// RetroAchievements RAM Mirror for NeoGeo — Option C + RTQuery  (v0x07)
//
// Each VBlank, reads a list of specific addresses from DDRAM (written by ARM),
// fetches byte values from 68K Work RAM (two 8-bit BRAMs), and writes them
// back to DDRAM.
//
// NeoGeo memory map for rcheevos (RC_CONSOLE_ARCADE = 27):
//   0x00000–0x0FFFF : 68K Work RAM (64KB)
//   0x10000+        : returns 0 (unsupported)
//
// NeoGeo Work RAM is split into two dpram #(15) instances:
//   WRAML (low byte,  M68K_DATA[7:0])  at word address M68K_ADDR[15:1]
//   WRAMU (high byte, M68K_DATA[15:8]) at word address M68K_ADDR[15:1]
//
// RA byte ordering (big-endian, native 68000 convention):
//   addr[0]=0 → upper byte (WRAMU)  — byte at even physical address
//   addr[0]=1 → lower byte (WRAML)  — byte at odd  physical address
//
// DDRAM Layout (at DDRAM_BASE, ARM phys 0x3D000000):
//   [0x00000] Header:   magic(32) + 0(8) + flags(8) + 0(16)
//   [0x00008] Frame:    frame_counter(32) + bsram_size(32)
//
//   [0x40000] AddrReq:  addr_count(32) + request_id(32)       (ARM → FPGA)
//   [0x40008] Addrs:    addr[0](32) + addr[1](32), ...        (2 per 64-bit word)
//
//   [0x48000] ValResp:  response_id(32) + response_frame(32)  (FPGA → ARM)
//   [0x48008] Values:   val[0..7](8b each), val[8..15], ...   (8 per 64-bit word)

module ra_ram_mirror_neogeo #(
	parameter [28:0] DDRAM_BASE = 29'h07A00000,  // ARM phys 0x3D000000 >> 3
	parameter BYPASS_BRAM = 0
)(
	input             clk,           // CLK_48M
	input             reset,
	input             vblank,        // ~nBNKB (active high during VBlank)

	// Work RAM BRAM read interface — shared address, two data outputs
	output reg [14:0] bram_addr,     // 15-bit word address for WRAML & WRAMU port B
	input       [7:0] wraml_dout,    // WRAML output (low byte)
	input       [7:0] wramu_dout,    // WRAMU output (high byte)

	// DDRAM write interface (toggle req/ack)
	output reg [28:0] ddram_wr_addr,
	output reg [63:0] ddram_wr_din,
	output reg  [7:0] ddram_wr_be,
	output reg        ddram_wr_req,
	input             ddram_wr_ack,

	// DDRAM read interface (toggle req/ack)
	output reg [28:0] ddram_rd_addr,
	output reg        ddram_rd_req,
	input             ddram_rd_ack,
	input      [63:0] ddram_rd_dout,

	// Status
	output reg        active,
	output reg [31:0] dbg_frame_counter
);

// ======================================================================
// Constants
// ======================================================================
localparam [28:0] ADDRLIST_BASE = DDRAM_BASE + 29'h8000;  // byte offset 0x40000 / 8
localparam [28:0] VALCACHE_BASE = DDRAM_BASE + 29'h9000;  // byte offset 0x48000 / 8
localparam [31:0] WRAM_LIMIT    = 32'h10000;              // 64KB boundary
localparam [12:0] MAX_ADDRS     = 13'd4096;

// Realtime query mailbox (Tier 1 smart cache)
localparam [28:0] QUERY_CTRL_ADDR = DDRAM_BASE + 29'hA000;
localparam [28:0] QUERY_REQ_BASE  = DDRAM_BASE + 29'hA001;
localparam [28:0] QUERY_RESP_BASE = DDRAM_BASE + 29'hA011;
localparam [3:0]  MAX_RT_QUERIES  = 4'd16;

// ======================================================================
// Clock domain crossing synchronizers for DDRAM ack
// ======================================================================
reg dwr_ack_s1, dwr_ack_s2;
reg drd_ack_s1, drd_ack_s2;
always @(posedge clk) begin
	dwr_ack_s1 <= ddram_wr_ack; dwr_ack_s2 <= dwr_ack_s1;
	drd_ack_s1 <= ddram_rd_ack; drd_ack_s2 <= drd_ack_s1;
end

// ======================================================================
// VBlank edge detection
// ======================================================================
reg vblank_prev;
wire vblank_rising = vblank & ~vblank_prev;
always @(posedge clk) vblank_prev <= vblank;

reg vblank_pending;
always @(posedge clk) begin
	if (reset)
		vblank_pending <= 1'b0;
	else if (vblank_rising)
		vblank_pending <= 1'b1;
	else if (state == S_IDLE && vblank_pending)
		vblank_pending <= 1'b0;
end

// ======================================================================
// State machine
// ======================================================================
localparam S_IDLE        = 6'd0;
localparam S_DD_WR_WAIT  = 6'd1;
localparam S_DD_RD_WAIT  = 6'd2;
localparam S_READ_HDR    = 6'd3;
localparam S_PARSE_HDR   = 6'd4;
localparam S_READ_PAIR   = 6'd5;
localparam S_PARSE_ADDR  = 6'd6;
localparam S_DISPATCH    = 6'd7;
localparam S_BRAM_WAIT   = 6'd8;
localparam S_BRAM_WAIT2  = 6'd9;
localparam S_STORE_VAL   = 6'd10;
localparam S_FLUSH_BUF   = 6'd11;
localparam S_WRITE_RESP  = 6'd12;
localparam S_WR_HDR0     = 6'd13;
localparam S_WR_HDR1     = 6'd14;
localparam S_WR_DBG      = 6'd15;
localparam S_WR_DBG2     = 6'd16;
localparam S_WR_DBG3     = 6'd17;
localparam S_WR_DBG4     = 6'd18;
localparam S_WR_DBG5     = 6'd19;
// RTQuery states
localparam S_QRY_PARSE   = 6'd20;
localparam S_QRY_RD_REQ  = 6'd21;
localparam S_QRY_FETCH   = 6'd22;
localparam S_QRY_BRAM_W1 = 6'd23;
localparam S_QRY_BRAM_W2 = 6'd24;
localparam S_QRY_WR_RESP = 6'd25;
localparam S_QRY_WR_CTRL = 6'd26;

reg [5:0]  state;
reg [5:0]  return_state;

reg [31:0] frame_counter;
always @(posedge clk) dbg_frame_counter <= frame_counter;

reg [63:0] rd_data;
reg [31:0] req_count;
reg [31:0] req_id;
reg [12:0] addr_idx;
reg [63:0] addr_word;
reg [31:0] cur_addr;
reg [63:0] collect_buf;
reg  [3:0] collect_cnt;
reg [12:0] val_word_idx;
reg  [7:0] fetch_byte;

// Debug counters
reg [15:0] dbg_ok_cnt;
reg [15:0] dbg_oob_cnt;
reg  [7:0] dbg_dispatch_cnt;

// Per-address debug capture for first 8 addresses
// Each record: {bram_addr[14:0], cur_addr[0], wraml_dout[7:0], wramu_dout[7:0]} = 32 bits
reg [31:0] dbg_rec0, dbg_rec1, dbg_rec2, dbg_rec3;
reg [31:0] dbg_rec4, dbg_rec5, dbg_rec6, dbg_rec7;

// RTQuery registers
reg  [7:0] qry_request_seq;
reg  [7:0] qry_last_seen_seq;
reg  [7:0] qry_num;
reg  [3:0] qry_idx;
reg [31:0] qry_addr;
reg  [7:0] qry_num_bytes;
reg [31:0] qry_value;
reg  [2:0] qry_byte_idx;
reg  [9:0] qry_poll_timer;
reg [19:0] ddram_wait_timeout;

// ======================================================================
// Main state machine
// ======================================================================
always @(posedge clk) begin
	if (reset) begin
		state        <= S_IDLE;
		active       <= 1'b0;
		frame_counter <= 32'd0;
		ddram_wr_req <= dwr_ack_s2;
		ddram_rd_req <= drd_ack_s2;
		qry_last_seen_seq <= 8'd0;
		qry_poll_timer <= 10'd0;
	end
	else begin
		case (state)

		S_IDLE: begin
			active <= 1'b0;
			if (vblank_pending) begin
				active <= 1'b1;
				qry_poll_timer   <= 10'd0;
				dbg_ok_cnt       <= 16'd0;
				dbg_oob_cnt      <= 16'd0;
				dbg_dispatch_cnt <= 8'd0;
				dbg_rec0 <= 32'd0; dbg_rec1 <= 32'd0;
				dbg_rec2 <= 32'd0; dbg_rec3 <= 32'd0;
				dbg_rec4 <= 32'd0; dbg_rec5 <= 32'd0;
				dbg_rec6 <= 32'd0; dbg_rec7 <= 32'd0;
				ddram_wr_addr <= DDRAM_BASE;
				ddram_wr_din  <= {16'd0, 8'h01, 8'd0, 32'h52414348};
				ddram_wr_be   <= 8'hFF;
				ddram_wr_req  <= ~ddram_wr_req;
				return_state  <= S_READ_HDR;
				state         <= S_DD_WR_WAIT;
			end
			else if (qry_poll_timer < 10'd1000) begin
				qry_poll_timer <= qry_poll_timer + 10'd1;
			end
			else begin
				qry_poll_timer <= 10'd0;
				ddram_rd_addr <= QUERY_CTRL_ADDR;
				ddram_rd_req  <= ~ddram_rd_req;
				return_state  <= S_QRY_PARSE;
				state         <= S_DD_RD_WAIT;
			end
		end

		S_DD_WR_WAIT: begin
			ddram_wait_timeout <= ddram_wait_timeout + 20'd1;
			if (ddram_wr_req == dwr_ack_s2) begin
				ddram_wait_timeout <= 20'd0;
				state <= return_state;
			end else if (ddram_wait_timeout >= 20'hFFFFF) begin
				ddram_wait_timeout <= 20'd0;
				state <= S_IDLE;
			end
		end

		S_DD_RD_WAIT: begin
			ddram_wait_timeout <= ddram_wait_timeout + 20'd1;
			if (ddram_rd_req == drd_ack_s2) begin
				ddram_wait_timeout <= 20'd0;
				rd_data <= ddram_rd_dout;
				state   <= return_state;
			end else if (ddram_wait_timeout >= 20'hFFFFF) begin
				ddram_wait_timeout <= 20'd0;
				state <= S_IDLE;
			end
		end

		S_READ_HDR: begin
			ddram_rd_addr <= ADDRLIST_BASE;
			ddram_rd_req  <= ~ddram_rd_req;
			return_state  <= S_PARSE_HDR;
			state         <= S_DD_RD_WAIT;
		end

		S_PARSE_HDR: begin
			req_id <= rd_data[63:32];
			if (rd_data[31:0] == 32'd0) begin
				req_count <= 32'd0;
				state     <= S_WRITE_RESP;
			end else begin
				req_count    <= (rd_data[31:0] > {19'd0, MAX_ADDRS}) ?
				                {19'd0, MAX_ADDRS} : rd_data[31:0];
				addr_idx     <= 13'd0;
				collect_cnt  <= 4'd0;
				collect_buf  <= 64'd0;
				val_word_idx <= 13'd0;
				state        <= S_READ_PAIR;
			end
		end

		S_READ_PAIR: begin
			ddram_rd_addr <= ADDRLIST_BASE + 29'd1 + {16'd0, addr_idx[12:1]};
			ddram_rd_req  <= ~ddram_rd_req;
			return_state  <= S_PARSE_ADDR;
			state         <= S_DD_RD_WAIT;
		end

		S_PARSE_ADDR: begin
			// NeoGeo: no address bit inversion needed (unlike MegaDrive ym6045)
			// BRAM word address = byte_addr[15:1]
			if (!addr_idx[0]) begin
				addr_word <= rd_data;
				cur_addr  <= rd_data[31:0];
				bram_addr <= rd_data[15:1];
			end else begin
				cur_addr  <= addr_word[63:32];
				bram_addr <= addr_word[47:33];
			end
			state <= S_DISPATCH;
		end

		S_DISPATCH: begin
			dbg_dispatch_cnt <= dbg_dispatch_cnt + 8'd1;
			if (BYPASS_BRAM) begin
				fetch_byte <= cur_addr[7:0] ^ 8'hA5;
				state      <= S_STORE_VAL;
			end
			else if (cur_addr < WRAM_LIMIT) begin
				dbg_ok_cnt <= dbg_ok_cnt + 16'd1;
				state      <= S_BRAM_WAIT;
			end
			else begin
				fetch_byte <= 8'd0;
				dbg_oob_cnt <= dbg_oob_cnt + 16'd1;
				state      <= S_STORE_VAL;
			end
		end

		S_BRAM_WAIT: begin
			state <= S_BRAM_WAIT2;
		end

		S_BRAM_WAIT2: begin
			// Capture per-address debug for first 8 addresses
			// Record: {bram_addr[14:0], cur_addr[0], wraml_dout[7:0], wramu_dout[7:0]}
			case (addr_idx[2:0])
				3'd0: if (addr_idx < 13'd8) dbg_rec0 <= {bram_addr, cur_addr[0], wraml_dout, wramu_dout};
				3'd1: if (addr_idx < 13'd8) dbg_rec1 <= {bram_addr, cur_addr[0], wraml_dout, wramu_dout};
				3'd2: if (addr_idx < 13'd8) dbg_rec2 <= {bram_addr, cur_addr[0], wraml_dout, wramu_dout};
				3'd3: if (addr_idx < 13'd8) dbg_rec3 <= {bram_addr, cur_addr[0], wraml_dout, wramu_dout};
				3'd4: if (addr_idx < 13'd8) dbg_rec4 <= {bram_addr, cur_addr[0], wraml_dout, wramu_dout};
				3'd5: if (addr_idx < 13'd8) dbg_rec5 <= {bram_addr, cur_addr[0], wraml_dout, wramu_dout};
				3'd6: if (addr_idx < 13'd8) dbg_rec6 <= {bram_addr, cur_addr[0], wraml_dout, wramu_dout};
				3'd7: if (addr_idx < 13'd8) dbg_rec7 <= {bram_addr, cur_addr[0], wraml_dout, wramu_dout};
			endcase
					// NeoGeo byte selection from two separate 8-bit BRAMs:
					//   68000 is big-endian:
					//     even address (addr[0]=0) → upper byte → WRAMU
					//     odd  address (addr[0]=1) → lower byte → WRAML
					// NOTE: This gives native 68K byte order. The ARM side
					// applies XOR to match RA/LE emulator convention.
					if (cur_addr[0])
						fetch_byte <= wraml_dout;
					else
						fetch_byte <= wramu_dout;
					state <= S_STORE_VAL;
		end

		S_STORE_VAL: begin
			case (collect_cnt[2:0])
				3'd0: collect_buf[ 7: 0] <= fetch_byte;
				3'd1: collect_buf[15: 8] <= fetch_byte;
				3'd2: collect_buf[23:16] <= fetch_byte;
				3'd3: collect_buf[31:24] <= fetch_byte;
				3'd4: collect_buf[39:32] <= fetch_byte;
				3'd5: collect_buf[47:40] <= fetch_byte;
				3'd6: collect_buf[55:48] <= fetch_byte;
				3'd7: collect_buf[63:56] <= fetch_byte;
			endcase
			collect_cnt <= collect_cnt + 4'd1;
			addr_idx    <= addr_idx + 13'd1;

			if (collect_cnt == 4'd7 || (addr_idx + 13'd1 >= req_count[12:0])) begin
				state <= S_FLUSH_BUF;
			end
			else if (addr_idx[0]) begin
				state <= S_READ_PAIR;
			end else begin
				state <= S_PARSE_ADDR;
			end
		end

		S_FLUSH_BUF: begin
			ddram_wr_addr <= VALCACHE_BASE + 29'd1 + {16'd0, val_word_idx};
			ddram_wr_din  <= collect_buf;
			ddram_wr_be   <= (collect_cnt == 4'd8) ? 8'hFF
			                 : ((8'd1 << collect_cnt[2:0]) - 8'd1);
			ddram_wr_req  <= ~ddram_wr_req;
			val_word_idx  <= val_word_idx + 13'd1;
			collect_cnt   <= 4'd0;
			collect_buf   <= 64'd0;

			if (addr_idx >= req_count[12:0]) begin
				return_state <= S_WRITE_RESP;
			end else if (!addr_idx[0]) begin
				return_state <= S_READ_PAIR;
			end else begin
				return_state <= S_PARSE_ADDR;
			end
			state <= S_DD_WR_WAIT;
		end

		S_WRITE_RESP: begin
			ddram_wr_addr <= VALCACHE_BASE;
			ddram_wr_din  <= {frame_counter + 32'd1, req_id};
			ddram_wr_be   <= 8'hFF;
			ddram_wr_req  <= ~ddram_wr_req;
			return_state  <= S_WR_HDR0;
			state         <= S_DD_WR_WAIT;
		end

		S_WR_HDR0: begin
			ddram_wr_addr <= DDRAM_BASE;
			ddram_wr_din  <= {16'd0, 8'h00, 8'd0, 32'h52414348};
			ddram_wr_be   <= 8'hFF;
			ddram_wr_req  <= ~ddram_wr_req;
			return_state  <= S_WR_HDR1;
			state         <= S_DD_WR_WAIT;
		end

		S_WR_HDR1: begin
			ddram_wr_addr <= DDRAM_BASE + 29'd1;
			ddram_wr_din  <= {32'd0, frame_counter + 32'd1};
			ddram_wr_be   <= 8'hFF;
			ddram_wr_req  <= ~ddram_wr_req;
			frame_counter <= frame_counter + 32'd1;
			return_state  <= S_WR_DBG;
			state         <= S_DD_WR_WAIT;
		end

		// Debug word 1 @ DDRAM_BASE+2 (offset 0x10):
		// {ver(8), dispatch(8), ok(16), oob(16), 0(16)}
		S_WR_DBG: begin
			ddram_wr_addr <= DDRAM_BASE + 29'd2;
			ddram_wr_din  <= {8'h07, dbg_dispatch_cnt, dbg_ok_cnt, dbg_oob_cnt, 16'd0};
			ddram_wr_be   <= 8'hFF;
			ddram_wr_req  <= ~ddram_wr_req;
			return_state  <= S_WR_DBG2;
			state         <= S_DD_WR_WAIT;
		end

		// Debug word 2 @ DDRAM_BASE+3 (offset 0x18): rec[1](32) | rec[0](32)
		S_WR_DBG2: begin
			ddram_wr_addr <= DDRAM_BASE + 29'd3;
			ddram_wr_din  <= {dbg_rec1, dbg_rec0};
			ddram_wr_be   <= 8'hFF;
			ddram_wr_req  <= ~ddram_wr_req;
			return_state  <= S_WR_DBG3;
			state         <= S_DD_WR_WAIT;
		end

		// Debug word 3 @ DDRAM_BASE+4 (offset 0x20): rec[3](32) | rec[2](32)
		S_WR_DBG3: begin
			ddram_wr_addr <= DDRAM_BASE + 29'd4;
			ddram_wr_din  <= {dbg_rec3, dbg_rec2};
			ddram_wr_be   <= 8'hFF;
			ddram_wr_req  <= ~ddram_wr_req;
			return_state  <= S_WR_DBG4;
			state         <= S_DD_WR_WAIT;
		end

		// Debug word 4 @ DDRAM_BASE+5 (offset 0x28): rec[5](32) | rec[4](32)
		S_WR_DBG4: begin
			ddram_wr_addr <= DDRAM_BASE + 29'd5;
			ddram_wr_din  <= {dbg_rec5, dbg_rec4};
			ddram_wr_be   <= 8'hFF;
			ddram_wr_req  <= ~ddram_wr_req;
			return_state  <= S_WR_DBG5;
			state         <= S_DD_WR_WAIT;
		end

		// Debug word 5 @ DDRAM_BASE+6 (offset 0x30): rec[7](32) | rec[6](32)
		S_WR_DBG5: begin
			ddram_wr_addr <= DDRAM_BASE + 29'd6;
			ddram_wr_din  <= {dbg_rec7, dbg_rec6};
			ddram_wr_be   <= 8'hFF;
			ddram_wr_req  <= ~ddram_wr_req;
			return_state  <= S_IDLE;
			state         <= S_DD_WR_WAIT;
		end

		// =============================================================
		// RTQuery States
		// =============================================================
		S_QRY_PARSE: begin
			if (rd_data[7:0] != qry_last_seen_seq && rd_data[15:8] != 8'd0) begin
				qry_request_seq <= rd_data[7:0];
				qry_num         <= (rd_data[15:8] > {4'd0, MAX_RT_QUERIES}) ?
				                   {4'd0, MAX_RT_QUERIES} : rd_data[15:8];
				qry_idx         <= 4'd0;
				state           <= S_QRY_RD_REQ;
			end else begin
				state <= S_IDLE;
			end
		end

		S_QRY_RD_REQ: begin
			ddram_rd_addr <= QUERY_REQ_BASE + {25'd0, qry_idx};
			ddram_rd_req  <= ~ddram_rd_req;
			return_state  <= S_QRY_FETCH;
			state         <= S_DD_RD_WAIT;
		end

		S_QRY_FETCH: begin
			qry_addr      <= rd_data[31:0];
			qry_num_bytes <= (rd_data[39:32] == 8'd0) ? 8'd1 : rd_data[39:32];
			qry_value     <= 32'd0;
			qry_byte_idx  <= 3'd0;
			if (rd_data[31:0] < WRAM_LIMIT) begin
				bram_addr <= rd_data[15:1];
				state     <= S_QRY_BRAM_W1;
			end else begin
				state <= S_QRY_WR_RESP;
			end
		end

		S_QRY_BRAM_W1: begin
			state <= S_QRY_BRAM_W2;
		end

		S_QRY_BRAM_W2: begin
			// Same byte selection as batch: addr[0]=0→WRAMU, addr[0]=1→WRAML
			if (qry_addr[0])
				qry_value <= qry_value | ({24'd0, wraml_dout} << (qry_byte_idx * 8));
			else
				qry_value <= qry_value | ({24'd0, wramu_dout} << (qry_byte_idx * 8));
			qry_byte_idx <= qry_byte_idx + 3'd1;
			if (qry_byte_idx + 3'd1 >= qry_num_bytes[2:0]) begin
				state <= S_QRY_WR_RESP;
			end else begin
				qry_addr  <= qry_addr + 32'd1;
				bram_addr <= (qry_addr + 32'd1) >> 1;
				state     <= S_QRY_BRAM_W1;
			end
		end

		S_QRY_WR_RESP: begin
			ddram_wr_addr <= QUERY_RESP_BASE + {25'd0, qry_idx};
			ddram_wr_din  <= {32'd0, qry_value};
			ddram_wr_be   <= 8'hFF;
			ddram_wr_req  <= ~ddram_wr_req;
			qry_idx       <= qry_idx + 4'd1;
			if (qry_idx + 4'd1 >= qry_num[3:0])
				return_state <= S_QRY_WR_CTRL;
			else
				return_state <= S_QRY_RD_REQ;
			state <= S_DD_WR_WAIT;
		end

		S_QRY_WR_CTRL: begin
			qry_last_seen_seq <= qry_request_seq;
			ddram_wr_addr     <= QUERY_CTRL_ADDR;
			ddram_wr_din      <= {24'd0, qry_request_seq, 16'd0, qry_num[7:0], qry_request_seq};
			ddram_wr_be       <= 8'hFF;
			ddram_wr_req      <= ~ddram_wr_req;
			return_state      <= S_IDLE;
			state             <= S_DD_WR_WAIT;
		end

		default: state <= S_IDLE;
		endcase
	end
end

endmodule
