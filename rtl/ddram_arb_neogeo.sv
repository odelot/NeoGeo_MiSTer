// DDRAM Arbiter for NeoGeo RetroAchievements
//
// Sits between the existing ddram module and the physical DDRAM interface.
// The existing ddram module (ADPCM + Z80 ROM) is the primary master.
// RA mirror is secondary (toggle req/ack protocol, read+write).
// RA requests are serviced when the primary master is idle on the bus.
//
// RA toggle signals may originate from a different clock domain
// (CLK_48M vs CLK_96M), so they are synchronized.

module ddram_arb_neogeo (
	input         clk,            // DDRAM_CLK (CLK_96M)

	// Physical DDRAM interface (directly to top-level DDRAM ports)
	input         PHY_BUSY,
	output  [7:0] PHY_BURSTCNT,
	output [28:0] PHY_ADDR,
	input  [63:0] PHY_DOUT,
	input         PHY_DOUT_READY,
	output        PHY_RD,
	output [63:0] PHY_DIN,
	output  [7:0] PHY_BE,
	output        PHY_WE,

	// Primary master DDRAM interface (existing ddram module)
	output        PRI_BUSY,
	input   [7:0] PRI_BURSTCNT,
	input  [28:0] PRI_ADDR,
	output [63:0] PRI_DOUT,
	output        PRI_DOUT_READY,
	input         PRI_RD,
	input  [63:0] PRI_DIN,
	input   [7:0] PRI_BE,
	input         PRI_WE,

	// RetroAchievements write channel (toggle req/ack)
	input  [28:0] ra_wr_addr,
	input  [63:0] ra_wr_din,
	input   [7:0] ra_wr_be,
	input         ra_wr_req,
	output reg    ra_wr_ack,

	// RetroAchievements read channel (toggle req/ack)
	input  [28:0] ra_rd_addr,
	input         ra_rd_req,
	output reg    ra_rd_ack,
	output reg [63:0] ra_rd_dout
);

// Synchronize RA toggle signals (may come from CLK_48M domain)
reg ra_wr_req_s1, ra_wr_req_s2;
reg ra_rd_req_s1, ra_rd_req_s2;
always @(posedge clk) begin
	ra_wr_req_s1 <= ra_wr_req; ra_wr_req_s2 <= ra_wr_req_s1;
	ra_rd_req_s1 <= ra_rd_req; ra_rd_req_s2 <= ra_rd_req_s1;
end

// State machine
localparam S_PASSTHRU = 2'd0;
localparam S_RA_WR    = 2'd1;
localparam S_RA_RD    = 2'd2;
localparam S_RA_WAIT  = 2'd3;

reg [1:0] state = S_PASSTHRU;

// Track pending primary master read bursts
reg        pri_rd_active = 0;
reg  [7:0] pri_burst_cnt = 0;

// Combinational mux
assign PHY_BURSTCNT = (state == S_PASSTHRU) ? PRI_BURSTCNT : 8'd1;
assign PHY_ADDR     = (state == S_PASSTHRU) ? PRI_ADDR     :
                      (state == S_RA_WR)    ? ra_wr_addr   : ra_rd_addr;
assign PHY_DIN      = (state == S_PASSTHRU) ? PRI_DIN      : ra_wr_din;
assign PHY_BE       = (state == S_PASSTHRU) ? PRI_BE       :
                      (state == S_RA_WR)    ? ra_wr_be     : 8'hFF;
assign PHY_WE       = (state == S_RA_WR)    ? 1'b1         :
                      (state == S_PASSTHRU) ? PRI_WE       : 1'b0;
assign PHY_RD       = (state == S_RA_RD)    ? 1'b1         :
                      (state == S_PASSTHRU) ? PRI_RD       : 1'b0;

assign PRI_BUSY       = (state != S_PASSTHRU) ? 1'b1 : PHY_BUSY;
assign PRI_DOUT       = PHY_DOUT;
assign PRI_DOUT_READY = (state == S_PASSTHRU) ? PHY_DOUT_READY : 1'b0;

always @(posedge clk) begin
	// Track pending primary read bursts
	if (state == S_PASSTHRU) begin
		if (PRI_RD && !PHY_BUSY) begin
			pri_rd_active <= 1'b1;
			pri_burst_cnt <= PRI_BURSTCNT;
		end
		if (pri_rd_active && PHY_DOUT_READY) begin
			if (pri_burst_cnt <= 8'd1)
				pri_rd_active <= 1'b0;
			else
				pri_burst_cnt <= pri_burst_cnt - 8'd1;
		end
	end

	case (state)
	S_PASSTHRU: begin
		// Only steal bus when primary is idle
		if (!PRI_WE && !PRI_RD && !PHY_BUSY && !pri_rd_active) begin
			if (ra_wr_req_s2 != ra_wr_ack)
				state <= S_RA_WR;
			else if (ra_rd_req_s2 != ra_rd_ack)
				state <= S_RA_RD;
		end
	end

	S_RA_WR: begin
		if (!PHY_BUSY) begin
			ra_wr_ack <= ra_wr_req_s2;
			state <= S_PASSTHRU;
		end
	end

	S_RA_RD: begin
		if (!PHY_BUSY) begin
			state <= S_RA_WAIT;
		end
	end

	S_RA_WAIT: begin
		if (PHY_DOUT_READY) begin
			ra_rd_dout <= PHY_DOUT;
			ra_rd_ack <= ra_rd_req_s2;
			state <= S_PASSTHRU;
		end
	end
	endcase
end

endmodule
