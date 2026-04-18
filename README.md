
# NeoGeo_MiSTer — RetroAchievements Fork

This is a fork of the [MiSTer-devel NeoGeo core](https://github.com/MiSTer-devel/NeoGeo_MiSTer) with modifications to support **RetroAchievements** on MiSTer FPGA.

> **Status:** Experimental / Proof of Concept
>
> This core must be used together with the [modified Main_MiSTer binary](https://github.com/odelot/Main_MiSTer).

## How to Test

Pre-built `.rbf` files are available on the [Releases](https://github.com/odelot/NeoGeo_MiSTer/releases) page — no need to compile the core yourself.

1. Download the latest `NeoGeo_*.rbf` from this repo's [Releases](https://github.com/odelot/NeoGeo_MiSTer/releases).
2. Copy it to `/media/fat/_Console/` (or `_Arcade/`) on your MiSTer SD card, replacing the original NeoGeo core.
3. You also need the **modified Main_MiSTer binary** — download it from [odelot/Main_MiSTer Releases](https://github.com/odelot/Main_MiSTer/releases) and follow its README to set up `retroachievements.cfg`.
4. Set up your NeoGeo ROMs and BIOS files as described in the [Original Core Features](#original-core-features) section below.
5. Reboot, load the NeoGeo core, and open a game that has achievements on [retroachievements.org](https://retroachievements.org/).

## What's Different from the Original

The [upstream NeoGeo core](https://github.com/MiSTer-devel/NeoGeo_MiSTer) is an FPGA implementation of the Neo Geo / MVS system by [Furrtek](https://www.patreon.com/furrtek/posts). This fork adds a **RAM mirroring layer** that exposes the 68K Work RAM to the ARM CPU for RetroAchievements evaluation, without changing any existing core behaviour.

### Files Added

| File | Purpose |
|------|---------|
| `rtl/ra_ram_mirror_neogeo.sv` | Reads 68K Work RAM from BRAM on every VBlank and writes requested values to DDRAM using the Selective Address (Option C) protocol |
| `rtl/ddram_arb_neogeo.sv` | DDRAM bus arbiter — shares DDRAM access between the existing controller (ADPCM / Z80 ROM) and the RA mirror |

### Files Modified

| File | Change |
|------|--------|
| `neogeo.sv` | Converts WRAML / WRAMU to dual-port RAM (`dpram`), adds CD-mode shadow BRAMs for 68K Work RAM, instantiates the RA mirror and DDRAM arbiter, inserts arbiter between existing `ddram` module and physical DDRAM interface |
| `files.qip` | Adds the two new `.sv` files to the Quartus project |

### How the RAM Mirror Works

The NeoGeo RA integration uses the **Selective Address (Option C)** protocol. The ARM binary writes a list of memory addresses it needs into a DDRAM request area; the FPGA reads only those addresses from the 68K Work RAM and writes the values back to a DDRAM response area.

**Memory exposed:**
- **68K Work RAM** (`$00000–$0FFFF`) — 64 KB

**RAM architecture:**
The core stores 68K Work RAM in two separate 15-bit dual-port BRAMs — WRAML (low byte, `M68K_DATA[7:0]`) and WRAMU (high byte, `M68K_DATA[15:8]`). The RA mirror reads Port B of both BRAMs using the byte address: even addresses (`addr[0]=0`) select WRAMU and odd addresses (`addr[0]=1`) select WRAML, following the 68K big-endian convention. No address inversion or byte reordering is needed.

**CD mode handling:**
In CD mode, the WRAML/WRAMU BRAMs are repurposed for Z80 RAM and the 68K Work RAM lives in SDRAM (`$100000–$10FFFF`). Shadow DPRAMs capture both CPU writes (via `~nWWL`/`~nWWU`) and DMA writes to keep the RA mirror working. The mirror automatically muxes between the MVS BRAMs and the CD shadow BRAMs depending on the active system mode.

**DDRAM arbitration:**
A custom arbiter (`ddram_arb_neogeo.sv`) sits between the existing DDRAM controller (ADPCM samples / Z80 ROM reads) and the physical DDRAM interface. The RA mirror steals bus cycles when the primary master is idle. Two-stage flip-flop synchronizers handle clock domain crossing between CLK_48M (RA mirror) and CLK_96M (DDRAM).

**Data flow:**
```
68K CPU / DMA
      ↓ writes
WRAML + WRAMU BRAMs (64 KB)          ARM (Main_MiSTer)
  [or CD shadow BRAMs]                     ↓
      ↓ Port B read                   writes address list
RA Mirror State Machine               to DDRAM 0x3D040000
      ↓                                    ↓
DDRAM Arbiter ←──────────────────→ reads values from
      ↓                            DDRAM 0x3D048000
Physical DDRAM                          ↓
(ARM phys 0x3D000000)             rcheevos evaluates
                                  achievement conditions
```

## Original Core Features

Below is the original README content from the upstream NeoGeo core.

---

### [SNK Neo Geo](https://en.wikipedia.org/wiki/Neo_Geo_(system)) for [MiSTer Platform](https://github.com/MiSTer-devel/Main_MiSTer/wiki)

FPGA implementation of the NEO GEO/MVS system by [Furrtek](https://www.patreon.com/furrtek/posts).

**Features:**
* Supports memory card saving
* MVS, AES, and CD system support
* Compatible with Darksoft ROM sets using provided XML
* Compatible with decrypted MAME ROM sets using your own XML
* Compatible with decrypted .neo ROM sets
* Compatible with gog.com ROMs using provided XML
* Support for Universe BIOS

Note: This core does not support encrypted ROMs. Make sure the ROM has no encrypted parts before using. MAME ROM pack includes many encrypted ROMs so it's not recommended for inexperienced users. Using the .neo conversion tool with a MAME ROM set will result in some ROMs still being encrypted. There is an alternate .neo conversion tool for the Darksoft ROM set that will give you a fully decrypted set.

[MAME to .neo conversion tool](https://github.com/city41/neosdconv)

[Darksoft to .neo conversion tool](https://gitlab.com/loic.petit/darksoft-to-neosd/)

**Installation:**
Copy the NeoGeo\_\*.rbf file to the 'Console' or 'Arcade' folder (your choice) on the SD card. ROMs should go in the 'games\NeoGeo' folder. For ease of use, it is strongly suggested that people use the Darksoft ROM pack. These can be either zipped or unzipped with minimal loading speed difference. Several things must be observed:
* When using an unzipped Darksoft ROM set, each game's ROM files must be in their own folder and the folders containing the ROM sets must be named to match the XML (MAME standard names)
* ROMs can be placed inside sub-folders for organization
* Zipped ROMs must **not** contain folders.

In addition, several bios files must be placed in the 'games\NeoGeo' folder for the core to function properly:
* 000-lo.lo
* sfix.sfix
* sp-s2.sp1 (MVS)
* neo-epo.sp1 (AES)
* uni-bios.rom

These bios files must be placed in the 'games\NeoGeo-CD' folder:
* top-sp1.bin (CD)
* neocd.bin (CDZ)
* uni-bioscd.rom (CD and CDZ)

Sometimes these files may be named slightly different depending on where they are obtained, but they must be renamed to match the filenames above to work with MiSTer. You may choose between using the original system BIOS (sp-s2.sp1/neo-epo.sp1) or UniBios (uni-bios.rom). Using UniBios is recommended, and can be obtained [here](http://unibios.free.fr/).

Lastly, **romsets.xml** from the release folder must also be placed in the directory. The provided XML is for Darksoft ROMs only, you must make your own for MAME ROMs. This file describes to the core where the ROM sets are located and how to load them. **gog-romsets.xml** can be used (renamed to **romsets.xml**) for games purchased from gog.com (which also include all the needed bios files), see comments in gog-romsets.xml.

**Saving and Loading:**
In AES mode, all saves are to the memory card only. In MVS mode, some games and UniBios save their settings to a special area of battery backed ram built into the system, while game data can still be saved to a memory card.

**RAM and Game Sizes:**
Neo Geo uses very large ROMs. About 84% of the library will fit onto a 32 megabyte SDRAM module. Another 12% will fit onto a 64 megabyte SDRAM module. The remaining 8 games require a 128 megabyte module. For more information about which games can be loaded with which sized RAM, open romsets.xml in your favorite text editor or github. The games are organized by size.
