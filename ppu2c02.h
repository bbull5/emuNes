#pragma once

#include <cstdint>
#include <memory>

#include "Cartridge.h"


class ppu2c02 {
	public:
		ppu2c02();
		~ppu2c02();

		/* Read/Write With Main Bus */
		void cpu_write(uint16_t addr, uint8_t data);
		uint8_t cpu_read(uint16_t addr);

		/* Read/Write With PPU Bus */
		void ppu_write(uint16_t addr, uint8_t data);
		uint8_t ppu_read(uint16_t addr);

		/* PPU Interface*/
		void connect_cart(const std::shared_ptr<Cartridge>& cartridge);
		void clock();

		/* Public flag to signal when current frame has been drawn */
		bool frame_done = false;

	private:
		std::shared_ptr<Cartridge> cart;	//Local pointer to cartridge

		/* Internal PPU Memory */
		uint8_t nameTable[2][1024];			//There are two name tables in the NES hardware
											//It is a 1KB area of memory used to lay out
											//backgrounds.
											//Each byte in the name table controls one 8x8 pixel
											//character cell, and each nametable has 30 rows of
											//of 32 tiles each, for 960(0x2C0) bytes.
											//With each tile being 8x8 pixels, this makes a total
											//of 256x240 pixels in one map, the smae size as one 
											//full screen
											//Each name table has a corresponding attribute table

		uint8_t paletteTable[32];

		/* Variables used internally by PPU */
		int16_t scanline = 0;	//This variable is signed, because counting
		//scanlines always starts at -1
		uint16_t cycles = 0;


	private:
		/* PPU Registers (not completed) */
		/* Note that for all PPU regtisters, each bit in the register 
		   represents a specific function or state of the PPU. Refer to 
		   the PPU programming guide on the nesdev wiki for documentation 
		   on each register. */
		/* In the following implementation, each register is represented as
		   a bitmask. This is likely not the simplest approach, but for now 
		   treat it as a workable prototype. 
		   
		   Update: It is likely to be simpler and involve less overhead to 
		   represent each register as a single uint8_t data type. See the 
		   implementation of the 6502 status register for an example. */
		union {
			struct {
				uint8_t nametableX : 1;
				uint8_t nametableY : 1;
				uint8_t incrementVRAMAddr : 1;
				uint8_t spritePatternAddress : 1;
				uint8_t bkgrndPatternAddress : 1;
				uint8_t spriteSize : 1;
				uint8_t slaveMode : 1;		//Not used
				uint8_t enableNMI : 1;
			};
			uint8_t reg;
		}ctrl;
		
		
		union {
			struct {
				uint8_t unused : 5;
				uint8_t spriteOverflow : 1;
				uint8_t spriteZero : 1;
				uint8_t verticalBlank : 1;
			};
			uint8_t reg;
		} status;


		union {
			struct {
				uint8_t grayscale : 1;
				uint8_t renderBkgrndLeft : 1;
				uint8_t renderSpritesLeft : 1;
				uint8_t renderBkgrnd : 1;
				uint8_t renderSprites : 1;
				uint8_t emphasizeRed : 1;
				uint8_t emphasizeGreen : 1;
				uint8_t empahsizeBlue : 1;
			};
			uint8_t reg;
		} mask;



};