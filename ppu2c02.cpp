#include "ppu2c02.h"


ppu2c02::ppu2c02() {

}

ppu2c02::~ppu2c02() {

}


//Functionality omitted until PPU fully implemented
//For now, we just have the PPU registers layed out
void ppu2c02::cpu_write(uint16_t addr, uint8_t data) {
	switch (addr) {
	case 0x0000:	//PPUCTRL
		break;
	case 0x0001:	//PPUMASK
		break;
	case 0x0002:	//PPUSTATUS
		break;
	case 0x0003:	//OAMADDR (OAM read/write address)
		break;
	case 0x0004:	//OAMDATA (OAM data read/write)
		break;
	case 0x0005:	//PPUSCROLL - fine scroll position (two writes: X scroll, Y scroll)
		break;
	case 0x0006:	//PPUADDR (PPU read/write address)
		break;
	case 0x0007:	//PPUDATA (PPU data read/write)
		break;
	}
}

//Functionality omitted until PPU fully implemented.
//For now, we just have the PPU registers layed out
uint8_t ppu2c02::cpu_read(uint16_t addr) {
	uint8_t data = 0x00;

	switch (addr) {
	case 0x0000:	//PPUCTRL - Used by CPU to configure PPU state
		break;
	case 0x0001:	//PPUMASK - Decides if backgrounds or sprites are being drawn and behavior at screen edge
		break;
	case 0x0002:	//PPUSTATUS - 
		break;
	case 0x0003:	//OAMADDR (OAM read/write address)
		break;
	case 0x0004:	//OAMDATA (OAM data read/write)
		break;
	case 0x0005:	//PPUSCROLL - fine scroll position (two writes: X scroll, Y scroll)
		break;
	case 0x0006:	//PPUADDR (PPU read/write address)
		break;
	case 0x0007:	//PPUDATA (PPU data read/write)
		break;
	}

	return data;
}


//Does nothing for now
void ppu2c02::ppu_write(uint16_t addr, uint8_t data) {
	uint8_t d = 0x00;
	addr &= 0x3FFF;
}


//Does nothing for now
uint8_t ppu2c02::ppu_read(uint16_t addr) {
	uint8_t data = 0x00;
	addr &= 0x3FFF;

	return data;
}


void ppu2c02::connect_cart(const std::shared_ptr<Cartridge>& cartridge) {
	this->cart = cartridge;
}


/* Controls the internal frequency of the PPU which in turn dicates the 
   graphical output. 
   
   The NES has a resolution of 256x240 pixels. Note that the PPU scanlines
   are larger than that resolution (both vertically and horizontally). This
   is because the PPU uses the "off-screen" portion of its cycle to update its
   internal state without worrying about what its graphical output looks like */
void ppu2c02::clock() {
	cycles++;

	//A scanline is completed every 341 cycles
	if (cycles >= 341) {
		cycles = 0;
		scanline++;
		
		//There are 261 scanlines in a frame
		if (scanline >= 261) {
			scanline = -1;
			frame_done = true;
		}
	}
}