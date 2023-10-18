#include "Mapper_0.h"


Mapper_0::Mapper_0(uint8_t nPrgBanks, uint8_t nChrBanks) : Mapper(nPrgBanks, nChrBanks) {

}

Mapper_0::~Mapper_0() {

}


/* The CPU interfaces with the PRG ROM, so cpu_map_write and cpu_map_read 
   handle addresses provided to the cartridge mapper by the CPU */
/* Mapper 0 can have either 16KB or 32KB for the PRG ROM (specified in
   the iNES file header)

   if PRG ROM is 16KB (16KB address space is mirrored):
   CPU ADDRESS				PRG ROM ADDRESS
   0x8000 - 0xBFFF			0x0000 - 0x3FFF
   0xC000 - 0xFFFF			0x0000 - 0x3FFF
   if PRG ROM is 32KB (no mirroring):
   CPU ADDRESS				PRG ROM ADDRESS
   0x8000 - 0xFFFF			0x0000 - 0x7FFF
 */
bool Mapper_0::cpu_map_write(uint16_t addr, uint16_t& mappedAddr) {
	if (addr >= 0x8000 && addr <= 0x7FFF) {
		mappedAddr = addr & (numPrgBanks > 1 ? 0x7FFF : 0x3FFF);

		return true;
	}

	return false;
}


bool Mapper_0::cpu_map_read(uint16_t addr, uint16_t &mappedAddr) {
	if (addr >= 0x8000 && addr <= 0xFFFF) {
		mappedAddr = addr & (numPrgBanks > 1 ? 0x7FFF : 0x3FFF);

		return true;
	}

	return false;
}


/* The PPU interfaces with the CHR ROM, so ppu_map_write and ppu_map_read
   handle addresses provided to the cartridge mapper by the PPU */
/* There is no mapping required for the PPU 0

	PPU ADDRESS				CHR ROM ADDRESS
	0x0000 - 0x1FFF         0x0000 - 0x1FFF
*/

bool Mapper_0::ppu_map_write(uint16_t addr, uint16_t& mappedAddr) {
	if (addr >= 0x0000 && addr <= 0x1FFF) {
		//If there is no CHR ROM on the cartridge, that address space
		//is treated as RAM
		if (numChrBanks == 0) {
			mappedAddr = addr;

			return true;
		}
	}

	return false;
}


bool Mapper_0::ppu_map_read(uint16_t addr, uint16_t& mappedAddr) {
	//There is no mapping required for the PPU
	if (addr >= 0x0000 && addr <= 0x1FFF) {
		mappedAddr = addr;

		return true;
	}

	return false;
}
