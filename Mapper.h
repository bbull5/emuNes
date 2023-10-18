//This class serves as an abstract base class for all implemented mappers. Each individual
//mapper inherits from this class.

#pragma once

#include <cstdint>


class Mapper {
	public:
		Mapper(uint8_t nPrgBanks, uint8_t nChrBanks);
		~Mapper();

		/* Read/Write With Main Bus */
		virtual bool cpu_map_write(uint16_t addr, uint16_t &mappedAddr) = 0;
		virtual bool cpu_map_read(uint16_t addr, uint16_t &mappedAddr) = 0;
		
		/* Read/Write With PPU Bus */
		virtual bool ppu_map_write(uint16_t addr, uint16_t &mappedAddr) = 0;
		virtual bool ppu_map_read(uint16_t addr, uint16_t &mappedAddr) = 0;


	protected:
		/* Number of "chunks" of each kind of ROM */
		uint8_t numPrgBanks = 0;
		uint8_t numChrBanks = 0;


	private:


};