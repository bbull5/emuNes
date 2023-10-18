#pragma once

#include <cstdint>
#include <string>
#include <fstream>
#include <vector>

#include "Mapper_0.h"


class Cartridge {
	public:
		Cartridge(std::string& fileName);	//Argument is path to iNES file
		~Cartridge();


		/* Read/Write With Main Bus */
		void cpu_write(uint16_t addr, uint8_t data);
		uint8_t cpu_read(uint16_t addr, uint8_t &data);

		/* Read/Write With PPU Bus */
		void ppu_write(uint16_t addr, uint8_t data);
		uint8_t ppu_read(uint16_t addr, uint8_t &data);

	private:
		std::vector<uint8_t> prgRom;		//Program ROM (interfaces with CPU)
		std::vector<uint8_t> chrRom;		//Character ROM (interfaces with PPU)

		uint8_t mapperID = 0;				//Mapper to be used
		uint8_t numPrgRom = 0;				//Number of PRG ROM banks
		uint8_t numChrRom = 0;				//Number of CHR ROM banks

		std::shared_ptr<Mapper> mapperPtr;	//Points to the appropriate mapper class
};