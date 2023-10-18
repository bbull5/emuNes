#include "Cartridge.h"


Cartridge::Cartridge(std::string& fileName) {
	//iNES File Header
	struct Header {
		char name[4];
		uint8_t prgRomChunks;
		uint8_t chrRomChunks;
		uint8_t mapper1;
		uint8_t mapper2;
		uint8_t prgRomSize;
		uint8_t tvSystem1;
		uint8_t tvSystem2;
		char unused[5];
	} header;

	//Open and read iNES header
	std::ifstream nesFile;
	nesFile.open(fileName, std::ifstream::binary);
	if (nesFile.is_open()) {
		//Read iNES file header
		nesFile.read((char*)&header, sizeof(Header));

		//The first 512 bytes after the header is used for training
		//data. This is treated as junk by this emulator and can be
		//ignored.
		//See iNES file format page on nesdev.wiki for more details
		if (header.mapper1 & 0x04) {
			nesFile.seekg(512, std::ios_base::cur);
		}

		//Determine mapper ID
		mapperID = ((header.mapper2 >> 4) << 4) | header.mapper1 >> 4;

		//Determine iNES file type
		//There are three iNES file types. For the time being, we
		//will only handle type 1
		uint8_t fileType = 1;

		//Placeholder
		if (fileType == 0) {

		}

		if (fileType == 1) {
			//Use the PRG ROM size entry in header to resize the 
			//prgRom vector, and read in the PRG ROM data from the iNES file
			numPrgRom = header.prgRomChunks;
			prgRom.resize(numPrgRom * 16384);
			nesFile.read((char*)prgRom.data(), prgRom.size());

			//Use the CHR ROM size entry in header to resize the
			//chrRom vector, and read in the CHR ROM data from the iNES file
			numChrRom = header.chrRomChunks;
			chrRom.resize(numChrRom * 8192);
			nesFile.read((char*)chrRom.data(), chrRom.size());
		}

		//Placeholder
		if (fileType == 2) {

		}

		//Load correct mapper
		switch (mapperID) {
		case 0: mapperPtr = std::make_shared<Mapper_0>(numPrgRom, numChrRom);
			break;
		}

		nesFile.close();
	}
}

Cartridge::~Cartridge() {

}



void Cartridge::cpu_write(uint16_t addr, uint8_t data) {
	uint16_t mappedAddr = 0x0000;

	if (mapperPtr->cpu_map_write(addr, mappedAddr)) {
		prgRom[mappedAddr] = data;
	}
}


uint8_t Cartridge::cpu_read(uint16_t addr, uint8_t &data) {
	uint16_t mappedAddr = 0x0000;

	if (mapperPtr->cpu_map_read(addr, mappedAddr)) {
		data = prgRom[mappedAddr];
	}

	return data;
}


void Cartridge::ppu_write(uint16_t addr, uint8_t data) {
	uint16_t mappedAddr = 0x0000;

	if (mapperPtr->ppu_map_read(addr, mappedAddr)) {
		chrRom[mappedAddr] = data;
	}
}


uint8_t Cartridge::ppu_read(uint16_t addr, uint8_t &data) {
	uint16_t mappedAddr = 0x0000;

	if (mapperPtr->cpu_map_write(addr, mappedAddr)) {
		data = chrRom[mappedAddr];
	}

	return data;
}