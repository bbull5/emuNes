#pragma once

#include "Mapper.h"


class Mapper_0 : public Mapper {
	public:
		Mapper_0(uint8_t nPrgBanks, uint8_t nChrBanks);
		~Mapper_0();


		bool cpu_map_write(uint16_t addr, uint16_t &mappedAddr) override;
		bool cpu_map_read(uint16_t addr, uint16_t &mappedAddr) override;
		bool ppu_map_write(uint16_t addr, uint16_t &mappedAddr) override;
		bool ppu_map_read(uint16_t addr, uint16_t &mappedAddr) override;
};