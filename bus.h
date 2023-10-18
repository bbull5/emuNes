#pragma once

#include <cstdint>
#include <array>
#include <memory>

#include "cpu6502.h"
#include "ppu2c02.h"
#include "Cartridge.h"



class Bus {
	public:
		Bus();
		~Bus();


		/* Main Bus Read/Write */
		void cpu_write(uint16_t addr, uint8_t data);
		uint8_t cpu_read(uint16_t addr);	//Add bool read flag (later)
		

		/* Components Connected to the Main Bus */
		cpu6502 cpu;						//6502 CPU
		
		/* System RAM: 2KB */
		std::array<uint8_t, 64 * 1024> sysRam;

		/* PPU */
		ppu2c02 ppu;

		/* Cartridge */
		std::shared_ptr<Cartridge> cart;

		//APU

		//Controllers


		void insert_cartridge(const std::shared_ptr<Cartridge>& cart);

		/* Internal System Functions */
		void reset();
		void clock();

	private:
		uint32_t numCycles = 0;
};


