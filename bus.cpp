#include "bus.h"


Bus::Bus() {
	cpu.connect_to_bus(this);
}


Bus::~Bus() {

}


void Bus::cpu_write(uint16_t addr, uint8_t data) {
	/* The NES RAM takes up 8KB of address space, but only 
	    2KB is available for read/write. The 2KB of usable space
		is mirrored through the full 8KB address range. When 
		writing to a memory location, a bitwise AND is applied to
		the target address. This is equivalent to addr % 2048. */
	if (addr >= 0x0000 && addr <= 0x1FFF) {
		sysRam[addr & 0x07FF] = data;
	}

	/* The PPU has only 8 registers in it address space. The memory 
	   addresses of these registers are mirrored throughout the full
	   addressable space available to the CPU. When the CPU targets one
	   of the PPU's registers, a bitwise AND is applied to the target
	   address. This is equivalent to addr % 8. */
	else if (addr >= 0x2000 && addr <= 0x3FFF) {
		ppu.cpu_write(addr & 0x0007, data);
	}
}


uint8_t Bus::cpu_read(uint16_t addr) {
	uint8_t data = 0x00;

	//sysRam is mirrored every 2048 bytes
	if (addr >= 0x0000 && addr <= 0x1FFF) {
		data = sysRam[addr & 0x07FF];
	}
	//PPU registers are mirrored every 8 bytesf
	else if (addr >= 0x2000 && addr <= 0x3FFF) {
		data = ppu.cpu_read(addr & 0x0007);
	}

	return data;
}


/* Connects cartridge object to Main Bus */
void Bus::insert_cartridge(const std::shared_ptr<Cartridge>& cart) {
	this->cart = cart;
	ppu.connect_cart(cart);
}


/* Returns Main Bus (and all connected components) to a known, initial state */
void Bus::reset() {
	cpu.reset();
	numCycles = 0;
}


/* Running frequency is controlled by the caller of this function */
void Bus::clock() {
	/* The fastest clock frequency is the PPU's. The PPU's clock ticks 
	   each time this function is called */
	ppu.clock();

	/* The PPU clock is 3x faster than the CPU clock, so we only tick the
	    CPU clock every 3 ticks of the PPU clock */
	if (numCycles % 3 == 0) {
		cpu.clock();
	}

	/* Tracks total number of system clock cycles */
	numCycles++;
}