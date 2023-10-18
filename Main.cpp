#include <iostream>
#include <bitset>

#include "cpu6502.h"
#include "Bus.h"




/* Debug Function Prototypes */
void show_cpu_state(Bus b);
void show_ram(Bus b);
void show_cpu_internals(Bus b);

void display_debugger(Bus b) {
	show_cpu_state(b);
	show_cpu_internals(b);
	show_ram(b);
}

int main() {
	Bus nes;


	/* Hardcoded Test Program */
	nes.sysRam[0x0000] = 0xA2;		//LDX #10		2 CYCLES
	nes.sysRam[0x0001] = 0x0A;
	nes.sysRam[0x0002] = 0x8E;		//STX $0000		4 CYCLES
	nes.sysRam[0x0003] = 0x00;
	nes.sysRam[0x0004] = 0x00;
	nes.sysRam[0x0005] = 0xA2;		//LDX #03		2 CYCLES
	nes.sysRam[0x006] = 0x03;
	nes.sysRam[0x0007] = 0x8E;		//STX $0001		2 CYCLES
	nes.sysRam[0x0008] = 0x01;
	nes.sysRam[0x0009] = 0x00;
	nes.sysRam[0x000A] = 0xAC;		//LDY $0000		4 CYCLES
	nes.sysRam[0x000B] = 0x00;
	nes.sysRam[0x000C] = 0x00;
	nes.sysRam[0x000D] = 0xAD;		//LDA $0000		4 CYCLES
	nes.sysRam[0x000E] = 0x00;
	nes.sysRam[0x000F] = 0X00;
	nes.sysRam[0x0010] = 0x18;		//CLC			2 CYCLES
	nes.sysRam[0x0011] = 0x6D;		//ADC $0001		4 CYCLES
	nes.sysRam[0x0012] = 0x01;
	nes.sysRam[0x0013] = 0x00;
	nes.sysRam[0x0014] = 0x88;		//DEY			2 CYCLES
	nes.sysRam[0x0015] = 0xD0;		//BNE			2(3)(4) CYCLES
	nes.sysRam[0x0016] = 0xFA;
	nes.sysRam[0x0017] = 0x8D;		//STA $0002		4 CYCLES
	nes.sysRam[0x0018] = 0x02;
	nes.sysRam[0x0019] = 0x00;
	nes.sysRam[0x001A] = 0xEA;
	nes.sysRam[0x001B] = 0xEA;
	nes.sysRam[0x001C] = 0xEA;


	/* Debug Loop */
	char cmd = 's';

	while (cmd == 's' || cmd == 'i') {
		display_debugger(nes);

		do {
			nes.cpu.clock();
		} while (!nes.cpu.instruction_complete());

		std::cin >> cmd;

		system("cls");
	}



	return 0;
}



void show_cpu_state(Bus b) {
	//Binary representation of status register
	std::bitset<8> sr(b.cpu.status);

	std::cout << std::hex << "Acc:      " << (int)(b.cpu.acc) << std::endl;
	std::cout << "X:        " << (int)(b.cpu.x) << std::endl;
	std::cout << "Y:        " << (int)(b.cpu.y) << std::endl;
	std::cout << "SP:       " << (int)(b.cpu.sp) << std::endl;
	std::cout << "Status:   " << sr << std::endl;
	std::cout << "PC:       " << (int)(b.cpu.pc) << std::endl;
	std::cout << "\n";
	std::cout << "Current Op: " << (int)(b.sysRam[b.cpu.pc]) << std::endl;
	std::cout << "\n";
}

void show_ram(Bus b) {
	std::cout << std::hex;

	for (int i = 0; i < 256; i += 16) {
		std::cout << "$0xx" << i << "    ";
		for (int j = 0; j < 16; ++j) {
			std::cout << (int)(b.sysRam[i + j]) << "  ";
		}
		std::cout << "\n";
	}

	std::cout << "\n\n";
	
	for (int i = 32768; i < 33024; i += 16) {
		std::cout << "$0xx" << i << "    ";
		for (int j = 0; j < 16; ++j) {
			std::cout << (int)(b.sysRam[i + j]) << "  ";
		}
		std::cout << "\n";
	}

	std::cout << "\n\n";
}

void show_cpu_internals(Bus b) {
	std::cout << std::hex;

	std::cout << "Abs Addr " << (int)(b.cpu.addrAbs) << std::endl;
	std::cout << "Rel Addr " << (int)(b.cpu.addrRel) << std::endl;
	std::cout << "Fetched " << (int)(b.cpu.fetchedByte) << std::endl;
	
	std::cout << "\n\n";
}