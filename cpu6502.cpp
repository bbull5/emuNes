#include "cpu6502.h"
#include "bus.h"

cpu6502::cpu6502() {
	using CPU = cpu6502;	//Within this constructor, "CPU" is an alias for the cpu6502 class

	//This vector serves as a lookup table for all combinations of OPCODES and addressing modes. It is a list of Instruction structs (as defined in the cpu6502 header file). 
	InstructionLookupTable =
	{
		{&CPU::BRK, &CPU::IMM, 7}, {&CPU::ORA, &CPU::IDX, 6}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::ORA, &CPU::ZP0, 3}, {&CPU::ASL, &CPU::ZP0, 5}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::PHP, &CPU::IMP, 3}, {&CPU::ORA, &CPU::IMM, 2}, {&CPU::ASL, &CPU::ACC, 2}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::ORA, &CPU::ABS, 4}, {&CPU::ASL, &CPU::ABS, 6}, {&CPU::XXX, &CPU::IMP, 2},
		{&CPU::BPL, &CPU::REL, 2}, {&CPU::ORA, &CPU::IDY, 5}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::ORA, &CPU::ZPX, 4}, {&CPU::ASL, &CPU::ZPX, 6}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::CLC, &CPU::IMP, 2}, {&CPU::ORA, &CPU::ABY, 4}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::ORA, &CPU::ABX, 4}, {&CPU::ASL, &CPU::ABX, 7}, {&CPU::XXX, &CPU::IMP, 2},
		{&CPU::JSR, &CPU::ABS, 6}, {&CPU::AND, &CPU::IDX, 6}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::BIT, &CPU::ZP0, 3}, {&CPU::AND, &CPU::ZP0, 3}, {&CPU::ROL, &CPU::ZP0, 5}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::PLP, &CPU::IMP, 4}, {&CPU::AND, &CPU::IMM, 2}, {&CPU::ROL, &CPU::ACC, 2}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::BIT, &CPU::ABS, 4}, {&CPU::AND, &CPU::ABS, 4}, {&CPU::ROL, &CPU::ABS, 6}, {&CPU::XXX, &CPU::IMP, 2},
		{&CPU::BMI, &CPU::REL, 2}, {&CPU::AND, &CPU::IDY, 5}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::AND, &CPU::ZPX, 4}, {&CPU::ROL, &CPU::ZPX, 6}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::SEC, &CPU::IMP, 2}, {&CPU::AND, &CPU::ABY, 4}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::AND, &CPU::ABX, 4}, {&CPU::ROL, &CPU::ABX, 7}, {&CPU::XXX, &CPU::IMP, 2},
		{&CPU::RTI, &CPU::IMP, 6}, {&CPU::EOR, &CPU::IDX, 6}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::EOR, &CPU::ZP0, 3}, {&CPU::LSR, &CPU::ZP0, 5}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::PHA, &CPU::IMP, 3}, {&CPU::EOR, &CPU::IMM, 2}, {&CPU::LSR, &CPU::ACC, 2}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::JMP, &CPU::ABS, 3}, {&CPU::EOR, &CPU::ABS, 4}, {&CPU::LSR, &CPU::ABS, 6}, {&CPU::XXX, &CPU::IMP, 2},
		{&CPU::BVC, &CPU::REL, 2}, {&CPU::EOR, &CPU::IDY, 5}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::EOR, &CPU::ZPX, 4}, {&CPU::LSR, &CPU::ZPX, 6}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::CLI, &CPU::IMP, 2}, {&CPU::EOR, &CPU::ABY, 4}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::EOR, &CPU::ABX, 4}, {&CPU::LSR, &CPU::ABX, 7}, {&CPU::XXX, &CPU::IMP, 2},
		{&CPU::RTS, &CPU::IMP, 6}, {&CPU::ADC, &CPU::IDX, 6}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::ADC, &CPU::ZP0, 3}, {&CPU::ROR, &CPU::ZP0, 5}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::PLA, &CPU::IMP, 4}, {&CPU::ADC, &CPU::IMM, 2}, {&CPU::ROR, &CPU::ACC, 2}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::JMP, &CPU::IND, 5}, {&CPU::ADC, &CPU::ABS, 4}, {&CPU::ROR, &CPU::ABS, 6}, {&CPU::XXX, &CPU::IMP, 2},
		{&CPU::BVS, &CPU::REL, 2}, {&CPU::ADC, &CPU::IDY, 5}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::ADC, &CPU::ZPX, 4}, {&CPU::ROR, &CPU::ZPX, 6}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::SEI, &CPU::IMP, 2}, {&CPU::ADC, &CPU::ABY, 4}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::ADC, &CPU::ABX, 4}, {&CPU::ROR, &CPU::ABX, 7}, {&CPU::XXX, &CPU::IMP, 2},
		{&CPU::XXX, &CPU::IMP, 2}, {&CPU::STA, &CPU::IDX, 6}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::STY, &CPU::ZP0, 3}, {&CPU::STA, &CPU::ZP0, 3}, {&CPU::STX, &CPU::ZP0, 3}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::DEY, &CPU::IMP, 2}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::TXA, &CPU::IMP, 2}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::STY, &CPU::ABS, 4}, {&CPU::STA, &CPU::ABS, 4}, {&CPU::STX, &CPU::ABS, 4}, {&CPU::XXX, &CPU::IMP, 2},
		{&CPU::BCC, &CPU::REL, 2}, {&CPU::STA, &CPU::IDY, 6}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::STY, &CPU::ZPX, 4}, {&CPU::STA, &CPU::ZPX, 4}, {&CPU::STX, &CPU::ZPY, 4}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::TYA, &CPU::IMP, 2}, {&CPU::STA, &CPU::ABY, 5}, {&CPU::TXS, &CPU::IMP, 2}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::STA, &CPU::ABX, 5}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::XXX, &CPU::IMP, 2},
		{&CPU::LDY, &CPU::IMM, 2}, {&CPU::LDA, &CPU::IDX, 6}, {&CPU::LDX, &CPU::IMM, 2}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::LDY, &CPU::ZP0, 3}, {&CPU::LDA, &CPU::ZP0, 3}, {&CPU::LDX, &CPU::ZP0, 3}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::TAY, &CPU::IMP, 2}, {&CPU::LDA, &CPU::IMM, 2}, {&CPU::TAX, &CPU::IMP, 2}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::LDY, &CPU::ABS, 4}, {&CPU::LDA, &CPU::ABS, 4}, {&CPU::LDX, &CPU::ABS, 4}, {&CPU::XXX, &CPU::IMP, 2},
		{&CPU::BCS, &CPU::REL, 2}, {&CPU::LDA, &CPU::IDY, 5}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::LDY, &CPU::ZPX, 4}, {&CPU::LDA, &CPU::ZPX, 4}, {&CPU::LDX, &CPU::ZPY, 4}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::CLV, &CPU::IMP, 2}, {&CPU::LDA, &CPU::ABY, 4}, {&CPU::TSX, &CPU::IMP, 2}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::LDY, &CPU::ABX, 4}, {&CPU::LDA, &CPU::ABX, 4}, {&CPU::LDX, &CPU::ABY, 4}, {&CPU::XXX, &CPU::IMP, 2},
		{&CPU::CPY, &CPU::IMM, 2}, {&CPU::CMP, &CPU::IDX, 6}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::CPY, &CPU::ZP0, 3}, {&CPU::CMP, &CPU::ZP0, 3}, {&CPU::DEC, &CPU::ZP0, 5}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::INY, &CPU::IMP, 2}, {&CPU::CMP, &CPU::IMM, 2}, {&CPU::DEX, &CPU::IMP, 2}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::CPY, &CPU::ABS, 4}, {&CPU::CMP, &CPU::ABS, 4}, {&CPU::DEC, &CPU::ABS, 6}, {&CPU::XXX, &CPU::IMP, 2},
		{&CPU::BNE, &CPU::REL, 2}, {&CPU::CMP, &CPU::IDY, 5}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::CMP, &CPU::ZPX, 4}, {&CPU::DEC, &CPU::ZPX, 6}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::CLD, &CPU::IMP, 2}, {&CPU::CMP, &CPU::ABY, 4}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::CMP, &CPU::ABX, 4}, {&CPU::DEC, &CPU::ABX, 7}, {&CPU::XXX, &CPU::IMP, 2},
		{&CPU::CPX, &CPU::IMM, 2}, {&CPU::SEC, &CPU::IDX, 6}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::CPX, &CPU::ZP0, 3}, {&CPU::SEC, &CPU::ZP0, 3}, {&CPU::INC, &CPU::ZP0, 5}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::INX, &CPU::IMP, 2}, {&CPU::SEC, &CPU::IMM, 2}, {&CPU::NOP, &CPU::IMP, 2}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::CPX, &CPU::ABS, 4}, {&CPU::SEC, &CPU::ABS, 4}, {&CPU::INC, &CPU::ABS, 6}, {&CPU::XXX, &CPU::IMP, 2},
		{&CPU::BEQ, &CPU::REL, 2}, {&CPU::SEC, &CPU::IDY, 5}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::SEC, &CPU::ZPX, 4}, {&CPU::INC, &CPU::ZPX, 6}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::SED, &CPU::IMP, 2}, {&CPU::SEC, &CPU::ABY, 4}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::XXX, &CPU::IMP, 2}, {&CPU::SEC, &CPU::ABX, 4}, {&CPU::INC, &CPU::ABX, 7}, {&CPU::XXX, &CPU::IMP, 2}
	};
}


cpu6502::~cpu6502() {
	
	//Destructor does nothing (I think)

}


uint8_t cpu6502::fetch() {
	if (!(InstructionLookupTable[opcode].addrMode == &cpu6502::IMP)) {
		fetchedByte = read_bus(addrAbs);
	}

	return fetchedByte;
}



/* EXTERNAL SIGNAL FUNCTIONS */

void cpu6502::clock() {
	if (cycles == 0) {
		opcode = read_bus(pc);
		pc++;

		status |= 0x20;		//Unused bit is always set

		//Get number of cycles for current instruction
		cycles = InstructionLookupTable[opcode].cycles;

		//Get address mode and opcode
		uint8_t extraCycle01 = (this->*InstructionLookupTable[opcode].addrMode)();
		uint8_t extraCycle02 = (this->*InstructionLookupTable[opcode].op)();

		cycles += (extraCycle01 + extraCycle02);
	}

	cycles--;
}


//RESET
//Places the CPU into a predetermined, known state
void cpu6502::reset() {
	//0xFFFE is reserved in memory to handle interrupt requests.
	//After the interrupt is serviced, set the program counter to 
	//this reserved memory location
	addrAbs = 0xFFFC;
	uint16_t loByte = read_bus(addrAbs);
	uint16_t hiByte = read_bus(addrAbs + 1);

	pc = (hiByte << 8) | loByte;

	//Zero out all registers
	acc = x = y = 0x00;
	sp = 0xFD;
	//Zero out the status register but leave the unused bit (bit 6) on
	status = 0x00;		
	status |= 0x20;
	
	addrRel = 0x0000;
	addrAbs = 0x0000;
	fetchedByte = 0x00;

	cycles = 8;
}


//INTERRUPT REQUEST
void cpu6502::irq() {
	//Interrupt request is serviced iff interrupt disable flag not on
	if (!(status & (1 << 2))) {
		//Push low byte of program counter to stack
		write_bus(0x0100 + sp, (pc >> 8) & 0x00FF);
		sp--;
		//Push high byte of program counter to stack
		write_bus(0x0100 + sp, pc & 0x00FF);
		sp--;
		
		//Set status flags
		status |= 0x04;		//Turn interrupt disable flag on
		status &= 0xFB;		//Turn decimal flag off
		status |= 0x20;		//Unused bit is always set

		//0xFFFE is reserved in memoroy to handle interrupt requests.
		//After the interrupt is serviced, set the program counter to 
		//this reserved memory location
		addrAbs = 0xFFFE;
		uint16_t loByte = read_bus(addrAbs);
		uint16_t hiByte = read_bus(addrAbs + 1);

		pc = (hiByte << 8) | loByte;

		//Interrupts take time
		cycles = 7;
	}
}


//NON-MASKABLE INTERRUPT REQUEST
void cpu6502::nmi() {
	write_bus(0x0100 + sp, (pc >> 8) & 0x00FF);
	sp--;
	write_bus(0x0100 + sp, pc & 0x00FF);
	sp--;

	//Set status flags
	status |= 0x04;		//Turn interrupt disable flag on
	status &= 0xFB;		//Turn decimal flag off
	status |= 0x20;		//Unused bit is always set

	addrAbs = 0xFFFA;
	uint16_t loByte = read_bus(addrAbs);
	uint16_t hiByte = read_bus(addrAbs + 1);

	pc = (hiByte << 8) | loByte;

	cycles = 8;
}


bool cpu6502::instruction_complete() {
	return cycles == 0;
}



/* BUS INTERFACE FUNCTIONS */
uint8_t cpu6502::read_bus(uint16_t addr) {
	return bus->cpu_read(addr);
}


void cpu6502::write_bus(uint16_t addr, uint8_t data) {
	bus->cpu_write(addr, data);
}




/* ADDRESSING MODE FUNCTIONS */
//All of these functions return values. These values indicate whether
//or not additional clock cycles are necessary, and how many cycles
//to add


//IMPLICIT
uint8_t cpu6502::IMP() {
	fetchedByte = acc;
	return 0;
}


//ACCCUMULATOR
uint8_t cpu6502::ACC() {
	
	//It may be possible to treat this as a duplicate of the IMP address mode

	return 0;
}


//IMMEDIATE
//Uses the address immediately after the current program counter address
uint8_t cpu6502::IMM() {
	addrAbs = pc++;
	return 0;
}


//ZERO PAGE
//The zero page is the range 0x0000 - 0x00FF in memory. Zero page addressing
//allows the address to be specified using only the low byte in the address 
//(e.g. 0x1A instead of 0x001A). This, of course, requires only 1 byte, rather
//than the 2 bytes usually required for memory addresses.
uint8_t cpu6502::ZP0() {
	addrAbs = read_bus(pc);
	pc++;
	addrAbs &= 0x00FF;
	return 0;
}


//ZERO PAGE WITH X OFFSET
//Works identically to the zero page addressing mode, except the address is
//offset by the value stored in the X register.
uint8_t cpu6502::ZPX() {
	addrAbs = read_bus(pc) + x;
	pc++;
	addrAbs &= 0x00FF;
	return 0;
}


//ZERO PAGE WITH Y OFFSET
//Works identically to the zero page addressing mode, except the address is
//offset by the valued stored in the Y register
uint8_t cpu6502::ZPY() {
	addrAbs = read_bus(pc) + y;
	pc++;
	addrAbs &= 0x00FF;
	return 0;
}


//RELATIVE
//This address mode is used only by branch instructions. The branch instruction
//contains a signed 8-bit offset (-128 - +127) which is added to the program counter
//if the branch condition is true.
uint8_t cpu6502::REL() {
	addrRel = read_bus(pc);
	pc++;

	if (addrRel & 0x80) {
		addrRel |= 0xFF00;
	}

	return 0;
}


//ABSOLUTE
//This address mode is used when an OPCODE supplies a full 16-bit address. This means that
//an instruction that uses the absolute address mode, provides the full address for the instruction
//to operate on
uint8_t cpu6502::ABS() {
	uint16_t loByte = read_bus(pc);
	pc++;
	uint16_t hiByte = read_bus(pc);
	pc++;

	addrAbs = (hiByte << 8) | loByte;

	return 0;
}


//ABSOLUTE WITH X OFFSET
//Works identically to the absolute address mode, but the value of the X register is added to the absolute address
//
//If incrementing addrAbs by x results in a page turn in memory (i.e. addrAbs + x results in an increment in the
//high byte), add 1 to the cycle count.
//Else, the instruction does not require an additional cycle
//
//Example:
//If addrAbs = 0x1FFF and x = 0x01, then 1FFF + 0x01 = 0x2000
//This operation requires a carry which means a page turn in memory and an additional cycle
uint8_t cpu6502::ABX() {
	uint16_t loByte = read_bus(pc);
	pc++;
	uint16_t hiByte = read_bus(pc);
	pc++;

	addrAbs = ((hiByte << 8) | loByte) + x;

	//If most significant byte of addrAbs != to the supplied high byte, a page turn is required
	if ((addrAbs & 0xFF00) != (hiByte << 8)) { return 1; }
	else { return 0; }
}


//ABSOLUTE WITH Y OFFSET
//Works identically to ABX, except the absolute address is offset by the value in the Y register
//instead of the X register
uint8_t cpu6502::ABY() {
	uint16_t loByte = read_bus(pc);
	pc++;
	uint16_t hiByte = read_bus(pc);
	pc++;

	addrAbs = ((hiByte << 8) | loByte) + y;

	if ((addrAbs & 0xFF00) != (hiByte << 8)) { return 1; }
	else { return 0; }
}


//INDIRECT
//!!!!(THERE IS A HARDWARE BUG THAT MUST BE EMULATED HERE)!!!! (done)
//
//This address mode acts like a pointer. The operand is the 16-bit address that holds the low byte
//of the target address. Then, the program counter is incremented and the high byte of the target
//adress is stored at this incremented address.
uint8_t cpu6502::IND() {
	uint16_t loByte = read_bus(pc);
	pc++;
	uint16_t hiByte = read_bus(pc);
	pc++;

	uint16_t ptr = (hiByte << 8) | loByte;

	loByte = read_bus(ptr);
	hiByte = read_bus(ptr + 1);

	if (loByte == 0x00FF) {
		addrAbs = (read_bus(ptr & 0xFF00) << 8) | read_bus(ptr);
	}
	else {
		addrAbs = (hiByte << 8) | loByte;
	}

	return 0;
}


//INDIRECT, X INDEXED
//
uint8_t cpu6502::IDX() {
	uint16_t index = read_bus(pc);
	pc++;

	uint16_t loByte = read_bus((index + x) & 0x00FF);
	uint16_t hiByte = read_bus((index + x + 1) & 0x00FF);

	addrAbs = (hiByte << 8) | loByte;

	return 0;
}


//INDIRECT, Y INDEXED
uint8_t cpu6502::IDY() {
	uint16_t index = read_bus(pc);
	pc++;

	uint16_t loByte = read_bus(index & 0x00FF);
	uint16_t hiByte = read_bus((index + 1) & 0x00FF);

	addrAbs = ((hiByte << 8) | loByte) + y;
	
	if ((addrAbs & 0xFF00) != (hiByte << 8)) { return 1; }
	else { return 0; }
}



/* OPCODE FUNCTIONS */


//ADD WITH CARRY
uint8_t cpu6502::ADC() {
	fetch();

	uint16_t sum = (uint16_t)acc + (uint16_t)fetchedByte + (uint16_t)(status & (1 << 0));

	if (sum > 255) {
		status |= 0x01;
	}
	if ((sum & 0x00FF) == 0) {
		status |= 0x02;
	}
	if (sum & 0x80) {
		status |= 0x80;
	}
	if (~((uint16_t)acc ^ (uint16_t)fetchedByte) & ((uint16_t)acc ^ (uint16_t)sum) & 0x0080) {
		status |= 0x40;
	}

	acc = sum & 0x00FF;

	return 1;
}


//AND MEMORY WITH ACCUMULATOR
uint8_t cpu6502::AND() {
	fetch();

	acc = acc & fetchedByte;

	if (acc == 0) { status |= 0x02; }
	if (acc & 0x80) { status |= 0x80; }

	return 1;
}


//ARITHMETIC SHIFT LEFT
//
//There might be some problems here. Reference material typecasted the shifted variable to uint16_t.
//I used 8-bits here because it seems simpler. Keep that in mind if there are bugs associated with this OPCODE
uint8_t cpu6502::ASL() {
	//Retrieve byte
	fetch();
	
	//Shift fetchedByte 1 bit to the left
	uint8_t shifted = fetchedByte << 1;

	//Set status register flags
	if ((shifted & 0xF0) > 0) {
		status |= 0x01;
	}
	if ((shifted & 0xFF) == 0x00) {
		status |= 0x02;
	}
	if (shifted > 0x80) {
		status |= 0x80;
	}

	//Place shifted into the location specified by the address mode
	if (InstructionLookupTable[opcode].addrMode == &cpu6502::IMP) {
		acc = shifted;
	}
	else {
		write_bus(addrAbs, shifted);
	}

	return 0;
}


//BRANCH IF CARRY FLAG FALSE
uint8_t cpu6502::BCC() {
	if (!(status & (1 << 0))) {
		cycles++;

		addrAbs = pc + addrRel;

		if ((addrAbs & 0xFF00) != (pc & 0xFF00)) {
			cycles++;
		}

		pc = addrAbs;
	}

	return 0;
}


//BRANCH ON CARRY FLAG SET
uint8_t cpu6502::BCS() {
	if (status & (1 << 0)) {
		cycles++;

		addrAbs = pc + addrRel;

		if ((addrAbs & 0xFF00) != (pc & 0xFF00)) {
			cycles++;
		}

		pc = addrAbs;
	}

	return 0;
}


//BRANCH ON EQUAL
uint8_t cpu6502::BEQ() {
	if (status & (1 << 1)) {
		cycles++;

		addrAbs = pc + addrRel;

		if ((addrAbs & 0xFF00) != (pc & 0xFF00)) {
			cycles++;
		}

		pc = addrAbs;
	}

	return 0;
}


//TEST BITS IN MEMORY WITH ACCUMULATOR
uint8_t cpu6502::BIT() {
	fetch();

	//Logical AND with accumulator and oeprand
	uint8_t byte = acc & fetchedByte;

	//Set status register flags
	if ((byte & 0xFF) == 0x00) {
		status |= 0x02;
	}
	if (byte & (1 << 6)) {
		status |= 0x40;
	}
	else {
		status &= 0xBF;
	}
	if (byte & (1 << 7)) {
		status |= 0x80;
	}
	else {
		status &= 0x7F;
	}

	return 0;
}


//BRANCH ON RESULT MINUS
uint8_t cpu6502::BMI() {
	if (status & (1 << 7)) {
		cycles++;

		addrAbs = pc + addrRel;

		if ((addrAbs & 0xFF00) != (pc & 0xFF00)) {
			cycles++;
		}

		pc = addrAbs;
	}

	return 0;
}


//BRANCH ON RESULT NOT ZERO
uint8_t cpu6502::BNE() {
	if (!(status & (1 << 1))) {
		cycles++;

		addrAbs = pc + addrRel;

		if ((addrAbs & 0xFF00) != (pc & 0xFF00)) {
			cycles++;
		}

		pc = addrAbs;
	}

	return 0;
}


//BRANCH ON RESULT POSITIVE (NEGATIVE FLAG 0FF)
uint8_t cpu6502::BPL() {
	if (!(status & (1 << 7))) {
		cycles++;

		addrAbs = pc + addrRel;

		if ((addrAbs & 0xFF00) != (pc & 0xFF00)) {
			cycles++;
		}

		pc = addrAbs;
	}

	return 0;
}


//BREAK
//Forces an interrupt
uint8_t cpu6502::BRK() {
	//Set interrupt flag
	status |= 0x04;
	//Set break flag
	status |= 0x08;

	//Write current program counter to stack
	write_bus(0x0100 + sp, (pc >> 8) * 0x00FF);
	sp--;
	write_bus(0x0100 + sp, pc & 0x00FF);
	sp--;

	//Write status register to stack
	write_bus(0x0100 + sp, status);
	sp--;

	//Memory location 0xFFFE is reserved for servicing breaks. Load this
	//address into the program counter
	pc = (uint16_t)read_bus(0xFFFE) | ((uint16_t)read_bus(0xFFFF) << 8);

	return 0;
}


//BRANCH ON OVERFLOW CLEAR
uint8_t cpu6502::BVC() {
	if (!(status & (1 << 6))) {
		cycles++;

		addrAbs = pc + addrRel;

		if ((addrAbs & 0xFF00) != (pc & 0xFF00)) {
			cycles++;
		}

		pc = addrAbs;
	}

	return 0;
}


//BRANCH ON OVERFLOW SET
uint8_t cpu6502::BVS() {
	if (status & (1 << 6)) {
		cycles++;
		
		addrAbs = pc + addrRel;

		if ((addrAbs & 0xFF00) != (pc & 0xFF00)) {
			cycles++;
		}

		pc = addrAbs;
	}

	return 0;
}


//CLEAR CARRY FLAG
uint8_t cpu6502::CLC() {
	status &= 0xFE;

	return 0;
}


//CLEAR DECIMAL FLAG
uint8_t cpu6502::CLD() {
	status &= 0xF7;

	return 0;
}


//CLEAR INTERRUPT FLAG
uint8_t cpu6502::CLI() {
	status &= 0XFB;

	return 0;
}


//CLEAR OVERFLOW FLAG
uint8_t cpu6502::CLV() {
	status &= 0xBF;

	return 0;
}


uint8_t cpu6502::CMP() {
	fetch();

	//Set status flags according to result of comparison
	if (acc >= fetchedByte) {
		status |= 0x01;
	}
	if (acc == fetchedByte) {
		status |= 0x02;
	}
	if ((acc & fetchedByte) & (1 << 7)) {
		status |= 0x80;
	}

	return 1;
}


uint8_t cpu6502::CPX() {
	fetch();

	//Set status flags according to result of comparison
	if (x >= fetchedByte) {
		status |= 0x01;
	}
	if (x == fetchedByte) {
		status |= 0x02;
	}
	if ((x & fetchedByte) & (1 << 7)) {
		status |= 0x80;
	}

	return 0;
}


uint8_t cpu6502::CPY() {
	fetch();

	//Set status flags according to result of comparison
	if (y >= fetchedByte) {
		status |= 0x01;
	}
	if (y == fetchedByte) {
		status |= 0x02;
	}
	if ((y & fetchedByte) & (1 << 7)) {
		status |= 0x80;
	}

	return 0;
}


uint8_t cpu6502::DEC() {
	fetch();

	//Decrement byte in memory by 1
	uint8_t byte = fetchedByte - 1;

	//Write the new value to the memory location
	write_bus(addrAbs, byte);

	//Set flags in status register
	if (byte == 0x00) {
		status |= 0x02;
	}
	if (byte & 0x80) {
		status |= 0x80;
	}

	return 0;
}


uint8_t cpu6502::DEX() {
	//Decrement X register
	x--;

	//Set flags in status register
	if (x == 0x00) {
		status |= 0x02;
	}
	if (x & 0x80) {
		status |= 0x80;
	}

	return 0;
}


uint8_t cpu6502::DEY() {
	//Decrement Y register
	y--;

	//Set flags in status register
	if (y == 0x00) {
		status |= 0x02;
	}
	if (y & 0x80) {
		status |= 0x80;
	}

	return 0;
}


uint8_t cpu6502::EOR() {
	fetch();

	//XOR accumulator with memory location 
	acc = acc ^ fetchedByte;

	//Set flags in status register
	if (acc == 0x00) {
		status |= 0x02;
	}
	if (acc & 0x80) {
		status |= 0x80;
	}

	return 0;
}

uint8_t cpu6502::INC() {
	fetch();

	//Increment memory location
	uint8_t byte = fetchedByte + 1;

	//Write incremented value to memory location
	write_bus(addrAbs, byte);

	//Set flags in status register
	if (byte == 0x00) {
		status |= 0x02;
	}
	if (byte & 0x80) {
		status |= 0x80;
	}

	return 0;
}


uint8_t cpu6502::INX() {
	//Increment X register
	x++;

	//Set flags in status register
	if (x == 0x00) {
		status |= 0x02;
	}
	if (x & 0x80) {
		status |= 0x80;
	}

	return 0;
}


uint8_t cpu6502::INY() {
	//Increment Y register
	y++;

	//Set flags in status register
	if (y == 0x00) {
		status |= 0x02;
	}
	if (y & 0x80) {
		status |= 0x80;
	}

	return 0;
}


//JUMP TO NEW MEMORY LOCATION
//Sets program counter to address specified by addrAbs
uint8_t cpu6502::JMP() {
	pc = addrAbs;

	return 0;
}


//JUMP TO NEW LOCATION, SAVING CURRENT PC TO STACK
uint8_t cpu6502::JSR() {
	pc--;

	//Write low byte of program counter to stack
	write_bus(0x0100 + sp, (pc >> 8) & 0x00FF);
	sp--;
	//Write high byte of program counter to stack
	write_bus(0x0100 + sp, pc & 0x00FF);
	sp--;

	//Program counter takes new memory location
	pc = addrAbs;

	return 0;
}


//LOAD VALUE AT MEMORY LOCATION INTO ACCUMULATOR
uint8_t cpu6502::LDA() {
	fetch();

	acc = fetchedByte;

	//Set flags in status register
	if (acc == 0x00) {
		status |= 0x02;
	}
	if (acc & 0x80) {
		status |= 0x80;
	}

	return 1;
}


//LOAD VALUE AT MEMORY LOCATION INTO X REGISTER
uint8_t cpu6502::LDX() {
	fetch();

	x = fetchedByte;

	//Set flags in status register
	if (x == 0x00) {
		status |= 0x02;
	}
	if (x & 0x80) {
		status |= 0x80;
	}

	return 0;
}


//LOAD VALUE AT MEMORY LOCATION INTO Y REGISTER
uint8_t cpu6502::LDY() {
	fetch();

	y = fetchedByte;

	//Set flags in status register
	if (y == 0x00) {
		status |= 0x02;
	}
	if (y & 0x80) {
		status |= 0x80;
	}

	return 0;
}


//SHIFT ONE BIT TO THE RIGHT
uint8_t cpu6502::LSR() {
	fetch();

	//Place LSB of original byte into carry flag
	if (fetchedByte & 0x01) {
		status |= 0x01;
	}

	uint8_t shifted = fetchedByte >> 1;

	//Set flags in status register
	if (shifted == 0x00) {
		status |= 0x02;
	}
	if (shifted & 0x80) {
		status |= 0x80;
	}

	return 0;
}


//NO OPERATION
//
//This instruction is executed when an the CPU attempts to service
//an illegal OPCODE
//Some games depend on illegal OPCODES, so this function needs to be 
//updated with functionality to handle these operations
uint8_t cpu6502::NOP() {	
	switch (opcode) {
		case 0x1C:
		case 0x3C:
		case 0x5C:
		case 0x7C:
		case 0xDC:
		case 0xFC:
			return 1;
			break;
	}

	return 0;
}


//INCLUSIVE BITWISE "OR"
uint8_t cpu6502::ORA() {
	fetch();

	acc = acc | fetchedByte;

	if (acc == 0x00) {
		status |= 0x00;
	}
	if (acc & 0x80) {
		status |= 0x80;
	}

	return 1;
}


//PUSH ACCUMULATOR ON STACK
uint8_t cpu6502::PHA() {
	write_bus(0x0100 + sp, acc);
	sp--;

	return 0;
}


//PUSH STATUS REGISTER ONTO STACK
uint8_t cpu6502::PHP() {
	//Set break flag and unused bit to 1
	status |= 0x10;
	status |= 0x20;
	
	write_bus(sp + 0x0100, status);
	sp--;

	return 0;
}


//PULL ACCUMULATOR FROM STACK
uint8_t cpu6502::PLA() {
	sp++;

	acc = read_bus(0x0100 + sp);

	if (acc == 0x00) {
		status |= 0x02;
	}
	if (acc & 0x80) {
		status |= 0x80;
	}

	return 0;
}


//PULL STATUS REGISTER FROM STACK
uint8_t cpu6502::PLP() {
	sp++;

	status = read_bus(0x0100 + sp);

	//Ensure that the unsued bit is set
	status |= 0x20;

	return 0;
}


//ROTATE 1 BIT TO THE LEFT
uint8_t cpu6502::ROL() {
	fetch();

	//Store MSB in carry flag
	if (fetchedByte & 0x80) {
		status |= 0x01;
	}
	
	//Shift fetched byte one bit to the left
	uint8_t shifted = fetchedByte << 1;

	//Rotate MSB (stored in carry flag) to LSB position
	if (status & 0x01) {
		shifted |= 0x01;
	}

	//Set flags in status register
	if (shifted & 0xF0) {
		status |= 0x01;
	}
	if ((shifted == 0x00)) {
		status |= 0x02;
	}
	if (shifted & 0x80) {
		status |= 0x80;
	}

	//Store shifted byte in accumulator if address mode is implied.
	//Otherwise, store it at the specified memory location
	if (InstructionLookupTable[opcode].addrMode == &cpu6502::IMP) {
		acc = shifted;
	}
	else {
		write_bus(addrAbs, shifted);
	}

	return 0;
}


//ROTATE 1 BIT TO THE RIGHT
uint8_t cpu6502::ROR() {
	fetch();

	//Store LSB in the carry flag
	if (status & 0x01) {
		status |= 0x01;
	}

	//Shift fetched byte one bit to the right
	uint8_t shifted = fetchedByte >> 1;

	//Rotate LSB (stored in carry flag) to the MSB position
	if (status & 0x01) {
		shifted |= 0x80;
	}

	//Set flags in status register
	if (shifted & 0xF0) {
		status |= 0x01;
	}
	if ((shifted == 0x00)) {
		status |= 0x02;
	}
	if (shifted & 0x80) {
		status |= 0x80;
	}

	//Store shifted byte in accumulator if address mode is implied.
	//Otherwise, store it at the specified memory location
	if (InstructionLookupTable[opcode].addrMode == &cpu6502::IMP) {
		acc = shifted;
	}
	else {
		write_bus(addrAbs, shifted);
	}

	return 0;
}


//RETURN FROM INTERRUPT
//The status register is read from the stack (break flag and the unused bit are ignored)
//
//!!!!!THIS MAY NOT BE THE CORRECT WAY TO IGNORE THE BREAK FLAG AND UNUSED BIT WHEN READING FROM STACK!!!!!!
uint8_t cpu6502::RTI() {
	sp++;
	status = read_bus(0x0100 + sp);
	
	//Break flag and unused bit are ignored
	/*
	status &= ~(0x10);
	status &= ~(0x20);
	*/
	status |= 0x10;
	status |= 0x20;
	sp++;

	//Read program counter from stack
	pc = (uint16_t)read_bus(0x0100 + sp);
	sp++;
	pc |= (uint16_t)read_bus(0x0100 + sp) << 8;

	return 0;
}


//RETURN FROM SUBROUTINE
uint8_t cpu6502::RTS() {
	//Read the program counter from the stack
	sp++;
	pc = (uint16_t)read_bus(0x0100 + sp);
	sp++;
	pc |= (uint16_t)read_bus(0x0100 + sp) << 8;

	//The address stored in the stack was the last operation performed
	//so the program counter needs to be incremented when returning from
	//the subroutine
	pc++;

	return 0;
}


//SUBTRACT WITH CARRY
uint8_t cpu6502::SBC() {
	fetch();

	uint16_t inverse = ((uint16_t)fetchedByte) ^ 0x00FF;
	uint16_t sum = (uint16_t)acc + inverse + (uint16_t)(status & (1 << 0));

	if (sum & 0xFF00) {
		status |= 0x01;
	}
	if ((sum & 0x00FF) == 0) {
		status |= 0x02;
	}
	if (sum & (uint16_t)acc & (sum ^ inverse) & 0x0080) {
		status |= 0x40;
	}
	if (sum & 0x0080) {
		status |= 0x80;
	}

	acc = sum & 0x00FF;

	return 1;
}


//SET CARRY FLAG
uint8_t cpu6502::SEC() {
	status |= 0x01;

	return 0;
}


//SET DECIMAL FLAG
uint8_t cpu6502::SED() {
	status |= 0x08;

	return 0;
}


//SET INTERRUPT FLAG
uint8_t cpu6502::SEI() {
	status |= 0x04;

	return 0;
}


//STORE ACCUMULATOR AT MEMORY LOCATION
uint8_t cpu6502::STA() {
	write_bus(addrAbs, acc);

	return 0;
}


//STORE X REGISTER AT MEMORY LOCATION
uint8_t cpu6502::STX() {
	write_bus(addrAbs, x);

	return 0;
}


//STORE Y REGISTER AT MEMORY LOCATION
uint8_t cpu6502::STY() {
	write_bus(addrAbs, y);

	return 0;
}


//TRANSFER ACCUMULATOR TO X REGISTER
uint8_t cpu6502::TAX() {
	x = acc;

	if (x == 0x00) {
		status |= 0x02;
	}
	if (x & 0x80) {
		status |= 0x80;
	}

	return 0;
}


//TRANSFER ACCUMULATOR TO Y REGISTER
uint8_t cpu6502::TAY() {
	y = acc;

	if (y == 0x00) {
		status |= 0x02;
	}
	if (y & 0x80) {
		status |= 0x80;
	}

	return 0;
}


//TRANSFER STACK POINTER TO X REGISTER
uint8_t cpu6502::TSX() {
	x = sp;

	if (x == 0x00) {
		status |= 0x02;
	}
	if (x & 0x80) {
		status |= 0x80;
	}

	return 0;
}


//TRANSFER X REGISTER TO ACCUMULATOR
uint8_t cpu6502::TXA() {
	acc = x;

	if (acc == 0x00) {
		status |= 0x02;
	}
	if (acc & 0x80) {
		status |= 0x80;
	}

	return 0;
}


//TRANSFER X REGISTER TO STACK POINTER
uint8_t cpu6502::TXS() {
	sp = x;

	return 0;
}


//TRANSFER Y REGISTER TO ACCUMULATOR
uint8_t cpu6502::TYA() {
	acc = y;

	if (acc = 0x00) {
		status |= 0x02;
	}
	if (acc & 0x80) {
		status |= 0x80;
	}

	return 0;
}


//ILLEGAL OPCODES
uint8_t cpu6502::XXX() {
	return 0;
}