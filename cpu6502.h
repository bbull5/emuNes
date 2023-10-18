#pragma once

#include <cstdint>
#include <vector>

class Bus;	//Forward declare bus class (this is done so that this implementation of the 6502
			//can be swapped out to a different bus implementation) (i.e. a different machine)


class cpu6502 {
	public:
		cpu6502();
		~cpu6502();		//Unnecessary?

		void connect_to_bus(Bus* b) { bus = b; }
		
		/* Registers */
		uint8_t acc = 0x00,			//Accumulator
			x = 0x00,				//X register
			y = 0x00,				//Y register
			sp = 0x00;				//Stack pointer
		uint16_t pc;				//Program counter	
		uint8_t status = 0x00;		//Status register	//The 8 bits in the status register are special in that they represent specific things
														//This chart is the symbolic representation of the bits in the status register:
														//1 => Carry flag [C]
														//2 => Zero flag  [Z]
														//3 => Interrupt  [I]
														//4 => Decimal	  [D] (not used by the NES)
														//5 => Break flag [B]
														//6 => Unused bit [U] (should always be set to 1)
														//7 => Overflow   [V]
														//8 => Negative   [N]


		/* CPU Internal Functions */
		uint8_t fetch();			//Used when an instruction needs data from a memory location
		uint8_t fetchedByte = 0x00;

		uint16_t addrAbs = 0x0000;	//Used when an instruction needs to read from a different address
		uint16_t addrRel = 0x00;	//Branch instructions use this to jump to a different address

		uint8_t opcode = 0x00;		//Stores the opcode	
		uint8_t cycles = 0;			//Number of cycles to complete instruction


		/* Addressing Modes */
		uint8_t IMP();			//Implicit
		uint8_t ACC();			//Accumulator
		uint8_t IMM();			//Immediate
		uint8_t ZP0();			//Zero Page
		uint8_t ZPX();			//Zero Page, X
		uint8_t ZPY();			//Zero Page, Y
		uint8_t REL();			//Relative
		uint8_t ABS();			//Absolute
		uint8_t ABX();			//Absolute, X
		uint8_t ABY();			//Absolute, Y
		uint8_t IND();			//Indirect
		uint8_t IDX();			//Indexed Indirect
		uint8_t IDY();			//Indirect Indexed


		/* OPCODES */
		//Refer to 6502 datasheet for details on specific OPCODES
		uint8_t ADC();	uint8_t AND();	uint8_t ASL();	uint8_t BCC();	uint8_t BCS();
		uint8_t BEQ();	uint8_t BIT();	uint8_t BMI();	uint8_t BNE();	uint8_t BPL();
		uint8_t BRK();	uint8_t BVC();	uint8_t BVS();	uint8_t CLC();	uint8_t CLD();
		uint8_t CLI();	uint8_t CLV();	uint8_t CMP();	uint8_t CPX();	uint8_t CPY();
		uint8_t DEC();	uint8_t DEX();	uint8_t DEY();	uint8_t EOR();	uint8_t INC();
		uint8_t INX();	uint8_t INY();	uint8_t JMP();	uint8_t JSR();	uint8_t LDA();
		uint8_t LDX();	uint8_t LDY();	uint8_t LSR();	uint8_t NOP();	uint8_t ORA();
		uint8_t PHA();	uint8_t PHP();	uint8_t PLA();	uint8_t PLP();	uint8_t ROL();
		uint8_t ROR();	uint8_t RTI();	uint8_t RTS();	uint8_t SBC();	uint8_t SEC();
		uint8_t SED();	uint8_t SEI();	uint8_t STA();	uint8_t STX();	uint8_t STY();
		uint8_t TAX();	uint8_t TAY();	uint8_t TSX();	uint8_t TXA();	uint8_t TXS();
		uint8_t TYA();

		uint8_t XXX();	//Catches/handles illegal OPCODES


		/* External Signal Functions */
		void clock();			//Executes one clock cycle
		void reset();			//Reset signal
		void irq();				//Interrupt request signal
		void nmi();				//Non-maskable interrupt request

		bool instruction_complete();


	private:
		Bus* bus = nullptr;
		
		uint8_t read_bus(uint16_t addr);					//Read byte from bus
		void write_bus(uint16_t addr, uint8_t data);		//Write byte to bus


		//This struct encapsulates all the necessary data to execute an instruction
		struct Instruction {
			uint8_t(cpu6502::* op)(void) = nullptr;			//Function ptr to opcode
			uint8_t(cpu6502::* addrMode)(void) = nullptr;	//Function ptr to address mode
			uint8_t cycles = 0;								//Clock cycles to complete instruction
		};

		std::vector<Instruction> InstructionLookupTable;
};
