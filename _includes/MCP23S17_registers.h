#ifndef __MCP23S17REG_H
	#define __MCP23S17REG_H
	
	#include <stdio.h>
	//------------------------- REGISTERS
//	const static byte			MCP23S17_IPOL = 	0x02;
//	const static byte			MCP23S17_GPINTEN =  0x04;
//	const static byte			MCP23S17_DEFVAL = 	0x06;
//	const static byte			MCP23S17_INTCON = 	0x08;
//	const static byte			MCP23S17_IOCON = 	0x0A;
//	const static byte			MCP23S17_GPPU = 	0x0C;
//	const static byte			MCP23S17_INTF = 	0x0E;
//	const static byte			MCP23S17_INTCAP = 	0x10;
//	const static byte			MCP23S17_GPIO = 	0x12;
//	const static byte			MCP23S17_OLAT = 	0x14;



	const static byte			MCP23S17_IODIR = 	0x00;
	const static byte			MCP23S17_IPOL = 	0x01;
	const static byte			MCP23S17_GPINTEN =  0x02;
	const static byte			MCP23S17_DEFVAL = 	0x03;
	const static byte			MCP23S17_INTCON = 	0x04;
	const static byte			MCP23S17_IOCON = 	0x05;
	const static byte			MCP23S17_GPPU = 	0x06;
	const static byte			MCP23S17_INTF = 	0x07;
	const static byte			MCP23S17_INTCAP = 	0x08;
	const static byte			MCP23S17_GPIO = 	0x09;
	const static byte			MCP23S17_OLAT = 	0x0A;
#endif
