//--------------------------------------------------------------------------//
//          Copyright (c) 2003, Data I/O Corporation                        //
//          10525 Willows Rd NE / Redmond, WA 98073 / (206) 881-6444        //
//__________________________________________________________________________//
/***************************************************************************

		PINFILE:    RTC_R7FA6E10F_LQFP100.CPP            Author: Vinod Dosapati

		Description: 

		Package: LQFP100

		History: 
							Version 1.0   

 ***************************************************************************/

#include "standard.hpp"     // defines BYTE, WORD, etc.
#include "PinDefines.hpp"   // #defines for pins and packages  
#include "DevParms.hpp"     // structure for pin files
#include "FlashAPI.hpp"     // defines for socket, socketstatus
#include "FlashAlg2.hpp"    // defines base class 

#include "rtc_synergy_cortexm33.hpp"  // class definition for C_R7FS

//-----------Begin standardized algorithm parameters-------------------------
DEVPARMS _parms = {
		MICRO,                  // DEVPARMS type  ---> MICRO
		3129,                   // adapter ID
		CONTINUITY1,            // continuity check
		0x0010,                 // mfg ID, 
		0x0001,                 // device ID (BYTE2&BYTE3)
		8,                      // data width
		0x2100000,              // device size address space of micro, ~34MB map, 256KB + 8kB DF + CFG
		 40,                    // number of sectors
		100,                    // number of pins
		330,                    // Vcc voltage
		330,                    // Vpp voltage / VDD2
		310,                    // Vih voltage
		0,                      // Alt voltage
		0,                      // erase time - not used
		0,                      // program time - not used
		false,                  // Sector Protect: device does not support protect
		false,                  // Software Data Protect
		false,                  // Boot Block Protect1
		SECTOR_BLANK_CHECK | USING_SPECIAL_FEATURES,  // alg_features
		0,            // reserved integer 1
		0,            // reserved integer 2
		0,            // reserved integer 3
		0,            // reserved integer 4
		330,          // voltage_vcc_high;   
		330,          // voltage_vpp_high;
		300,          // voltage_vih_high;
		330,          // voltage_vcc_low;     
		330,          // voltage_vpp_low;
		300,          // voltage_vih_low;
		0,            // voltage_vcc_prog;    
		0,            // voltage_vpp_prog;
		0             // voltage_vih_prog;
};


DEVPINS _pins[] = 
{
		NOCONN,      NO_MODE,  NOCONN,      NO_MODE,  NOCONN,      NO_MODE,  NOCONN,      NO_MODE,  NOCONN,      NO_MODE,  // pin  1 -  5
		NOCONN,      NO_MODE,   // pin  6
		NOCONN,      NO_MODE,   // pin  7
		VCC,         VCC_RES,   // pin  8 VBATT
		NOCONN,      NO_MODE,   // pin  9 VCL0 - 100nF to GND
		NOCONN,      NO_MODE,   // pin 10 XCIN - 10kOhm PD
		NOCONN,      NO_MODE,   // pin 11
		GND,         GND_RES,   // pin 12 VSS
		NOCONN,      NO_MODE,   // pin 13 XTAL 16MHz oscillator on the adapter
		NOCONN,      NO_MODE,   // pin 14 EXTAL 16MHz oscillator on the adapter
		NOCONN,      NO_MODE,   // pin 15
		NOCONN,      NO_MODE,   // pin 16
		NOCONN,      NO_MODE,   // pin 17
		NOCONN,      NO_MODE,   // pin 18
		NOCONN,      NO_MODE,   // pin 19
		A0,          MANUAL,    // pin 20 for cont. check only
		NOCONN,      NO_MODE,  NOCONN,      NO_MODE,  NOCONN,      NO_MODE,  NOCONN,      NO_MODE,  NOCONN,      NO_MODE,  // pin  21 - 25

		GND,         GND_RES,   // pin 26 VSS USB
		NOCONN,      NO_MODE,   // pin 27
		NOCONN,      NO_MODE,   // pin 28
		VCC,         VCC_RES,   // pin 29 VCC USB
		VCC,         VCC_RES,   // pin 30 VCC USB LDO
		NOCONN,      NO_MODE,   // pin 31
		NOCONN,      NO_MODE,   // pin 32
		NOCONN,      NO_MODE,   // pin 33 VCL1 - 100nF to GND
		GND,         GND_RES,   // pin 34 VSS
		NOCONN,      NO_MODE,   // pin 35 VLO - 10kOhm PD
		NOCONN,      NO_MODE,   // pin 36 VLO - 10kOhm PD
		GND,         GND_RES,   // pin 37 VCC_DCDC (step-down reg.)
		A24,         MANUAL,    // pin 38 RESET
		A20,         MANUAL,    // pin 39 MD
		NOCONN,      NO_MODE,   // pin 40 NMI - 4.7kOhm PU
		NOCONN,      NO_MODE,   // pin 41
		NOCONN,      NO_MODE,   // pin 42
		NOCONN,      NO_MODE,   // pin 43
		NOCONN,      NO_MODE,   // pin 44
		GND,         GND_RES,   // pin 45 VSS
		VCC,         VCC_RES,   // pin 46 VCC
		NOCONN,      NO_MODE,   // pin 47
		NOCONN,      NO_MODE,   // pin 48
		NOCONN,      NO_MODE,   // pin 49
		OE_NOT,      MANUAL,    // pin 50 SWCLK
		
		D7,          BUS,       // pin 51 SWDIO
		D0,          BUS,     	// pin 52 TxD
		CE,          MANUAL,  	// pin 53 RxD
		NOCONN,      NO_MODE,   // pin 54
		NOCONN,      NO_MODE,  NOCONN,    NO_MODE,  NOCONN,      NO_MODE,  NOCONN,      NO_MODE,  NOCONN,      NO_MODE,  // pin 55 - 59
		NOCONN,      NO_MODE,   // pin 60
		NOCONN,      NO_MODE,   // pin 61
		VCC,         VCC_RES,   // pin 62 VCC
		GND,         GND_RES,   // pin 63 VSS
		NOCONN,      NO_MODE,   // pin 64 VCL - 100nF to GND
		NOCONN,      NO_MODE,  NOCONN,      NO_MODE,  NOCONN,      NO_MODE,  NOCONN,      NO_MODE,  NOCONN,      NO_MODE,  // pin 65 - 69
		A1,          MANUAL,    // pin 70 for cont. check only
		NOCONN,      NO_MODE,  NOCONN,      NO_MODE,  NOCONN,      NO_MODE,  NOCONN,      NO_MODE,  NOCONN,      NO_MODE,  // pin 71 - 75

		NOCONN,      NO_MODE,  NOCONN,      NO_MODE,  NOCONN,      NO_MODE,  NOCONN,      NO_MODE,  NOCONN,      NO_MODE,  // pin 76 - 80
		NOCONN,      NO_MODE,   // pin 81 VCL2 - 100nF to GND
		VCC,         VCC_RES,   // pin 82 VCC
		GND,         GND_RES,   // pin 83 VSS
		NOCONN,      NO_MODE,   // pin 84
		NOCONN,      NO_MODE,   // pin 85
		GND,         GND_RES,   // pin 86 VREFL
		VCC,         VCC_RES,   // pin 87 VREFH
		VCC,         VCC_RES,   // pin 88 AVCC0
		GND,         GND_RES,   // pin 89 AVSS0
		GND,         GND_RES,   // pin 90 VREFL0
		VCC,         VCC_RES,   // pin 91 VREFH0
		NOCONN,      NO_MODE,   // pin 92
		NOCONN,      NO_MODE,   // pin 93 P007 - 10kOhm PD
		NOCONN,      NO_MODE,   // pin 94 P006 - 10kOhm PD
		D15,         MANUAL,    // pin 95 for cont. check only
		NOCONN,      NO_MODE,  NOCONN,      NO_MODE,  NOCONN,      NO_MODE,  NOCONN,      NO_MODE,  NOCONN,      NO_MODE,  // pin 96 - 100 P004 - P000  - 10kOhm PD
};      // endof PinOut

DEVSECTORS _sects[] = 
{
	// following the sectortable with BYTE addresses
	// Image addresses
    0x00000000,0x00001FFF,        // Block        0    8K
    0x00002000,0x00003FFF,        // Block        1    8K
    0x00004000,0x00005FFF,        // Block        2    8K
    0x00006000,0x00007FFF,        // Block        3    8K
    0x00008000,0x00009FFF,        // Block        4    8K
    0x0000A000,0x0000BFFF,        // Block        5    8K
    0x0000C000,0x0000DFFF,        // Block        6    8K
    0x0000E000,0x0000FFFF,        // Block        7    8K
    0x00010000,0x00017FFF,        // Block        8   32K
    0x00018000,0x0001FFFF,        // Block        9   32K
    0x00020000,0x00027FFF,        // Block       10   32K
    0x00028000,0x0002FFFF,        // Block       11   32K
    0x00030000,0x00037FFF,        // Block       12   32K
    0x00038000,0x0003FFFF,        // Block       13   32K
    0x00040000,0x00047FFF,        // Block       14   32K
    0x00048000,0x0004FFFF,        // Block       15   32K
    0x00050000,0x00057FFF,        // Block       16   32K
    0x00058000,0x0005FFFF,        // Block       17   32K
    0x00060000,0x00067FFF,        // Block       18   32K
    0x00068000,0x0006FFFF,        // Block       19   32K
    0x00070000,0x00077FFF,        // Block       20   32K
    0x00078000,0x0007FFFF,        // Block       21   32K
    0x00080000,0x00087FFF,        // Block       22   32K
    0x00088000,0x0008FFFF,        // Block       23   32K
    0x00090000,0x00097FFF,        // Block       24   32K
    0x00098000,0x0009FFFF,        // Block       25   32K
    0x000A0000,0x000A7FFF,        // Block       26   32K
    0x000A8000,0x000AFFFF,        // Block       27   32K
    0x000B0000,0x000B7FFF,        // Block       28   32K
    0x000B8000,0x000BFFFF,        // Block       29   32K
    0x000C0000,0x000C7FFF,        // Block       30   32K
    0x000C8000,0x000CFFFF,        // Block       31   32K
    0x000D0000,0x000D7FFF,        // Block       32   32K
    0x000D8000,0x000DFFFF,        // Block       33   32K
    0x000E0000,0x000E7FFF,        // Block       34   32K
    0x000E8000,0x000EFFFF,        // Block       35   32K
    0x000F0000,0x000F7FFF,        // Block       36   32K
    0x000F8000,0x000FFFFF,        // Block       37   32K
    0x01000000,0x01001FFF,        // Block       38    8K Data flash
    0x01040000,0x010401FF,        // Block       39   512 options

};    // endof Sector
//-----------End standardized algorithm parameters-------------------------

// Device/Algo PARAMETER
DEVICE_DESCRIPTOR_T _dev_descriptor = 
{
	//SIGNATURE - from debug log
	//RMB				  NOA	Typ	  BFV			  DID																				PTN
	{0x00,0x5B,0x8D,0x80, 0x04, 0x01, 0x01,0x06,0x19, 0x4E,0x4B,0x2A,0x93,0xC2,0xFA,0x44,0x53,0x35,0x37,0x35,0x36,0x41,0x0D,0x97,0x61, 0x52,0x37,0x46,0x41,0x36,0x45,0x31,0x30,0x46,0x32,0x43,0x46,0x50,0x20,0x20,0x20},
	//
	{//	KOA   SAD                  EAD                  EAU                  WAU                   RAU					CAU					//AREA INFO -from debug log
		{0x00, 0x00,0x00,0x00,0x00, 0x00,0x00,0xFF,0xFF, 0x00,0x00,0x20,0x00, 0x00,0x00,0x00,0x80, 0x00,0x00,0x00,0x01, 0x00,0x00,0x80,0x00}, //area 0
		{0x00, 0x00,0x01,0x00,0x00, 0x00,0x0F,0xFF,0xFF, 0x00,0x00,0x80,0x00, 0x00,0x00,0x00,0x80, 0x00,0x00,0x00,0x01, 0x00,0x00,0x80,0x00}, //area 1
		{0x10, 0x08,0x00,0x00,0x00, 0x08,0x00,0x1F,0xFF, 0x00,0x00,0x00,0x40, 0x00,0x00,0x00,0x04, 0x00,0x00,0x00,0x01, 0x00,0x00,0x04,0x00}, //area 2
		{0x20, 0x01,0x00,0xA1,0x00, 0x01,0x00,0xA2,0xFF, 0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x10, 0x00,0x00,0x00,0x01, 0x00,0x00,0x01,0x00}, //area 3
	},
	0x01040050, // ID[31:0] address in the config area - based on Ref.Manual, section "Configuration Area Bit Map"
		 9600,    // startup baud rate - based on the bootloader spec
	2000000,    // standard uart baud rate - - based on the bootloader spec & F0157 capability
		  1024     // read page size for the UART transmission (128/256/512/768/1024. With 1024 no step-through during verify!)
};

/*************************************************************************
FUNCTION  AlgoCreate()
ARGUMENTS   none
RETURNS   pointer to algorithm object
METHOD
EXCEPTIONS --  none

This function is called by LmAlginit to create an algorithm object 
*************************************************************************/  

RRAlgorithm* AlgoCreate (void)
{
	return (new RA4E1_RA6E1(&_parms, &_pins[0], &_sects[0], &_dev_descriptor));
}
