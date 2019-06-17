//--------------------------------------------------------------------------//
//          Copyright (c) 2003, Data I/O Corporation                        //
//          10525 Willows Rd NE / Redmond, WA 98073 / (206) 881-6444        //
//__________________________________________________________________________//
/*****************************************************************************
      
      
      PinFile: AND_ADSP_BF706KCPZ_QFN88.cpp          Author: Vinod Dosapati
      
      Description:
      Package: QFN88
     
*****************************************************************************/
#include "standard.hpp"     // defines BYTE, WORD, etc.
#include "PinDefines.hpp"   // #defines for pins and packages  
#include "DevParms.hpp"     // structure for pin files
#include "FlashAlg2.hpp"    // 
#include "Swd_F17x.hpp"     // C_SWD
#include "ANDBF706.hpp"     //Device class  
#include "AND_BF706_CODE.hpp"//bootloader code
   

//-----------Begin standardized algorithm parameters-------------------------
DEVPARMS _parms ={
		MICRO, 	                //DEVPARMS type ----> MICRO
    3382,   		       		 //Adaapter id
    CONTINUITY1,            // continutity check
    0x0000,                 // mfg ID       
    0x0023,                 // device ID (BYTE2&BYTE3)
    8,                      // data width
    0x500,               	  // device size address space of micro,programmable 0x48C bytes+lockbit+reserved 
		1,                   	  // number of sectors; (it must be the same as in the database, no matter whether it means area or block)
		88,                  	  // number of pins
    330,                    // Vcc voltage
    0,                      // Vpp voltage
    310,                    // Vih voltage
    0,                      // Alt voltage
    0,                      // erase time per block in ms
    0,                      // program time - not used
    false,                  // Sector Protect: device does not support protect
    false,                  // Software Data Protect
    false,                  // Boot Block Protect1
    0,  										// alg_features
    0,                      // reserved integer 1
    0,                      // reserved integer 2
    0,                      // reserved integer 3
    0,                      // reserved integer 4
    360,                    // voltage_vcc_high
    180,                    // voltage_vpp_high - not used
    330,                    // voltage_vih_high
    300,                    // voltage_vcc_low
    180,                    // voltage_vpp_low - not used
    270,                    // voltage_vih_low
    340,                    // voltage_vcc_prog
    180,                    // voltage_vpp_prog - not used
    320                     // voltage_vih_prog
};


DEVPINS _pins[] = 
{
	NOCONN,		NO_MODE,	// pin 1
	NOCONN,		NO_MODE,	// pin 2
	NOCONN,		NO_MODE,	// pin 3
	VCC,			VCC_RES,	// pin 4
	NOCONN,		NO_MODE,	// pin 5
	NOCONN,		NO_MODE,	// pin 6
	NOCONN,		NO_MODE,	// pin 7
	NOCONN,		NO_MODE,	// pin 8
	NOCONN,		NO_MODE,	// pin 9
	NOCONN,		NO_MODE,	// pin 10
	VCC,			VCC_RES,	// pin 11
	NOCONN,		NO_MODE,	// pin 12 testpin
	NOCONN,		NO_MODE,	// pin 13
	NOCONN,		NO_MODE,	// pin 14
	NOCONN,		NO_MODE,	// pin 15 testpin
	NOCONN,		NO_MODE,	// pin 16
	VCC,			VCC_RES,	// pin 17
	GND,			GND_RES,	// pin 18
	GND,			GND_RES,	// pin 19
	NOCONN,		NO_MODE,	// pin 20
	GND,			GND_RES,	// pin 21
	GND,			GND_RES,	// pin 22
	CE,				MANUAL,		// pin 23 SPI2-CS
	D3,				MANUAL,		// pin 24 SPI2_D3
	D2,				MANUAL,		// pin 25 PB_13 SPI2_D2
	VCC,			VCC_RES,	// pin 26
	GP1,			MANUAL,		// pin 27 SPI2-MOSI
	D0,				MANUAL,		// pin 28 SPI2-MISO
	A31,			MANUAL,		// pin 29	SPI2_CLK
	NOCONN,		NO_MODE,	// pin 30
	NOCONN,		NO_MODE,	// pin 31
	GND,			GND_RES,	// pin 32
	GND,			GND_RES,	// pin 33
	GND,			GND_RES,	// pin 34
	NOCONN,		NO_MODE,	// pin 35
	VCC,			VCC_RES,	// pin 36
	GND,			GND_RES,	// pin 37
	NOCONN,		NO_MODE,	// pin 38
	NOCONN,		NO_MODE,	// pin 39
	NOCONN,		NO_MODE,	// pin 40
	VCC,			VCC_RES,	// pin 41
	NOCONN,		NO_MODE,	// pin 42
	NOCONN,		NO_MODE,	// pin 43
	NOCONN,		NO_MODE,	// pin 44
	NOCONN,		NO_MODE,	// pin 45
	NOCONN,		NO_MODE,	// pin 46
	NOCONN,		NO_MODE,	// pin 47
	NOCONN,		NO_MODE,	// pin 48
	VCC,			VCC_RES,	// pin 49
	VCC,			VCC_RES,	// pin 50
	NOCONN,		NO_MODE,	// pin 51
	NOCONN,		NO_MODE,	// pin 52
	NOCONN,		NO_MODE,	// pin 53
	NOCONN,		NO_MODE,	// pin 54
	VCC,			VCC_RES,	// pin 55
	NOCONN,		NO_MODE,	// pin 56
	NOCONN,		NO_MODE,	// pin 57 SYS_CLKIN
	NOCONN,		NO_MODE,	// pin 58
	NOCONN,		NO_MODE,	// pin 59
	NOCONN,		NO_MODE,	// pin 60
	NOCONN,		NO_MODE,	// pin 61
	VCC,			VCC_RES,	// pin 62
	NOCONN,		NO_MODE,	// pin 63
	NOCONN,		NO_MODE,	// pin 64
	NOCONN,		NO_MODE,	// pin 65
	A20,			MANUAL,		// pin 66 SYS_BMODE0 
	A21,			MANUAL,		// pin 67 SYS_BMODE1
	A24,			MANUAL,		// pin 68 SYS_HWRST
	NOCONN,		NO_MODE,	// pin 69
	NOCONN,		NO_MODE,	// pin 70
	NOCONN,		NO_MODE,	// pin 71
	VCC,			VCC_RES,	// pin 72
	NOCONN,		NO_MODE,	// pin 73
	D1,				MANUAL,		// pin 74 SPI2_RDY
	NOCONN,		NO_MODE,	// pin 75
	GND,			GND_RES,	// pin 76
	NOCONN,		NO_MODE,	// pin 77 connected vis pullup resistor 10K
	NOCONN,		NO_MODE,	// pin 78
	NOCONN,		NO_MODE,	// pin 79
	NOCONN,		NO_MODE,	// pin 80
	NOCONN,		NO_MODE,	// pin 81
	VCC,			VCC_RES,	// pin 82
	NOCONN,		NO_MODE, 	// pin 83 JTAG_TDO optional
	D7,		    BUS,			// pin 84 SWDIO
	OE,		    MANUAL,		// pin 85 SWCLK
	NOCONN,		NO_MODE,	// pin 86 JTAG_TDI optional
	A15,			MANUAL,		// pin 87 JTAG_TRST
	NOCONN,		NO_MODE,	// pin 88
	GND,			GND_RES,	// pin EXTP1
};// end of pin defines


DEVSECTORS _sects[] = 
{
    0x00000000,0x0000048B,        // Block        0  1164
};// end of sector table

// Device/Algo PARAMETER
DEVICE_DESCRIPTOR_T _dev_descriptor = 
{
  0x11A00000,                       //start address in RAM
  &flash_code[0],                   //prog code
  sizeof(flash_code)/sizeof(BYTE)   //prog code size
};

// /*************************************************************************
// FUNCTION 	AlgoCreate()
// ARGUMENTS  	none
// RETURNS		pointer to algorithm object
// METHOD
// EXCEPTIONS --  none
// 
// This function is called by LmAlginit to create an algorithm object
// *************************************************************************/
RRAlgorithm* AlgoCreate (void)
{
    return (new C_AND_BF706xx(&_parms, &_pins[0],  &_sects[0], 12000000,&_dev_descriptor) );
}