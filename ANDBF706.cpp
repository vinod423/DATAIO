//--------------------------------------------------------------------------//
//          Copyright (c) 2003, Data I/O Corporation                        //
//          10525 Willows Rd NE / Redmond, WA 98073 / (206) 881-6444        //
//__________________________________________________________________________//
/*****************************************************************************
 
    ALGORITHM FILE: ANDBF706.cpp           Author: Vinod Dosapati
    
    Description: Analog Devices - BlackFin OTP programming support
              The programming application including the customer setup will be loaded as .ldr (absolute binary format) file and executed.
    
    Supports: 
             06/13/2019 - BF706
    
******************************************************************************/
#define ALG_DEBUG 1        // 1-per function, 2-per block, 3 add block info

#ifdef  ALG_DEBUG

#include "rr_printf.h"      // for PRINTF (debugging
#endif
#include "assert.h"
#include "standard.hpp"     // defines BYTE, WORD, etc.
#include "string.h"         // common strings
#include "PinDefines.hpp"   // #defines for pins and packages  
#include "DevParms.hpp"     // structure for pin files
#include "MicroSecDelay.h"  //
#include "FlashAlg2.hpp"    // base class 
#include "FlashAlg.inl"     // empty implementations of FlashAlg base class...needed to satisfy the linker. 
#include "ose.h"            // for millisecond delay
#include "stdio.h"          // defines sprintf
#include "LM_Phapi.hpp"
#include "HwTypes.hpp"
#include "FPGARegisters.hpp"

#include "StdWiggler.hpp"
#include "ANDBF706.hpp"                        //  device class
#include "MPC_funcs.hpp"



static const char* const __file = __FILE__;

char msgbuff[MESSAGE_LENGTH+1] = {0}; // This will be used for text handling to log.ext file


// *************************************************************************
// FUNCTION    ctor ()
// ARGUMENTS
// RETURNS
// METHOD
// EXCEPTIONS  none
// *************************************************************************
C_AND_BF706xx::C_AND_BF706xx (DEVPARMS* dparms_p, DEVPINS* dpins_p, DEVSECTORS* dsectors_p) 
                              : FlashAlg2 (dparms_p, dpins_p, dsectors_p)
                  
{
  PRINTF("The device algo is constructed\n");
}

// *************************************************************************
// FUNCTION   dtor()
// ARGUMENTS
// "ZRETURNS
// METHOD
// EXCEPTIONS none
// *************************************************************************
C_AND_BF706xx::~C_AND_BF706xx (void)
{
  PRINTF("Device algo is killed\n");
}

// *************************************************************************
// FUNCTION   Initialize()
// ARGUMENTS  none
// RETURNS
// METHOD
//     Power up the device using parameters from the pin file,
//     Pin modes and relays are already set-up
// EXCEPTIONS none
// *************************************************************************
void C_AND_BF706xx::Initialize(void)
{
  //DWORD param = 0;

#if (ALG_DEBUG > 0)
    PRINTF(" NOTE: %s Build Time / date is: %s / %s \n", __FILE__, __TIME__, __DATE__);
#endif

  // always call base class implementation
  FlashAlg::Initialize();

  // Initialize FPGA access object
  m_fpga_p = StdWiggler::Construct();
  m_fpga_p->Initialize ();
  m_fpga_p->SetSerialParams ( SPI2_MOSI,   // Serial In -> writing to the device
              SPI2_MISO,    // Serial Out-> reading from the deivce
              SPI2_CLK,   // Shift Clock pin
              90000,    // Shit clock in Hz
              m_fpga_p->EDGE_FALLING,
              m_fpga_p->MSB_FIRST,
              m_fpga_p->SINGLE_BIT);

   return;
}
// *************************************************************************
// FUNCTION       PowerUp()
// ARGUMENTS      none
// RETURNS
// METHOD
//                Initialize/re-initialize statics and call the DoPowerUp Power 
//                to power-up the DUTs
// EXCEPTIONS --  none
// *************************************************************************
void C_AND_BF706xx::PowerUp()
{
    PRINTF("C_AND_BF706xx::PowerUp()\n");                    // debug statements sent to command window

    DoPowerUp();

    return;
}


// *************************************************************************
// FUNCTION       DoPowerUp()
// ARGUMENTS      none
// RETURNS
// METHOD
//                Power up the device using parameters from the pin file,
//                Pin modes and relays are already set-up
// EXCEPTIONS --  none
// *************************************************************************
void C_AND_BF706xx::DoPowerUp()
{
  #if (ALG_DEBUG > 1)
      PRINTF("C_AND_BF706xx::DoPowerUp()\n");                    // debug statements sent to command window
  #endif
  
  m_fpga_p->ParDataCompare();     //keep data lines in input

  m_prg_api_p->SetVpullDir(HwTypes::DOWN);
    
  //power-up reset Datasheet Pg-60
  m_prg_api_p->PinSet(JTAG_TRST, LOGIC_0);
  m_prg_api_p->PinSet(SYS_HWRST, LOGIC_0);
  m_prg_api_p->VccSet(m_devparms_p->voltage_vcc, false);
  m_prg_api_p->VihSet(m_devparms_p->voltage_vih, false);
  DELAY_MS(10);
  m_prg_api_p->SetVpullDir(HwTypes::UP);

  /*set boot modes pins to no boot Mode
  SYS_BMODE[1:0]
  00 - No boot . processer goes to idle state after pre boot operations.
  01 - SPI Master Boot 
  10 - SPI Slave Boot 
  11 - UART Boot Mode
   we make use of  SPI slave boot mode so the device blackfin acts as Slave.
   our programmer will drive the clk and load the Boot Application into the Device.
   Refer Booting Modes Table. 
  */
  m_prg_api_p->PinSet(A20, LOGIC_0); // SYS_BMODE0
  m_prg_api_p->PinSet(A21, LOGIC_1); // SYS_BMODE1
 
 //set SPI2 Pins
 m_prg_api_p->PinSet(SPI2_CLK, LOGIC_1);
 m_prg_api_p->PinSet(SPI2_MOSI, LOGIC_1);
 m_prg_api_p->PinSet(SPI2_SS, LOGIC_1);

  m_prg_api_p->PinSet(SYS_HWRST, LOGIC_1);
  DELAY_MS(1);

  if (m_prg_api_p->SysEvtChk())
    PRINTF(" OC @ line %d\n", __LINE__);

  PRINTF("Device is powered-up\n");
   return;
   
}
// *************************************************************************
// FUNCTION     PowerDown()
// ARGUMENTS
// RETURNS
// METHOD
//     power down
// EXCEPTIONS --  none
// *************************************************************************
void C_AND_BF706xx::PowerDown()
{
#if (ALG_DEBUG > 2)
     PRINTF("C_AND_BF706xx::PowerDown()\n");                  // debug statements sent to command window
#endif

  DoPowerDown();

  return;
}

// *************************************************************************
// FUNCTION       DoPowerDown()
// ARGUMENTS
// RETURNS
// METHOD
//                power down
// EXCEPTIONS --  none
// *************************************************************************
void C_AND_BF706xx::DoPowerDown()
{
  #if (ALG_DEBUG > 2)
      PRINTF("C_AND_BF706xx::DoPowerDown()\n");                  // debug statements sent to command window
  #endif

  m_prg_api_p->PinSet(SYS_HWRST, LOGIC_0); //reset the device.
  DELAY_MS(1);
  m_prg_api_p->VihSet(0);
  m_prg_api_p->VccSet(0);                  // bring drivers down
  DELAY_MS(50);

  if (m_prg_api_p->SysEvtChk())
    PRINTF(" OC @ line %d\n", __LINE__);
  
  return;
}

//Search for the end of the application in the programmer's ("device's") memory map.
//It assumes, the application does not contain 512 x 0xFF in sequence.
//Returns the length in bytes.
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ //
DWORD C_AND_BF706xx::GetApplicationLength(void)
{
  DWORD byteCnt;
  WORD FFblockSize;
 
  FFblockSize = 0;
  for (byteCnt = 0; byteCnt < m_devparms_p->device_size; byteCnt += 4)
  {
    if (GetDataFromRam_32Bit(byteCnt,  (DWORD)m_srcdata_bp) != 0xFFFFFFFF)
    {
      FFblockSize = 0;
    }
    if (FFblockSize == 128)
      break;
    
    FFblockSize++;  
  }
  
  if (byteCnt == m_devparms_p->device_size)
    byteCnt = 0; //end of application did not found - force the programming to fail
    
  //adjust byteCnt
  byteCnt = byteCnt - (FFblockSize * 4) + 4;
  
  return byteCnt;
}  


// *************************************************************************
// FUNCTION IDCheck()
// ARGUMENTS
// RETURNS true if part ID == expected ID, false if not
// METHOD
//     Read device ID and compare to expected
// EXCEPTIONS --  none
// *************************************************************************
bool C_AND_BF706xx::IDCheck()
{
  PRINTF("C_AND_BF706xx::IDCheck()\n");                                 // debug statements sent to command window
  bool device_id_ok = true;
  
  return device_id_ok;
}


// *************************************************************************
// FUNCTION    BlankCheck()
// ARGUMENTS
// RETURNS     true if part is blank, false if not
// METHOD
//             Test part for blank state
// EXCEPTIONS  none
// *************************************************************************
C_AND_BF706xx::DEV_STAT_E C_AND_BF706xx::BlankCheck()
{
  //DEV_STAT_E blankcheck_stat = OPERATION_OK;
  PRINTF("C_AND_BF706xx::BlankCheck()\n");

  return OPERATION_OK;

}
// *************************************************************************
// FUNCTION     Verify()
// ARGUMENTS    none
// RETURNS      DEV_STAT_E - device status enumeration
// METHOD       Verify device data against image on a per block basis
// EXCEPTIONS   none
// *************************************************************************
C_AND_BF706xx::DEV_STAT_E C_AND_BF706xx::Verify()
{
  //DEV_STAT_E verify_stat = OPERATION_OK;
  PRINTF("C_AND_BF706xx::Verify()\n");

  return OPERATION_OK;
}
// *************************************************************************
// FUNCTION    Read()
// ARGUMENTS   none
// RETURNS     DEV_STAT_E - device status enumeration
// METHOD      Read device to image on a per block basis
// EXCEPTIONS  none
// *************************************************************************
C_AND_BF706xx::DEV_STAT_E C_AND_BF706xx::Read()
{

  PRINTF("C_AND_BF706xx::Read()\n");
  
  return OPERATION_OK;
}
 
// *************************************************************************
// FUNCTION    Erase()
// ARGUMENTS   none
// RETURNS     DEV_STAT_E - device status enumeration
// METHOD      CHIP ERASE
// EXCEPTIONS  none
// *************************************************************************
C_AND_BF706xx::DEV_STAT_E C_AND_BF706xx::Erase()
{
  //DEV_STAT_E erase_stat = OPERATION_OK;
  PRINTF("C_AND_BF706xx::Erase()\n");

  return OPERATION_OK;
}
// *************************************************************************
// FUNCTION    Program()
// ARGUMENTS   none
// RETURNS     DEV_STAT_E - device status enumeration
// METHOD      none
//             
// EXCEPTIONS  none
// *************************************************************************
C_AND_BF706xx::DEV_STAT_E C_AND_BF706xx::Program()
{
  BYTE databuffer[1];
  DWORD byteCnt;
  DWORD appLength;
  SOCKET_STATUS_T socket_stat;
  DWORD maxRetries;
  volatile BYTE *srcbase = (volatile BYTE*)m_srcdata_bp;
  DEV_STAT_E program_stat = OPERATION_OK;

  PRINTF("C_AND_BF706xx::Program()\n");

  appLength = GetApplicationLength();
  PRINTF(" .LDR lnegth: 0x%X\n", appLength);
  
  if (appLength == 0)
  {
    sprintf(msgbuff, "Critical error: the length of the application couldn't be detected. Operation aborted.");
    m_prg_api_p->Write2EventLog(msgbuff);
    return PROGRAM_ERR;
  }
  
  databuffer[0] = HOST_START_SINGLE_BIT_MODE;
   
  m_fpga_p->FastPinSet( SPI2_SS, LOGIC_0 );
  m_fpga_p->SerialWrite (&databuffer[0], 8);
  m_fpga_p->FastPinSet( SPI2_SS, LOGIC_1 );
  

  for (byteCnt = 0; byteCnt < appLength; byteCnt++)
  {
    m_fpga_p->FastPinSet(SPI2_SS, LOGIC_0);
    m_fpga_p->SerialWrite((BYTE*)(srcbase+byteCnt), 8);
    m_fpga_p->FastPinSet(SPI2_SS, LOGIC_1);
  }
  
  maxRetries = 1000;
  do
  {
    socket_stat = m_fpga_p->ParDataCompare(0x0000, APP_STATUS_PIN_D2_MASK);
  } while (maxRetries-- && socket_stat);

  if (CompareFailed(socket_stat))
  {
    if (!m_prg_api_p->MisCompare(DeviceOperation::PROGRAM, socket_stat, 0, 0))
    {
      m_prg_api_p->Write2EventLog("C_AND_BF706xx::Program() - APP timeout error!");
      return PROGRAM_ERR;
    }
  }
  
  maxRetries = 5000;
  do
  {
    DELAY_MS(1);
    socket_stat = m_fpga_p->ParDataCompare(0xFFFF, PRG_STATUS_PIN_D3_MASK);
  } while (maxRetries-- && socket_stat);

  if (CompareFailed(socket_stat))
  {
    if (!m_prg_api_p->MisCompare(DeviceOperation::PROGRAM, socket_stat, 0, 0))
    {
      m_prg_api_p->Write2EventLog("C_AND_BF706xx::Program() - Failed!");
      return PROGRAM_ERR;
    }
  }
  
  return program_stat;
}
// *************************************************************************
// FUNCTION    Secure()
// ARGUMENTS   none
// RETURNS     DEV_STAT_E - device status enumeration
// METHOD      Program Option Bytes on x16 basis
// EXCEPTIONS  none
// *************************************************************************
C_AND_BF706xx::DEV_STAT_E C_AND_BF706xx::Secure()
{
  //DEV_STAT_E secure_stat = OPERATION_OK;
  PRINTF("C_AND_BF706xx::Secure()\n");

  return OPERATION_OK;
}
/********************************************************************************/

