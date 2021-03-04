// $Header: /psbu/RoadRunner/sw/src/rralg/rtcrv40f.cpp 29    10/22/20 1:15a Dosapav $
//--------------------------------------------------------------------------//
//          Copyright (c) 1999->2004, Data I/O Corporation                  //
//          10525 Willows Rd NE / Redmond, WA 98073 / (425) 881-6444        //
//__________________________________________________________________________//

/***************************************************************************

    ALGORITHM FILE: rtcrv40f.cpp          Author: Stefan Konrad

    DESCRIPTION   : Implements the base class for Renesas' SINGLE VOLTAGE 
                    FLASH MICROCONTROLLERS with MONOS flash, RV40F technology
                    
                    Based on the specification:
                    TTS-SG-13-0117-02a.pdf - Flash Serial Programming Flow of PG-FP5 for RV40F System Specification
                   
    SUPPORTS      :  
                      FLASH MACRO    DEVICE-Type    Notes
                      -----------    -----------    -----------
                      RV40F           RH850/F1L     supported
                      RV40F           RH850/F1H     supported
                      RV40F           RH850/D1x     supported
                      RV40F           RH850/P1x     supported
                      
    DATA I/O SPID :  

                    MM/DD/YY
    Version 1.0   : 05/22/14 - Initial release
                               To-do-list (aka not supported yet):
                               - OTP-Bit handling (blocks can be set OTP): done (RPI)
                               - Lock-Bit handling: done (RPI)
                               - ID authentication handling (password protection): done (RPI)
                               - Serial Programming Disable (SPD) cmd: done (RPI)
                               - ICU-S (Cryptography unit) handling: done (RPI)
                               - PR5 file interpreter (upgrade PFI script): done
            1.01  : 07/02/14 - bugfix in DeviceInit(): initialized socket_stat to 0; affected MasterRead cmd.
                             - prepared Lock-/OTP-Bit, SPD programming but not released yet (SFM interface not implemented yet)
                             - updated PFI script
            1.1   : 09/30/15 - added blank check for data flash
            1.2   : 11/03/15 - prepared EXTENDED_OPTION commands
                             - bugfix for Startup-sequence: replaced fpga-SerialWrite.
            1.3   : 03/04/16 - added VerifyDFBlock(...) to allow verify of data units smaller than 16bytes
            
            2.0   : 08/19/16 - added RPI file support (in RH850DataMapper.dll Ver. 1.0.0.3 ) with ALL device features supported
                               For upgrading existing device supports in SEMI/DSE consider the following:
                               (1) select Secure Device: Possible_SF (the algo will check what to do)
                               (2) select the device footnotes: 1797,2537,1795,(2552),2539
                               (2.1) !!!Caution: for devices with TYPE6 = 0x48 (DF unit = 16 bytes): add RTC_RH850_FILL_DF to SFM, inclusively footnote 2552!
                               (3) add ID Code to SFM (RTC_RH850_ID16_OLD)
                               (4) update the PRM_T t_xxxx_prm struct in the device parameter file using the latest version of pfi.wsf script in VSS.
            2.1   : 08/23/16 - skip ID code verify when device is Read Protected. Abort overprogramming of ICU_S devices, if ICU_S area contains data.
            2.1.1 : 09/06/16 - fixed BC error, when Clear command was called during erase.
            2.2   : 11/16/16 - added feature: Code flash fill up can be disabled.
            3.0   : 11/29/16 - speedup using FPGA sequencer F172 rev. 9. Programming time reduced to a quarter.
                               For upgrading existing device supports in SEMI/DSE consider the following:
                               (1) Disable blankcheck.
                               (2) Select ERASE: Do always.
                               (3) add RTC_RH850_FILL_CF to SFM, inclusively footnote 2657! - works for RPI files only!!! 
                                   (if no RPI file AND CF fill-up is disabled, code flash won't be programmed & verified! - fixed w/ DLL version 1.0.0.7)
            3.5   : 03/21/17 - enabled ICU-S region erase cmd (if supported by the device)
            3.6   : 04/11/17 - prepared for RH850/P1x-C family 
            4.0   : 04/14/17 - added RH850/P1x-C class (based on spec. EETS-ER-0047-0.4)
            4.1   : 07/07/17 - fixed ID code handling in RH850/P1x-C class.
                               adapted TYP7 handling to F1K family (ICU-S 2kB supported)
            4.2   : 11/28/17 - added VerifyArea(...), added comments, fixed READ_CMD for areas smaller than MAX_PAGE_SIZE
            4.21  : 03/16/18 - adapted ICU-S error message in Secure()
            5.0   : 06/06/18 - refactored option parameters init. Moved ID_CODE programming from Secure() into program/verify block to allow SNS support in that area. ID_AUTH must stay in Secure().
            5.1   : 07/04/18 - bugfix for m_ICU_S_RegionSize (introduced on 6/6/18): set to 0 in Initialize.
            5.2   : 07/12/18 - fixed potential buffer overflow in ReadDataFrame(BYTE* buffer_p) routine, used in DeviceInit()
            5.3   : 07/19/18 - fix for the change on 6/6/18: in Secure() replaced in the Sector check PROGRAM_SECTOR_OP flag by BLANK_CHECK_OP, so Secure() flow executes even if options sector contains SN#.
                             - in Secure() skip programming of ID_CODE for AUTHentication if options sector does contain SN.
            5.4   : 08/13/18 - bugfix for m_ICU_S_RegionSize (introduced on 6/6/18): init it in DeviceInit() correctly. Added: WORD get_ICU_S_RegionSize(void);
            5.5.  : 09/04/18 - fixed handling for devices, where ICU-S erase is prohibited & validated. Reprogramming is possible when the datafile does not contain data for ICU-S area and ICU-S not requested again.
            5.6   : 02/14/19 - introduced a reserved parameter to handle JTAG init frequency for slower devices. -Vinod                                                  
            5.7   : 01/15/20 - fixed ICU-M programming in C_RV40F flow. Wrong mask used on OPBT8, thus ICU-M activating did not work.
            6.0   : 09/29/20 - added an extra class to implements customized routine for Kimball electronic.Refer DSR#62314.
            6.1		: 02/08/21 - Adapted SendFrame, GetDataFrame, ReadDataFrame to speedup.
            6.2   : 02/10/21 - Modified Read Function, to enable reading of  Data flash using Exteneded Read Cmd for the Supported devices.
            6.3 	: 02/10/21 -  IDCode from SF Menu will be used to unlock the Device. And the ID in RPI will be Programmed.   
            7.0   : 03/02/21 - Reprogramming and validation of ICUS Flow has been Changed.
***************************************************************************/
#define ALG_DEBUG 2 // 1-per function, 2-per block, 3 add block info

#ifdef ALG_DEBUG
#include "rr_printf.h" // for PRINTF (debugging
#endif
#include "assert.h"
#include "standard.hpp"    // defines BYTE, WORD, etc.
#include "string.h"        // common strings
#include "PinDefines.hpp"  // #defines for pins and packages
#include "DevParms.hpp"    // structure for pin files
#include "MicroSecDelay.h" //
#include "FlashAlg2.hpp"   // base class
#include "FlashAlg.inl"    // empty implementations of FlashAlg base class...needed to satisfy the linker.
#include "ose.h"           // for millisecond delay
#include "stdio.h"         // defines sprintf
#include "LM_Phapi.hpp"

#include "JTAG_Boost_API.hpp"
#include "StdWiggler.hpp"
#include "rtcrv40f.hpp" // class definition for C_RV40F
#include "MPC_funcs.hpp"

static const char *const __file = __FILE__;

char msgbuff[MESSAGE_LENGTH + 1] = {0};

/////////////////////////////////////////////////////////////////////////////////////////
//         SocketNumChange ()
// Inputs: skt: number of DUT
// Return: number of socket
// METHOD:   Translates DUT# in socket#. Socket# is used when communicating to
//      the user interface.
/////////////////////////////////////////////////////////////////////////////////////////
WORD SocketNumChange(int skt)
{
  return WORD(SKT_CHG_FACTOR - skt);
}

/////////////////////////////////////////////////////////////////////////////////////////
//          SwapEndian ()
// Inputs:  val: 32 bit (4 bytes) value
// Return:  swapped value
/////////////////////////////////////////////////////////////////////////////////////////
DWORD SwapEndian(DWORD val)
{
  DWORD ret_val = 0;
  for (int i = 3; i >= 0; i--)
  {
    ret_val |= (LOBYTE(val)) << (i * 8);
    val >>= 8;
  }
  return ret_val;
} //SwapEndian()

//////////////////////////// CRC32 algorithm ///////////////////////////////
// crc32h.c -- package to compute 32-bit CRC one byte at a time using
//             the high-bit first (Big-Endian) bit ordering convention
//
// Synopsis:
//
// gen_crc_table() -- generates a 256-word table containing all CRC
//                    remainders for every possible 8-bit byte. It
//                    must be executed (once) before any CRC updates.
//
// unsigned update_crc(crc_accum, data_blk_ptr, data_blk_size)
//
// unsigned crc_accum; char *data_blk_ptr; int data_blk_size;
//
// Returns the updated value of the CRC accumulator after
// processing each byte in the addressed block of data.
//
// It is assumed that an unsigned long is at least 32 bits wide and
// that the predefined type char occupies one 8-bit byte of storage.
//
// The generator polynomial used for this version of the package is
// x^32+x^26+x^23+x^22+x^16+x^12+x^11+x^10+x^8+x^7+x^5+x^4+x^2+x^1+x^0
// as specified in the Autodin/Ethernet/ADCCP protocol standards.
// Other degree 32 polynomials may be substituted by re-defining the
// symbol POLYNOMIAL below. Lower degree polynomials must first be
// multiplied by an appropriate power of x. The representation used
// is that the coefficient of x^0 is stored in the LSB of the 32-bit
// word and the coefficient of x^31 is stored in the most significant
// bit. The CRC is to be appended to the data most significant byte
// first. For those protocols in which bytes are transmitted MSB
// first and in the same order as they are encountered in the block
// this convention results in the CRC remainder being transmitted with
// the coefficient of x^31 first and with that of x^0 last (just as
// would be done by a hardware shift register mechanization).
//
// The table lookup technique was adapted from the algorithm described
// by Avram Perez, Byte-wise CRC Calculations, IEEE Micro 3, 40 (1983).

#define POLYNOMIAL 0x04c11db7L

static DWORD crc_table[256];

// generate the table of CRC remainders for all possible bytes
void gen_crc_table()
{
  register int i, j;
  register DWORD crc_accum;

  for (i = 0; i < 256; i++)
  {
    crc_accum = ((DWORD)i << 24);

    for (j = 0; j < 8; j++)
    {
      if (crc_accum & 0x80000000L)
        crc_accum = (crc_accum << 1) ^ POLYNOMIAL;
      else
        crc_accum = (crc_accum << 1);
    }
    crc_table[i] = crc_accum;
  }
  return;
}

// update the CRC on the data block one byte at a time
DWORD update_crc(DWORD crc_accum, BYTE *data_blk_ptr, DWORD data_blk_size)
{
  register DWORD i, j;

  for (j = 0; j < data_blk_size; j++)
  {
    i = ((DWORD)(crc_accum >> 24) ^ *data_blk_ptr++) & 0xff;
    crc_accum = (crc_accum << 8) ^ crc_table[i];
  }
  return crc_accum;
}

// Algorithm based on the device specs:
// - RSO-14-006866-05 (Mar/2016)        : Flash serial programming flow
// - RSO-14-002349-11/TTS-SG-13-0119-03 : Device Type Code of RV40F
// - IDF-15-009927-01                   : Renesas Flash Programmer Image File (RPI) Format Specification
//
//
// Device Type Code table:
//
// Idx  Type  Class        Bit   Init   Content          Comments
//----------------------------------------------------------------------------------------
//  0   TYP1  DeviceCode   7-4    -     DeviceCode       1: RH850
//                         3-0    0     CustomCode       0: Standard device, 1: CC-Cube (aka P1x-C: not supported yet, some new flows and commands).
//----------------------------------------------------------------------------------------
//  1   TYP2  Bootfirm     7-4    0     Boot revision    reserved for custom
//                         3-2    0     Reserved
//                           1    -     Area Erase       0: not supported,  1: supported
//                           0    0     Ext. func.       0: not supported,  1: supported
//----------------------------------------------------------------------------------------
//  2   TYP3  Function     7-2    1     Reserved
//                           1    -     Lock-bit UB      0: not supported, 1: supported
//                           0    -     Lock-bit Code    0: not supported, 1: supported
//----------------------------------------------------------------------------------------
//  3   TYP4  Function       7    0     Reserved
//                           6    -     OTP              0: not supported, 1: supported
//                           5    0     Reserved
//                           4    0     Reserved
//                           3    -     Endian           0: not supported, 1: supported
//                           2    0     Reserved
//                           1    -     Reset Vector     0: not supported, 1: supported
//                           0    -     OFS              0: not supported, 1: supported
//----------------------------------------------------------------------------------------
//  4   TYP5  Clock        7-3    0     Reserved
//                           2    -     Int OSC f. RH850 0: not supported, 1: supported
//                           1    -     Int OSC f. USB   0: not supported, 1: supported
//                           0    -     int OSC f. CSI   0: not supported, 1: supported
//----------------------------------------------------------------------------------------
//  5   TYP6  Mode           7    0     Reserved
//                         6-4    -     Min Prog Unit (data)
//                                                           1: 2byte, 2: 4byte, 3: 8byte, ..., 8: 256byte, 9: 512byte
//                         3-0    -     Min Prog Unit (code)
//----------------------------------------------------------------------------------------
//  6   TYP7  Security       7    0     Reserved
//                           6    -     Trusted Memory       0: not supported, 1: supported
//                           5    -     ICU Erase            0: Permit/not sup, 1: prohibit
//                           4    0     Reserved
//                           3    -     ICU Option byte      0: not supported, 1: supported
//                           2    -     ICU-S (2K)           0: not supported, 1: supported (Intelligent cryptographic unit-slave: when activated -> last 2kB of dataflash will be hidden).
//                           1    -     ICU-M                0: not supported, 1: supported (Intelligent cryptographic unit-master: when activated -> parts of code and dataflash will be hidden, no access to the device)
//                           0    -     ICU-S (1K)           0: not supported, 1: supported (Intelligent cryptographic unit-slave: when activated -> last 1kB of dataflash will be hidden).
//----------------------------------------------------------------------------------------
//  7   TYP8  Reserved     7-0    0     Reserved
//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------

/////////////////////////////////////////////////////////////////////////////////////////
//          C_RV40F ctor ()
// Inputs:
//
// Return:
//
// METHOD:
////////////////////////////////////////////////////////////////////////////////
C_RV40F::C_RV40F(DEVPARMS *dparms_p, DEVPINS *dpins_p, DEVSECTORS *dsectors_p,
                 BYTE opfreq_parm, PRM_T *dprm_p, ALG_CODE *tBoot_code_p)
    : FlashAlg2(dparms_p, dpins_p, dsectors_p),
      m_op_frequency(opfreq_parm),
      m_tRV40F_Param_p(dprm_p),
      m_tBoot_code_p(tBoot_code_p)
{

  m_fpga_p = NULL;

} //C_RV40F ctor()

/*******************************************************************************
*         C_RV40F dtor ()
* Inputs:   
*       
* Return:   
*       
* METHOD:     
*******************************************************************************/
C_RV40F::~C_RV40F()
{

  delete m_fpga_p;
  m_fpga_p = NULL;
}

/*******************************************************************************
*         Initialize ()
* Inputs:   
*       
* Return:   
*       
* METHOD:     
*******************************************************************************/
void C_RV40F::Initialize(void)
{
  int i, j;
  DWORD addrCnt;
  DWORD param;

  DWORD fpga_ver = LM_Phapi::Get()->FPGAVerGet();

  WORD minProgUnitCode;
  // call base class impplementation
  FlashAlg2::Initialize();

  // Initialize standard FPGA access object
  m_fpga_p = StdWiggler::Construct();
  if (!m_fpga_p)
    m_prg_api_p->ThrowException(ALG_LOGIC_ERR_S, __LINE__, __file);
  m_fpga_p->Initialize((DWORD)m_op_frequency * 100000, STARTUP_SCI_FREQUENCY);

  // If available, initialize special FPGA access object
  // The algo uses FPGA-speedup for transfering 1kB page data during programming
  m_fpga_speedup_supported = false;
  if (fpga_ver == 0xF172)
  { //min. revision = 9
    PRINTF("C_RV40F:Initialize() - Sequencer FPGA detected \n");
    DWORD JTAG_init_freq = m_devparms_p->reserved1;
    if (0 == JTAG_init_freq)
      JTAG_init_freq = 4800000;
    //                       TCK            TDI  TDO     MSB
    JTAGBoost_p->Initialize(JTAG_init_freq, OE, NULL_NAME, A25, D0, 0, true); //FLSCI3SCI cycle time tKCYSF = 200ns (max. 5MHz).
    if (!JTAGBoost_p->IsSupported(JTAGBoost_p->JT_TDI_A24_27))
      PRINTF("... wrong revision. \n");
    else
    {
      m_fpga_speedup_supported = true;

      //Bank 0: SOD+LN+WRITE_CMD
      JTAGBoost_p->SetRAMValues(0x81040113, 0, 0, 0, 32, 0, false);
      //Bank 1 - 256: 1024 data bytes
      //MAX_PAGE_SIZE = 1024B = 256 x 4B
      for (DWORD bank_nr = 1; bank_nr < 256; bank_nr++)
        JTAGBoost_p->SetRAMValues(0x00000000, 0, 0, 0, 32, bank_nr, false);
      JTAGBoost_p->SetRAMValues(0x00000000, 0, 0, 0, 32, bank_nr, true); //last frame
    }
  }

  for (j = 0; j < MAX_SOCKET_NUM; j++) // intialize buffer with -1(non-responsive state)
  {
    m_ICU_S_Status[j] = SOCKETS_HAVE_DIFFERENT_STATUS;
  }

  m_fTarget_initialized = false;
  m_fStartupMode = false;
  m_reset_H_flmd0_pulse_start_wait = 0;

  m_sector_quantity = m_devparms_p->sector_quantity;
  for (int block = 0; block < m_sector_quantity; block++)
  {
    if ((m_devsectors_p[block].begin_address & PROT_OFFSET) == PROT_OFFSET)
      m_option_data_block = (WORD)block;
    if ((m_devsectors_p[block].begin_address & DF_START_IN_IMAGE) == DF_START_IN_IMAGE)
    {
      m_DF_block_nr = (WORD)block; //First DF sector
      break;
    }
  } //--end of all blocks

  ALG_ASSERT(((m_tRV40F_Param_p->TYPE[6] & 0x5) && (m_DF_block_nr != (m_sector_quantity - 1))) == false); //The current algorithm does NOT work correctly, when standard ICU_S feature is available and more than one data flash sector exists!

  m_id_len = 16;                      //[bytes]
  m_signature_len = SIGNATURE_LENGTH; //58 bytes

  //Initialize special data
  //let's get the required data
  char temp_buf[33];
  char *ret_val = temp_buf;
  if (!m_prg_api_p->SpecFeatureParmGet("ID Code", &ret_val))
  {
    strcat(msgbuff, "Parameter <<ID Code>> missed, check Database !!!");
    m_prg_api_p->ThrowException(PARAM_UNAVAILABLE_ERR_S, __LINE__, __file, msgbuff);
  }
  for (i = 0; i < m_id_len; i++)
  {
    m_IDCODE_buffer[i] = ret_val[m_id_len - 1 - i];
  }

  m_ID_buffer_p = &m_srcdata_bp[ID_OFFSET];

  if (!m_prg_api_p->SpecFeatureParmGet("OPBT_StartAddr", &param))
  {
    strcat(msgbuff, "Hidden parameter <<OPBT_StartAddr>> missed (required for DLL), check Database !!!");
    m_prg_api_p->ThrowException(PARAM_UNAVAILABLE_ERR_S, __LINE__, __file, msgbuff);
  }

  if (m_prg_api_p->SpecFeatureParmGet("Verify mode", &param))
    PRINTF("<<Verify mode>> found\n");
  else
  {
    strcat(msgbuff, "<<Verify mode>> missed, check Database !!!");
    m_prg_api_p->ThrowException(PARAM_UNAVAILABLE_ERR_S, __LINE__, __file, msgbuff);
  }
  if (param & 2)
    m_VerifyType = CHECKSUM_VERIFY;
  else
    m_VerifyType = DATA_VERIFY;

  m_DF_erase_unit = (DWORD)m_tRV40F_Param_p->SIGNATURE[55]; //Bytes
  //find out the minimum programming unit for Data Flash
  minProgUnitCode = (m_tRV40F_Param_p->TYPE[5] >> 4) & 0x7;
  m_DF_write_unit = 1 << minProgUnitCode; //Bytes

  m_fDF_filled0xFF = false; //default
  if (m_DF_write_unit == 16)
  {
    if (m_prg_api_p->SpecFeatureParmGet("Data Flash - fill up with 0xFF", &param))
      PRINTF("<<Data Flash - fill up with 0xFF>> found: %Xh \n", param);
    else
    {
      strcat(msgbuff, "<<Data Flash - fill up with 0xFF>> missed, check Database !!!");
      m_prg_api_p->ThrowException(PARAM_UNAVAILABLE_ERR_S, __LINE__, __file, msgbuff);
    }
    if (param)
      m_fDF_filled0xFF = true;
  } //-- OF if (m_DF_write_unit == 16)

  m_fCF_filled0xFF = true; //default
  if (m_prg_api_p->SpecFeatureParmGet("Code Flash - fill up with 0xFF", &param))
  {
    PRINTF("<<Code Flash - fill up with 0xFF>> found: %Xh \n", param);
    if (param == 0)
      m_fCF_filled0xFF = false;
  }
  m_ICU_S_RegionSize = 0;
  m_optionSupportedByDev = 0;
  m_optionSelectedByUser = 0;

  if (m_tRV40F_Param_p->TYPE[1] == 0x20) //TYP2
    m_optionSupportedByDev |= CFG_WRITE;

  if (m_tRV40F_Param_p->TYPE[2] & 0x01) //TYP3
    m_optionSupportedByDev |= LB;

  if (m_tRV40F_Param_p->TYPE[3] & 0x40) //TYP4
    m_optionSupportedByDev |= OTP;

  if (m_tRV40F_Param_p->TYPE[6] & 0x01) //TYP7.Bit0
  {
    m_optionSupportedByDev |= ICU_S; //1kB
    m_ICU_S_RegionSize = 0x400;
  }
  if (m_tRV40F_Param_p->TYPE[6] & 0x02) //TYP7.Bit1
  {
    m_optionSupportedByDev |= OPBTEX;
    m_optionSupportedByDev |= ICU_M;
  }
  if (m_tRV40F_Param_p->TYPE[6] & 0x04) //TYP7.Bit2
  {
    m_optionSupportedByDev |= ICU_S; //2kB
    m_ICU_S_RegionSize = 0x800;
  }
  if (m_tRV40F_Param_p->TYPE[6] & 0x20)
    m_optionSupportedByDev |= ICU_S_ERASE_PROHIBITED;

  //check for selected features
  for (addrCnt = 0; addrCnt < LB_LENGTH; addrCnt++)
  {
    if (GetDataFromRam_8Bit(LOCK_BIT_OFFSET + addrCnt, (DWORD)m_srcdata_bp) != 0xFF)
    {
      m_optionSelectedByUser |= LB;
      break;
    }
  }
  for (addrCnt = 0; addrCnt < LB_LENGTH; addrCnt++)
  {
    if (GetDataFromRam_8Bit(OTP_BIT_OFFSET + addrCnt, (DWORD)m_srcdata_bp) != 0xFF)
    {
      m_optionSelectedByUser |= OTP;
      break;
    }
  }

  for (addrCnt = 0; addrCnt < OPBT_LENGTH; addrCnt++)
  {
    if (GetDataFromRam_8Bit(OPBT_OFFSET(0) + addrCnt, (DWORD)m_srcdata_bp) != 0xFF)
    {
      m_optionSelectedByUser |= OPBT;
      break;
    }
  }
  for (addrCnt = 0; addrCnt < EXT_OPBT_LENGTH; addrCnt++)
  {
    if (GetDataFromRam_8Bit(OPBT_OFFSET(0) + OPBT_LENGTH + addrCnt, (DWORD)m_srcdata_bp) != 0xFF)
    {
      m_optionSelectedByUser |= OPBTEX;
      break;
    }
  }

  if (GetDataFromRam_8Bit(IDSEL_OFFSET, (DWORD)m_srcdata_bp) == 0x00)
    m_optionSelectedByUser |= ID_CODE;
  if (GetDataFromRam_8Bit(ID_AUTH_OFFSET, (DWORD)m_srcdata_bp) == 0x00)
    m_optionSelectedByUser |= ID_AUTH;
  if (GetDataFromRam_8Bit(ICU_S_OFFSET, (DWORD)m_srcdata_bp) == 0x00)
    m_optionSelectedByUser |= ICU_S;
  if (GetDataFromRam_8Bit(SPD_OFFSET, (DWORD)m_srcdata_bp) == 0x00)
    m_optionSelectedByUser |= SPD;

  gen_crc_table();

  //m_devparms_p->reserved4 is used for delay settings in Power Up/Down routines
  //LOBYTE = delay setting after Power Up [100ms]
  //HIBYTE = delay setting after Power Down [100ms]

} //C_RV40F::Initialize()

/*******************************************************************************
*         C_RV40F::DoPowerUp ()
* Inputs:   
*       
* Return:   
*       
* METHOD:     
*******************************************************************************/
void C_RV40F::DoPowerUp(void)
{
  PRINTF("C_RV40F::DoPowerUp()\n"); // debug statements

  m_akt_skt_msk = 0;
  for (WORD dut = 0; dut < MAX_SOCKET_NUM; dut++)
  {
    // If socket is enabled
    if (SOCKET_ENABLE == m_prg_api_p->SocketStatusGet((WORD)(SKT_CHG_FACTOR - dut - 1)))
      m_akt_skt_msk |= (1 << dut);
  } //-- OF for (WORD dut=0; dut<MAX_SOCKET_NUM; dut++)

  // Power up the device
  m_fpga_p->ParDataCompare();               //on some adapter datapins are eventually connected to DRST. Keep it LOW.
  m_prg_api_p->PinSet(_RESET_PIN, LOGIC_0); //POR
  m_prg_api_p->PinSet(JP0_2_SCK_PIN, LOGIC_0);
  m_prg_api_p->PinSet(JP0_0_SI_PIN, LOGIC_0);
  m_prg_api_p->PinSet(FLMD0_PIN, LOGIC_0);

  m_prg_api_p->SetVpullDir(HwTypes::UP); // pull up D0 (SO)

  if (m_prg_api_p->PinExists(JP0_4_TRST_PIN))
    m_prg_api_p->PinSet(JP0_4_TRST_PIN, LOGIC_0);
  if (m_prg_api_p->PinExists(POWERGOOD_PIN))
    m_prg_api_p->PinSet(POWERGOOD_PIN, LOGIC_0);
  if (m_prg_api_p->PinExists(MODE0_PIN))
    m_prg_api_p->PinSet(MODE0_PIN, LOGIC_1); //don't care in programming mode
  if (m_prg_api_p->PinExists(MODE1_PIN))
    m_prg_api_p->PinSet(MODE1_PIN, LOGIC_1); //don't care in programming mode
  if (m_prg_api_p->PinExists(JP0_3_TMS_PIN))
    m_prg_api_p->PinSet(JP0_3_TMS_PIN, LOGIC_0); //don't care in programming mode

  if (m_prg_api_p->PinExists(FLMD1_PIN))
    m_prg_api_p->PinSet(FLMD1_PIN, LOGIC_0);

  //Voltage regulators for the core power supply are supplied with higher voltage.
  //The core supply must be supplied first then the rest
  //Thus the power sequence depends on the voltage level settings:
  //m_prg_api_p->SetAdapterPower(HwTypes::ON);          // Power ON +5V_SW - for testing
  if (m_prg_api_p->PinExists(VDD_AUX_EN_PIN)) //if voltage regulator has ENABLE pin set it HIGH
    m_prg_api_p->PinSet(VDD_AUX_EN_PIN, LOGIC_1);
  if (m_prg_api_p->PinExists(VREG_VS0_PIN))
    m_prg_api_p->PinSet(VREG_VS0_PIN, LOGIC_1);
  if (m_prg_api_p->PinExists(VREG_VS1_PIN))
    m_prg_api_p->PinSet(VREG_VS1_PIN, LOGIC_1);
  if (m_prg_api_p->PinExists(VREG_VS2_PIN))
    m_prg_api_p->PinSet(VREG_VS2_PIN, LOGIC_1);

  if (m_devparms_p->voltage_vpp > m_devparms_p->voltage_vcc)
  { //G-Node G4 is used for voltage regulator supply (core supply)
    if (m_prg_api_p->PinExists(VDD_AUX_PIN))
    {
      m_prg_api_p->VppSet(m_devparms_p->voltage_vpp, false);
      //MicroSecDelay(200);
      m_prg_api_p->PinSet(VDD_AUX_PIN, SV);
    }
    m_prg_api_p->VccSet(m_devparms_p->voltage_vcc, false);
    m_prg_api_p->VihSet(m_devparms_p->voltage_vih, false);
  }
  else
  { //VCC is used for voltage regulator supply (core supply)
    m_prg_api_p->VccSet(m_devparms_p->voltage_vcc, false);
    m_prg_api_p->VihSet(m_devparms_p->voltage_vih, false);
    if (m_prg_api_p->PinExists(VDD_AUX_PIN))
    {
      m_prg_api_p->VppSet(m_devparms_p->voltage_vpp, true);
      MicroSecDelay(200);
      m_prg_api_p->PinSet(VDD_AUX_PIN, SV);
    }
  }
  MicroSecDelay(100);
  if (LOBYTE(m_devparms_p->reserved4))
    DELAY_MS(LOBYTE(m_devparms_p->reserved4) * 100); //wait for regulator stabilisation

  if (m_prg_api_p->PinExists(POWERGOOD_PIN))
    m_prg_api_p->PinSet(POWERGOOD_PIN, LOGIC_1);
  DELAY_MS(50);

  ResetToProgrammingMode();

  if (m_prg_api_p->SysEvtChk())
    PRINTF(" OC @ line %d\n", __LINE__);
  return;
}

/*******************************************************************************
*         C_RV40F::DoPowerDown ()
* Inputs:   
*       
* Return:   
*       
* METHOD:     
*******************************************************************************/
void C_RV40F::DoPowerDown(void)
{
  PRINTF("C_RV40F::DoPowerDown()\n"); // debug statements

  DWORD reset_L_vddoff_wait = 10000;

  m_prg_api_p->PinSet(_RESET_PIN, LOGIC_0);
  WaitLoop(reset_L_vddoff_wait);

  if (m_devparms_p->voltage_vpp > m_devparms_p->voltage_vcc)
  { //G-Node G4 is used for voltage regulator supply (core supply)
    m_prg_api_p->VihSet(0, false);
    m_prg_api_p->VccSet(0, false);
    if (m_prg_api_p->PinExists(VDD_AUX_PIN))
    {
      m_prg_api_p->PinSet(VDD_AUX_PIN, LOGIC_0); //power down core
      m_prg_api_p->VppSet(0, false);
    }
  }
  else
  { //VCC is used for voltage regulator supply (core supply)
    m_prg_api_p->VihSet(0, false);
    if (m_prg_api_p->PinExists(VDD_AUX_PIN))
    {
      m_prg_api_p->PinSet(VDD_AUX_PIN, LOGIC_0);
      m_prg_api_p->VppSet(0, false);
      MicroSecDelay(200);
    }
    m_prg_api_p->VccSet(0, false);
  }

  if (m_prg_api_p->PinExists(VDD_AUX_EN_PIN))
    m_prg_api_p->PinSet(VDD_AUX_EN_PIN, LOGIC_0);
  //m_prg_api_p->SetAdapterPower(HwTypes::OFF);          // Power OFF +5V_SW for testing
  DELAY_MS(100);
  if (HIBYTE(m_devparms_p->reserved4))
    DELAY_MS(HIBYTE(m_devparms_p->reserved4) * 100); //wait for regulator cool down

  return;
}

/*******************************************************************************
*         C_RV40F::ResetToProgrammingMode ()
* Inputs:   
*       
* Return:   
*       
* METHOD:     
*******************************************************************************/
void C_RV40F::ResetToProgrammingMode(void)
{
  BYTE modePulseCnt;
  SOCKET_STATUS_T socket_stat;
  DWORD reset_H_flmd0_pulse_start_wait, sync_detect_wait;
  DWORD vdd_flmd_H_wait = 100000;    //PMT_0F[1]
  DWORD flmd_H_reset_H_wait = 10000; //PMT_0F[2]

  m_fTarget_initialized = false;
  m_fStartupMode = true;
  m_sys_clk = m_op_frequency / 10;                 //[MHz]
  m_fpga_p->SetSerialParams(JP0_0_SI_PIN,          // Serial In -> writing to the device
                            JP0_1_SO_PIN,          // Serial Out-> reading from the deivce
                            JP0_2_SCK_PIN,         // Shift Clock pin
                            STARTUP_SCI_FREQUENCY, // Shift clock in Hz
                            m_fpga_p->EDGE_FALLING,
                            m_fpga_p->MSB_FIRST);

  //Intention of the loop:
  //  Devices with the same marking can have different fw versions with different timings,
  //  especially the crucially important reset_H_flmd0_pulse_start_wait.
  //  The loop will try the different delays in the list in th ehope to find the right setting
  int i = 0;
  if (m_reset_H_flmd0_pulse_start_wait)
    reset_H_flmd0_pulse_start_wait = m_reset_H_flmd0_pulse_start_wait; //already determined
  else
    reset_H_flmd0_pulse_start_wait = m_tRV40F_Param_p->RST_H_FLMD0_DLY_LIST[0];
  do
  {
    sync_detect_wait = 2800000; //PMT_0F[6]
    //Reset the device
    m_prg_api_p->PinSet(_RESET_PIN, LOGIC_0);
    WaitLoop(vdd_flmd_H_wait);

    //set programming mode
    m_prg_api_p->PinSet(FLMD0_PIN, LOGIC_1);
    if (m_prg_api_p->PinExists(FLMD1_PIN))
      m_prg_api_p->PinSet(FLMD1_PIN, LOGIC_0);

    m_prg_api_p->PinSet(JP0_0_SI_PIN, LOGIC_1);
    m_prg_api_p->PinSet(JP0_2_SCK_PIN, LOGIC_1);
    WaitLoop(flmd_H_reset_H_wait);

    //DELAY_MS(3000); //for testing only

    m_fpga_p->FastPinSet(_RESET_PIN, LOGIC_1);
    WaitLoop(reset_H_flmd0_pulse_start_wait);

    // ATTN - timing critical loop - device will not get into mode if pulse length is out of spec
    LOCK_IRQ(); // assert global IRQ lock
    for (modePulseCnt = 0; modePulseCnt < COMMODE_CSI_SEL_CNT; modePulseCnt++)
    {
      m_fpga_p->FastPinSet(FLMD0_PIN, LOGIC_0); // pulse - FastPinSet function has 0us to 20us overhead
      WaitLoop(20);
      // wait minimum acceptable time for pulse width
      m_fpga_p->FastPinSet(FLMD0_PIN, LOGIC_1);
      WaitLoop(20);
      // Measured - Max tCH,tCL=55us; Min tCH,tCL=36 with compiler optimization
      // Measured - Max tCH,tCL=60us; Min tCH,tCL=40 with debug mode
      // Device spec states Max tCH,tCL=100us; Min tCH,tCL=10us
    }
    UNLOCK_IRQ();

    do
    {
      socket_stat = m_fpga_p->ParDataCompare(LOGIC_0, 0xFFFE); //D0 = SO
    } while (sync_detect_wait-- && socket_stat);

    if ((socket_stat & m_akt_skt_msk) != m_akt_skt_msk) //at least one device responded
      m_reset_H_flmd0_pulse_start_wait = reset_H_flmd0_pulse_start_wait;
    else
    {
      i++;
      reset_H_flmd0_pulse_start_wait = m_tRV40F_Param_p->RST_H_FLMD0_DLY_LIST[i];
    }
    if ((i == MODE_ENTRY_DELAY_MAXCNT) || (reset_H_flmd0_pulse_start_wait == 0))
      break; //game over
  } while ((socket_stat & m_akt_skt_msk) == m_akt_skt_msk);

  return;
}

/*******************************************************************************
*         C_RV40F::PowerUp ()
* Inputs:   
*       
* Return:   
*       
* METHOD:     
*******************************************************************************/
void C_RV40F::PowerUp(void)
{
#if (ALG_DEBUG > 0)
  PRINTF("C_RV40F::PowerUp()\n"); // debug statements
#endif

  m_prg_api_p->SetOverCurrentCurrentLevel(HwTypes::VPP_OC, 300); //[mA]
  m_prg_api_p->SetOverCurrentCurrentLevel(HwTypes::VCC_OC, 350); //[mA]
  DoPowerUp();

  return;
}

/*******************************************************************************
*         C_RV40F::PowerDown ()
* Inputs:   
*       
* Return:   
*       
* METHOD:     
*******************************************************************************/
void C_RV40F::PowerDown(void)
{

  DoPowerDown();

#if (ALG_DEBUG > 0)
  PRINTF("C_RV40F::PowerDown()\n"); // debug statements
#endif

  return;
} //C_RV40F::PowerDown()

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ //
void C_RV40F::WriteCmdBuffer(const BYTE offset, const BYTE value)
{
  ALG_ASSERT(offset < CMD_BUFFER_SIZE);

  if (offset == 0)
    m_current_CMD = value;

  m_cmd_buffer[offset] = value;
  return;
};

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ //
void C_RV40F::WriteCmdBuffer(const BYTE cmd, const DWORD param1, const DWORD param2)
{
  m_current_CMD = cmd;

  m_cmd_buffer[0] = cmd;
  m_cmd_buffer[1] = (BYTE)(param1 >> 24);
  m_cmd_buffer[2] = (BYTE)(param1 >> 16);
  m_cmd_buffer[3] = (BYTE)(param1 >> 8);
  m_cmd_buffer[4] = (BYTE)(param1);
  m_cmd_buffer[5] = (BYTE)(param2 >> 24);
  m_cmd_buffer[6] = (BYTE)(param2 >> 16);
  m_cmd_buffer[7] = (BYTE)(param2 >> 8);
  m_cmd_buffer[8] = (BYTE)(param2);

  return;
};

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ //
void C_RV40F::WaitLoop(const DWORD delay_us)
{
  if (delay_us)
  {
    if (delay_us < DELAY_THRESHOLD)
      MicroSecDelay((WORD)delay_us);
    else
    {
      DELAY_MS((WORD)(delay_us / DELAY_THRESHOLD));
      if (delay_us % DELAY_THRESHOLD)
        MicroSecDelay((WORD)(delay_us % DELAY_THRESHOLD));
    }
  }

  return;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ //
bool C_RV40F::GetSockets_ICU_S_Status()
{
  int i, sockets_ICU_S_status ;
  bool ret_val = true;

  //check for the first socket with known status and use as reference
  for (i = 0; i < MAX_SOCKET_NUM; i++)
  {
    if (m_ICU_S_Status[i] >= 0)
    {
      sockets_ICU_S_status  = m_ICU_S_Status[i]; //Taking 1st encountered 1 or 0 as reference
      break;
    }
  }

  //comapre other skt values with reference
  for (i = 0; i < MAX_SOCKET_NUM; i++)
  {
    if (m_ICU_S_Status[i] >= 0)
    {
      if (sockets_ICU_S_status  != m_ICU_S_Status[i])
        ret_val = false; //socket have mixed devices - use single socket mode where applicable
    }
  }

  if (ret_val)
    m_All_skts_ICU_S_INVALID = sockets_ICU_S_status ;

  return ret_val;
}

// SerialWrite(const BYTE* data_p)
// Used for the initial setup, when low speed is available.
// The StdWriggler-SerialWrite doesn't work for some devices (the data-hold timing is too short at rising-clk)
// This function implements the SCI-write sequence properly. Data is latched at rising edge of clk.
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ //
int C_RV40F::SerialWrite(const BYTE *data_p)
{
  int i, j;

  for (i = 0; i < 8; i++)
  {
    if ((data_p[0] >> (7 - i)) & 1)
      m_fpga_p->FastPinSet(JP0_0_SI_PIN, LOGIC_1);
    else
      m_fpga_p->FastPinSet(JP0_0_SI_PIN, LOGIC_0);

    //fSCK ~ 10kHz

    m_fpga_p->FastPinSet(JP0_2_SCK_PIN, LOGIC_0);
    MicroSecDelay(52);
    m_fpga_p->FastPinSet(JP0_2_SCK_PIN, LOGIC_1);
    MicroSecDelay(52);
  }

  return 0;
}

// Implements the frame format for RV40F devices
// Load data serially via SI, HS is done via SO
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ //
int C_RV40F::SendFrame(const FRAMESTART_T startType, const FRAMEEND_T endType, const BYTE *buffer_p, const WORD length, BYTE includeACK)
{
  DWORD bytecnt;
  WORD local_length;
  BYTE data8, checksum = 0;
  bool ret_value = true;
  DWORD ulMaxRetries = DEFAULT_TIMEOUT;
  SOCKET_STATUS_T socket_stat;

  if (includeACK)
    local_length = length + 1;
  else
    local_length = length;

  //synchronize with uC
  //socket_stat = WaitUntilDeviceReady(m_current_op_mode, LOGIC_0, DEFAULT_TIMEOUT);
  if (m_current_op_mode == DeviceOperation::READ)
    m_prg_api_p->SetSocketReadMode(HwTypes::GANG_RD_MODE); //to allow compare function

  do
  {
    socket_stat = m_fpga_p->ParDataCompare(LOGIC_0, 0xFFFE); //D0 = SO
  } while (ulMaxRetries-- && socket_stat);

  if (m_current_op_mode == DeviceOperation::READ)
    m_prg_api_p->SetSocketReadMode(HwTypes::SINGLE_SKT_RD_MODE);

  if (CompareFailed(socket_stat))
  {
    if (!m_prg_api_p->MisCompare(m_current_op_mode, socket_stat, 0, 0))
    {
      m_prg_api_p->Write2EventLog("C_RV40F::SendFrame timeout error!");
      return false;
    }
  }

  if (length == MAX_PAGE_SIZE && m_fpga_speedup_supported)
  {
    JTAGBoost_p->SetTDI((DWORD *)buffer_p, MAX_PAGE_SIZE / 4, 1);
    JTAGBoost_p->StartOp(0);
    checksum = 0x18; //(0x04 + 0x01 (LN)) + 0x13 (Program_CMD)
    for (bytecnt = 0; bytecnt < MAX_PAGE_SIZE; bytecnt++)
      checksum += buffer_p[bytecnt];
  }
  else
  {
    //-----------------------------------------------------//
    if (m_fStartupMode)
      SerialWrite((BYTE *)&startType);
    else
      m_fpga_p->SerialWrite((BYTE *)&startType, 8);
    //-----------------------------------------------------//
    data8 = HIBYTE(local_length);
    checksum += data8;
    if (m_fStartupMode)
      SerialWrite(&data8);
    else
      m_fpga_p->SerialWrite(&data8, 8);
    //-----------------------------------------------------//
    data8 = LOBYTE(local_length);
    checksum += data8;
    if (m_fStartupMode)
      SerialWrite(&data8);
    else
      m_fpga_p->SerialWrite(&data8, 8);
    //-----------------------------------------------------//
    if (includeACK)
    {
      checksum += includeACK;
      if (m_fStartupMode)
        SerialWrite(&includeACK);
      else
        m_fpga_p->SerialWrite(&includeACK, 8);
    }
    //-----------------------------------------------------//
    for (bytecnt = 0; bytecnt < length; bytecnt++)
    {
      checksum += buffer_p[bytecnt];
      if (m_fStartupMode)
        SerialWrite(&buffer_p[bytecnt]);
      else
        m_fpga_p->SerialWrite(&buffer_p[bytecnt], 8);
    }
  }
  //-----------------------------------------------------//
  checksum = 0x00 - checksum;
  if (m_fStartupMode)
    SerialWrite(&checksum);
  else
    m_fpga_p->SerialWrite(&checksum, 8);
  //-----------------------------------------------------//
  if (m_fStartupMode)
    SerialWrite((BYTE *)&endType);
  else
    m_fpga_p->SerialWrite((BYTE *)&endType, 8);

  return ret_value;
}

///////////////////////////////////////////////////////////////////////
//WaitUntilDeviceReady(DEV_OP_E opMode, const WORD pinLvl, DWORD timeout)
//
//Waits for the READY signal on SO (the level depends on the send or receive state)
///////////////////////////////////////////////////////////////////////
BYTE C_RV40F::WaitUntilDeviceReady(DEV_OP_E opMode, const WORD pinLvl, DWORD timeout)
{
  BYTE socket_stat;

  if (opMode == DeviceOperation::READ)
    m_prg_api_p->SetSocketReadMode(HwTypes::GANG_RD_MODE); //to allow compare function
  do
  {
    socket_stat = m_fpga_p->ParDataCompare(pinLvl, 0xFFFE); //D0 = SO
  } while (timeout-- && socket_stat);
  if (opMode == DeviceOperation::READ)
    m_prg_api_p->SetSocketReadMode(HwTypes::SINGLE_SKT_RD_MODE);

  return socket_stat;
}

// Implements the frame format for RV40F devices
// Verify data received serially via SO against data in buffer_p
// The return value is false, if an error occured and all sockets are wrong.
// It does not matter which part of the frame failed, the appropriated socket is switched off.
// In our "closed" gang environment this makes sense. In ICP/ISP (or single socket programming),
// the different error codes could be cheked. Then the further process could be decided based on the error code (checksum error, or BUSY code)
// Parameter: check_length: count of bytes to be checked in the data frame. If it is lower than length, the rest of data bytes would be ignored.
//            inlcudeACK:   before receiving data, check ACK code
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ //
int C_RV40F::GetDataFrame(const FRAMEEND_T endType, BYTE *buffer_p, const WORD length, const WORD check_length, BYTE includeACK, bool fLongWait)
{
  int ret_value = true;
  WORD local_length;
  SOCKET_STATUS_T socket_stat;
  BYTE checksum = 0;
  BYTE expectedData, readData;
  DWORD bytecnt;
  DWORD ulMaxRetries;

  if (fLongWait)
    ulMaxRetries = ERASE_TIMEOUT;
  else
    ulMaxRetries = DEFAULT_TIMEOUT;

  if (includeACK)
    local_length = length + 1;
  else
    local_length = length;

  //synchronize with uC
  //socket_stat = WaitUntilDeviceReady(m_current_op_mode, LOGIC_1, ulMaxRetries);
  if (m_current_op_mode == DeviceOperation::READ)
    m_prg_api_p->SetSocketReadMode(HwTypes::GANG_RD_MODE); //to allow compare function

  do
  {
    socket_stat = m_fpga_p->ParDataCompare(LOGIC_1, 0xFFFE); //D0 = SO
  } while (ulMaxRetries-- && socket_stat);

  if (m_current_op_mode == DeviceOperation::READ)
    m_prg_api_p->SetSocketReadMode(HwTypes::SINGLE_SKT_RD_MODE);

  if (CompareFailed(socket_stat))
  {
    if (!m_prg_api_p->MisCompare(m_current_op_mode, socket_stat, 0, LOGIC_1))
    {
      m_prg_api_p->Write2EventLog("C_RV40F::GetFrame timeout error!");
      return false;
    }
  }

  // Frame Header
  expectedData = SOD;
  if (m_current_op_mode == DeviceOperation::READ)
  {
    if (m_fpga_p->SerialRead(&readData, 8))
      return false;
    else if (readData != expectedData)
    {
      PRINTF("C_RV40F::GetDataFrame() - Check status for cmd %Xh failed. SOD not recognized. Expected: %Xh, Actual: %Xh\n", (WORD)m_current_CMD, expectedData, readData);
      return false; //frame must fit
    }
  }
  else //VERIFYMODE
    socket_stat = m_fpga_p->SerialCompare(&expectedData, &COMPARE_ALL_MASK, 8);

  if (CompareFailed(socket_stat))
  {
    PRINTF("C_RV40F::GetDataFrame() - Check status for command %Xh failed. SOD not recognized.\n", (WORD)m_current_CMD);
    if (!m_prg_api_p->MisCompare(m_current_op_mode, socket_stat, 0, expectedData))
      return false;
  }

  expectedData = HIBYTE(local_length);
  checksum += expectedData;
  if (m_current_op_mode == DeviceOperation::READ)
  {
    if (m_fpga_p->SerialRead(&readData, 8))
      return false;
    else if (readData != expectedData)
    {
      PRINTF("C_RV40F::GetDataFrame() - Check status for cmd %Xh failed. Unexpected frame length (MSB). Expected: %Xh, Actual: %Xh\n", (WORD)m_current_CMD, expectedData, readData);
      return false; //frame must fit
    }
  }
  else //VERIFYMODE
    socket_stat = m_fpga_p->SerialCompare(&expectedData, &COMPARE_ALL_MASK, 8);
  if (CompareFailed(socket_stat))
  {
    PRINTF("C_RV40F::GetDataFrame() - Check status for command %Xh failed. Unexpected frame length.\n", (WORD)m_current_CMD);
    if (!m_prg_api_p->MisCompare(m_current_op_mode, socket_stat, 0, expectedData))
      return false;
  }

  expectedData = LOBYTE(local_length);
  checksum += expectedData;
  if (m_current_op_mode == DeviceOperation::READ)
  {
    if (m_fpga_p->SerialRead(&readData, 8))
      return false;
    else if (readData != expectedData)
    {
      PRINTF("C_RV40F::GetDataFrame() - Check status for cmd %Xh failed. Unexpected frame length (LSB). Expected: %Xh, Actual: %Xh\n", (WORD)m_current_CMD, expectedData, readData);
      return false; //frame must fit
    }
  }
  else //VERIFYMODE
    socket_stat = m_fpga_p->SerialCompare(&expectedData, &COMPARE_ALL_MASK, 8);
  if (CompareFailed(socket_stat))
  {
    PRINTF("C_RV40F::GetDataFrame() - Check status for command %Xh failed. Unexpected frame length.\n", (WORD)m_current_CMD);
    if (!m_prg_api_p->MisCompare(m_current_op_mode, socket_stat, 0, expectedData))
      return false;
  }

  if (includeACK)
  {
    checksum += includeACK;
    if (m_current_op_mode == DeviceOperation::READ)
    {
      if (m_fpga_p->SerialRead(&readData, 8))
        return false;
      else if (readData != includeACK)
      {
        PRINTF("C_RV40F::GetDataFrame() - Check status for cmd %Xh failed. Expected ACK code: %Xh, Actual: %Xh\n", (WORD)m_current_CMD, includeACK, readData);
        return false; //frame must fit
      }
    }
    else //VERIFYMODE
      socket_stat = m_fpga_p->SerialCompare(&includeACK, &COMPARE_ALL_MASK, 8);
    if (CompareFailed(socket_stat))
    {
      PRINTF("C_RV40F::GetDataFrame() - Check status for command %Xh failed. Unexpected ACK code.\n", (WORD)m_current_CMD);
      if (!m_prg_api_p->MisCompare(m_current_op_mode, socket_stat, 0, includeACK))
        return false;
    }
  } //-- OF if (includeACK)

  // Parameter
  for (bytecnt = 0; bytecnt < length; bytecnt++)
  {
    if (m_current_op_mode == DeviceOperation::READ)
    {
      if (m_fpga_p->SerialRead(&buffer_p[bytecnt], 8))
        ret_value = false;
      checksum += buffer_p[bytecnt];
    }
    else //VERIFYMODE
    {
      checksum += buffer_p[bytecnt];
      expectedData = buffer_p[bytecnt];
      socket_stat = m_fpga_p->SerialCompare(&expectedData, &COMPARE_ALL_MASK, 8);
    }

    if (CompareFailed(socket_stat))
    {
      if (check_length == length || bytecnt < check_length)
      {
        PRINTF("C_RV40F::GetDataFrame() - Check status for command %Xh failed. Unexpected data.\n", (WORD)m_current_CMD);
        if (!m_prg_api_p->MisCompare(m_current_op_mode, socket_stat, 0, expectedData))
          return false;
      }
    }
  }

  // Check sum
  expectedData = 0x00 - checksum;
  if (m_current_op_mode == DeviceOperation::READ)
  {
    if (m_fpga_p->SerialRead(&readData, 8))
      ret_value = false;
    else if (readData != expectedData)
    {
      PRINTF("C_RV40F::GetDataFrame() - Check status for cmd %Xh failed. Unexpected checksum. Expected: %Xh, Actual: %Xh\n", (WORD)m_current_CMD, expectedData, readData);
      return false; //frame must fit
    }
  }
  else //VERIFYMODE
    socket_stat = m_fpga_p->SerialCompare(&expectedData, &COMPARE_ALL_MASK, 8);
  if (CompareFailed(socket_stat))
  {
    if (check_length == length)
    {
      PRINTF("C_RV40F::GetDataFrame() - Check status for command %Xh failed. Unexpected checksum.\n", (WORD)m_current_CMD);
      if (!m_prg_api_p->MisCompare(m_current_op_mode, socket_stat, 0, expectedData))
        return false;
    }
  }

  // End of frame
  expectedData = endType;
  if (m_current_op_mode == DeviceOperation::READ)
  {
    if (m_fpga_p->SerialRead(&readData, 8))
      return false;
    else if (readData != expectedData)
    {
      PRINTF("C_RV40F::GetDataFrame() - Check status for command %Xh failed. Frame end mark not recognized.\n", (WORD)m_current_CMD);
      return false; //frame must fit
    }
  }
  else //VERIFYMODE
    socket_stat = m_fpga_p->SerialCompare(&expectedData, &COMPARE_ALL_MASK, 8);
  if (CompareFailed(socket_stat))
  {
    PRINTF("C_RV40F::GetDataFrame() - Check status for command %Xh failed. Frame end mark not recognized.\n", (WORD)m_current_CMD);
    if (!m_prg_api_p->MisCompare(m_current_op_mode, socket_stat, 0, expectedData))
      return false;
  }

  return ret_value;
}

// Implements the frame format for RV40F devices
// Works in READ mode only (NO GANG mode). Saves the data in buffer_p - make sure the buffer is big enough!
// The return value is false, if a communication error occured.
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ //
int C_RV40F::ReadDataFrame(BYTE *buffer_p)
{
  WORD length;
  DWORD bytecnt;
  BYTE checksum;
  BYTE local_buffer[4];
  SOCKET_STATUS_T socket_stat;
  DWORD ulMaxRetries = DEFAULT_TIMEOUT;
  int ret_value = true;

  //synchronize with uC
  socket_stat = WaitUntilDeviceReady(m_current_op_mode, LOGIC_1, DEFAULT_TIMEOUT);

  if (CompareFailed(socket_stat))
  {
    if (!m_prg_api_p->MisCompare(m_current_op_mode, socket_stat, 0, LOGIC_1))
    {
      m_prg_api_p->Write2EventLog("C_RV40F::ReadFrame timeout error!");
      return false;
    }
  }

  // Read first 3 bytes: SOD + LN(2)
  if (m_fpga_p->SerialRead(&local_buffer[0], 24))
    return false;
  length = ((WORD)local_buffer[1] << 8) | (WORD)local_buffer[2];
  if (length > (MAX_PAGE_SIZE + 1)) //RES + max. LEN
  {
    m_prg_api_p->Write2EventLog("C_RV40F::ReadFrame size exceeded buffer limit.");
    return false;
  }
  checksum = local_buffer[1] + local_buffer[2];

  // Parameter
  for (bytecnt = 0; bytecnt < length; bytecnt++)
  {
    if (m_fpga_p->SerialRead(&buffer_p[bytecnt], 8))
      return false;
    checksum += buffer_p[bytecnt];
  }

  // Check sum
  if (m_fpga_p->SerialRead(&local_buffer[0], 8))
    return false;
  checksum += local_buffer[0];
  if (checksum)
  {
    sprintf(msgbuff, "C_RV40F::ReadFrame checksum ERROR while executing cmd 0x%02X.", buffer_p[0]);
    m_prg_api_p->Write2EventLog(msgbuff);
    return false;
  }
  // End of frame
  if (m_fpga_p->SerialRead(&local_buffer[1], 8))
    return false;

  return ret_value;
}

//Checks, if the data flash area contains data for programming.
//Therefore it searches for 0x00 marked bytes in the appropriate DF_MARKER area.
//Returns false, if area contains data.
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ //
bool C_RV40F::DF_IsAreaEmpty(DWORD startAddress, DWORD areaSize)
{
  DWORD addrCnt;
  DWORD memAddress = startAddress + DF_MARKER_OFFSET;
  bool ret_val = true;

  for (addrCnt = 0; addrCnt < areaSize; addrCnt++)
  {
    if (GetDataFromRam_8Bit(memAddress + addrCnt, (DWORD)m_srcdata_bp) == 0)
    {
      ret_val = false;
      break;
    }
  }

  return ret_val;
}

//Checks, if the code flash area contains data for programming.
//Returns false, if area contains data.
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ //
bool C_RV40F::CF_IsAreaEmpty(DWORD startAddress, DWORD areaSize)
{
  DWORD blockCnt;
  DWORD blockSize = areaSize / MIN_PAGE_SIZE;
  DWORD memAddress = startAddress / MIN_PAGE_SIZE + CF_MARKER_OFFSET;
  bool ret_val = true;

  for (blockCnt = 0; blockCnt < blockSize; blockCnt++)
  {
    if (GetDataFromRam_8Bit(memAddress + blockCnt, (DWORD)m_srcdata_bp) == 0)
    {
      ret_val = false;
      break;
    }
  }

  return ret_val;
}

//Checks, if the code flash area is filled up with data for programming.
//Returns false, if area filled up with data.
//Returns true, if area contains gaps.
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ //
bool C_RV40F::CF_IsAreaFragmented(DWORD startAddress, DWORD areaSize)
{
  DWORD blockCnt;
  DWORD blockSize = areaSize / MIN_PAGE_SIZE;
  DWORD memAddress = startAddress / MIN_PAGE_SIZE + CF_MARKER_OFFSET;
  bool ret_val = false;

  for (blockCnt = 0; blockCnt < blockSize; blockCnt++)
  {
    if (GetDataFromRam_8Bit(memAddress + blockCnt, (DWORD)m_srcdata_bp))
    {
      ret_val = true;
      break;
    }
  }

  return ret_val;
}

//check whether ICU_S area is empty - not allowed!
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ //
/*bool C_RV40F::check_ICU_S_area(void)
{
  bool area_included = true;
  WORD ICU_S_RegionSize = get_ICU_S_RegionSize();

  if (ICU_S_RegionSize && DF_IsAreaEmpty(m_devsectors_p[m_DF_block_nr].end_address + 1 - ICU_S_RegionSize, ICU_S_RegionSize))
  {
    m_prg_api_p->Write2EventLog("ICU-S requested but ICU-S area is not included in the data file -> Operation aborted.");
    m_prg_api_p->Write2EventLog("Include ICU-S area when you intend to use ICU-S feature.");
    area_included = false;
  }

  return area_included;
}*/

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ //
C_RV40F::DEV_STAT_E C_RV40F::VerifyArea(DWORD startInMem, DWORD startInDev, DWORD areaSize)
{
  volatile BYTE *srcbase = (volatile BYTE *)m_srcdata_bp;

  WriteCmdBuffer(READ_CMD, startInDev, startInDev + areaSize - 1);
  if (false == SendFrame(SOH, ETX, GetCmdBufferP(), 9))
    return VERIFY_ERR;
  if (false == GetDataFrame(ETX, GetCmdBufferP(), CHECK_ST1, CHECK_ST1))
    return VERIFY_ERR;
  if (false == SendFrame(SOD, ETX, GetCmdBufferP(), 1)) //reverse ACK
    return VERIFY_ERR;

  if (false == GetDataFrame(ETX, (BYTE *)(srcbase + startInMem), (WORD)areaSize, (WORD)areaSize, READ_CMD))
  {
    PRINTF("C_RV40F::Verify fail at address 0x%X\n", startInMem);
    return VERIFY_ERR;
  }

  return OPERATION_OK;
}

// Verify DATA FLASH data received serially via SO against data in buffer_p when the data is marked in buffer_p + DF_MARKER_OFFSET
// The return value is false, if an error occured and all sockets are wrong.
// It does not matter which part of the frame failed, the appropriated socket is switched off.
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ //
int C_RV40F::VerifyDFblock(const FRAMEEND_T endType, BYTE *buffer_p, WORD pageSZ)
{
  int ret_value = true;
  SOCKET_STATUS_T socket_stat;
  BYTE expectedData;
  DWORD bytecnt;
  DWORD ulMaxRetries = DEFAULT_TIMEOUT;

  //synchronize with uC
  do
  {
    socket_stat = m_fpga_p->ParDataCompare(LOGIC_1, 0xFFFE); //D0 = SO
  } while (ulMaxRetries-- && socket_stat);

  if (CompareFailed(socket_stat))
  {
    if (!m_prg_api_p->MisCompare(m_current_op_mode, socket_stat, 0, LOGIC_1))
    {
      m_prg_api_p->Write2EventLog("C_RV40F::GetFrame timeout error!");
      return false;
    }
  }

  // Frame Header
  expectedData = SOD;
  socket_stat = m_fpga_p->SerialCompare(&expectedData, &COMPARE_ALL_MASK, 8);
  if (CompareFailed(socket_stat))
  {
    PRINTF("C_RV40F::GetDataFrame() - Check status for command %Xh failed. SOD not recognized.\n", (WORD)m_current_CMD);
    if (!m_prg_api_p->MisCompare(m_current_op_mode, socket_stat, 0, expectedData))
      return false;
  }

  expectedData = HIBYTE(pageSZ + 1);
  socket_stat = m_fpga_p->SerialCompare(&expectedData, &COMPARE_ALL_MASK, 8);
  if (CompareFailed(socket_stat))
  {
    PRINTF("C_RV40F::GetDataFrame() - Check status for command %Xh failed. Unexpected frame length.\n", (WORD)m_current_CMD);
    if (!m_prg_api_p->MisCompare(m_current_op_mode, socket_stat, 0, expectedData))
      return false;
  }

  expectedData = LOBYTE(pageSZ + 1);
  socket_stat = m_fpga_p->SerialCompare(&expectedData, &COMPARE_ALL_MASK, 8);
  if (CompareFailed(socket_stat))
  {
    PRINTF("C_RV40F::GetDataFrame() - Check status for command %Xh failed. Unexpected frame length.\n", (WORD)m_current_CMD);
    if (!m_prg_api_p->MisCompare(m_current_op_mode, socket_stat, 0, expectedData))
      return false;
  }

  socket_stat = m_fpga_p->SerialCompare(&READ_CMD, &COMPARE_ALL_MASK, 8);
  if (CompareFailed(socket_stat))
  {
    PRINTF("C_RV40F::GetDataFrame() - Check status for command %Xh failed. Unexpected ACK code.\n", (WORD)m_current_CMD);
    if (!m_prg_api_p->MisCompare(m_current_op_mode, socket_stat, 0, READ_CMD))
      return false;
  }

  // DATA
  for (bytecnt = 0; bytecnt < pageSZ; bytecnt++)
  {
    if (false == m_fDF_filled0xFF)
      socket_stat = m_fpga_p->SerialCompare(&buffer_p[bytecnt], &buffer_p[bytecnt + DF_MARKER_OFFSET], 8); //check marked data only
    else
      socket_stat = m_fpga_p->SerialCompare(&buffer_p[bytecnt], &COMPARE_ALL_MASK, 8);

    if (CompareFailed(socket_stat))
    {
      PRINTF("C_RV40F::GetDataFrame() - Check status for command %Xh failed. Unexpected data in DF.\n", (WORD)m_current_CMD);
      if (!m_prg_api_p->MisCompare(m_current_op_mode, socket_stat, 0xDDDD, expectedData))
        return false;
    }
  }

  // Check sum
  expectedData = 0x00; //ignore it
  m_fpga_p->SerialCompare(&expectedData, &COMPARE_NOTHING, 8);

  // End of frame
  expectedData = endType;
  socket_stat = m_fpga_p->SerialCompare(&expectedData, &COMPARE_ALL_MASK, 8);
  if (CompareFailed(socket_stat))
  {
    PRINTF("C_RV40F::GetDataFrame() - Check status for command %Xh failed. Frame end mark not recognized.\n", (WORD)m_current_CMD);
    if (!m_prg_api_p->MisCompare(m_current_op_mode, socket_stat, 0, expectedData))
      return false;
  }

  return ret_value;
}

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ //
// implements start-up sequence of the algorithm: RESET command + alg. param. settings
bool C_RV40F::DeviceInit(void)
{

  BYTE sendAck = 0x55;
  BYTE expectedData;
  BYTE data_buffer[MAX_PAGE_SIZE + 6]; //for local reads
  SOCKET_STATUS_E skt_stat;
  BYTE bytecnt, skt_mask, failed_skt_mask;
  int nDUT;
  DEV_OP_E saved_current_op_mode = m_current_op_mode;
  BYTE expectedPROTmode = UNKNOWN;
  BYTE expectedICUSmode = UNKNOWN;
  bool device_init_ok = true;
  SOCKET_STATUS_T socket_stat = 0;

  if (m_fTarget_initialized)
    return true; //device already up and running (programming mode activated, frequency set and eventually new bootloader loaded)

  //generic Boot Device Check Processing
  expectedData = 0xC1; //generic boot device check code
  SerialWrite((BYTE *)&sendAck);
  MicroSecDelay(500);
  if (m_current_op_mode == DeviceOperation::READ)
  {
    if (m_fpga_p->SerialRead(&data_buffer[0], 8))
      return false;
    else if (data_buffer[0] != expectedData)
    {
      PRINTF("C_RV40F::DeviceInit() - Generic boot device check failed. Expected: %Xh, Actual: %Xh\n", expectedData, data_buffer[0]);
      return false;
    }
  }
  else //VERIFYMODE
    socket_stat = m_fpga_p->SerialCompare(&expectedData, &COMPARE_ALL_MASK, 8);

  if (CompareFailed(socket_stat))
  {
    PRINTF("C_RV40F::DeviceInit() - Generic boot device check failed.\n");
    if (!m_prg_api_p->MisCompare(m_current_op_mode, socket_stat, 0, expectedData))
      return false;
  }

  // Device Type Get processing
  m_fCfgClearCmdReq = false;
  m_fReadProtected = false;
  m_optionSupportedByDev &= ~(DWORD)ID_AUTH; //set it to initial value. it depends on the device settings

  //Collect data socket by socket
  failed_skt_mask = 0;
  WORD socket_mask = LM_Phapi::Get()->ActiveDUTMaskGet();
  m_current_op_mode = DeviceOperation::READ;
  for (nDUT = MAX_SOCKET_NUM - 1; nDUT >= 0; nDUT--)
  {
    // check only active sockets
    skt_stat = m_prg_api_p->SocketStatusGet(SocketNumChange(nDUT + 1));
    if (skt_stat == SOCKET_ENABLE)
    {
      // activate this one socket for reading
      // Note: ALL OTHER SOCKETS WILL BE DISABLED
      skt_mask = 1 << nDUT;
      LM_Phapi::Get()->SetSingleSktMode((WORD)(nDUT + 1));

      m_current_CMD = DEVICE_TYPE_GET_CMD;
      if (false == SendFrame(SOH, ETX, &DEVICE_TYPE_GET_CMD, 1))
        failed_skt_mask |= skt_mask;
      if (false == ReadDataFrame(data_buffer))
        failed_skt_mask |= skt_mask;
      if (data_buffer[0] != DEVICE_TYPE_GET_CMD)
      {
        sprintf(msgbuff, "\tSocket%d: execution of DEVICE_TYPE_GET_CMD failed. Error code: 0x%02X", SocketNumChange(nDUT + 1), data_buffer[0]);
        m_prg_api_p->Write2EventLog(msgbuff);
        failed_skt_mask |= skt_mask;
      }

      if (failed_skt_mask & skt_mask)
        continue;

      if (false == SendFrame(SOD, ETX, &DEVICE_TYPE_GET_CMD, 1)) //reverse ACK
        failed_skt_mask |= skt_mask;
      if (false == ReadDataFrame(data_buffer))
        failed_skt_mask |= skt_mask;
      if (data_buffer[0] != DEVICE_TYPE_GET_CMD)
      {
        sprintf(msgbuff, "\tSocket%d: DEVICE_TYPE_GET_CMD failed. Error code: 0x%02X", SocketNumChange(nDUT + 1), data_buffer[0]);
        m_prg_api_p->Write2EventLog(msgbuff);
        failed_skt_mask |= skt_mask;
      }

      if (failed_skt_mask & skt_mask)
        continue;

      //debug infos:
      DWORD dwFreqValMin, dwFreqValMax;
      PRINTF("Socket%d device type: %02X %02X %02X %02X %02X %02X %02X %02X\n", SocketNumChange(nDUT + 1), data_buffer[1], data_buffer[2], data_buffer[3], data_buffer[4], data_buffer[5], data_buffer[6], data_buffer[7], data_buffer[8]);
      dwFreqValMax = ((DWORD)data_buffer[9] << 24) | ((DWORD)data_buffer[10] << 16) | ((DWORD)data_buffer[11] << 8) | data_buffer[12];
      dwFreqValMin = ((DWORD)data_buffer[13] << 24) | ((DWORD)data_buffer[14] << 16) | ((DWORD)data_buffer[15] << 8) | data_buffer[16];
      PRINTF("Min - Max OSC freq: %d - %d [MHz]\n", dwFreqValMin, dwFreqValMax);
      dwFreqValMax = ((DWORD)data_buffer[17] << 24) | ((DWORD)data_buffer[18] << 16) | ((DWORD)data_buffer[19] << 8) | data_buffer[20];
      dwFreqValMin = ((DWORD)data_buffer[21] << 24) | ((DWORD)data_buffer[22] << 16) | ((DWORD)data_buffer[23] << 8) | data_buffer[24];
      PRINTF("Min - Max CPU freq: %d - %d [MHz]\n", dwFreqValMin, dwFreqValMax);

      //check device type
      for (bytecnt = 0; bytecnt < 8; bytecnt++)
      {
        if (data_buffer[1 + bytecnt] != m_tRV40F_Param_p->TYPE[bytecnt])
        {
          // disable the DUT
          failed_skt_mask |= skt_mask;
          sprintf(msgbuff, "\tSocket%d failed at signature byte %d. Expected: 0x%02X, actual: 0x%02X", SocketNumChange(nDUT + 1), bytecnt, m_tRV40F_Param_p->TYPE[bytecnt], data_buffer[1 + bytecnt]);
          m_prg_api_p->Write2EventLog(msgbuff);
        }
      } //-- OF for (bytecnt = 0

      if (failed_skt_mask & skt_mask)
        continue;
    } //-- OF if (skt_stat==SOCKET_ENABLE)
  }   //-- OF for (nDUT=MAX_SOCKET_NUM-1; nDUT>=0; nDUT--)
  // Re-Enable all enabled sockets
  LM_Phapi::Get()->SetGangSktMode(socket_mask);
  m_current_op_mode = saved_current_op_mode;

  // disable the DUT
  if (CompareFailed(failed_skt_mask))
    if (!m_prg_api_p->MisCompare(DeviceOperation::VERIFY, failed_skt_mask, 0, 0))
      return false;

  if (false == CheckFlashID())
    return false;

  if (false == SetFrequency(m_op_frequency))
    return false;

  failed_skt_mask = 0;
  // Get the status of all sockets (need to re-enable sockets)
  socket_mask = LM_Phapi::Get()->ActiveDUTMaskGet();
  m_current_op_mode = DeviceOperation::READ;
  for (nDUT = MAX_SOCKET_NUM - 1; nDUT >= 0; nDUT--)
  {
    // check only active sockets
    skt_stat = m_prg_api_p->SocketStatusGet(SocketNumChange(nDUT + 1));
    if (skt_stat == SOCKET_ENABLE)
    {
      // activate this one socket for reading
      // Note: ALL OTHER SOCKETS WILL BE DISABLED
      skt_mask = 1 << nDUT;
      LM_Phapi::Get()->SetSingleSktMode((WORD)(nDUT + 1));

      m_current_CMD = INQUIRY_CMD;
      if (false == SendFrame(SOH, ETX, &INQUIRY_CMD, 1))
        failed_skt_mask |= skt_mask;
      if (false == ReadDataFrame(data_buffer))
        failed_skt_mask |= skt_mask;
      if (data_buffer[0] != INQUIRY_CMD)
      {
        failed_skt_mask |= skt_mask;
        sprintf(msgbuff, "\tSocket%d: INQUIRY_CMD failed. Error code: 0x%02X", SocketNumChange(nDUT + 1), data_buffer[0]);
        m_prg_api_p->Write2EventLog(msgbuff);
      }
      if (failed_skt_mask & skt_mask)
        continue;

      //ID authentication mode Get command determines which security mode is supported:
      //0xFF: command protection mode - legacy NEC style with security bits
      //0x00: ID authentication mode  - Renesas style with password protection
      m_current_CMD = ID_AUTH_MODE_GET_CMD;
      if (false == SendFrame(SOH, ETX, &ID_AUTH_MODE_GET_CMD, 1))
        failed_skt_mask |= skt_mask;
      if (false == ReadDataFrame(data_buffer))
        failed_skt_mask |= skt_mask;
      if (data_buffer[0] != ID_AUTH_MODE_GET_CMD)
      {
        failed_skt_mask |= skt_mask;
        sprintf(msgbuff, "\tSocket%d: Execution of ID_AUTH_MODE_GET_CMD failed. Error code: 0x%02X", SocketNumChange(nDUT + 1), data_buffer[0]);
        m_prg_api_p->Write2EventLog(msgbuff);
      }

      if (failed_skt_mask & skt_mask)
        continue;

      if (false == SendFrame(SOD, ETX, &ID_AUTH_MODE_GET_CMD, 1)) //reverse ACK
        failed_skt_mask |= skt_mask;
      if (false == ReadDataFrame(data_buffer))
        failed_skt_mask |= skt_mask;
      if (data_buffer[0] != ID_AUTH_MODE_GET_CMD)
      {
        failed_skt_mask |= skt_mask;
        sprintf(msgbuff, "\tSocket%d: ID_AUTH_MODE_GET_CMD failed. Error code: 0x%02X", SocketNumChange(nDUT + 1), data_buffer[0]);
        m_prg_api_p->Write2EventLog(msgbuff);
      }

      if (failed_skt_mask & skt_mask)
        continue;

      if (expectedPROTmode == UNKNOWN)
      {
        expectedPROTmode = data_buffer[1]; //log it for the first device
        if (expectedPROTmode == ID_AUTHENTICATION_MODE)
        {
          m_optionSupportedByDev |= ID_AUTH;
          if (GetDataFromRam_8Bit(ID_AUTH_OFFSET, (DWORD)m_srcdata_bp))
            m_fCfgClearCmdReq = true;
        }
      }

      if (data_buffer[1] != expectedPROTmode)
      {
        failed_skt_mask |= skt_mask;
        sprintf(msgbuff, "\tSocket%d: Different protection mode found, expected: 0x%02X, actual: 0x%02X", SocketNumChange(nDUT + 1), expectedPROTmode, data_buffer[1]);
        m_prg_api_p->Write2EventLog(msgbuff);
      }
      if (failed_skt_mask & skt_mask)
        continue;

      if (m_optionSupportedByDev & ID_AUTH)
      { //ID AUTH mode: send password for authentication
        WriteCmdBuffer(0, ID_AUTH_CHECK_CMD);
        if (false == SendFrame(SOH, ETX, m_IDCODE_buffer, m_id_len, ID_AUTH_CHECK_CMD))
          failed_skt_mask |= skt_mask;
        if (false == ReadDataFrame(data_buffer))
          failed_skt_mask |= skt_mask;

        if (data_buffer[0] != ID_AUTH_CHECK_CMD)
        {
          failed_skt_mask |= skt_mask;
          sprintf(msgbuff, "\tSocket%d: Execution of ID_AUTH_CHECK_CMD failed. Error code: 0x%02X", SocketNumChange(nDUT + 1), data_buffer[0]);
          m_prg_api_p->Write2EventLog(msgbuff);
        }
        if (failed_skt_mask & skt_mask)
          continue;
      }
      else
      { //command protection mode: check protection status
        m_current_CMD = PROTECTION_GET_CMD;
        if (false == SendFrame(SOH, ETX, &PROTECTION_GET_CMD, 1))
          failed_skt_mask |= skt_mask;

        if (false == ReadDataFrame(data_buffer))
          failed_skt_mask |= skt_mask;

        if (data_buffer[0] != PROTECTION_GET_CMD)
        {
          failed_skt_mask |= skt_mask;
          sprintf(msgbuff, "\tSocket%d: Execution of PROTECTION_GET_CMD failed. Error code: 0x%02X", SocketNumChange(nDUT + 1), data_buffer[0]);
          m_prg_api_p->Write2EventLog(msgbuff);
        }
        if (failed_skt_mask & skt_mask)
          continue;

        if (false == SendFrame(SOD, ETX, &PROTECTION_GET_CMD, 1)) //reverse ACK
          failed_skt_mask |= skt_mask;
        if (false == ReadDataFrame(data_buffer))
          failed_skt_mask |= skt_mask;

        if (data_buffer[0] != PROTECTION_GET_CMD)
        {
          failed_skt_mask |= skt_mask;
          sprintf(msgbuff, "\tSocket%d: PROTECTION_GET_CMD failed. Error code: 0x%02X", SocketNumChange(nDUT + 1), data_buffer[0]);
          m_prg_api_p->Write2EventLog(msgbuff);
        }
        if (failed_skt_mask & skt_mask)
          continue;

        if (data_buffer[1] != 0xFF)
          m_fCfgClearCmdReq = true;
      } //-- OF if (m_optionSupportedByDev & ID_AUTH)

      if (m_optionSupportedByDev & ICU_S)
      {
        data_buffer[0] = ICU_S_INVALID; //Assuming all the device are blank
        if (false == SendFrame(SOH, ETX, data_buffer, 1, ICU_S_MODE_CHECK_CMD))
          failed_skt_mask |= skt_mask;
        if (false == ReadDataFrame(data_buffer))
          failed_skt_mask |= skt_mask;

        if (data_buffer[0] == ICU_S_MODE_CHECK_CMD)
        {
          expectedICUSmode = ICU_S_INVALID;
          /*ICU_s Invalid*/
          PRINTF("Socket%d: ICU_S is Invalid.\n ", SocketNumChange(nDUT + 1));
        }
        else
        {
          if ((data_buffer[0] == ICU_S_MODE_CHECK_ERR) && (data_buffer[1] == ICU_S_STATUS_VERIFY_ERR)) // Sends NACK if device is valid
          {
            expectedICUSmode = ICU_S_VALID;
            /*ICU_S is Valid*/
            PRINTF("Socket%d: ICU_S is Valid.\n", SocketNumChange(nDUT + 1));
          }
          else
          {
            failed_skt_mask |= skt_mask;
            sprintf(msgbuff, "\tSocket%d: ICU_S_MODE_CHECK_CMD result: 0x%02X, expected: 0x%02X", SocketNumChange(nDUT + 1), data_buffer[0], ICU_S_MODE_CHECK_CMD);
            m_prg_api_p->Write2EventLog(msgbuff);
          }
        }

        if (failed_skt_mask & skt_mask)
          continue;

        if (expectedICUSmode == ICU_S_INVALID)
          m_ICU_S_Status[nDUT] = ICU_S_RESPONSIVE_INVALID; //ICU-S not used --> whole data flash is available
        else
        {
          m_ICU_S_Status[nDUT] = ICU_S_RESPONSIVE_VALID;
          if (m_optionSupportedByDev & ICU_S_ERASE_PROHIBITED)
          {
            sprintf(msgbuff, "\tWARNING: Socket%d ICU-S validated! Last %d bytes of data flash can't be handled.", SocketNumChange(nDUT + 1), m_ICU_S_RegionSize);
            m_prg_api_p->Write2EventLog(msgbuff);
            if ((m_prg_api_p->GetRunningDeviceOp() != ::READ))
            {
              if (!(m_optionSelectedByUser & ICU_S))
              {
                failed_skt_mask |= skt_mask;
                sprintf(msgbuff, "\tSocket%d: Device has a valid ICU-S Config and it conflicts with the job settings", SocketNumChange(nDUT + 1));
                m_ICU_S_Status[nDUT] = SOCKETS_HAVE_DIFFERENT_STATUS; //setting to unknown status, because its an error
                m_prg_api_p->Write2EventLog(msgbuff);
              }
            }
          }
          else
            m_fCfgClearCmdReq = true;
        }
      } //-- OF if (m_optionSupportedByDev & ICU_S)

      if (m_optionSupportedByDev & OTP)
      {
        WriteCmdBuffer(0, OTP_GET_CMD);
        if (false == SendFrame(SOH, ETX, GetCmdBufferP(), 1))
          failed_skt_mask |= skt_mask;
        if (false == ReadDataFrame(data_buffer))
          failed_skt_mask |= skt_mask;

        if (data_buffer[0] != OTP_GET_CMD)
        {
          failed_skt_mask |= skt_mask;
          sprintf(msgbuff, "\tSocket%d: Execution of OTP_GET_CMD failed. Error code: 0x%02X", SocketNumChange(nDUT + 1), data_buffer[0]);
          m_prg_api_p->Write2EventLog(msgbuff);
        }
        if (failed_skt_mask & skt_mask)
          continue;

        if (false == SendFrame(SOD, ETX, GetCmdBufferP(), 1)) //reverse ACK
          failed_skt_mask |= skt_mask;
        if (false == ReadDataFrame(data_buffer))
          failed_skt_mask |= skt_mask;

        if (data_buffer[0] != OTP_GET_CMD)
        {
          failed_skt_mask |= skt_mask;
          sprintf(msgbuff, "\tSocket%d: OTP_GET_CMD failed. Error code: 0x%02X", SocketNumChange(nDUT + 1), data_buffer[0]);
          m_prg_api_p->Write2EventLog(msgbuff);
        }
        if (failed_skt_mask & skt_mask)
          continue;
        for (bytecnt = 0; bytecnt < LB_LENGTH; bytecnt++)
        {
          if (data_buffer[1 + bytecnt] != 0xFF)
            break;
        }
        if (bytecnt != LB_LENGTH)
        {
          sprintf(msgbuff, "\tSocket%d: At least one sector is OTP.", SocketNumChange(nDUT + 1));
          m_prg_api_p->Write2EventLog(msgbuff);
        }
      } //-- OF if (m_optionSupportedByDev & OTP)
    }   //-- OF if (skt_stat==SOCKET_ENABLE)
  }     //-- OF for (nDUT=MAX_SOCKET_NUM-1; nDUT>=0; nDUT--)
  // Re-Enable all enabled sockets
  LM_Phapi::Get()->SetGangSktMode(socket_mask);
  m_current_op_mode = saved_current_op_mode;

  // disable the DUT
  if (CompareFailed(failed_skt_mask))
    if (!m_prg_api_p->MisCompare(DeviceOperation::VERIFY, failed_skt_mask, 0, 0))
      return false;

  PRINTF("C_RV40F::Device Init() - RESET done\n"); // debug statements

  if (m_tBoot_code_p != NULL)
  {
    //for devices with erroneous firmware new firmware has to be loaded as replacement and executed
    device_init_ok = InitializeFlashFirmware(m_tBoot_code_p);
  }

  if (false == GetSockets_ICU_S_Status())
    PRINTF("use single socket Mode for ICU_S Devices.\n");

  // check for any system events
  if (m_prg_api_p->SysEvtChk())
    PRINTF(" OC @ line %d\n", __LINE__);

  if (device_init_ok)
    m_fTarget_initialized = true;

  return device_init_ok;
}

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ //
int C_RV40F::SetFrequency(const BYTE chFrequency) //unit: 100kHz
{
#if (ALG_DEBUG > 1)
  PRINTF("C_RV40F::SetFrequency\n"); // debug statements
#endif

  DWORD FOSC = (DWORD)chFrequency * 100000;
  BYTE buffer[9];

  WriteCmdBuffer(FREQUENCY_SET_CMD, FOSC, m_tRV40F_Param_p->FCPU);
  if (false == SendFrame(SOH, ETX, GetCmdBufferP(), 9))
    return false;

  if (false == GetDataFrame(ETX, GetCmdBufferP(), 1, CHECK_ST1))
    return false;

  if (false == SendFrame(SOD, ETX, GetCmdBufferP(), 1)) //reverse ACK
    return false;

  buffer[0] = (BYTE)(m_tRV40F_Param_p->FCPU >> 24);
  buffer[1] = (BYTE)(m_tRV40F_Param_p->FCPU >> 16);
  buffer[2] = (BYTE)(m_tRV40F_Param_p->FCPU >> 8);
  buffer[3] = (BYTE)(m_tRV40F_Param_p->FCPU);
  if (false == GetDataFrame(ETX, buffer, 8, 4, FREQUENCY_SET_CMD)) //don't check the peripheral frq.
    return false;

  MicroSecDelay(1000);
  //switch to higher speed
  DWORD shift_frequency = m_tRV40F_Param_p->RANSET[0];
  m_fpga_p->SetSerialParams(JP0_0_SI_PIN,    // Serial In -> writing to the device
                            JP0_1_SO_PIN,    // Serial Out-> reading from the deivce
                            JP0_2_SCK_PIN,   // Shift Clock pin
                            shift_frequency, // Shift clock in Hz
                            m_fpga_p->EDGE_FALLING,
                            m_fpga_p->MSB_FIRST);

  m_sys_clk = m_tRV40F_Param_p->FCPU / 1000000; //consider higher PLL-frequency, the wait times will be shorter
  m_fStartupMode = false;

  return true;
}

// Loads new firmware replacing the erroneous one in the RAM of the device
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ //
int C_RV40F::InitializeFlashFirmware(ALG_CODE *tCode_p)
{
  int ret_value;
  BYTE frameEndType;
  WORD byteCnt, transferByteCnt;
  WORD dataLength;

  //
  WriteCmdBuffer(BOOTSTRAP_CMD, tCode_p->destinationAddress, tCode_p->code_length);
  if (false == SendFrame(SOH, ETX, GetCmdBufferP(), 9))
    return false;

  if (false == GetDataFrame(ETX, GetCmdBufferP(), CHECK_ST1, CHECK_ST1))
    return false;

  byteCnt = 0;
  do
  {
    transferByteCnt = tCode_p->code_length - byteCnt;
    if (transferByteCnt / MAX_PAGE_SIZE)
      dataLength = MAX_PAGE_SIZE;
    else
      dataLength = transferByteCnt % MAX_PAGE_SIZE;

    if (byteCnt + dataLength >= tCode_p->code_length) //last data block?
      frameEndType = ETX;                             //end of all data
    else
      frameEndType = ETB; //end of block

    if (false == SendFrame(SOD, frameEndType, (BYTE *)&tCode_p->code_p[byteCnt], dataLength, BOOTSTRAP_CMD))
      return false;

    //check transferring status
    ret_value = GetDataFrame(ETX, GetCmdBufferP(), CHECK_ST1, CHECK_ST1);

    byteCnt += dataLength;
  } while (ret_value == true && byteCnt < tCode_p->code_length);

  return ret_value;
}

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ //
C_RV40F::DEV_STAT_E C_RV40F::RV_ProtBits(void)
{
  BYTE data_buffer[PROT_LENGTH];
  DEV_STAT_E op_stat = OPERATION_OK;
  const CURRENT_OP_STATUS *current_op_stat_p = m_prg_api_p->CurrentOpStatusGet();

  m_current_op_mode = current_op_stat_p->operation;

  //get security bytes
  if (m_current_op_mode != DeviceOperation::READ)
  {                                                                          //prepare data for verify
    data_buffer[0] = ~GetDataFromRam_8Bit(PROT_OFFSET, (DWORD)m_srcdata_bp); //bits are inverted
  }
  WriteCmdBuffer(0, PROTECTION_GET_CMD);
  if (false == SendFrame(SOH, ETX, GetCmdBufferP(), 1))
    return VERIFY_ERR;
  if (false == GetDataFrame(ETX, GetCmdBufferP(), CHECK_ST1, CHECK_ST1))
    return VERIFY_ERR;
  if (false == SendFrame(SOD, ETX, GetCmdBufferP(), 1)) //reverse ACK
    return VERIFY_ERR;
  if (false == GetDataFrame(ETX, data_buffer, PROT_LENGTH, PROT_LENGTH, PROTECTION_GET_CMD))
    return VERIFY_ERR;

  if (m_current_op_mode == DeviceOperation::READ)
  {                                                                               //put data to memory
    StoreDataToRam_8Bit(PROT_OFFSET, (BYTE)~data_buffer[0], (DWORD)m_srcdata_bp); //inverted
  }

  return op_stat;
}

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ //
C_RV40F::DEV_STAT_E C_RV40F::RV_OPBT(void)
{
  DEV_STAT_E op_stat = OPERATION_OK;
  volatile BYTE *srcbase = (volatile BYTE *)m_srcdata_bp;
  const CURRENT_OP_STATUS *current_op_stat_p = m_prg_api_p->CurrentOpStatusGet();

  m_current_op_mode = current_op_stat_p->operation;

  //get option bytes (OPB)
  WriteCmdBuffer(0, OPTION_GET_CMD);
  if (false == SendFrame(SOH, ETX, GetCmdBufferP(), 1))
    return VERIFY_ERR;
  if (false == GetDataFrame(ETX, GetCmdBufferP(), CHECK_ST1, CHECK_ST1))
    return VERIFY_ERR;
  if (false == SendFrame(SOD, ETX, GetCmdBufferP(), 1)) //reverse ACK
    return VERIFY_ERR;
  if (false == GetDataFrame(ETX, (BYTE *)(srcbase + OPBT_OFFSET(0)), OPBT_LENGTH, OPBT_LENGTH, OPTION_GET_CMD))
    return VERIFY_ERR;

  return op_stat;
}

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ //
C_RV40F::DEV_STAT_E C_RV40F::RV_IDCode(void)
{
  DEV_STAT_E op_stat = OPERATION_OK;
  volatile BYTE *srcbase = (volatile BYTE *)m_srcdata_bp;
  const CURRENT_OP_STATUS *current_op_stat_p = m_prg_api_p->CurrentOpStatusGet();

  m_current_op_mode = current_op_stat_p->operation;

  WriteCmdBuffer(0, IDCODE_GET_CMD);
  if (false == SendFrame(SOH, ETX, GetCmdBufferP(), 1))
    return VERIFY_ERR;
  if (false == GetDataFrame(ETX, GetCmdBufferP(), CHECK_ST1, CHECK_ST1))
    return VERIFY_ERR;
  if (false == SendFrame(SOD, ETX, GetCmdBufferP(), 1)) //reverse ACK
    return VERIFY_ERR;
  if (false == GetDataFrame(ETX, (BYTE *)(srcbase + ID_OFFSET), m_id_len, m_id_len, IDCODE_GET_CMD))
    return VERIFY_ERR;

  return op_stat;
}

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ //
C_RV40F::DEV_STAT_E C_RV40F::Prog_IDCode(const BYTE cmd)
{
  volatile BYTE *srcbase = (volatile BYTE *)m_srcdata_bp;
  DEV_STAT_E op_stat = OPERATION_OK;

  WriteCmdBuffer(0, cmd);
  if (false == SendFrame(SOH, ETX, (BYTE *)(srcbase + ID_OFFSET), m_id_len, cmd))
    return PROGRAM_ERR;
  if (false == GetDataFrame(ETX, GetCmdBufferP(), CHECK_ST1, CHECK_ST1))
    return PROGRAM_ERR;

  return op_stat;
}

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ //
C_RV40F::DEV_STAT_E C_RV40F::RV_BlockProtBits(BYTE blockProtCmd)
{
  DWORD memOffset;
  DEV_STAT_E op_stat = OPERATION_OK;
  volatile BYTE *srcbase = (volatile BYTE *)m_srcdata_bp;
  const CURRENT_OP_STATUS *current_op_stat_p = m_prg_api_p->CurrentOpStatusGet();

  m_current_op_mode = current_op_stat_p->operation;

  if (blockProtCmd == OTP_GET_CMD)
    memOffset = OTP_BIT_OFFSET;
  else
    memOffset = LOCK_BIT_OFFSET;

  WriteCmdBuffer(0, blockProtCmd);
  if (false == SendFrame(SOH, ETX, GetCmdBufferP(), 1))
    return VERIFY_ERR;
  if (false == GetDataFrame(ETX, GetCmdBufferP(), CHECK_ST1, CHECK_ST1))
    return VERIFY_ERR;
  if (false == SendFrame(SOD, ETX, GetCmdBufferP(), 1)) //reverse ACK
    return VERIFY_ERR;
  if (false == GetDataFrame(ETX, (BYTE *)(srcbase + memOffset), LB_LENGTH, LB_LENGTH, blockProtCmd))
    return VERIFY_ERR;

  return op_stat;
}

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ //
C_RV40F::DEV_STAT_E C_RV40F::RV_DeviceConfig(void)
{
  DEV_STAT_E op_stat = OPERATION_OK;

  if ((m_optionSupportedByDev & ID_AUTH) == 0)
  {
    if (OPERATION_OK != RV_ProtBits())
    {
      m_prg_api_p->Write2EventLog("Verify of the protection bits failed.");
      return VERIFY_ERR;
    }
    if (m_fReadProtected)
      m_prg_api_p->Write2EventLog("WARNING: Verify of the ID Code not possible!");
    else if (OPERATION_OK != RV_IDCode())
    {
      m_prg_api_p->Write2EventLog("Verify of the ID Code failed.");
      return VERIFY_ERR;
    }
  }
  if (OPERATION_OK != RV_OPBT())
  {
    m_prg_api_p->Write2EventLog("Verify of the option bytes failed.");
    return VERIFY_ERR;
  }
  if (m_optionSupportedByDev & LB)
    if (OPERATION_OK != RV_BlockProtBits(LOCKBIT_GET_CMD))
    {
      m_prg_api_p->Write2EventLog("Verify of the block LOCK bits failed.");
      return VERIFY_ERR;
    }
  if (m_optionSupportedByDev & OTP)
    if (OPERATION_OK != RV_BlockProtBits(OTP_GET_CMD))
    {
      m_prg_api_p->Write2EventLog("Verify of the block OTP bits failed.");
      return VERIFY_ERR;
    }

  return op_stat;
}

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ //
BYTE C_RV40F::swapBits(const BYTE dataByte, const BYTE bit_1, const BYTE bit_2)
{
  BYTE tempByte, tempBit_1, tempBit_2;

  tempByte = dataByte & (~(1 << bit_1)) & (~(1 << bit_2));
  tempBit_1 = (dataByte >> bit_1) & 1;
  tempBit_2 = (dataByte >> bit_2) & 1;
  tempByte |= (tempBit_1 << bit_2);
  tempByte |= (tempBit_2 << bit_1);

  return tempByte;
}

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ //
DWORD C_RV40F::GetDFAdressInDevice(DWORD address)
{
  return (0xFD200000 + address); //re-mapping: CH file 0xFF200000 <--> RR map 0x2000000
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
C_RV40F::DEV_STAT_E C_RV40F::Erase_DataFlash_Area(DWORD DF_startaddress_in_device, DWORD DF_endaddress_in_device)
{
  DWORD data_area_size;

  DWORD address, startaddress_in_device;
  bool fResetReq = false;

  DEV_STAT_E erase_stat = OPERATION_OK;

  //DF_startaddress_in_device = GetDFAdressInDevice(m_devsectors_p[block].begin_address);
  //erase data flash area (RV40F flash)
  data_area_size = (DF_endaddress_in_device - DF_startaddress_in_device) + 1;

  for (address = 0; address < data_area_size && erase_stat == OPERATION_OK; address += m_DF_erase_unit)
  {
    startaddress_in_device = DF_startaddress_in_device + address;

    WriteCmdBuffer(ERASE_CMD, startaddress_in_device, 0);

    if (false == SendFrame(SOH, ETX, GetCmdBufferP(), 5))
      return WSM_BUSY_ERR;

    if (false == GetDataFrame(ETX, GetCmdBufferP(), CHECK_ST1, CHECK_ST1))
    {
      erase_stat = BLOCK_ERASE_ERR;
      PRINTF("C_RV40F::Erasing failed at data flash address %08Xh\n", startaddress_in_device);
    }
  } //-- OF for (address = 0; address < ; address += )
  return erase_stat;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ //
C_RV40F::DEV_STAT_E C_RV40F::Erase_ICU_Area()
{
  DWORD address;
  BYTE status_expected;
  BYTE status_busy = 0xD6;
  DEV_STAT_E erase_stat = OPERATION_OK;

  for (address = 0; address < m_ICU_S_RegionSize && erase_stat == OPERATION_OK; address += (m_DF_erase_unit / 2))
  {
    m_current_CMD = ICU_REGION_ERASE_CMD;
    if (false == SendFrame(SOH, ETX, &ICU_REGION_ERASE_CMD, 1))
      return WSM_BUSY_ERR;

    if (address == (m_ICU_S_RegionSize - (m_DF_erase_unit / 2)))
      status_expected = ICU_REGION_ERASE_CMD;
    else
      status_expected = status_busy;

    if (false == GetDataFrame(ETX, &status_expected, CHECK_ST1, CHECK_ST1, ICU_REGION_ERASE_CMD, LONG_DELAY))
      erase_stat = BLOCK_ERASE_ERR;
  } //-- OF for (address = 0; address < ; address += )

  return erase_stat;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ //
C_RV40F::DEV_STAT_E C_RV40F::Program_DataFlash_Area(WORD block, WORD ICU_S_RegionSize)
{
  volatile BYTE *srcbase = (volatile BYTE *)m_srcdata_bp;
  DEV_STAT_E prog_stat = OPERATION_OK;

  DWORD address, endaddress, startaddress;
  DWORD startaddress_in_device, endaddress_in_device;
  DWORD areaSize;
  WORD blockSize;
  BYTE frameEndType;

  address = m_devsectors_p[block].begin_address;
  endaddress = m_devsectors_p[block].end_address - ICU_S_RegionSize;
  if (m_fDF_filled0xFF == false)
  {
    do
    {
      //search for words to be programmed (marked with 0x00 in the MARKER area)
      while (DF_IsAreaEmpty(address, m_DF_write_unit) && address <= endaddress)
        address += m_DF_write_unit;
      if (address > endaddress)
        break; //reached end of DF sector
      startaddress = address;
      do
      {
        address += m_DF_write_unit;
        if (0 == (address % MAX_PAGE_SIZE))
          break;
      } while (!DF_IsAreaEmpty(address, m_DF_write_unit));

      startaddress_in_device = GetDFAdressInDevice(startaddress);
      endaddress_in_device = GetDFAdressInDevice(address - 1);
      areaSize = (WORD)(endaddress_in_device - startaddress_in_device + 1);

      WriteCmdBuffer(PROGRAM_CMD, startaddress_in_device, endaddress_in_device);

      if (false == SendFrame(SOH, ETX, GetCmdBufferP(), 9))
        return WSM_BUSY_ERR;

      if (false == GetDataFrame(ETX, GetCmdBufferP(), CHECK_ST1, CHECK_ST1))
        return PROGRAM_ERR;

      if (false == SendFrame(SOD, ETX, (BYTE *)(srcbase + startaddress), areaSize, PROGRAM_CMD))
        return WSM_BUSY_ERR;

      if (false == GetDataFrame(ETX, GetCmdBufferP(), CHECK_ST1, CHECK_ST1))
      {
        prog_stat = PROGRAM_ERR;
        PRINTF("C_RV40F::Programming fail at DF address 0x%X\n", startaddress);
      }
    } while (address <= endaddress && prog_stat == OPERATION_OK);
  } // OF if(m_fDF_filled0xFF == false)
  else
  {
    startaddress_in_device = GetDFAdressInDevice(address);
    startaddress = address;
    endaddress_in_device = GetDFAdressInDevice(endaddress);
    areaSize = (WORD)(endaddress_in_device - startaddress_in_device + 1);
    if (areaSize < MAX_PAGE_SIZE)
      blockSize = (WORD)areaSize;
    else
      blockSize = MAX_PAGE_SIZE;

    WriteCmdBuffer(PROGRAM_CMD, startaddress_in_device, endaddress_in_device);

    if (false == SendFrame(SOH, ETX, GetCmdBufferP(), 9))
      return WSM_BUSY_ERR;

    if (false == GetDataFrame(ETX, GetCmdBufferP(), CHECK_ST1, CHECK_ST1))
      return PROGRAM_ERR;

    for (address = startaddress;
         address <= endaddress && prog_stat == OPERATION_OK;
         address += blockSize)
    {
      if (address + blockSize > endaddress) //last data block?
        frameEndType = ETX;                 //end of all data
      else
        frameEndType = ETB; //end of block

      if (false == SendFrame(SOD, frameEndType, (BYTE *)(srcbase + address), blockSize, PROGRAM_CMD))
        return WSM_BUSY_ERR;
      if (false == GetDataFrame(ETX, GetCmdBufferP(), CHECK_ST1, CHECK_ST1))
      {
        prog_stat = PROGRAM_ERR;
        PRINTF("C_RV40F::Programming fail at 0x%X\n", address);
      }
    }
  }

  return prog_stat;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ //
C_RV40F::DEV_STAT_E C_RV40F::Verify_DataFlash_Area(WORD block, WORD ICU_S_RegionSize)
{

  volatile BYTE *srcbase = (volatile BYTE *)m_srcdata_bp;
  DWORD startaddress, endaddress;
  DWORD startaddress_in_device, endaddress_in_device;
  DWORD address;
  WORD blockSize;
  DWORD areaSize;
  BYTE frameEndType;
  DEV_STAT_E verify_stat = OPERATION_OK;

  startaddress = m_devsectors_p[block].begin_address;
  endaddress = m_devsectors_p[block].end_address - ICU_S_RegionSize;
  startaddress_in_device = GetDFAdressInDevice(startaddress);
  endaddress_in_device = GetDFAdressInDevice(endaddress);
  areaSize = endaddress_in_device - startaddress_in_device + 1;

  if (areaSize < MAX_PAGE_SIZE)
    blockSize = (WORD)areaSize;
  else
    blockSize = MAX_PAGE_SIZE;

  //use read command here, because CRC and VERIFY commands request at least 1024byte blocks
  WriteCmdBuffer(READ_CMD, startaddress_in_device, endaddress_in_device);

  if (false == SendFrame(SOH, ETX, GetCmdBufferP(), 9))
    return VERIFY_ERR;
  if (false == GetDataFrame(ETX, GetCmdBufferP(), CHECK_ST1, CHECK_ST1))
    return VERIFY_ERR;

  for (address = startaddress;
       address <= endaddress && verify_stat == OPERATION_OK;
       address += blockSize)
  {
    if (false == SendFrame(SOD, ETX, GetCmdBufferP(), 1)) //reverse ACK
      return VERIFY_ERR;

    if (address + blockSize > endaddress) //last data block?
      frameEndType = ETX;                 //end of all data
    else
      frameEndType = ETB; //end of block

    if (false == VerifyDFblock(frameEndType, (BYTE *)(srcbase + address), blockSize))
      return VERIFY_ERR;
  }

  return verify_stat;
}
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ //
int C_RV40F::CheckFlashID(void)
{
  //base class has nothing to do
  return true;
}
//-- END C_RV40F definitions

// -- Begin entry points for programmer system
/*************************************************************************
FUNCTION IDCheck()
ARGUMENTS
RETURNS true if part ID == expected ID, false if not
METHOD
    Read device ID's and compare to expected
EXCEPTIONS --  none
*************************************************************************/
bool C_RV40F::IDCheck()
{
#if (ALG_DEBUG > 0)
  PRINTF("C_RV40F::IDCheck() (Signature + EXTRAs)\n"); // debug statements sent to command window
#endif

  //LM_Phapi::Get()->ScopePinSet (1);
  BYTE data_buffer[2 * SIGNATURE_LENGTH];
  BYTE dev_name[DEVICE_NAME_LENGTH + 1];
  BYTE stat_buffer[1];
  SOCKET_STATUS_E skt_stat;
  BYTE bytecnt, skt_mask, failed_skt_mask;
  int nDUT;
  bool device_id_ok = true;

  m_current_op_mode = DeviceOperation::IDCHECK;

  if (false == DeviceInit())
  {
    m_reset_H_flmd0_pulse_start_wait = 0; //re-start RST_H_FLMD0_DLY-pulse search in RESET
    return false;
  }

  failed_skt_mask = 0;
  // Get the status of all sockets (needed to re-enable sockets)
  WORD socket_mask = LM_Phapi::Get()->ActiveDUTMaskGet();
  m_current_op_mode = DeviceOperation::READ;

  for (nDUT = MAX_SOCKET_NUM - 1; nDUT >= 0; nDUT--)
  {
    m_current_CMD = SIGNATURE_GET_CMD;
    // check only active sockets
    skt_stat = m_prg_api_p->SocketStatusGet(SocketNumChange(nDUT + 1));
    if (skt_stat == SOCKET_ENABLE)
    {
      // active this one socket for reading
      // Note: AL OTHER SOCKETS WILL BE DISABLED
      skt_mask = 1 << nDUT;
      LM_Phapi::Get()->SetSingleSktMode((WORD)(nDUT + 1));

      if (false == SendFrame(SOH, ETX, &SIGNATURE_GET_CMD, 1))
        failed_skt_mask |= skt_mask;
      if (false == GetDataFrame(ETX, stat_buffer, CHECK_ST1, CHECK_ST1))
        failed_skt_mask |= skt_mask;
      if (stat_buffer[0] != SIGNATURE_GET_CMD)
      {
        // disable the DUT
        if (!m_prg_api_p->MisCompare(DeviceOperation::IDCHECK, skt_mask, 0, SIGNATURE_GET_CMD))
          failed_skt_mask |= skt_mask;
      }

      if (failed_skt_mask & skt_mask)
        continue;

      if (false == SendFrame(SOD, ETX, stat_buffer, 1)) //reverse ACK
        failed_skt_mask |= skt_mask;
      if (false == GetDataFrame(ETX, data_buffer, m_signature_len, m_signature_len, SIGNATURE_GET_CMD))
        failed_skt_mask |= skt_mask;

      if (failed_skt_mask & skt_mask)
        continue;

      for (bytecnt = 0; bytecnt < DEVICE_NAME_LENGTH; bytecnt++) //device name
        dev_name[bytecnt] = data_buffer[bytecnt];
      dev_name[DEVICE_NAME_LENGTH] = '\0'; //string-end for device name

      sprintf(msgbuff, "\tSocket%d device name: %s", SocketNumChange(nDUT + 1), &dev_name[0]);
      m_prg_api_p->Write2EventLog(msgbuff);

      //check ID
      for (bytecnt = 0; bytecnt < SIGNATURE_LENGTH; bytecnt++)
      {
        if (data_buffer[bytecnt] != m_tRV40F_Param_p->SIGNATURE[bytecnt])
        {
          // disable the DUT
          failed_skt_mask |= skt_mask;
          if (!m_prg_api_p->MisCompare(DeviceOperation::IDCHECK, skt_mask, bytecnt, m_tRV40F_Param_p->SIGNATURE[bytecnt]))
          {
            sprintf(msgbuff, "\tSocket%d failed at signature byte %d. Expected: 0x%02X, actual: 0x%02X", SocketNumChange(nDUT + 1), bytecnt, m_tRV40F_Param_p->SIGNATURE[bytecnt], data_buffer[bytecnt]);
            m_prg_api_p->Write2EventLog(msgbuff);
          }
        }
      } //-- OF for (bytecnt = 0

      if (failed_skt_mask & skt_mask)
        continue;

      if ((m_optionSupportedByDev & ID_AUTH) == 0)
      { //command protection mode
        //check protection status
        m_current_CMD = PROTECTION_GET_CMD;
        if (false == SendFrame(SOH, ETX, &PROTECTION_GET_CMD, 1))
          device_id_ok = false;

        if (false == GetDataFrame(ETX, stat_buffer, CHECK_ST1, CHECK_ST1))
          device_id_ok = false;

        if (stat_buffer[0] != PROTECTION_GET_CMD)
        {
          // disable the DUT
          if (!m_prg_api_p->MisCompare(DeviceOperation::IDCHECK, skt_mask, 0, PROTECTION_GET_CMD))
            failed_skt_mask |= (1 << nDUT);
        }

        if (false == SendFrame(SOD, ETX, &PROTECTION_GET_CMD, 1)) //reverse ACK
          device_id_ok = false;
        if (false == GetDataFrame(ETX, data_buffer, PROT_LENGTH, PROT_LENGTH, PROTECTION_GET_CMD))
          device_id_ok = false;

        if (!(data_buffer[0] & PROT_BYTE_READPROT_MSK))
        {
          m_fReadProtected = true;
          sprintf(msgbuff, "\tSocket%d security state-> device is read protected.", SocketNumChange(nDUT + 1));
          m_prg_api_p->Write2EventLog(msgbuff);
        }
        if (!(data_buffer[0] & PROT_BYTE_ERASPROT_MSK))
        {
          sprintf(msgbuff, "\tSocket%d security state-> device is erase protected.", SocketNumChange(nDUT + 1));
          m_prg_api_p->Write2EventLog(msgbuff);
        }
        if (!(data_buffer[0] & PROT_BYTE_PROGPROT_MSK))
        {
          sprintf(msgbuff, "\tSocket%d security state-> device is write protected.", SocketNumChange(nDUT + 1));
          m_prg_api_p->Write2EventLog(msgbuff);
        }
      } //-- if ((m_optionSupportedByDev&ID_AUTH) == 0)
    }   //-- OF if (skt_stat==SOCKET_ENABLE)
  }     //-- OF for (nDUT=MAX_SOCKET_NUM-1; nDUT>=0; nDUT--)

  // Re-Enable all enabled sockets
  LM_Phapi::Get()->SetGangSktMode(socket_mask);
  m_current_op_mode = DeviceOperation::IDCHECK;

  if ((failed_skt_mask & m_akt_skt_msk) == m_akt_skt_msk)
  {
    device_id_ok = false;
    m_reset_H_flmd0_pulse_start_wait = 0; //re-start RST_H_FLMD0_DLY-pulse search in RESET
  }

  //LM_Phapi::Get()->ScopePinSet (0);
  return device_id_ok;
}

////////////////////////////////////////////////////////////////////////////////
//          BlankCheck ()
// Inputs:
//
// Return:
//
// METHOD:
////////////////////////////////////////////////////////////////////////////////
C_RV40F::DEV_STAT_E C_RV40F::BlankCheck()
{
#if (ALG_DEBUG > 0)
  PRINTF("C_RV40F::BlankCheck()\n");
#endif

  //Blank check shouln't be called for the RH850 family, because erasing is essential for the data retention time.

  DEV_STAT_E blank_stat = OPERATION_OK;

  m_current_op_mode = DeviceOperation::BLANKCHECK;

  if (false == DeviceInit())
    return BLANKCHECK_ERR;

  if (m_fCfgClearCmdReq)
    return BLANKCHECK_ERR;

  DWORD startaddress, endaddress, startaddress_in_device, endaddress_in_device;
  WORD block = 0;
  do
  {
    // see if block has to be programmed
    if (m_prg_api_p->GetSectorFlag(SectorOp::BLANK_CHECK_OP, block))
    {
      if (block == m_option_data_block)
        continue;

      startaddress = m_devsectors_p[block].begin_address;
      endaddress = m_devsectors_p[block].end_address;
      if (block >= m_DF_block_nr)
      {
        endaddress -= m_ICU_S_RegionSize;
        startaddress_in_device = GetDFAdressInDevice(startaddress);
        endaddress_in_device = GetDFAdressInDevice(endaddress);
      }
      else
      {
        startaddress_in_device = startaddress;
        endaddress_in_device = endaddress;
      }

      WriteCmdBuffer(BLANKCHECK_CMD, startaddress_in_device, endaddress_in_device);

      if (false == SendFrame(SOH, ETX, GetCmdBufferP(), 9))
        return WSM_BUSY_ERR;

      if (false == GetDataFrame(ETX, GetCmdBufferP(), CHECK_ST1, CHECK_ST1))
        blank_stat = BLANKCHECK_ERR;

      // check for any system events
      if (m_prg_api_p->SysEvtChk())
        return HARDWARE_ERR; // O.C. or Adapter change - return immediately
    }                        // end of if 'sector' was to be programmed
  } while (++block < m_sector_quantity && blank_stat == OPERATION_OK);

  if (blank_stat)
  {
    blank_stat = BLANKCHECK_ERR;
    ResetToProgrammingMode(); //added, in order to get all devices synchronized when an error occured
  }

  return blank_stat;
}

//*************************************************************************
//FUNCTION    Read()
//ARGUMENTS   none
//RETURNS     DEV_STAT_E - device status enumeration
//METHOD      Read device to image on a per block basis
//EXCEPTIONS --  none
//**********************************************************************
C_RV40F::DEV_STAT_E C_RV40F::Read()
{
#if (ALG_DEBUG > 0)
  // debug statements
  PRINTF("C_RV40F::Read()\n");
#endif

  DWORD areaSize;
  WORD blockSize;
  BYTE stat_buffer[1];
  DWORD blankAreaSize;
  WORD packetLength;
  BYTE checksum;
  BYTE socket_stat;
  BYTE read_cmd;
  DEV_STAT_E read_stat = OPERATION_OK;

  m_current_op_mode = DeviceOperation::READ;

  if (false == DeviceInit())
    return READ_ERR;

  DWORD startaddress, endaddress, startaddress_in_device, endaddress_in_device, address;
  BYTE frameEndType;
  volatile BYTE *srcbase = (volatile BYTE *)m_srcdata_bp;

  if (m_DF_write_unit == 16)
    read_cmd = READ_CMD;
  else
    read_cmd = EXTENDED_READ_CMD;

  WORD block = 0;
  do
  {
    // see if block has to be programmed
    if (m_prg_api_p->GetSectorFlag(SectorOp::PROGRAM_SECTOR_OP, block))
    {
      if (block == m_option_data_block)
      {
        if ((m_optionSupportedByDev & ID_AUTH) == 0) //either AUTH mode or cmd prot.
        {
          if (OPERATION_OK != RV_ProtBits())
            return READ_ERR;
          if (OPERATION_OK != RV_IDCode())
            return READ_ERR;
        }
        if (OPERATION_OK != RV_OPBT())
          return READ_ERR;
        if (OPERATION_OK != RV_BlockProtBits(LOCKBIT_GET_CMD))
          return READ_ERR;
        if (OPERATION_OK != RV_BlockProtBits(OTP_GET_CMD))
          return READ_ERR;
        continue; //go forward with next block
      }
      //if (block >= m_DF_block_nr && m_fDF_filled0xFF == false)
      //continue; //dataflash can not be read without the information which addresses are programmed

      startaddress = m_devsectors_p[block].begin_address;
      endaddress = m_devsectors_p[block].end_address;

      if (block >= m_DF_block_nr)
      {
        if (m_All_skts_ICU_S_INVALID == ICU_S_RESPONSIVE_VALID)
          endaddress -= m_ICU_S_RegionSize;

        startaddress_in_device = GetDFAdressInDevice(startaddress);
        endaddress_in_device = GetDFAdressInDevice(endaddress);
      }
      else
      {
        startaddress_in_device = startaddress;
        endaddress_in_device = endaddress;
      }

      areaSize = endaddress_in_device - startaddress_in_device + 1;
      if (areaSize < MAX_PAGE_SIZE)
        blockSize = (WORD)areaSize;
      else
        blockSize = MAX_PAGE_SIZE;

      WriteCmdBuffer(read_cmd, startaddress_in_device, endaddress_in_device);

      if (false == SendFrame(SOH, ETX, GetCmdBufferP(), 9))
        return WSM_BUSY_ERR;

      if (false == GetDataFrame(ETX, stat_buffer, CHECK_ST1, CHECK_ST1))
        return READ_ERR;

      if (stat_buffer[0] != read_cmd)
      {
        // disable the DUT
        PRINTF("Check status for cmd %Xh failed. Expected: %Xh, Actual: %Xh\n", (WORD)m_current_CMD, EXTENDED_READ_CMD, stat_buffer[0]);
        if (!m_prg_api_p->MisCompare(DeviceOperation::READ, 0xF, 0, EXTENDED_READ_CMD))
          return READ_ERR;
      }

      m_comm_buffer[0] = 0x00;
      address = startaddress;
      do
      {
        if (false == SendFrame(SOD, ETX, GetCmdBufferP(), 1)) //reverse ACK
          return WSM_BUSY_ERR;
        socket_stat = WaitUntilDeviceReady(DeviceOperation::READ, LOGIC_1, DEFAULT_TIMEOUT);
        if (CompareFailed(socket_stat))
        {
          if (!m_prg_api_p->MisCompare(DeviceOperation::READ, socket_stat, 0, LOGIC_1))
          {
            m_prg_api_p->Write2EventLog(" SO timeout error!");
            return READ_ERR;
          }
        }
        //Receive Data Frame
        //read 3 Bytes: SOD + LN
        if (m_fpga_p->SerialRead(&m_comm_buffer[0], 24))
          return READ_ERR;
        packetLength = ((WORD)m_comm_buffer[1] << 8) | (WORD)m_comm_buffer[2];
        if (packetLength > (MAX_PAGE_SIZE + 1)) //RES + max. LEN
        {
          m_prg_api_p->Write2EventLog("Packet size exceeded buffer limit");
          return READ_ERR;
        }
        //read the rest of the packet
        if (m_fpga_p->SerialRead(&m_comm_buffer[3], 8 * (packetLength + 2)))
          return READ_ERR;
        //we have all data now
        checksum = 0;
        for (int i = 1; i <= (packetLength + 3); i++) //incl. CS
          checksum += m_comm_buffer[i];

        if (checksum)
        {
          m_prg_api_p->Write2EventLog("Checksum ERROR while executing READ_VALID_DATA cmd");
          return READ_ERR;
        }
        if (m_comm_buffer[3] == read_cmd)
        {
          memcpy(&m_srcdata_bp[address], &m_comm_buffer[4], packetLength - 1);
          if (!(m_DF_write_unit == 16))
          {
            if (block >= m_DF_block_nr)
              memset(&m_srcdata_bp[DF_MARKER_OFFSET + address], 0x00, packetLength - 1); //set MARKER for valid data -> then the image can be used for LFM jobs.
            else
              memset(&m_srcdata_bp[CF_MARKER_OFFSET + (address / 256)], 0x00, (packetLength - 1) / 256); //set MARKER for valid data -> then the image can be used for LFM jobs.
          }
          address += packetLength - 1;
        }
        else if (m_comm_buffer[3] == EXTENDED_READ_CMD_ERR)
        {
          if (m_comm_buffer[2] == 5) // a length value 5 and NACK points to blank data ----recored the blank area size
          {
            blankAreaSize = ((DWORD)m_comm_buffer[4] << 24) | ((DWORD)m_comm_buffer[5] << 16) | ((DWORD)m_comm_buffer[6] << 8) | (DWORD)m_comm_buffer[7];
            address += blankAreaSize;
            continue; //blank area contains invalid data, therefore skip it
          }
          else
          {
            PRINTF("C_RV40F:  Receive Data Frame - ACK Failed. Excpected ACK code: %Xh, Actual: %Xh\n", READ_CMD, m_comm_buffer[4]);
            return READ_ERR;
          }
        }
        else
        {
          PRINTF("C_RV40F:  Receive Data Frame - ACK Failed. Excpected ACK code: %Xh, Actual: %Xh\n", READ_CMD, m_comm_buffer[4]);
          return READ_ERR;
        }

      } while (address <= endaddress && read_stat == OPERATION_OK);

      // check for any system events
      if (m_prg_api_p->SysEvtChk())
        return HARDWARE_ERR; // O.C. or Adapter change - return immediately
    }                        // end of if 'sector' was to be programmed
  } while (++block < m_sector_quantity && read_stat == OPERATION_OK);

  return read_stat;
}

//*************************************************************************
//FUNCTION    Erase()
//ARGUMENTS   none
//RETURNS     DEV_STAT_E - device status enumeration
//METHOD      CHIP ERASE
//EXCEPTIONS --  none
//*************************************************************************
C_RV40F::DEV_STAT_E C_RV40F::Erase()
{
#if (ALG_DEBUG > 0)
  PRINTF("C_RV40F::Erase()\n"); // debug statements
#endif

  DWORD dataFlashSize;
  DWORD DF_startaddress_in_device, DF_endaddress_in_device;
  DWORD address, startaddress_in_device;
  bool fResetReq = false;
  SOCKET_STATUS_E skt_stat;
  BYTE skt_mask, failed_skt_mask;
  int nDUT, i;
  DEV_STAT_E erase_stat = OPERATION_OK;

  m_current_op_mode = DeviceOperation::ERASE;

  if (false == DeviceInit())
    return BLOCK_ERASE_ERR;

  if ((m_optionSupportedByDev & ICU_S) && (m_optionSelectedByUser & ICU_S))
  { //check whether ICU_S area is empty - not allowed!
    if (m_ICU_S_RegionSize && DF_IsAreaEmpty(m_devsectors_p[m_DF_block_nr].end_address + 1 - m_ICU_S_RegionSize, m_ICU_S_RegionSize))
    {
      m_prg_api_p->Write2EventLog("ICU-S requested but ICU-S area is not included in the data file -> Operation aborted.");
      m_prg_api_p->Write2EventLog("Include ICU-S area when you intend to use ICU-S feature.");
      return BLOCK_ERASE_ERR;
    }
  }

  WORD block = 0;
  do
  {
    // see if block has to be erased
    if (m_prg_api_p->GetSectorFlag(SectorOp::ERASE_SECTOR_OP, block))
    {
      if (block == m_option_data_block || block >= m_DF_block_nr)
        continue;

      startaddress_in_device = m_devsectors_p[block].begin_address;

      WriteCmdBuffer(ERASE_CMD, startaddress_in_device, 0);

      if (false == SendFrame(SOH, ETX, GetCmdBufferP(), 5))
        return WSM_BUSY_ERR;

      if (false == GetDataFrame(ETX, GetCmdBufferP(), CHECK_ST1, CHECK_ST1))
      {
        erase_stat = BLOCK_ERASE_ERR;
        PRINTF("C_RV40F::Erasing failed in block %d\n", block);
      }

      // check for any system events
      if (m_prg_api_p->SysEvtChk())
        return HARDWARE_ERR; // O.C. or Adapter change - return immediately
    }                        // end of if 'sector' was to be programmed
  } while (++block < m_sector_quantity && erase_stat == OPERATION_OK);

  // check if DF has to be erased
  block = m_DF_block_nr;
  do
  {
    if (m_prg_api_p->GetSectorFlag(SectorOp::ERASE_SECTOR_OP, block))
    {
      DF_startaddress_in_device = GetDFAdressInDevice(m_devsectors_p[block].begin_address);
      DF_endaddress_in_device = GetDFAdressInDevice(m_devsectors_p[block].end_address) - m_ICU_S_RegionSize;

      if (OPERATION_OK != Erase_DataFlash_Area(DF_startaddress_in_device, DF_endaddress_in_device))
        erase_stat = BLOCK_ERASE_ERR;

      if (m_ICU_S_RegionSize)
      {
        DF_startaddress_in_device = (GetDFAdressInDevice(m_devsectors_p[m_DF_block_nr].end_address) + 1) - m_ICU_S_RegionSize;
        DF_endaddress_in_device = GetDFAdressInDevice(m_devsectors_p[m_DF_block_nr].end_address);
        if (false == GetSockets_ICU_S_Status()) //skts with different validation status
        {
          //Collect data socket by socket
          failed_skt_mask = 0;
          WORD socket_mask = LM_Phapi::Get()->ActiveDUTMaskGet();
          m_current_op_mode = DeviceOperation::READ;
          for (nDUT = MAX_SOCKET_NUM - 1; nDUT >= 0; nDUT--)
          {
            // check only active sockets
            skt_stat = m_prg_api_p->SocketStatusGet(SocketNumChange(nDUT + 1));
            if (skt_stat == SOCKET_ENABLE)
            {
              // activate this one socket for reading
              // Note: ALL OTHER SOCKETS WILL BE DISABLED
              skt_mask = 1 << nDUT;
              LM_Phapi::Get()->SetSingleSktMode((WORD)(nDUT + 1));

              if (m_ICU_S_Status[nDUT] == ICU_S_RESPONSIVE_INVALID)
              {
                if (OPERATION_OK != Erase_DataFlash_Area(DF_startaddress_in_device, DF_endaddress_in_device))
                  failed_skt_mask |= skt_mask;
              }
              else
              {
                if ((m_ICU_S_Status[nDUT] == ICU_S_RESPONSIVE_VALID) &&  (!(m_optionSupportedByDev & ICU_S_ERASE_PROHIBITED)))
                {
                  if (OPERATION_OK != Erase_ICU_Area())
                  {
                    failed_skt_mask |= skt_mask;
                    erase_stat == BLOCK_ERASE_ERR;
                  }

                  if (erase_stat == OPERATION_OK)
                  {
                    m_ICU_S_Status[nDUT] == ICU_S_RESPONSIVE_INVALID;
                    fResetReq = true; //ICU-S protection is lifted after the next RESET
                  }
                } //end if((m_ICU_S_Status[nDUT] == ICU_S_RESPONSIVE_VALID) && (!(m_optionSupportedByDev & ICU_S_ERASE_PROHIBITED)))
              }

              if (failed_skt_mask & skt_mask)
                continue;
            } //-- OF if (skt_stat==SOCKET_ENABLE)
          }   //-- OF for (nDUT=MAX_SOCKET_NUM-1; nDUT>=0; nDUT--)
          // Re-Enable all enabled sockets
          LM_Phapi::Get()->SetGangSktMode(socket_mask);
          m_current_op_mode = DeviceOperation::ERASE;

          // disable the DUT
          if (CompareFailed(failed_skt_mask))
            if (!m_prg_api_p->MisCompare(DeviceOperation::ERASE, failed_skt_mask, 0, 0))
              erase_stat = BLOCK_ERASE_ERR;

        } //end of  if(false== GetSockets_ICU_S_Status())
        else
        {
          if (m_All_skts_ICU_S_INVALID == ICU_S_RESPONSIVE_INVALID)
          {

            if (OPERATION_OK != Erase_DataFlash_Area(DF_startaddress_in_device, DF_endaddress_in_device))
              erase_stat = BLOCK_ERASE_ERR;
          }
          else
          {
            if(!(m_optionSupportedByDev & ICU_S_ERASE_PROHIBITED))
            {
              if (OPERATION_OK != Erase_ICU_Area())
                erase_stat = BLOCK_ERASE_ERR;

              if (erase_stat == OPERATION_OK)
              {
                for (i = 0; i < MAX_SOCKET_NUM; i++)
                {
                  if (m_ICU_S_Status[i] == ICU_S_RESPONSIVE_VALID)
                  {
                    m_ICU_S_Status[i] = ICU_S_RESPONSIVE_INVALID; // set valid nDUT's to Invalid
                  }
                }
                fResetReq = true; //ICU-S protection is lifted after the next RESET
              }
            } //end if (!(m_optionSupportedByDev & ICU_S_ERASE_PROHIBITED))
          }   // end else if (m_All_skts_ICU_S_INVALID == ICU_S_RESPONSIVE_INVALID)
        }     // end else   if(false== GetSockets_ICU_S_Status())

      } //end of if(m_ICU_S_RegionSize)
    }   // end of if (m_prg_api_p->GetSectorFlag(SectorOp::ERASE_SECTOR_OP, block))
  } while (++block < m_sector_quantity && erase_stat == OPERATION_OK);

  //do options area last and only if necessary
  if (erase_stat == OPERATION_OK && m_fCfgClearCmdReq && m_prg_api_p->GetSectorFlag(SectorOp::ERASE_SECTOR_OP, m_option_data_block))
  {
    for (address = 0; address < OPBT_LENGTH; address++)
    {
      if (GetDataFromRam_8Bit(OPBT_OFFSET(0) + address, (DWORD)m_srcdata_bp) != 0xFF)
        break;
    }
    if (address == OPBT_LENGTH)
    { //abort if OPBT area empty. Otherwise device could be rendered useless
      m_prg_api_p->Write2EventLog("OPBT0-7 settings missed. Erasing aborted to prevent unprogrammed option bytes!");
      return BLOCK_ERASE_ERR;
    }

    //erase options area
    m_current_CMD = CONFIG_CLEAR_CMD;
    if (false == SendFrame(SOH, ETX, &CONFIG_CLEAR_CMD, 1))
      return WSM_BUSY_ERR;

    if (false == GetDataFrame(ETX, &m_current_CMD, CHECK_ST1, CHECK_ST1, 0, LONG_DELAY))
      erase_stat = BLOCK_ERASE_ERR;
    m_fCfgClearCmdReq = false;
  } //-- OF if (m_prg_api_p->GetSectorFlag(SectorOp::ERASE_SECTOR_OP, m_option_data_block))

  if (fResetReq)
    ResetToProgrammingMode();

  // check for any system events
  if (m_prg_api_p->SysEvtChk())
    return HARDWARE_ERR; // O.C. or Adapter change - return immediately

  return erase_stat;
}

////////////////////////////////////////////////////////////////////////////////
//          Program ()
// Inputs:
//
// Return:
//
// METHOD:
////////////////////////////////////////////////////////////////////////////////
C_RV40F::DEV_STAT_E C_RV40F::Program()
{
#if (ALG_DEBUG > 0)
  PRINTF("C_RV40F::Program()\n"); // debug statements
#endif

  volatile BYTE *srcbase = (volatile BYTE *)m_srcdata_bp;
  DEV_STAT_E prog_stat = OPERATION_OK;

  DWORD startaddress, endaddress;
  DWORD startaddress_in_device, endaddress_in_device, address;
  BYTE frameEndType;
  DWORD areaSize;
  WORD blockSize;
  BYTE socket_stat;
  SOCKET_STATUS_E skt_stat;
  BYTE skt_mask, failed_skt_mask;
  int nDUT;

  m_current_op_mode = DeviceOperation::PROGRAM;

  if (false == DeviceInit())
    return PROGRAM_ERR;

  WORD block = 0;
  do
  {
    // see if block has to be programmed
    if (m_prg_api_p->GetSectorFlag(SectorOp::PROGRAM_SECTOR_OP, block))
    {
      if (block == m_option_data_block)
      {
        if ((m_optionSelectedByUser & ID_CODE) && (m_optionSelectedByUser & ID_AUTH) == 0)
        { //simply program the ID code - has no side effects
          if (OPERATION_OK != Prog_IDCode(IDCODE_SET_CMD))
            return PROGRAM_ERR;
        }
        continue; //rest of the options will be programmed in secure procedure
      }
      if (block >= m_DF_block_nr)
        continue; //dataflash handled separately

      startaddress = m_devsectors_p[block].begin_address;
      endaddress = m_devsectors_p[block].end_address;

      startaddress_in_device = startaddress;
      endaddress_in_device = endaddress;
      if (m_fCF_filled0xFF == false)
      {
        if (CF_IsAreaEmpty(startaddress, endaddress - startaddress + 1))
          continue; //skip
        if (CF_IsAreaFragmented(startaddress, endaddress - startaddress + 1))
        { //non-homogeneous CF area, it contains gaps
          //program areas with data only
          address = startaddress;
          do
          {
            //search for words to be programmed (marked with 0x00 in the MARKER area)
            while (GetDataFromRam_8Bit(address / MIN_PAGE_SIZE + CF_MARKER_OFFSET, (DWORD)m_srcdata_bp) && address <= endaddress)
              address += MIN_PAGE_SIZE;
            if (address > endaddress)
              break; //reached end of CF sector
            startaddress = address;
            do
            {
              address += MIN_PAGE_SIZE;
              if (0 == (address % MAX_PAGE_SIZE))
                break;
            } while (GetDataFromRam_8Bit(address / MIN_PAGE_SIZE + CF_MARKER_OFFSET, (DWORD)m_srcdata_bp) == 0);

            startaddress_in_device = startaddress;
            endaddress_in_device = address - 1;
            areaSize = endaddress_in_device - startaddress_in_device + 1;

            WriteCmdBuffer(PROGRAM_CMD, startaddress_in_device, endaddress_in_device);

            if (false == SendFrame(SOH, ETX, GetCmdBufferP(), 9))
              return WSM_BUSY_ERR;

            if (false == GetDataFrame(ETX, GetCmdBufferP(), CHECK_ST1, CHECK_ST1))
              return PROGRAM_ERR;

            if (false == SendFrame(SOD, ETX, (BYTE *)(srcbase + startaddress), (WORD)areaSize, PROGRAM_CMD))
              return WSM_BUSY_ERR;

            if (false == GetDataFrame(ETX, GetCmdBufferP(), CHECK_ST1, CHECK_ST1))
            {
              prog_stat = PROGRAM_ERR;
              PRINTF("C_RV40F::Programming fail at address 0x%X\n", startaddress);
            }
          } while (address <= endaddress && prog_stat == OPERATION_OK);

          continue;
        }
      } //-- OF if (m_fCF_filled0xFF == false)

      areaSize = endaddress_in_device - startaddress_in_device + 1;
      if (areaSize < MAX_PAGE_SIZE)
        blockSize = (WORD)areaSize;
      else
        blockSize = MAX_PAGE_SIZE;

      WriteCmdBuffer(PROGRAM_CMD, startaddress_in_device, endaddress_in_device);

      if (false == SendFrame(SOH, ETX, GetCmdBufferP(), 9))
        return WSM_BUSY_ERR;

      if (false == GetDataFrame(ETX, GetCmdBufferP(), CHECK_ST1, CHECK_ST1))
        return PROGRAM_ERR;

      for (address = startaddress;
           address <= endaddress && prog_stat == OPERATION_OK;
           address += blockSize)
      {
        if (address + blockSize > endaddress) //last data block?
          frameEndType = ETX;                 //end of all data
        else
          frameEndType = ETB; //end of block

        if (false == SendFrame(SOD, frameEndType, (BYTE *)(srcbase + address), blockSize, PROGRAM_CMD))
          return WSM_BUSY_ERR;

        if (false == GetDataFrame(ETX, GetCmdBufferP(), CHECK_ST1, CHECK_ST1))
        {
          prog_stat = PROGRAM_ERR;
          PRINTF("C_RV40F::Programming fail at 0x%X\n", address);
        }
      }

      // check for any system events
      if (m_prg_api_p->SysEvtChk())
        return HARDWARE_ERR; // O.C. or Adapter change - return immediately
    }                        // end of if 'sector' was to be programmed
  } while (++block < m_sector_quantity && prog_stat == OPERATION_OK);

  if (prog_stat != OPERATION_OK)
    return prog_stat;

  //data flash handling
  block = m_DF_block_nr;

  if ((m_optionSupportedByDev & ICU_S) && (!(GetSockets_ICU_S_Status())))
  {
    if (m_prg_api_p->GetSectorFlag(SectorOp::PROGRAM_SECTOR_OP, block))
    {
      //Collect data socket by socket
      failed_skt_mask = 0;
      WORD socket_mask = LM_Phapi::Get()->ActiveDUTMaskGet();
      m_current_op_mode = DeviceOperation::READ;
      for (nDUT = MAX_SOCKET_NUM - 1; nDUT >= 0; nDUT--)
      {
        // check only active sockets
        skt_stat = m_prg_api_p->SocketStatusGet(SocketNumChange(nDUT + 1));
        if (skt_stat == SOCKET_ENABLE)
        {
          // activate this one socket for reading
          // Note: ALL OTHER SOCKETS WILL BE DISABLED
          skt_mask = 1 << nDUT;
          LM_Phapi::Get()->SetSingleSktMode((WORD)(nDUT + 1));
          do
          {
            if (m_ICU_S_Status[nDUT] == ICU_S_RESPONSIVE_INVALID)
            {
              if (OPERATION_OK != Program_DataFlash_Area(block, 0))
                failed_skt_mask |= skt_mask;
            } //-- OF if(m_ICU_S_Status[nDUT])
            else if (m_ICU_S_Status[nDUT] == ICU_S_RESPONSIVE_VALID)
            {
              if (OPERATION_OK != Program_DataFlash_Area(block, m_ICU_S_RegionSize))
                failed_skt_mask |= skt_mask;
            }
          } while (++block < m_sector_quantity && prog_stat == OPERATION_OK);

          if (failed_skt_mask & skt_mask)
            continue;
        } //-- OF if (skt_stat==SOCKET_ENABLE)
      }   //-- OF for (nDUT=MAX_SOCKET_NUM-1; nDUT>=0; nDUT--)
      // Re-Enable all enabled sockets
      LM_Phapi::Get()->SetGangSktMode(socket_mask);
      m_current_op_mode = DeviceOperation::PROGRAM;

      // disable the DUT
      if (CompareFailed(failed_skt_mask))
        if (!m_prg_api_p->MisCompare(DeviceOperation::PROGRAM, failed_skt_mask, 0, 0))
          return PROGRAM_ERR;
    } //-- OF if (m_prg_api_p->GetSectorFlag(SectorOp::PROGRAM_SECTOR_OP, block))
  }   // enf of if ((m_optionSupportedByDev & ICU_S) && (!(GetSockets_ICU_S_Status())))
  else
  {
    do
    {
      if (m_prg_api_p->GetSectorFlag(SectorOp::PROGRAM_SECTOR_OP, block))
      {
        if (m_All_skts_ICU_S_INVALID == 1)
        {
          if (OPERATION_OK != Program_DataFlash_Area(block, 0))
            return PROGRAM_ERR;
        }
        else
        {
          if (OPERATION_OK != Program_DataFlash_Area(block, m_ICU_S_RegionSize))
            return PROGRAM_ERR;
        }
      } //-- OF if 'sector' was to be programmed
    } while (++block < m_sector_quantity && prog_stat == OPERATION_OK);

  } //end of else if ((m_optionSupportedByDev & ICU_S) && (!(GetSockets_ICU_S_Status())))

  // check for any system events
  if (m_prg_api_p->SysEvtChk())
    return HARDWARE_ERR; // O.C. or Adapter change - return immediately

  return prog_stat;
} //C_RV40F::Program()

////////////////////////////////////////////////////////////////////////////////
//FUNCTION    Verify()
//ARGUMENTS
//RETURNS     DEV_STAT_E - device status enumeration
//METHOD      verify device data towards image on a per block basis
//EXCEPTIONS --  none
////////////////////////////////////////////////////////////////////////////////
C_RV40F::DEV_STAT_E C_RV40F::Verify()
{
#if (ALG_DEBUG > 0)
  PRINTF("C_RV40F::Verify()\n");
#endif

  const CURRENT_OP_STATUS *current_op_stat_p = m_prg_api_p->CurrentOpStatusGet();
  volatile BYTE *srcbase = (volatile BYTE *)m_srcdata_bp;

  DWORD startaddress, endaddress;
  DWORD startaddress_in_device, endaddress_in_device;
  DWORD address;
  BYTE frameEndType;
  BYTE data_buffer[16];
  WORD block;
  WORD blockSize;
  DWORD areaSize;
  DWORD checkSum; //CRC32
  DEV_STAT_E verify_stat = OPERATION_OK;
  SOCKET_STATUS_E skt_stat;
  BYTE skt_mask, failed_skt_mask;
  int nDUT;

  m_current_op_mode = current_op_stat_p->operation;
  DEV_OP_E saved_current_op_mode = m_current_op_mode;

  if (false == DeviceInit())
    return VERIFY_ERR;

  //blockwise operation
  block = 0;
  do
  {
    // see if block has to be programmed
    if (m_prg_api_p->GetSectorFlag(SectorOp::PROGRAM_SECTOR_OP, block))
    {
      if (block == m_option_data_block)
      { //special handling
        if (m_current_op_mode == STAND_ALONE_VERIFY || m_current_op_mode == READ_VERIFY)
        {
          if (OPERATION_OK != RV_DeviceConfig())
          {
            return VERIFY_ERR;
          }
          continue; //go forward with next block
        }
        if ((m_optionSelectedByUser & ID_CODE) && (m_optionSelectedByUser & ID_AUTH) == 0)
        {
          if (OPERATION_OK != RV_IDCode())
          {
            return VERIFY_ERR;
          }
        }
        continue; //go forward with next block
      }
      if (block >= m_DF_block_nr)
        continue; //dataflash handled separately
      startaddress = m_devsectors_p[block].begin_address;
      endaddress = m_devsectors_p[block].end_address;

      startaddress_in_device = startaddress;
      endaddress_in_device = endaddress;
      if (m_fCF_filled0xFF == false)
      {
        if (CF_IsAreaEmpty(startaddress, endaddress - startaddress + 1))
          continue; //skip
        if (CF_IsAreaFragmented(startaddress, endaddress - startaddress + 1))
        { //non-homogeneous CF area, it contains gaps
          //verify areas with data only
          address = startaddress;
          do
          {
            //search for words to be programmed (marked with 0x00 in the MARKER area)
            while (GetDataFromRam_8Bit(address / MIN_PAGE_SIZE + CF_MARKER_OFFSET, (DWORD)m_srcdata_bp) && address <= endaddress)
              address += MIN_PAGE_SIZE;
            if (address > endaddress)
              break; //reached end of CF sector
            startaddress = address;
            do
            {
              address += MIN_PAGE_SIZE;
              if (0 == (address % MAX_PAGE_SIZE))
                break;
            } while (GetDataFromRam_8Bit(address / MIN_PAGE_SIZE + CF_MARKER_OFFSET, (DWORD)m_srcdata_bp) == 0);

            startaddress_in_device = startaddress;
            endaddress_in_device = address - 1;
            areaSize = endaddress_in_device - startaddress_in_device + 1;

            verify_stat = VerifyArea(startaddress, startaddress_in_device, areaSize);
          } while (address <= endaddress && verify_stat == OPERATION_OK);

          continue;
        }
      } //-- OF if (m_fCF_filled0xFF == false)

      if (m_VerifyType == CHECKSUM_VERIFY)
        m_current_CMD = CRC_CMD;
      else
        m_current_CMD = VERIFY_CMD;
      WriteCmdBuffer(m_current_CMD, startaddress_in_device, endaddress_in_device);

      if (false == SendFrame(SOH, ETX, GetCmdBufferP(), 9))
        return WSM_BUSY_ERR;
      if (false == GetDataFrame(ETX, GetCmdBufferP(), CHECK_ST1, CHECK_ST1))
        return VERIFY_ERR;

      checkSum = 0xFFFFFFFF; //init checksum
      //verify loop
      for (address = startaddress;
           address <= endaddress && verify_stat == OPERATION_OK;
           address += MAX_PAGE_SIZE)
      {
        if (m_current_CMD == CRC_CMD)
        {
          checkSum = update_crc(checkSum, (BYTE *)(srcbase + address), MAX_PAGE_SIZE); //CRC32
        }
        else
        {
          if (address + MAX_PAGE_SIZE > endaddress) //last data block?
            frameEndType = ETX;                     //end of all data
          else
            frameEndType = ETB; //end of block

          if (false == SendFrame(SOD, frameEndType, (BYTE *)(srcbase + address), MAX_PAGE_SIZE, VERIFY_CMD))
            return WSM_BUSY_ERR;

          if (false == GetDataFrame(ETX, GetCmdBufferP(), CHECK_ST1, CHECK_ST1))
          {
            verify_stat = VERIFY_ERR;
            PRINTF("C_RV40F::Verify fail at 0x%X\n", address);
          }
        } //if (m_current_CMD ==
      }   //-- OF for (address = startaddress;

      if (m_current_CMD == CRC_CMD)
      {
        if (false == SendFrame(SOD, ETX, GetCmdBufferP(), 1)) //reverse ACK
          return VERIFY_ERR;
        data_buffer[0] = (BYTE)(checkSum >> 24);
        data_buffer[1] = (BYTE)(checkSum >> 16);
        data_buffer[2] = (BYTE)(checkSum >> 8);
        data_buffer[3] = (BYTE)(checkSum >> 0);
        if (false == GetDataFrame(ETX, data_buffer, 4, 4, CRC_CMD))
        {
          verify_stat = VERIFY_ERR;
          PRINTF("C_RV40F::CRC check failed in block %d\n", block);
        }
      }

      // check for any system events
      if (m_prg_api_p->SysEvtChk())
        return HARDWARE_ERR; // O.C. or Adapter change - return immediately
    }                        // end of if 'sector' was to be programmed
  } while (++block < m_sector_quantity && verify_stat == OPERATION_OK);

  if (verify_stat != OPERATION_OK)
    return verify_stat;

  //data flash verify
  block = m_DF_block_nr;
  if ((m_optionSupportedByDev & ICU_S) && (!(GetSockets_ICU_S_Status())))
  {
    if (m_prg_api_p->GetSectorFlag(SectorOp::PROGRAM_SECTOR_OP, block))
    {
      //Collect data socket by socket
      failed_skt_mask = 0;
      WORD socket_mask = LM_Phapi::Get()->ActiveDUTMaskGet();
      m_current_op_mode = DeviceOperation::READ;
      for (nDUT = MAX_SOCKET_NUM - 1; nDUT >= 0; nDUT--)
      {
        // check only active sockets
        skt_stat = m_prg_api_p->SocketStatusGet(SocketNumChange(nDUT + 1));
        if (skt_stat == SOCKET_ENABLE)
        {
          // activate this one socket for reading
          // Note: ALL OTHER SOCKETS WILL BE DISABLED
          skt_mask = 1 << nDUT;
          LM_Phapi::Get()->SetSingleSktMode((WORD)(nDUT + 1));
          do
          {

            if (m_ICU_S_Status[nDUT] == ICU_S_RESPONSIVE_INVALID)
            {
              if (OPERATION_OK != Verify_DataFlash_Area(block, 0))
                failed_skt_mask |= skt_mask;
            } //-- OF if(m_ICU_S_Status[nDUT])
            else if (m_ICU_S_Status[nDUT] == ICU_S_RESPONSIVE_VALID)
            {
              if (OPERATION_OK != Verify_DataFlash_Area(block, m_ICU_S_RegionSize))
                failed_skt_mask |= skt_mask;
            }

          } while (++block < m_sector_quantity && verify_stat == OPERATION_OK);

          if (failed_skt_mask & skt_mask)
            continue;

        } //-- OF if (skt_stat==SOCKET_ENABLE)
      }   //-- OF for (nDUT=MAX_SOCKET_NUM-1; nDUT>=0; nDUT--)
      // Re-Enable all enabled sockets
      LM_Phapi::Get()->SetGangSktMode(socket_mask);
      m_current_op_mode = saved_current_op_mode;

      // disable the DUT
      if (CompareFailed(failed_skt_mask))
        if (!m_prg_api_p->MisCompare(DeviceOperation::VERIFY, failed_skt_mask, 0, 0))
          return VERIFY_ERR;
    } //-- OF if (m_prg_api_p->GetSectorFlag(SectorOp::PROGRAM_SECTOR_OP, block))
  }   //end of if ((m_optionSupportedByDev & ICU_S) && (!(GetSockets_ICU_S_Status())))
  else
  {
    do
    {
      if (m_prg_api_p->GetSectorFlag(SectorOp::PROGRAM_SECTOR_OP, block))
      {
        if (m_All_skts_ICU_S_INVALID == 1)
        {
          if (OPERATION_OK != Verify_DataFlash_Area(block, 0))
            return VERIFY_ERR;
        }
        else
        {
          if (OPERATION_OK != Verify_DataFlash_Area(block, m_ICU_S_RegionSize))
            return VERIFY_ERR;
        }

      } // end of if 'sector' selected
    } while (++block < m_sector_quantity && verify_stat == OPERATION_OK);
  }

  // check for any system events
  if (m_prg_api_p->SysEvtChk())
    return HARDWARE_ERR; // O.C. or Adapter change - return immediately

  return verify_stat;
} //C_RV40F::Verify()

////////////////////////////////////////////////////////////////////////////////
//          C_RV40F::Secure() ()
// Inputs:
//
// Return:
//
// METHOD:
////////////////////////////////////////////////////////////////////////////////
C_RV40F::DEV_STAT_E C_RV40F::Secure()
{
#if (ALG_DEBUG > 0)
  PRINTF("C_RV40F::Secure()\n"); // debug statements
#endif

  DWORD addrCnt;
  BYTE data_buffer[PROT_LENGTH];
  BYTE OPBTEX_buffer[EXT_OPBT_LENGTH];
  volatile BYTE *srcbase = (volatile BYTE *)m_srcdata_bp;
  bool optionDataBlockHasSN;
  DEV_STAT_E secure_stat = OPERATION_OK;
  SOCKET_STATUS_E skt_stat;
  BYTE skt_mask, failed_skt_mask;
  int nDUT;

  m_current_op_mode = DeviceOperation::SECURE;
  //This procedure is called last, thus it is in charge of
  // - setting of the (extended)option bytes
  // - setting of the Lock Bits for the sectors
  // - setting of the ICU mode (ICU-S validation)
  // - setting of the ID CODE (AUTH) or protection bits (06/07/2018 - ID CODE (but not ID CODE for AUTH!) will be programmed during Program() - allowing SNs in ID CODE)
  // - setting of the OTP bits for the sectors
  // - setting of "Serial Programming Disable" (Caution: SPD makes any further access by the programmer impossible!)

  if (!m_prg_api_p->GetSectorFlag(SectorOp::BLANK_CHECK_OP, m_option_data_block))
    return secure_stat; //nothing to do

  optionDataBlockHasSN = true;
  if (m_prg_api_p->GetSectorFlag(SectorOp::BLANK_CHECK_OP, m_option_data_block) && m_prg_api_p->GetSectorFlag(SectorOp::PROGRAM_SECTOR_OP, m_option_data_block))
    optionDataBlockHasSN = false;

  if (m_optionSelectedByUser & OPBT)
  {
    //program Option-bytes (OPBT)
    WriteCmdBuffer(0, OPTION_SET_CMD);
    if (false == SendFrame(SOH, ETX, (BYTE *)(srcbase + OPBT_OFFSET(0)), OPBT_LENGTH, OPTION_SET_CMD))
      return SECURE_ERR;
    if (false == GetDataFrame(ETX, GetCmdBufferP(), CHECK_ST1, CHECK_ST1))
      return SECURE_ERR;

    //verify
    if (OPERATION_OK != RV_OPBT())
      return SECURE_ERR;
  }

  if ((m_optionSupportedByDev & OPBTEX) && (m_optionSelectedByUser & OPBTEX))
  {
    if (m_optionSupportedByDev & ICU_M)
    { //OPBT8..12 related to ICU-M setup
      WriteCmdBuffer(0, EXTENDED_OPTION1_SET_CMD);
      if (false == SendFrame(SOH, ETX, (BYTE *)(srcbase + OPBT_OFFSET(12)), 4, EXTENDED_OPTION1_SET_CMD))
        return SECURE_ERR;
      if (false == GetDataFrame(ETX, GetCmdBufferP(), CHECK_ST1, CHECK_ST1))
        return SECURE_ERR;
      //prepare data for Extended Option Byte 2 Set cmd
      if (GetDataFromRam_8Bit(OPBT_OFFSET(8) + 3, (DWORD)m_srcdata_bp) & 0xF0)
        OPBTEX_buffer[0] = 0xFF;
      else
        OPBTEX_buffer[0] = 0x00;

      for (addrCnt = 0; addrCnt < 4; addrCnt++)
      {
        OPBTEX_buffer[1 + addrCnt] = GetDataFromRam_8Bit(OPBT_OFFSET(10) + addrCnt, (DWORD)m_srcdata_bp);
        OPBTEX_buffer[5 + addrCnt] = GetDataFromRam_8Bit(OPBT_OFFSET(11) + addrCnt, (DWORD)m_srcdata_bp);
        OPBTEX_buffer[9 + addrCnt] = GetDataFromRam_8Bit(OPBT_OFFSET(9) + addrCnt, (DWORD)m_srcdata_bp);
      }

      if (GetDataFromRam_8Bit(OPBT_OFFSET(8) + 2, (DWORD)m_srcdata_bp) & 0xF0)
        OPBTEX_buffer[13] = 0xFF;
      else
        OPBTEX_buffer[13] = 0x00;

      if (GetDataFromRam_8Bit(OPBT_OFFSET(8) + 1, (DWORD)m_srcdata_bp) & 0xF0)
        OPBTEX_buffer[14] = 0xFF;
      else
        OPBTEX_buffer[14] = 0x00;

      //Caution: with this command ICU-M mode is activated. After the next Reset the device can not be accessed possibly.
      WriteCmdBuffer(0, EXTENDED_OPTION2_SET_CMD);
      if (false == SendFrame(SOH, ETX, &OPBTEX_buffer[0], 15, EXTENDED_OPTION2_SET_CMD))
        return SECURE_ERR;
      if (false == GetDataFrame(ETX, GetCmdBufferP(), CHECK_ST1, CHECK_ST1))
        return SECURE_ERR;
    }
    else
    { //ICU-S Option - not tested yet
      WriteCmdBuffer(0, ICU_S_OPTION_SET_CMD);
      if (false == SendFrame(SOH, ETX, (BYTE *)(srcbase + OPBT_OFFSET(9)), 4, ICU_S_OPTION_SET_CMD))
        return SECURE_ERR;
      if (false == GetDataFrame(ETX, GetCmdBufferP(), CHECK_ST1, CHECK_ST1))
        return SECURE_ERR;
    }
  } //-- OF if ((m_optionSupportedByDev & OPBTEX) && (m_optionSelectedByUser & OPBTEX))

  if ((m_optionSupportedByDev & LB) && (m_optionSelectedByUser & LB))
  { //Programming and erasure by self programming of an area for which the lock bit is set and the lock bit function is enabled are prohibited.
    WriteCmdBuffer(0, LOCKBIT_SET_CMD);
    if (false == SendFrame(SOH, ETX, (BYTE *)(srcbase + LOCK_BIT_OFFSET), LB_LENGTH, LOCKBIT_SET_CMD))
      return SECURE_ERR;
    if (false == GetDataFrame(ETX, GetCmdBufferP(), CHECK_ST1, CHECK_ST1))
      return SECURE_ERR;

    //verify
    if (OPERATION_OK != RV_BlockProtBits(LOCKBIT_GET_CMD))
      return SECURE_ERR;
  } //--if (m_fLockBitsSupported)

  if ((m_optionSupportedByDev & ICU_S) && (m_optionSelectedByUser & ICU_S))
  {
    if (GetSockets_ICU_S_Status())
    {
      if (m_All_skts_ICU_S_INVALID == ICU_S_RESPONSIVE_INVALID)
      {
        WriteCmdBuffer(0, ICU_S_VALIDATE_CMD);
        if (false == SendFrame(SOH, ETX, GetCmdBufferP(), 1))
          return SECURE_ERR;
        if (false == GetDataFrame(ETX, GetCmdBufferP(), CHECK_ST1, CHECK_ST1))
          return SECURE_ERR;
      }
    }
    else
    {
      //Collect data socket by socket
      failed_skt_mask = 0;
      WORD socket_mask = LM_Phapi::Get()->ActiveDUTMaskGet();
      m_current_op_mode = DeviceOperation::READ;
      for (nDUT = MAX_SOCKET_NUM - 1; nDUT >= 0; nDUT--)
      {
        // check only active sockets
        skt_stat = m_prg_api_p->SocketStatusGet(SocketNumChange(nDUT + 1));
        if ((skt_stat == SOCKET_ENABLE) && (m_ICU_S_Status[nDUT] == ICU_S_RESPONSIVE_INVALID))
        {
          // activate this one socket for reading
          // Note: ALL OTHER SOCKETS WILL BE DISABLED
          skt_mask = 1 << nDUT;
          LM_Phapi::Get()->SetSingleSktMode((WORD)(nDUT + 1));

          WriteCmdBuffer(0, ICU_S_VALIDATE_CMD);
          if (false == SendFrame(SOH, ETX, GetCmdBufferP(), 1))
          {
            failed_skt_mask |= skt_mask;
            secure_stat = SECURE_ERR;
          }
          if (false == GetDataFrame(ETX, GetCmdBufferP(), CHECK_ST1, CHECK_ST1))
          {
            failed_skt_mask |= skt_mask;
            secure_stat = SECURE_ERR;
          }
        } //-- OF if (skt_stat==SOCKET_ENABLE)
      }   //-- OF for (nDUT=MAX_SOCKET_NUM-1; nDUT>=0; nDUT--)
      // Re-Enable all enabled sockets
      LM_Phapi::Get()->SetGangSktMode(socket_mask);
      m_current_op_mode = DeviceOperation::SECURE;

      // disable the DUT
      if (CompareFailed(failed_skt_mask))
        if (!m_prg_api_p->MisCompare(DeviceOperation::SECURE, failed_skt_mask, 0, 0))
          return SECURE_ERR;
    }
  }

  if ((m_optionSelectedByUser & ID_AUTH) == 0)
  { //command protection mode
    //prepare security data
    //security flags
    data_buffer[0] = ~GetDataFromRam_8Bit(PROT_OFFSET, (DWORD)m_srcdata_bp); //bits are inverted
    if (data_buffer[0] != 0x00 && data_buffer[0] != 0xFF)
    {
      WriteCmdBuffer(0, PROTECTION_SET_CMD);
      if (false == SendFrame(SOH, ETX, data_buffer, PROT_LENGTH, PROTECTION_SET_CMD))
        return SECURE_ERR;
      if (false == GetDataFrame(ETX, GetCmdBufferP(), CHECK_ST1, CHECK_ST1))
        return SECURE_ERR;

      // check for any system events
      if (m_prg_api_p->SysEvtChk())
        PRINTF(" OC @ line %d\n", __LINE__);

      //verify
      if (OPERATION_OK != RV_ProtBits())
        return SECURE_ERR;
    }
  }

  if ((m_optionSelectedByUser & ID_AUTH) && (optionDataBlockHasSN == false))
  { //when ID_AUTH is programmed, after the next RESET the access must be authenticated w/ the correct ID
    if (OPERATION_OK != Prog_IDCode(ID_AUTH_SET_CMD))
      return SECURE_ERR;
  }

  if ((m_optionSupportedByDev & OTP) && (m_optionSelectedByUser & OTP))
  { //Caution: when OTP bits are set, erasing of the appopriate blocks AND option area are prohibited!

    WriteCmdBuffer(0, OTP_SET_CMD);
    if (false == SendFrame(SOH, ETX, (BYTE *)(srcbase + OTP_BIT_OFFSET), LB_LENGTH, OTP_SET_CMD))
      return SECURE_ERR;
    if (false == GetDataFrame(ETX, GetCmdBufferP(), CHECK_ST1, CHECK_ST1))
      return SECURE_ERR;

    //verify
    if (OPERATION_OK != RV_BlockProtBits(OTP_GET_CMD))
      return SECURE_ERR;
  } //--if (m_fOtpBitsSupported)

  if (m_optionSelectedByUser & SPD)
  {
    WriteCmdBuffer(0, SP_DISABLE_CMD);
    if (false == SendFrame(SOH, ETX, GetCmdBufferP(), 1))
      return SECURE_ERR;
    if (false == GetDataFrame(ETX, GetCmdBufferP(), CHECK_ST1, CHECK_ST1))
      return SECURE_ERR;
    PRINTF("C_RV40F::Secure() - SPD set!\n");
  }

  return secure_stat;
} //C_RV40F::Secure()
//-- END C_RV40F definitions

/////////////////////////////////////////////////////
//Test Cases (new features/RPI):                   //
//LockBit: sector 0          : X                   //
//         sector > 0x800000 : X (R7F701503)       //
//         sector UB         : X                   //
//OTP                        : - (just SIM)        //
//ID                         : X                   //
//ID AUTH                    : X                   //
//ICU-S                      : X (R7F701054)       //
//ICU-M: (OPBT8..12)         : X (R7F701503)       //
//SPD (device damage!)       : X (R7F701054)       //
//                                                 //
//Verify job                 : X                   //
//Load from master           : X                   //
/////////////////////////////////////////////////////

//-- BEGIN C_RV40F_P1XC definitions
// - supports CC-Cube/P1x-C device series (Bosch specific). Based on spec EETS-ER-0047-0.4
/////////////////////////////////////////////////////////////////////////////////////////
//          C_RV40F_P1XC ctor ()
// Inputs:
//
// Return:
//
// METHOD:
////////////////////////////////////////////////////////////////////////////////
C_RV40F_P1XC::C_RV40F_P1XC(DEVPARMS *dparms_p, DEVPINS *dpins_p, DEVSECTORS *dsectors_p,
                           BYTE opfreq_parm, PRM_T *dprm_p, ALG_CODE *tBoot_code_p)
    : C_RV40F(dparms_p, dpins_p, dsectors_p, opfreq_parm, dprm_p, tBoot_code_p)
{
  //empty
} //C_RV40F_P1XC ctor()

/*******************************************************************************
*         Initialize ()
* Inputs:   
*       
* Return:   
*       
* METHOD:     
*******************************************************************************/
void C_RV40F_P1XC::Initialize(void)
{
  int i;
  WORD SFM_id_len;

  // call base class impplementation
  C_RV40F::Initialize();

  //sanity checks
  if (m_optionSupportedByDev & CFG_WRITE)
  {
    if (GetDataFromRam_32Bit(P1XC_OPBT1_OFFSET, (DWORD)m_srcdata_bp) != 0xFFFFFFFF)
      if (GetDataFromRam_32Bit(P1XC_RSTVCT_OFFSET, (DWORD)m_srcdata_bp) != 0xFFFFFFFF)
        m_optionSelectedByUser |= CFG_WRITE;
  }

  m_id_len = 32;        //[bytes]
  m_signature_len = 72; //[bytes]

  //Initialize special data
  //let's get the required data
  SFM_id_len = 16; //only 16 bytes per SFM field

  char temp_buf[33];
  char *ret_val = temp_buf;
  if (!m_prg_api_p->SpecFeatureParmGet("ID Code", &ret_val))
  {
    strcat(msgbuff, "Parameter <<ID Code>> missed, check Database !!!");
    m_prg_api_p->ThrowException(PARAM_UNAVAILABLE_ERR_S, __LINE__, __file, msgbuff);
  }
  for (i = 0; i < SFM_id_len; i++)
  {
    m_IDCODE_buffer[i] = ret_val[SFM_id_len - 1 - i];
  }

  if (!m_prg_api_p->SpecFeatureParmGet("ID Code (16..31)", &ret_val))
  {
    strcat(msgbuff, "Parameter <<ID Code (16..31)>> missed, check Database !!!");
    m_prg_api_p->ThrowException(PARAM_UNAVAILABLE_ERR_S, __LINE__, __file, msgbuff);
  }
  for (i = 0; i < SFM_id_len; i++)
  {
    m_IDCODE_buffer[16 + i] = ret_val[SFM_id_len - 1 - i];
  }

  if (!m_prg_api_p->SpecFeatureParmGet("CFID Code (0..15)", &ret_val))
  {
    strcat(msgbuff, "Parameter <<CFID Code (0..15)>> missed, check Database !!!");
    m_prg_api_p->ThrowException(PARAM_UNAVAILABLE_ERR_S, __LINE__, __file, msgbuff);
  }
  for (i = 0; i < SFM_id_len; i++)
  {
    m_IDCODE_buffer[32 + i] = ret_val[SFM_id_len - 1 - i];
  }

  if (!m_prg_api_p->SpecFeatureParmGet("CFID Code (16..31)", &ret_val))
  {
    strcat(msgbuff, "Parameter <<CFID Code (16..31)>> missed, check Database !!!");
    m_prg_api_p->ThrowException(PARAM_UNAVAILABLE_ERR_S, __LINE__, __file, msgbuff);
  }
  for (i = 0; i < SFM_id_len; i++)
  {
    m_IDCODE_buffer[48 + i] = ret_val[SFM_id_len - 1 - i];
  }

  if (!m_prg_api_p->SpecFeatureParmGet("DFID Code (0..15)", &ret_val))
  {
    strcat(msgbuff, "Parameter <<DFID Code (0..15)>> missed, check Database !!!");
    m_prg_api_p->ThrowException(PARAM_UNAVAILABLE_ERR_S, __LINE__, __file, msgbuff);
  }
  for (i = 0; i < SFM_id_len; i++)
  {
    m_IDCODE_buffer[64 + i] = ret_val[SFM_id_len - 1 - i];
  }

  if (!m_prg_api_p->SpecFeatureParmGet("DFID Code (16..31)", &ret_val))
  {
    strcat(msgbuff, "Parameter <<DFID Code (16..31)>> missed, check Database !!!");
    m_prg_api_p->ThrowException(PARAM_UNAVAILABLE_ERR_S, __LINE__, __file, msgbuff);
  }
  for (i = 0; i < SFM_id_len; i++)
  {
    m_IDCODE_buffer[80 + i] = ret_val[SFM_id_len - 1 - i];
  }

  if (m_prg_api_p->JobFlagGet(AlgorithmTypes::DEVICE_ERASE_FLAG))
  { //Programming job
    m_ID_buffer_p = &m_IDCODE_buffer[0];
  }
  else
  { //verify job
    //######################################################################################
    // check if this is an LFM job
    const char *str_p;
    str_p = m_prg_api_p->JobStringGet(AlgorithmTypes::DATA_FILENAME);
    if ((str_p != NULL) && (strlen(str_p) > 0))
    {
      if (!(strncmp(str_p, "Master device", 13)))
      {
        m_ID_buffer_p = &m_IDCODE_buffer[0];
      }
      else
      { //verify only job
        m_ID_buffer_p = &m_srcdata_bp[ID_OFFSET];
      }
    }
    else
    {
      m_prg_api_p->ThrowException(ALG_ERR_S, __LINE__, __FILE__, "JobStringGet failed");
    }
    //######################################################################################
  }

} //C_RV40F_P1XC::Initialize()

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ //
DWORD C_RV40F_P1XC::GetDFAdressInDevice(DWORD address)
{
  DWORD dataFlashAddress = 0;

  if ((address & 0xFFFC0000) == DF_START_IN_IMAGE)
    dataFlashAddress = 0xFD200000 + address;                   //re-mapping: CH file 0xFF200000 <--> RR map 0x2000000
  if ((address & 0xFFFC0000) == (DF_START_IN_IMAGE + 0x40000)) //2nd DF area
    dataFlashAddress = 0xFD2C0000 + address;                   //re-mapping: CH file 0xFF300000 <--> RR map 0x2040000

  return dataFlashAddress;
}

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ //
int C_RV40F_P1XC::CheckFlashID(void)
{
  BYTE pwSel = 3; //CFID + DFID

  m_current_CMD = FLASH_ID_CHECK_CMD;
  WriteCmdBuffer(0, FLASH_ID_CHECK_CMD);
  WriteCmdBuffer(1, pwSel);
  for (BYTE i = 0; i < (2 * m_id_len); i++)
    WriteCmdBuffer(2 + i, m_ID_buffer_p[32 + i]);

  if (false == SendFrame(SOH, ETX, GetCmdBufferP(), 2 + 2 * m_id_len))
    return false;
  if (false == GetDataFrame(ETX, GetCmdBufferP(), CHECK_ST1, CHECK_ST1))
    return false;

  return true;
}

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ //
C_RV40F_P1XC::DEV_STAT_E C_RV40F_P1XC::RV_IDCode(void)
{
  DEV_STAT_E op_stat = OPERATION_OK;
  volatile BYTE *srcbase = (volatile BYTE *)m_srcdata_bp;
  const CURRENT_OP_STATUS *current_op_stat_p = m_prg_api_p->CurrentOpStatusGet();

  m_current_op_mode = current_op_stat_p->operation;

  if ((m_optionSelectedByUser & CFG_WRITE) == 0)
  { //if CFG_WRITE method is used, then ID_CODE is part of the cfg area - it will be handled in Secure()
    WriteCmdBuffer(0, IDCODE_GET_CMD);
    if (false == SendFrame(SOH, ETX, GetCmdBufferP(), 1))
      return VERIFY_ERR;
    if (false == GetDataFrame(ETX, GetCmdBufferP(), CHECK_ST1, CHECK_ST1))
      return VERIFY_ERR;
    if (false == SendFrame(SOD, ETX, GetCmdBufferP(), 1)) //reverse ACK
      return VERIFY_ERR;
    if (false == GetDataFrame(ETX, (BYTE *)(srcbase + ID_OFFSET), 3 * m_id_len, 3 * m_id_len, IDCODE_GET_CMD)) //returns ID + CFID + DFID
      return VERIFY_ERR;
  }

  return op_stat;
}

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ //
C_RV40F_P1XC::DEV_STAT_E C_RV40F_P1XC::Prog_IDCode(const BYTE cmd)
{
  volatile BYTE *srcbase = (volatile BYTE *)m_srcdata_bp;
  DEV_STAT_E op_stat = OPERATION_OK;

  if ((m_optionSelectedByUser & CFG_WRITE) == 0)
  { //if CFG_WRITE method is used, then ID_CODE is part of the cfg area - it will be programmed in Secure()
    WriteCmdBuffer(0, cmd);
    if (false == SendFrame(SOH, ETX, (BYTE *)(srcbase + ID_OFFSET), 3 * m_id_len, cmd))
      return PROGRAM_ERR;
    if (false == GetDataFrame(ETX, GetCmdBufferP(), CHECK_ST1, CHECK_ST1))
      return PROGRAM_ERR;
  }

  return op_stat;
}

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ //
C_RV40F_P1XC::DEV_STAT_E C_RV40F_P1XC::RV_OPBT(void)
{
  DEV_STAT_E op_stat = OPERATION_OK;
  volatile BYTE *srcbase = (volatile BYTE *)m_srcdata_bp;
  const CURRENT_OP_STATUS *current_op_stat_p = m_prg_api_p->CurrentOpStatusGet();

  m_current_op_mode = current_op_stat_p->operation;

  for (BYTE opbtSel = 0; opbtSel < 4; opbtSel++)
  {
    //get option bytes (OPB)
    WriteCmdBuffer(0, OPTION_GET_CMD);
    WriteCmdBuffer(1, opbtSel);
    if (false == SendFrame(SOH, ETX, GetCmdBufferP(), 2))
      return VERIFY_ERR;
    if (false == GetDataFrame(ETX, GetCmdBufferP(), CHECK_ST1, CHECK_ST1))
      return VERIFY_ERR;
    if (false == SendFrame(SOD, ETX, GetCmdBufferP(), 1)) //reverse ACK
      return VERIFY_ERR;
    if (false == GetDataFrame(ETX, (BYTE *)(srcbase + OPBT_OFFSET(0) + 16 * opbtSel), 16, 16, OPTION_GET_CMD)) //4 OPBT words at once
      return VERIFY_ERR;
  } //-- OF for (BYTE opbtSel = 0; opbtSel < 4; opbtSel++)

  return op_stat;
}

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ //
C_RV40F_P1XC::DEV_STAT_E C_RV40F_P1XC::RV_DeviceConfig(void)
{
  WORD i;
  volatile BYTE *srcbase = (volatile BYTE *)m_srcdata_bp;
  DEV_STAT_E op_stat = OPERATION_OK;

  if (m_optionSelectedByUser & CFG_WRITE)
  {
    for (WORD cfgAddr = 0; cfgAddr < P1XC_CFG_DATA_LENGTH; cfgAddr += 0x10)
    {
      WriteCmdBuffer(0, CONFIG_VERIFY_CMD);
      WriteCmdBuffer(1, 0x00);
      WriteCmdBuffer(2, 0x00);
      WriteCmdBuffer(3, 0x00);
      WriteCmdBuffer(4, 0x40 + (BYTE)cfgAddr);
      for (i = 0; i < 16; i++)
        WriteCmdBuffer(5 + i, *(BYTE *)(srcbase + P1XC_CFG_DATA_OFFSET + i));
      if (false == SendFrame(SOH, ETX, GetCmdBufferP(), 21))
        return VERIFY_ERR;
      if (false == GetDataFrame(ETX, GetCmdBufferP(), CHECK_ST1, CHECK_ST1))
        return VERIFY_ERR;
    } //-- OF for (WORD cfgAddr = 0x40; cfgAddr < 0x100; cfgAddr+= 0x10)
  }   //-- OF if (m_optionSelectedByUser & CFG_WRITE)
  else
  {
    //standard methods for config programming

    if ((m_optionSupportedByDev & ID_AUTH) == 0)
    {
      if (OPERATION_OK != RV_ProtBits())
      {
        m_prg_api_p->Write2EventLog("Verify of the protection bits failed.");
        return VERIFY_ERR;
      }
      if (m_fReadProtected)
        m_prg_api_p->Write2EventLog("WARNING: Verify of the ID Code not possible!");
      else if (OPERATION_OK != RV_IDCode())
      {
        m_prg_api_p->Write2EventLog("Verify of the ID Code failed.");
        return VERIFY_ERR;
      }
    }
    if (OPERATION_OK != RV_OPBT())
    {
      m_prg_api_p->Write2EventLog("Verify of the option bytes failed.");
      return VERIFY_ERR;
    }
  } //-- OF if (m_optionSelectedByUser & CFG_WRITE)

  if (m_optionSupportedByDev & LB)
    if (OPERATION_OK != RV_BlockProtBits(LOCKBIT_GET_CMD))
    {
      m_prg_api_p->Write2EventLog("Verify of the block LOCK bits failed.");
      return VERIFY_ERR;
    }
  if (m_optionSupportedByDev & OTP)
    if (OPERATION_OK != RV_BlockProtBits(OTP_GET_CMD))
    {
      m_prg_api_p->Write2EventLog("Verify of the block OTP bits failed.");
      return VERIFY_ERR;
    }

  return op_stat;
}
//-- END C_RV40F_P1XC definitions

// -- Begin entry points for programmer system
////////////////////////////////////////////////////////////////////////////////
//          C_RV40F_P1XC::Secure() ()
// Inputs:
//
// Return:
//
// METHOD:
////////////////////////////////////////////////////////////////////////////////
C_RV40F_P1XC::DEV_STAT_E C_RV40F_P1XC::Secure()
{
#if (ALG_DEBUG > 0)
  PRINTF("C_RV40F_P1XC::Secure()\n"); // debug statements
#endif

  DWORD i, addrCnt;
  WORD cfgAddr;
  BYTE data_buffer[PROT_LENGTH];
  volatile BYTE *srcbase = (volatile BYTE *)m_srcdata_bp;
  bool optionDataBlockHasSN;
  DEV_STAT_E secure_stat = OPERATION_OK;
  SOCKET_STATUS_E skt_stat;
  BYTE skt_mask, failed_skt_mask;
  int nDUT;

  m_current_op_mode = DeviceOperation::SECURE;
  //This procedure is called last, thus it is in charge of
  // - setting of the (extended)option bytes
  // - setting of the Lock Bits for the sectors
  // - setting of the ICU mode (ICU-S validation)
  // - setting of the ID CODE (AUTH) or protection bits (06/07/2018 - If not CFG_WRITE mode, ID CODE (but not ID CODE for AUTH!) will be programmed during Program() - allowing SNs in ID CODE)
  // - setting of the OTP bits for the sectors
  // - setting of "Serial Programming Disable" (Caution: SPD makes any further access by the programmer impossible!)

  if (!m_prg_api_p->GetSectorFlag(SectorOp::BLANK_CHECK_OP, m_option_data_block))
    return secure_stat; //nothing to do

  optionDataBlockHasSN = true;
  if (m_prg_api_p->GetSectorFlag(SectorOp::BLANK_CHECK_OP, m_option_data_block) && m_prg_api_p->GetSectorFlag(SectorOp::PROGRAM_SECTOR_OP, m_option_data_block))
    optionDataBlockHasSN = false;

  //Test data for config: default values
  //BYTE cfg_buffer[256] =
  //{
  //  0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF, //0x00
  //  0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF, //0x10
  //  0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF, //0x20
  //  0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF, //0x30
  //  0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF, //0x40 - Security
  //  0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF, //0x50 - ID I
  //  0x00,0x00,0x00,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF, //0x60 - reset vector
  //  0xFF,0xFF,0xBF,0x7F,0x38,0xC9,0xFF,0xBF,0xFF,0xFF,0xFF,0xBF,0xFF,0xFF,0xFF,0xFF, //0x70 - OPBT0 - 3
  //  0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF, //0x80 - OPBT4 - 7
  //  0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF, //0x90 - OPBT8 - 11
  //  0xFF,0xFF,0xFF,0xFF,0x9F,0xFF,0xFF,0xFF,0xFA,0xFC,0xFF,0xFF,0xBC,0xFF,0xFF,0xFF, //0xA0 - OPBT12 - 15
  //  0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF, //0xB0 - ID II
  //  0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF, //0xC0 - CFID I
  //  0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF, //0xD0 - CFIF II
  //  0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF, //0xE0 - DFID I
  //  0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF  //0xF0 - DFID II
  //};

  if (m_optionSelectedByUser & CFG_WRITE)
  {
    for (cfgAddr = 0; cfgAddr < P1XC_CFG_DATA_LENGTH; cfgAddr += 0x10)
    {
      WriteCmdBuffer(0, CONFIG_WRITE_CMD);
      WriteCmdBuffer(1, 0x00);
      WriteCmdBuffer(2, 0x00);
      WriteCmdBuffer(3, 0x00);
      WriteCmdBuffer(4, 0x40 + (BYTE)cfgAddr);
      for (i = 0; i < 16; i++)
        WriteCmdBuffer(5 + i, *(BYTE *)(srcbase + P1XC_CFG_DATA_OFFSET + i));
      if (false == SendFrame(SOH, ETX, GetCmdBufferP(), 21))
        return SECURE_ERR;
      if (false == GetDataFrame(ETX, GetCmdBufferP(), CHECK_ST1, CHECK_ST1))
        return SECURE_ERR;
    } //-- OF for (WORD cfgAddr = 0x40; cfgAddr < 0x100; cfgAddr+= 0x10)

    //verify
    for (cfgAddr = 0; cfgAddr < P1XC_CFG_DATA_LENGTH; cfgAddr += 0x10)
    {
      WriteCmdBuffer(0, CONFIG_VERIFY_CMD);
      WriteCmdBuffer(1, 0x00);
      WriteCmdBuffer(2, 0x00);
      WriteCmdBuffer(3, 0x00);
      WriteCmdBuffer(4, 0x40 + (BYTE)cfgAddr);
      for (i = 0; i < 16; i++)
        WriteCmdBuffer(5 + i, *(BYTE *)(srcbase + P1XC_CFG_DATA_OFFSET + i));
      if (false == SendFrame(SOH, ETX, GetCmdBufferP(), 21))
        return SECURE_ERR;
      if (false == GetDataFrame(ETX, GetCmdBufferP(), CHECK_ST1, CHECK_ST1))
        return SECURE_ERR;
    } //-- OF for (WORD cfgAddr = 0x40; cfgAddr < 0x100; cfgAddr+= 0x10)
  }   //-- OF if (m_optionSelectedByUser & CFG_WRITE)
  else
  {
    //standard methods for config programming

    /*if ((m_optionSupportedByDev & ICU_S) && (m_optionSelectedByUser & ICU_S))
    {
      if (check_ICU_S_area() == false)
        return SECURE_ERR;
    }*/

    if (m_optionSelectedByUser & OPBT)
    {
      //program Option-bytes (OPBT)
      for (BYTE opbtSel = 0; opbtSel < 4; opbtSel++)
      {
        for (addrCnt = 0; addrCnt < 16; addrCnt++)
        {
          if (GetDataFromRam_8Bit(OPBT_OFFSET(0) + opbtSel * 16 + addrCnt, (DWORD)m_srcdata_bp) != 0xFF)
            break;
        }
        if (addrCnt == 16)
          continue;

        WriteCmdBuffer(0, OPTION_SET_CMD);
        WriteCmdBuffer(1, opbtSel);
        for (i = 0; i < 16; i++)
          WriteCmdBuffer(2 + i, *(BYTE *)(srcbase + OPBT_OFFSET(0) + opbtSel * 16 + i));
        if (false == SendFrame(SOH, ETX, GetCmdBufferP(), 18))
          return SECURE_ERR;
        if (false == GetDataFrame(ETX, GetCmdBufferP(), CHECK_ST1, CHECK_ST1))
          return SECURE_ERR;
      } //-- OF for (BYTE opbtSel = 0; opbtSel < 4; opbtSel++)

      //verify
      if (OPERATION_OK != RV_OPBT())
        return SECURE_ERR;
    }

    if ((m_optionSupportedByDev & ICU_S) && (m_optionSelectedByUser & ICU_S))
    {
      if (GetSockets_ICU_S_Status())
      {
        if (m_All_skts_ICU_S_INVALID == ICU_S_RESPONSIVE_INVALID)
        {
          WriteCmdBuffer(0, ICU_S_VALIDATE_CMD);
          if (false == SendFrame(SOH, ETX, GetCmdBufferP(), 1))
            return SECURE_ERR;
          if (false == GetDataFrame(ETX, GetCmdBufferP(), CHECK_ST1, CHECK_ST1))
            return SECURE_ERR;
        }
      }
      else
      {
        //Collect data socket by socket
        failed_skt_mask = 0;
        WORD socket_mask = LM_Phapi::Get()->ActiveDUTMaskGet();
        m_current_op_mode = DeviceOperation::READ;
        for (nDUT = MAX_SOCKET_NUM - 1; nDUT >= 0; nDUT--)
        {
          // check only active sockets
          skt_stat = m_prg_api_p->SocketStatusGet(SocketNumChange(nDUT + 1));
          if ((skt_stat == SOCKET_ENABLE) && (m_ICU_S_Status[nDUT] == ICU_S_RESPONSIVE_INVALID))
          {
            // activate this one socket for reading
            // Note: ALL OTHER SOCKETS WILL BE DISABLED
            skt_mask = 1 << nDUT;
            LM_Phapi::Get()->SetSingleSktMode((WORD)(nDUT + 1));
            
            WriteCmdBuffer(0, ICU_S_VALIDATE_CMD);
            if (false == SendFrame(SOH, ETX, GetCmdBufferP(), 1))
            {
              failed_skt_mask |= skt_mask;
              secure_stat = SECURE_ERR;
            }
            if (false == GetDataFrame(ETX, GetCmdBufferP(), CHECK_ST1, CHECK_ST1))
            {
              failed_skt_mask |= skt_mask;
              secure_stat = SECURE_ERR;
            }
          } //-- OF if (skt_stat==SOCKET_ENABLE)
        }   //-- OF for (nDUT=MAX_SOCKET_NUM-1; nDUT>=0; nDUT--)
        // Re-Enable all enabled sockets
        LM_Phapi::Get()->SetGangSktMode(socket_mask);
        m_current_op_mode = DeviceOperation::SECURE;

        // disable the DUT
        if (CompareFailed(failed_skt_mask))
          if (!m_prg_api_p->MisCompare(DeviceOperation::SECURE, failed_skt_mask, 0, 0))
            return SECURE_ERR;
      }
    }

    if ((m_optionSelectedByUser & ID_AUTH) && (optionDataBlockHasSN == false))
    { //when ID_AUTH is programmed, after the next RESET the access must be authenticated w/ the correct ID
      if (OPERATION_OK != Prog_IDCode(ID_AUTH_SET_CMD))
        return SECURE_ERR;
    }

    if ((m_optionSelectedByUser & ID_AUTH) == 0)
    { //command protection mode
      //prepare security data
      //security flags
      data_buffer[0] = ~GetDataFromRam_8Bit(PROT_OFFSET, (DWORD)m_srcdata_bp); //bits are inverted
      if (data_buffer[0] != 0x00 && data_buffer[0] != 0xFF)
      {
        WriteCmdBuffer(0, PROTECTION_SET_CMD);
        if (false == SendFrame(SOH, ETX, data_buffer, PROT_LENGTH, PROTECTION_SET_CMD))
          return SECURE_ERR;
        if (false == GetDataFrame(ETX, GetCmdBufferP(), CHECK_ST1, CHECK_ST1))
          return SECURE_ERR;

        // check for any system events
        if (m_prg_api_p->SysEvtChk())
          PRINTF(" OC @ line %d\n", __LINE__);

        //verify
        if (OPERATION_OK != RV_ProtBits())
          return SECURE_ERR;
      }
    }

    if (m_optionSelectedByUser & SPD)
    {
      WriteCmdBuffer(0, SP_DISABLE_CMD);
      if (false == SendFrame(SOH, ETX, GetCmdBufferP(), 1))
        return SECURE_ERR;
      if (false == GetDataFrame(ETX, GetCmdBufferP(), CHECK_ST1, CHECK_ST1))
        return SECURE_ERR;
      PRINTF("C_RV40F::Secure() - SPD set!\n");
    }
  } //-- OF if (m_optionSelectedByUser & CFG_WRITE)

  if ((m_optionSupportedByDev & LB) && (m_optionSelectedByUser & LB))
  { //Programming and erasure by self programming of an area for which the lock bit is set and the lock bit function is enabled are prohibited.
    WriteCmdBuffer(0, LOCKBIT_SET_CMD);
    if (false == SendFrame(SOH, ETX, (BYTE *)(srcbase + LOCK_BIT_OFFSET), LB_LENGTH, LOCKBIT_SET_CMD))
      return SECURE_ERR;
    if (false == GetDataFrame(ETX, GetCmdBufferP(), CHECK_ST1, CHECK_ST1))
      return SECURE_ERR;

    //verify
    if (OPERATION_OK != RV_BlockProtBits(LOCKBIT_GET_CMD))
      return SECURE_ERR;
  } //--if (m_fLockBitsSupported)

  if ((m_optionSupportedByDev & OTP) && (m_optionSelectedByUser & OTP))
  { //Caution: when OTP bits are set, erasing of the appopriate blocks AND option area are prohibited!

    WriteCmdBuffer(0, OTP_SET_CMD);
    if (false == SendFrame(SOH, ETX, (BYTE *)(srcbase + OTP_BIT_OFFSET), LB_LENGTH, OTP_SET_CMD))
      return SECURE_ERR;
    if (false == GetDataFrame(ETX, GetCmdBufferP(), CHECK_ST1, CHECK_ST1))
      return SECURE_ERR;

    //verify
    if (OPERATION_OK != RV_BlockProtBits(OTP_GET_CMD))
      return SECURE_ERR;
  } //--if (m_fOtpBitsSupported)

  return secure_stat;
} //C_RV40F_P1XC::Secure()
//-- END C_RV40F_P1XC definitions

/////////////////////////////////////////////////////
//Test Cases for P1x-C       :                    //
//Config Write               :  (R7F701327A)      //
//ID AUTH                    :                    //
/////////////////////////////////////////////////////

//-- BEGIN C_RV40F_Kimball definitions
//
/////////////////////////////////////////////////////////////////////////////////////////
//          C_RV40F_Kimball ctor ()
// Inputs:
//
// Return:
//
// METHOD:
////////////////////////////////////////////////////////////////////////////////
C_RV40F_Kimball::C_RV40F_Kimball(DEVPARMS *dparms_p, DEVPINS *dpins_p, DEVSECTORS *dsectors_p,
                                 BYTE opfreq_parm, PRM_T *dprm_p, DEVICE_DESCRIPTOR_T *descriptor_p, ALG_CODE *tBoot_code_p)
    : C_RV40F_P1XC(dparms_p, dpins_p, dsectors_p, opfreq_parm, dprm_p, tBoot_code_p)
{
  m_descriptor_p = descriptor_p;
  PRINTF("C_RV40F_Kimball::kimball custom constructed()\n"); //empty
  WORD block = 0;
  do
  {
    block++;
  } while (m_devsectors_p[block].begin_address <
           m_descriptor_p->code_length);
  m_sectors2load = block;

} //C_RV40F_P1XC ctor()

///Begin of erase routine for Kimball
//*************************************************************************
//FUNCTION    Erase_CF_Block()
//Inputs   		it erases the selected(m_sectors2load) sector blocks
//RETURNS     DEV_STAT_E - device status enumeration
//METHOD       block erase
//EXCEPTIONS --  none
//*************************************************************************
C_RV40F_Kimball::DEV_STAT_E C_RV40F_Kimball::Erase_CF_Block()
{
#if (ALG_DEBUG > 0)
  PRINTF("C_RV40F_Kimball::Erase_CF_Block()\n"); // debug statements
#endif

  DWORD address, startaddress_in_device;
  bool fResetReq = false;
  DEV_STAT_E erase_stat = OPERATION_OK;

  m_current_op_mode = DeviceOperation::ERASE;

  if (false == DeviceInit())
    return BLOCK_ERASE_ERR;

  WORD block = 0;
  do
  {
    // see if block has to be erased
    if (m_prg_api_p->GetSectorFlag(SectorOp::ERASE_SECTOR_OP, block))
    {
      if (block == m_option_data_block || block >= m_DF_block_nr)
        continue;

      startaddress_in_device = m_devsectors_p[block].begin_address;
      PRINTF("C_RV40F_Kimball::currently erasing block %d\n", block);

      WriteCmdBuffer(ERASE_CMD, startaddress_in_device, 0);

      if (false == SendFrame(SOH, ETX, GetCmdBufferP(), 5))
        return WSM_BUSY_ERR;

      if (false == GetDataFrame(ETX, GetCmdBufferP(), CHECK_ST1, CHECK_ST1))
      {
        erase_stat = BLOCK_ERASE_ERR;
        PRINTF("C_RV40F_Kimball::Erasing failed in block %d\n", block);
      }

      // check for any system events
      if (m_prg_api_p->SysEvtChk())
        return HARDWARE_ERR; // O.C. or Adapter change - return immediately
    }                        // end of if 'sector' was to be programmed
  } while (++block < m_sectors2load && erase_stat == OPERATION_OK);

  if (fResetReq)
    ResetToProgrammingMode();

  // check for any system events
  if (m_prg_api_p->SysEvtChk())
    return HARDWARE_ERR; // O.C. or Adapter change - return immediately

  return erase_stat;
}

/*******************************************************************************
*         C_RV40F_Kimball::Program_CF_Block()
* Inputs: it programs the selected(m_sectors2load) sector blocks
*       
* Return:   
*       
* METHOD:     
*******************************************************************************/
C_RV40F_Kimball::DEV_STAT_E C_RV40F_Kimball::Program_CF_Block()
{

#if (ALG_DEBUG > 0)
  PRINTF("C_RV40F_Kimball::Program_CF_Block()\n"); // debug statements
#endif

  volatile BYTE *srcbase = m_descriptor_p->code_p;
  DEV_STAT_E prog_stat = OPERATION_OK;

  DWORD startaddress, endaddress;
  DWORD startaddress_in_device, endaddress_in_device, address;
  BYTE frameEndType;
  DWORD areaSize;
  WORD blockSize;

  m_current_op_mode = DeviceOperation::PROGRAM;

  if (false == DeviceInit())
    return PROGRAM_ERR;

  WORD block = 0;
  do
  {
    // see if block has to be programmed
    if (m_prg_api_p->GetSectorFlag(SectorOp::PROGRAM_SECTOR_OP, block))
    {

      startaddress = m_devsectors_p[block].begin_address;
      endaddress = m_devsectors_p[block].end_address;

      startaddress_in_device = startaddress;
      endaddress_in_device = endaddress;
      areaSize = endaddress_in_device - startaddress_in_device + 1;
      if (areaSize < MAX_PAGE_SIZE)
        blockSize = (WORD)areaSize;
      else
        blockSize = MAX_PAGE_SIZE;

      WriteCmdBuffer(PROGRAM_CMD, startaddress_in_device, endaddress_in_device);

      if (false == SendFrame(SOH, ETX, GetCmdBufferP(), 9))
        return WSM_BUSY_ERR;

      if (false == GetDataFrame(ETX, GetCmdBufferP(), CHECK_ST1, CHECK_ST1))
        return PROGRAM_ERR;

      for (address = startaddress;
           address <= endaddress && prog_stat == OPERATION_OK;
           address += blockSize)
      {
        if (address + blockSize > endaddress) //last data block?
          frameEndType = ETX;                 //end of all data
        else
          frameEndType = ETB; //end of block

        if (false == SendFrame(SOD, frameEndType, (BYTE *)(srcbase + address), blockSize, PROGRAM_CMD))
          return WSM_BUSY_ERR;

        if (false == GetDataFrame(ETX, GetCmdBufferP(), CHECK_ST1, CHECK_ST1))
        {
          prog_stat = PROGRAM_ERR;
          PRINTF("C_RV40F_Kimball::Programming fail at 0x%X\n", address);
        }
      }

      // check for any system events
      if (m_prg_api_p->SysEvtChk())
        return HARDWARE_ERR; // O.C. or Adapter change - return immediately
    }                        // end of if 'sector' was to be programmed
  } while (++block < m_sectors2load && prog_stat == OPERATION_OK);

  if (prog_stat != OPERATION_OK)
    return prog_stat;

  return prog_stat;
} //C_RV40F_Kimball::Second_file_Program()

////////////////////////////////////////////////////////////////////////////////
//FUNCTION    Verify_CF_Block()
//Inputs   		it verifies the selected(m_sectors2load) sector blocks
//RETURNS     DEV_STAT_E - device status enumeration
//METHOD      verify device data towards image on a per block basis
//EXCEPTIONS --  none
////////////////////////////////////////////////////////////////////////////////
C_RV40F_Kimball::DEV_STAT_E C_RV40F_Kimball::Verify_CF_Block()
{
#if (ALG_DEBUG > 0)
  PRINTF("C_RV40F_Kimball::Verify_CF_Block()\n");
#endif

  volatile BYTE *srcbase = m_descriptor_p->code_p;

  DWORD startaddress, endaddress;
  DWORD startaddress_in_device, endaddress_in_device;
  DWORD address;
  BYTE frameEndType;
  WORD block;
  DEV_STAT_E verify_stat = OPERATION_OK;

  m_current_op_mode = DeviceOperation::VERIFY;

  if (false == DeviceInit())
    return VERIFY_ERR;

  //blockwise operation
  block = 0;
  do
  {
    if (m_prg_api_p->GetSectorFlag(SectorOp::PROGRAM_SECTOR_OP, block))
    {
      startaddress = m_devsectors_p[block].begin_address;
      endaddress = m_devsectors_p[block].end_address;

      startaddress_in_device = startaddress;
      endaddress_in_device = endaddress;
      m_current_CMD = VERIFY_CMD;
      WriteCmdBuffer(m_current_CMD, startaddress_in_device, endaddress_in_device);

      if (false == SendFrame(SOH, ETX, GetCmdBufferP(), 9))
        return WSM_BUSY_ERR;
      if (false == GetDataFrame(ETX, GetCmdBufferP(), CHECK_ST1, CHECK_ST1))
        return VERIFY_ERR;

      //verify loop
      for (address = startaddress;
           address <= endaddress && verify_stat == OPERATION_OK;
           address += MAX_PAGE_SIZE)
      {
        if (address + MAX_PAGE_SIZE > endaddress) //last data block?
          frameEndType = ETX;                     //end of all data
        else
          frameEndType = ETB; //end of block

        if (false == SendFrame(SOD, frameEndType, (BYTE *)(srcbase + address), MAX_PAGE_SIZE, VERIFY_CMD))
          return WSM_BUSY_ERR;

        if (false == GetDataFrame(ETX, GetCmdBufferP(), CHECK_ST1, CHECK_ST1))
        {
          verify_stat = VERIFY_ERR;
          PRINTF("C_RV40F_Kimball::Verify fail at 0x%X\n", address);
        }
      }
    }
  } while (++block < m_sectors2load && verify_stat == OPERATION_OK);

  // check for any system events
  if (m_prg_api_p->SysEvtChk())
    return HARDWARE_ERR; // O.C. or Adapter change - return immediately

  return verify_stat;
} //C_RV40F_Kimball::Verify_CF_Block()

// -- Begin entry points for programmer system
////////////////////////////////////////////////////////////////////////////////
//          C_RV40F_Kimball::Secure()
// Inputs:
//
// Return:
//
// METHOD:
////////////////////////////////////////////////////////////////////////////////
C_RV40F_Kimball::DEV_STAT_E C_RV40F_Kimball::Secure()
{
#if (ALG_DEBUG > 0)
  PRINTF("C_RV40F_Kimball::Secure()\n"); // debug statements
#endif

  DEV_STAT_E secure_stat = OPERATION_OK;

  //call base class impplementation
  secure_stat = C_RV40F_P1XC::Secure();
  if (secure_stat != OPERATION_OK)
    return SECURE_ERR;

  DoPowerDown();
  DELAY_MS(2000);

  PRINTF("C_RV40F_Kimball::DoPowerUp()\n"); // debug statements

  m_akt_skt_msk = 0;
  for (WORD dut = 0; dut < MAX_SOCKET_NUM; dut++)
  {
    // If socket is enabled
    if (SOCKET_ENABLE == m_prg_api_p->SocketStatusGet((WORD)(SKT_CHG_FACTOR - dut - 1)))
      m_akt_skt_msk |= (1 << dut);
  } //-- OF for (WORD dut=0; dut<MAX_SOCKET_NUM; dut++)

  // Power up the device
  m_fpga_p->ParDataCompare();               //on some adapter datapins are eventually connected to DRST. Keep it LOW.
  m_prg_api_p->PinSet(_RESET_PIN, LOGIC_0); //POR
  m_prg_api_p->PinSet(JP0_2_SCK_PIN, LOGIC_0);
  m_prg_api_p->PinSet(JP0_0_SI_PIN, LOGIC_0);
  m_prg_api_p->PinSet(FLMD0_PIN, LOGIC_0);

  m_prg_api_p->SetVpullDir(HwTypes::UP); // pull up D0 (SO)

  if (m_prg_api_p->PinExists(JP0_4_TRST_PIN))
    m_prg_api_p->PinSet(JP0_4_TRST_PIN, LOGIC_0);
  if (m_prg_api_p->PinExists(POWERGOOD_PIN))
    m_prg_api_p->PinSet(POWERGOOD_PIN, LOGIC_0);
  if (m_prg_api_p->PinExists(MODE0_PIN))
    m_prg_api_p->PinSet(MODE0_PIN, LOGIC_1); //don't care in programming mode
  if (m_prg_api_p->PinExists(MODE1_PIN))
    m_prg_api_p->PinSet(MODE1_PIN, LOGIC_1); //don't care in programming mode
  if (m_prg_api_p->PinExists(JP0_3_TMS_PIN))
    m_prg_api_p->PinSet(JP0_3_TMS_PIN, LOGIC_0); //don't care in programming mode

  if (m_prg_api_p->PinExists(FLMD1_PIN))
    m_prg_api_p->PinSet(FLMD1_PIN, LOGIC_0);

  //Voltage regulators for the core power supply are supplied with higher voltage.
  //The core supply must be supplied first then the rest
  //Thus the power sequence depends on the voltage level settings:
  //m_prg_api_p->SetAdapterPower(HwTypes::ON);          // Power ON +5V_SW - for testing
  if (m_prg_api_p->PinExists(VDD_AUX_EN_PIN)) //if voltage regulator has ENABLE pin set it HIGH
    m_prg_api_p->PinSet(VDD_AUX_EN_PIN, LOGIC_1);
  if (m_prg_api_p->PinExists(VREG_VS0_PIN))
    m_prg_api_p->PinSet(VREG_VS0_PIN, LOGIC_1);
  if (m_prg_api_p->PinExists(VREG_VS1_PIN))
    m_prg_api_p->PinSet(VREG_VS1_PIN, LOGIC_1);
  if (m_prg_api_p->PinExists(VREG_VS2_PIN))
    m_prg_api_p->PinSet(VREG_VS2_PIN, LOGIC_1);

  if (m_devparms_p->voltage_vpp > m_devparms_p->voltage_vcc)
  { //G-Node G4 is used for voltage regulator supply (core supply)
    if (m_prg_api_p->PinExists(VDD_AUX_PIN))
    {
      m_prg_api_p->VppSet(m_devparms_p->voltage_vpp, false);
      //MicroSecDelay(200);
      m_prg_api_p->PinSet(VDD_AUX_PIN, SV);
    }
    m_prg_api_p->VccSet(m_devparms_p->voltage_vcc, false);
    m_prg_api_p->VihSet(m_devparms_p->voltage_vih, false);
  }
  else
  { //VCC is used for voltage regulator supply (core supply)
    m_prg_api_p->VccSet(m_devparms_p->voltage_vcc, false);
    m_prg_api_p->VihSet(m_devparms_p->voltage_vih, false);
    if (m_prg_api_p->PinExists(VDD_AUX_PIN))
    {
      m_prg_api_p->VppSet(m_devparms_p->voltage_vpp, true);
      MicroSecDelay(200);
      m_prg_api_p->PinSet(VDD_AUX_PIN, SV);
    }
  }
  MicroSecDelay(100);
  if (LOBYTE(m_devparms_p->reserved4))
    DELAY_MS(LOBYTE(m_devparms_p->reserved4) * 100); //wait for regulator stabilisation

  if (m_prg_api_p->PinExists(POWERGOOD_PIN))
    m_prg_api_p->PinSet(POWERGOOD_PIN, LOGIC_1);
  DELAY_MS(50);

  m_prg_api_p->PinSet(_RESET_PIN, LOGIC_1);
  MicroSecDelay(1200);
  m_prg_api_p->PinSet(FLMD0_PIN, LOGIC_1);
  //m_prg_api_p->PinGroupDirSet (A16_A23,   PIN_INPUT);

  DELAY_MS(m_descriptor_p->app_runtime);

  //m_prg_api_p->PinGroupDirSet (A16_A23,   PIN_OUTPUT);

  ResetToProgrammingMode();

  if (m_prg_api_p->SysEvtChk())
    PRINTF(" OC @ line %d\n", __LINE__);

  secure_stat = Erase_CF_Block();
  if (secure_stat != OPERATION_OK)
    return SECURE_ERR;
  secure_stat = Program_CF_Block();
  if (secure_stat != OPERATION_OK)
    return SECURE_ERR;
  secure_stat = Verify_CF_Block();
  if (secure_stat != OPERATION_OK)
    return SECURE_ERR;

  return secure_stat;
} //C_RV40F_Kimball::Secure()
