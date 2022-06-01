// $Header: /psbu/RoadRunner/sw/src/rralg/rtc_synergy.cpp 7     7/21/21 5:41a Dosapav $
//--------------------------------------------------------------------------//
//          Copyright (c) 1999->2004, Data I/O Corporation                  //
//          10525 Willows Rd NE / Redmond, WA 98073 / (425) 881-6444        //
//__________________________________________________________________________//

/***************************************************************************

		ALGORITHM FILE:     RA4E1_RA6E1.CPP    Author: Vinod kumar Dosapati

		DESCRIPTION:
		DATA I/O SPID :
		WIDTH:  x8
		SUPPORTS:           RA family with Cortex-M33 using UART interface

												Specification: R01AN6024EJ0100 Rev.1.00  (System Specifications  Standard Boot Firmware for the RA4E1 and RA6E1)

												requires special FPGA F157 rev.6+ for UART support --> FC3+ only.
												algorithm base extracted from Synergy family.

		DATA I/O SPID :

		Version 1.0   :  - Initial release



***************************************************************************/

#include "assert.h"
#include "standard.hpp"	   // defines BYTE, WORD, etc.
#include "string.h"		   // common strings
#include "PinDefines.hpp"  // #defines for pins and packages
#include "DevParms.hpp"	   // structure for pin files
#include "MicroSecDelay.h" //
#include "FlashAlg2.hpp"   // base class
#include "FlashAlg.inl"	   // empty implementations of FlashAlg base class...needed to satisfy the linker.
#include "ose.h"		   // for millisecond delay
#include "stdio.h"		   // defines sprintf

#include "StdWiggler.hpp"
#include "rtc_synergy_cortexm33.hpp" // class definition for RA4E1_RA6E1
#include "LM_Phapi.hpp"
#include "MPC_funcs.hpp"

extern "C"
{
#include "osetypes.h"
#include "cpu.h"
}

static const char *const __file = __FILE__;

char msgbuff[MSG_LEN + 1] = {0}; // This will be used for text handling to log.ext file

static BYTE RxDBuffer[MAX_SOCKET_NUM][RCV_BUF_SZ]; // receiver buffer for UART
static BYTE ErrBuffer[MAX_SOCKET_NUM][RCV_BUF_SZ]; // error buffer for UART

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

////////////////////////////////////////////////////////////////////////////////
//          RA4E1_RA6E1 ctor ()
// Inputs:
//
// Return:
//
// METHOD:
////////////////////////////////////////////////////////////////////////////////
RA4E1_RA6E1::RA4E1_RA6E1(DEVPARMS *dparms_p, DEVPINS *dpins_p, DEVSECTORS *dsectors_p, DEVICE_DESCRIPTOR_T *devInfo_p)
	: FlashAlg2(dparms_p, dpins_p, dsectors_p)
{

	m_fpga_p = NULL;
	m_devInfo_p = devInfo_p;

} // RA4E1_RA6E1 ctor()

/*******************************************************************************
 *         Initialize ()
 * Inputs:
 *
 * Return:
 *
 * METHOD:
 *******************************************************************************/
void RA4E1_RA6E1::Initialize(void)
{
	DWORD br_max;
	int i;
	DWORD param;
	char msg[100] = {0};
	char temp_buf[33];
	char *SF_pw_p = temp_buf;

	// call base class impplementation
	FlashAlg2::Initialize();

	br_max = ((DWORD)m_devInfo_p->signature[4] << 24) | ((DWORD)m_devInfo_p->signature[5] << 16) | ((DWORD)m_devInfo_p->signature[6] << 8) | (DWORD)m_devInfo_p->signature[7];
	ALG_ASSERT(m_devInfo_p->baud_rate <= br_max);

	m_Signature_logged = false;
	m_UART_initialized = false;
	m_DF_block_nr = m_devparms_p->sector_quantity - 2;
	m_CFG_block_nr = m_devparms_p->sector_quantity - 1;

	// Initialize special data
	// let's get the required data
	m_UF_filled0xFF = false; // default
	if (m_prg_api_p->SpecFeatureParmGet("User Flash - fill up gaps with 0xFF", &param))
	{
		PRINTF("<<User Flash - fill up gaps with 0xFF>> found: %Xh \n", param);
		if (param == 0xFF)
			m_UF_filled0xFF = true;
	}

	if (!m_prg_api_p->SpecFeatureParmGet("ID Code", &SF_pw_p))
	{
		strcat(msgbuff, "Parameter <<ID Code>> missed, check Database !!!");
		m_prg_api_p->ThrowException(PARAM_UNAVAILABLE_ERR_S, __LINE__, __file, msgbuff);
	}
	for (i = 0; i < PW_LEN; i++)
		m_IDCODE_buffer[i] = SF_pw_p[i];

	m_ID_buffer_p = &m_IDCODE_buffer[0];
	for (i = 0; i < PW_LEN; i++)
	{
		if (m_IDCODE_buffer[i] != 0xFF)
			break;
	}

	if (i == PW_LEN)
	{
		// straight forward
		for (i = 0; i < PW_LEN; i++)
			m_IDCODE_buffer[i] = m_srcdata_bp[m_devInfo_p->ID_address + PW_LEN - 1 - i]; // MSB-LSB (ID[127..0])
	}

	// initalize receive buffer for UART
	m_uart_rcv_buffer.nBufferSize = RCV_BUF_SZ;
	for (int nDUT = MAX_SOCKET_NUM - 1; nDUT >= 0; nDUT--)
	{
		m_uart_rcv_buffer.pBuffer[nDUT] = &RxDBuffer[nDUT][0];
		m_uart_rcv_buffer.pErrorBuffer[nDUT] = &ErrBuffer[nDUT][0];
	} //-- OF for (nDUT=MAX_SOCKET_NUM-1; nDUT>=0; nDUT--)

} // RA4E1_RA6E1::Initialize()

/*******************************************************************************
 *         DoPowerUp ()
 * Inputs:
 *
 * Return:
 *
 * METHOD:
 *******************************************************************************/
void RA4E1_RA6E1::DoPowerUp(void)
{
#if (ALG_DEBUG > 0)
	PRINTF("RA4E1_RA6E1::DoPowerUp()\n"); // debug statements
#endif

	// Power up the device
	m_prg_api_p->PinSet(RES_PIN, LOGIC_0);
	m_prg_api_p->PinSet(MD_PIN, LOGIC_0);
	m_prg_api_p->VccSet(m_devparms_p->voltage_vcc, false);
	m_prg_api_p->VihSet(m_devparms_p->voltage_vih, true);

	if (m_prg_api_p->SysEvtChk())
		PRINTF(" OC @ line %d\n", __LINE__);

	return;
}

/*******************************************************************************
 *         DoPowerDown ()
 * Inputs:
 *
 * Return:
 *
 * METHOD:
 *******************************************************************************/
void RA4E1_RA6E1::DoPowerDown(void)
{
#if (ALG_DEBUG > 0)
	PRINTF("RA4E1_RA6E1::DoPowerDown()\n"); // debug statements
#endif

	m_prg_api_p->PinSet(RES_PIN, LOGIC_0);
	m_prg_api_p->PinSet(MD_PIN, LOGIC_0);
	DELAY_MS(10);
	m_prg_api_p->VihSet(0, false);
	m_prg_api_p->VccSet(0, true);

	m_UART_initialized = false;
	m_fpga_p->UARTInit();
	*m_devbase_p; // set databus Hi-Z
	return;
}

/*******************************************************************************
 *         ResetToProgrammingMode ()
 * Inputs:
 *
 * Return:
 *
 * METHOD:
 *******************************************************************************/
bool RA4E1_RA6E1::ResetToProgrammingMode(void)
{
	DWORD retryCnt;
	BYTE tx_buffer[1];
	BYTE rx_buffer[1];
	DWORD retryCntMax = 10;
	BYTE socket_stat = 0;

	// Reset the device
	*m_devbase_p; // set databus Hi-Z
	m_prg_api_p->PinSet(RES_PIN, LOGIC_0);
	m_prg_api_p->PinSet(MD_PIN, LOGIC_0);
	DELAY_MS(20);
	m_UART_initialized = false;
	m_prg_api_p->PinSet(RES_PIN, LOGIC_1);
	DELAY_MS(2000);

	// prepare UART interface
	m_bit_time = (WORD)((DWORD)1000000 / m_devInfo_p->startup_baud);
	if (m_fpga_p->UARTInit(TXD_PIN, RXD_PIN, m_devInfo_p->startup_baud, UART_8N1) == false)
	{
		PRINTF("UART Initialization failed\n");
		return false; // all sockets failed
	}

	m_fpga_p->UARTResetBuffer(&m_uart_rcv_buffer); // clear rcv buffer
	tx_buffer[0] = 0x00;						   // initial pulse for SCI initialization
	rx_buffer[0] = STATUS_CODE_ACK;

	for (retryCnt = 0; retryCnt < retryCntMax; retryCnt++)
	{
		MicroSecDelay(5000);
		m_fpga_p->UARTSend(&tx_buffer[0], 1, true);
		socket_stat = Verify_RxData(&rx_buffer[0], 1);
		if (ComparePassed(socket_stat))
			break;
	}

	if (CompareFailed(socket_stat))
	{
		if (!m_prg_api_p->MisCompare(AlgorithmTypes::POWERUP, socket_stat, 0, 0))
		{
			sprintf(msgbuff, "ACK missed. Communication error. Device possibly damaged...");
			m_prg_api_p->Write2EventLog(msgbuff);
			return false;
		}
	}

	m_fpga_p->UARTResetBuffer(&m_uart_rcv_buffer); // clear rcv buffer
	tx_buffer[0] = GENERIC_CODE;
	rx_buffer[0] = BOOT_CODE_ACK_C6;

	m_fpga_p->UARTSend(&tx_buffer[0], 1, true);
	socket_stat = Verify_RxData(&rx_buffer[0], 1);
	if (CompareFailed(socket_stat))
	{
		if (!m_prg_api_p->MisCompare(AlgorithmTypes::POWERUP, socket_stat, 0, 0))
		{
			sprintf(msgbuff, "Boot code missed. Communication error. Device possibly damaged...");
			m_prg_api_p->Write2EventLog(msgbuff);
			return false;
		}
	}

	return true;
}

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ //
// implements start-up sequence of the algorithm: INQUIRY command + alg. param. settings
//
// METHOD:
//
int RA4E1_RA6E1::DeviceInit(void)
{
	BYTE socket_stat;
	BYTE baud_param[4];
	DWORD timeout;
	bool device_init_ok = true;

	if (false == ResetToProgrammingMode())
		return false;

	SendPacket(SOH, INQUIRY_CMD);
	socket_stat = Verify_RxData(&STATUS_CODE_ACK, 9, INQUIRY_CMD, TIMEOUT_100MS, false, true);
	if (CompareFailed(socket_stat))
	{
		if (!m_prg_api_p->MisCompare(DeviceOperation::POWERUP, socket_stat, 0x0, 0xeeee))
		{
			m_prg_api_p->Write2EventLog("RA4E1_RA6E1::DeviceInit() - ID authentication failed!");
			return false;
		}
	}

	// increase baud rate
	baud_param[0] = (BYTE)(m_devInfo_p->baud_rate >> 24);
	baud_param[1] = (BYTE)(m_devInfo_p->baud_rate >> 16);
	baud_param[2] = (BYTE)(m_devInfo_p->baud_rate >> 8);
	baud_param[3] = (BYTE)(m_devInfo_p->baud_rate);
	SendPacket(SOH, BAUD_SET_CMD, &baud_param[0], 4);
	socket_stat = Verify_RxData(&STATUS_CODE_ACK, 9, BAUD_SET_CMD, TIMEOUT_100MS, true, true);
	if (CompareFailed(socket_stat))
	{
		if (!m_prg_api_p->MisCompare(DeviceOperation::POWERUP, socket_stat, 0x0, 0xbdbd))
		{
			m_prg_api_p->Write2EventLog("RA4E1_RA6E1::DeviceInit() - baud setting failed!");
			return false;
		}
	}
	MicroSecDelay(1000);

	m_UART_initialized = true;
	m_bit_time = (WORD)((DWORD)1000000 / m_devInfo_p->baud_rate);
	if (m_fpga_p->UARTInit(TXD_PIN, RXD_PIN, m_devInfo_p->baud_rate, UART_8N1) == false)
	{
		PRINTF("UART Initialization failed\n");
		return false; // all sockets failed
	}

	// check for any system events
	if (m_prg_api_p->SysEvtChk())
		PRINTF(" OC @ line %d\n", __LINE__);

	return device_init_ok;
}

/*******************************************************************************
 *         PowerUp ()
 * Inputs:
 *
 * Return:
 *
 * METHOD:
 *******************************************************************************/
void RA4E1_RA6E1::PowerUp(void)
{
#if (ALG_DEBUG > 0)
	PRINTF("RA4E1_RA6E1::PowerUp()\n"); // debug statements
#endif

	// Initialize FPGA access object
	m_fpga_p = StdWiggler::Construct();
	if (!m_fpga_p)
		m_prg_api_p->ThrowException(ALG_LOGIC_ERR_S, __LINE__, __file);
	m_fpga_p->Initialize();

	m_prg_api_p->SetVpullDir(HwTypes::UP);
	DoPowerUp();

	return;
}

/*******************************************************************************
 *         PowerDown ()
 * Inputs:
 *
 * Return:
 *
 * METHOD:
 *******************************************************************************/
void RA4E1_RA6E1::PowerDown(void)
{

	DoPowerDown();

	delete m_fpga_p;
	m_fpga_p = NULL;

#if (ALG_DEBUG > 0)
	PRINTF("RA4E1_RA6E1::PowerDown()\n"); // debug statements
#endif

	return;
} // RA4E1_RA6E1::PowerDown()

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ //
// calculates the checksum for the frame buffer
// method: 0x00 - LNH - LNL - all of cmd/data information
BYTE RA4E1_RA6E1::CalcFrameCheckSum(const BYTE *buffer_p)
{
	DWORD bytecnt, data_cnt;
	BYTE checksum = 0;

	data_cnt = ((WORD)buffer_p[1] << 8) + buffer_p[2] + FRAME_SIZE - 2;
	for (bytecnt = 1; bytecnt < data_cnt; bytecnt++)
		checksum -= buffer_p[bytecnt];

	return checksum;
}

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ //
////////////////////////////////////////////////////////////////////////////////
// FUNCTION SendPacket()
// ARGUMENTS
// RETURNS void
// METHOD
//     Implementation of the UART flow for command and data packet transmission
// EXCEPTIONS --  none
////////////////////////////////////////////////////////////////////////////////
void RA4E1_RA6E1::SendPacket(const FRAMESTART_T startType, const BYTE cmd, const BYTE *param_p, WORD param_length)
{
	WORD byteCnt;
	BYTE sendByte;
	DWORD timeout = TIMEOUT_1S;
	BYTE checksum8 = 0;

	ALG_ASSERT(param_length <= TX_BUF_SZ);

	m_fpga_p->UARTResetBuffer(&m_uart_rcv_buffer); // new frame

	sendByte = (BYTE)startType;
	m_fpga_p->UARTSend(&sendByte, 1);
	sendByte = HIBYTE(param_length + 1); // LNH
	m_fpga_p->UARTSend(&sendByte, 1);
	checksum8 -= sendByte;
	sendByte = LOBYTE(param_length + 1); // LNL
	m_fpga_p->UARTSend(&sendByte, 1);
	checksum8 -= sendByte;
	m_fpga_p->UARTSend(&cmd, 1);
	checksum8 -= cmd;
	if (param_length)
	{
		m_fpga_p->UARTSend(param_p, param_length, false);
		for (byteCnt = 0; byteCnt < param_length; byteCnt++)
			checksum8 -= param_p[byteCnt];
		while (m_fpga_p->UARTSendIsBusy() && timeout--)
		{
		};
	}
	m_fpga_p->UARTSend(&checksum8, 1);
	sendByte = (BYTE)ETX;
	m_fpga_p->UARTSend(&sendByte, 1, false);

	return;
}

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ //
// Verify_RxData: compare received data against expected data
//                This function does not access the device but the UART-FPGA buffer and register
// Parameter: const BYTE* expected_p: address of the buffer with the expected data. If NULL, no compare
//            const WORD dataCnt: count of the expected data
//            const BYTE cmd: if used (!= 0xFF), packet frame will be checked, otherwise single bytes
//            DWORD timeout: wait time for response
//						bool verbose: if true, shows the error messages
//
// Return:  Bitmask with socket status:
//          Bit 0 - Compare status DUT1 - socket 4
//          Bit 1 - Compare status DUT2 - socket 3
//          Bit 2 - Compare status DUT3 - socket 2
//          Bit 3 - Compare status DUT4 - socket 1
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ //
BYTE RA4E1_RA6E1::Verify_RxData(const BYTE *expected_p, const WORD dataCnt, const BYTE cmd, DWORD timeout, bool verbose, bool status_check)
{
	int i, nDUT;
	WORD active_sockets;
	WORD socket_ready_mask;
	WORD cnt, expectedCnt;
	BYTE rx_buffer[RCV_BUF_SZ]; // buffer for expected response packet
	BYTE rxData;
	BYTE status = 0;

	if (cmd == 0xFF)
	{ // check single bytes
		expectedCnt = dataCnt;
		if (expected_p)
		{
			for (cnt = 0; cnt < dataCnt; cnt++)
				rx_buffer[cnt] = expected_p[cnt];
		} //-- if (expected_p)
	}
	else
	{ // check packet frame

		expectedCnt = dataCnt + FRAME_SIZE + 1;
		if (expected_p)
		{
			if (status_check)
			{
				rx_buffer[0] = (BYTE)SOD;
				rx_buffer[1] = HIBYTE(dataCnt + 1);
				rx_buffer[2] = LOBYTE(dataCnt + 1);
				rx_buffer[3] = cmd;
				rx_buffer[4] = 0x00; // status
				memset(&rx_buffer[5], FILL_BYTES, (dataCnt - 1));
				rx_buffer[4 + dataCnt] = CalcFrameCheckSum(&rx_buffer[0]);
				rx_buffer[5 + dataCnt] = (BYTE)ETX;
			}
			else
			{
				expectedCnt = dataCnt + FRAME_SIZE + 1;
				rx_buffer[0] = (BYTE)SOD;
				rx_buffer[1] = HIBYTE(dataCnt + 1);
				rx_buffer[2] = LOBYTE(dataCnt + 1);
				rx_buffer[3] = cmd;
				memcpy(&rx_buffer[4], expected_p, (dataCnt));
				rx_buffer[4 + dataCnt] = CalcFrameCheckSum(&rx_buffer[0]);
				rx_buffer[5 + dataCnt] = (BYTE)ETX;
			}
		} //-- if (expected_p)
	}

	active_sockets = LM_Phapi::Get()->ActiveDUTMaskGet();
	socket_ready_mask = 0;
	for (i = 0; i < timeout; i++)
	{
		m_fpga_p->UARTReceive(&m_uart_rcv_buffer); // fill the buffer

		for (nDUT = MAX_SOCKET_NUM - 1; nDUT >= 0; nDUT--)
		{
			if ((active_sockets & (1 << nDUT)) == 0)
				continue; // skip inactive socket

			if (m_uart_rcv_buffer.nNumBytes[nDUT] >= expectedCnt)
				socket_ready_mask |= 1 << nDUT;
		} //-- OF for (nDUT=MAX_SOCKET_NUM-1; nDUT>=0; nDUT--)
		if ((socket_ready_mask & active_sockets) == active_sockets)
			break; // sockets are ready
	}			   //-- OF  for (int i = 0; i < timeout; i++)

	if (expected_p == NULL || i == timeout)
	{
		status = (~socket_ready_mask) & active_sockets;
		return status;
	}

	for (nDUT = MAX_SOCKET_NUM - 1; nDUT >= 0; nDUT--)
	{
		if ((active_sockets & (1 << nDUT)) == 0)
			continue; // skip inactive socket

		// if (memcmp(rx_buffer, &m_uart_rcv_buffer.pBuffer[nDUT][0], expectedCnt) == 0)
		//	continue;

		for (cnt = 0; cnt < expectedCnt; cnt++)
		{
			rxData = m_uart_rcv_buffer.pBuffer[nDUT][cnt];
			if (rx_buffer[cnt] != rxData)
			{
				status |= 1 << nDUT;
				if (verbose)
				{
					sprintf(msgbuff, "Socket%d at packet pos. %d expected 0x%02X, received 0x%02X.", SocketNumChange(nDUT + 1), cnt, rx_buffer[cnt], rxData);
					m_prg_api_p->Write2EventLog(msgbuff);
					if (expectedCnt != m_uart_rcv_buffer.nNumBytes[nDUT])
					{
						sprintf(msgbuff, "Socket%d: expected %d bytes, received %d. Communication failed!", SocketNumChange(nDUT + 1), expectedCnt, m_uart_rcv_buffer.nNumBytes[nDUT]);
						m_prg_api_p->Write2EventLog(msgbuff);
					}
				}
				break;
			}
		} //-- OF for (cnt = 0; cnt < expectedCnt; cnt++)
	}	  //-- OF for (nDUT=MAX_SOCKET_NUM-1; nDUT>=0; nDUT--)

	return status;
}

////////////////////////////////////////////////////////////////////////////////
// FUNCTION    VerifyBlock()
// ARGUMENTS
// RETURNS     DEV_STAT_E - device status enumeration
// METHOD      verify device data towards image
//
//
// EXCEPTIONS --  none
////////////////////////////////////////////////////////////////////////////////
RA4E1_RA6E1::DEV_STAT_E RA4E1_RA6E1::VerifyBlock(DWORD start_in_mem, DWORD start_in_dev, DWORD area_size)
{
	BYTE socket_stat;
	DWORD address_in_mem, end_in_mem;
	DWORD address_in_dev;
	DWORD blockSize;
	BYTE param[8];
	DEV_STAT_E verify_stat = OPERATION_OK;

	address_in_mem = start_in_mem;
	address_in_dev = start_in_dev;
	end_in_mem = start_in_mem + area_size - 1;
	do
	{
		blockSize = end_in_mem - address_in_mem + 1;
		if (blockSize > m_devInfo_p->rd_page_size)
			blockSize = m_devInfo_p->rd_page_size;

		param[0] = (BYTE)(address_in_dev >> 24);
		param[1] = (BYTE)(address_in_dev >> 16);
		param[2] = (BYTE)(address_in_dev >> 8);
		param[3] = (BYTE)address_in_dev;
		param[4] = (BYTE)((address_in_dev + blockSize - 1) >> 24);
		param[5] = (BYTE)((address_in_dev + blockSize - 1) >> 16);
		param[6] = (BYTE)((address_in_dev + blockSize - 1) >> 8);
		param[7] = (BYTE)(address_in_dev + blockSize - 1);
		SendPacket(SOH, READ_CMD, &param[0], 8);
		socket_stat = Verify_RxData(&m_srcdata_bp[address_in_mem], (WORD)blockSize, READ_CMD, TIMEOUT_1S, true, false);
		if (CompareFailed(socket_stat))
		{ // call MisCompare here reporting which block has failed
			if (!m_prg_api_p->MisCompare(DeviceOperation::VERIFY, socket_stat, 0, STATUS_CODE_ACK))
			{
				sprintf(msgbuff, "RA4E1_RA6E1::VerifyBlock() - Data verify failed in range 0x%X-0x%X!", address_in_mem, address_in_mem + blockSize - 1);
				m_prg_api_p->Write2EventLog(msgbuff);
				verify_stat = VERIFY_ERR;
			}
		}

		address_in_mem += blockSize;
		address_in_dev += blockSize;
	} while (address_in_mem <= end_in_mem && verify_stat == OPERATION_OK);

	return verify_stat;
} // RA4E1_RA6E1::VerifyBlock()

////////////////////////////////////////////////////////////////////////////////
//          ProgramBlock ()
// Inputs:
//
// Return:
//
// METHOD:
//
////////////////////////////////////////////////////////////////////////////////
RA4E1_RA6E1::DEV_STAT_E RA4E1_RA6E1::ProgramBlock(DWORD start_in_mem, DWORD start_in_dev, DWORD area_size)
{
	BYTE socket_stat;
	DWORD address_in_mem, end_in_mem;
	DWORD blockSize;
	BYTE param[8];
	DEV_STAT_E prog_stat = OPERATION_OK;

	param[0] = (BYTE)(start_in_dev >> 24);
	param[1] = (BYTE)(start_in_dev >> 16);
	param[2] = (BYTE)(start_in_dev >> 8);
	param[3] = (BYTE)start_in_dev;
	param[4] = (BYTE)((start_in_dev + area_size - 1) >> 24);
	param[5] = (BYTE)((start_in_dev + area_size - 1) >> 16);
	param[6] = (BYTE)((start_in_dev + area_size - 1) >> 8);
	param[7] = (BYTE)(start_in_dev + area_size - 1);
	SendPacket(SOH, WRITE_CMD, &param[0], 8);
	socket_stat = Verify_RxData(&STATUS_CODE_ACK, 9, WRITE_CMD, TIMEOUT_10MS, true, true);
	if (CompareFailed(socket_stat))
	{ // call MisCompare here reporting which block has failed
		if (!m_prg_api_p->MisCompare(DeviceOperation::PROGRAM, socket_stat, 0, STATUS_CODE_ACK))
		{
			sprintf(msgbuff, "RA4E1_RA6E1::ProgramBlock() - WRITE command failed in range 0x%X-0x%X!", start_in_mem, start_in_mem + area_size - 1);
			m_prg_api_p->Write2EventLog(msgbuff);
			prog_stat = PROGRAM_ERR;
		}
	}

	address_in_mem = start_in_mem;
	end_in_mem = start_in_mem + area_size - 1;
	do
	{
		blockSize = end_in_mem - address_in_mem + 1;
		if (blockSize > TX_BUF_SZ)
			blockSize = TX_BUF_SZ;

		SendPacket(SOD, WRITE_CMD, &m_srcdata_bp[address_in_mem], (WORD)blockSize);
		socket_stat = Verify_RxData(&STATUS_CODE_ACK, 9, WRITE_CMD, TIMEOUT_1S, true, true);
		if (CompareFailed(socket_stat))
		{ // call MisCompare here reporting which block has failed
			if (!m_prg_api_p->MisCompare(DeviceOperation::PROGRAM, socket_stat, 0, STATUS_CODE_ACK))
			{
				sprintf(msgbuff, "RA4E1_RA6E1::ProgramBlock() - Data Write failed in range 0x%X-0x%X!", address_in_mem, address_in_mem + blockSize - 1);
				m_prg_api_p->Write2EventLog(msgbuff);
				prog_stat = PROGRAM_ERR;
			}
		}

		address_in_mem += blockSize;
	} while (address_in_mem <= end_in_mem && prog_stat == OPERATION_OK);

	return prog_stat;
} // RA4E1_RA6E1::ProgramBlock()

////////////////////////////////////////////////////////////////////////////////
// FUNCTION    GetDeviceAreaInfo()
// ARGUMENTS
// RETURNS     true, if area found, otherwise false
// METHOD      reads the requested information from the area_info array
//
//
// EXCEPTIONS --  none
////////////////////////////////////////////////////////////////////////////////
bool RA4E1_RA6E1::GetDeviceAreaInfo(BYTE areaType, DWORD &areaStart, DWORD &areaEnd, DWORD &area_write_unit)
{
	bool searchResult;
	int i;

	for (i = 0; i < MAX_AREA_CNT; i++)
	{
		if (areaType == m_devInfo_p->area_info[i][0])
		{
			areaStart = ((DWORD)m_devInfo_p->area_info[i][1] << 24) | ((DWORD)m_devInfo_p->area_info[i][2] << 16) | ((DWORD)m_devInfo_p->area_info[i][3] << 8) | (DWORD)m_devInfo_p->area_info[i][4];
			areaEnd = ((DWORD)m_devInfo_p->area_info[i][5] << 24) | ((DWORD)m_devInfo_p->area_info[i][6] << 16) | ((DWORD)m_devInfo_p->area_info[i][7] << 8) | (DWORD)m_devInfo_p->area_info[i][8];
			area_write_unit = ((DWORD)m_devInfo_p->area_info[i][13] << 24) | ((DWORD)m_devInfo_p->area_info[i][14] << 16) | ((DWORD)m_devInfo_p->area_info[i][15] << 8) | (DWORD)m_devInfo_p->area_info[i][16];
			break;
		}
	}

	if (i == MAX_AREA_CNT)
		searchResult = false;
	else
		searchResult = true;

	return searchResult;
} // RA4E1_RA6E1::GetDeviceAreaInfo()

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
// FUNCTION    GetAreaForProcessing(DWORD end_address, DWORD min_page_size, DWORD max_page_size, DWORD& address, DWORD& area_start_address, DWORD& area_end_address)
// ARGUMENTS   DWORD end_address: area end
//             DWORD min_page_size: minimum size of programming page in bytes
//             DWORD max_page_size: maximum size of programming page in bytes
//             DWORD& address: actual address in the area
//             DWORD& area_start_address: reference to the area start address valiable
//             DWORD& area_end_address: reference to the area end address valiable
// RETURNS     false, if area is blank
//             true, if non-blank data was found. area_start_address & area_end_address show the actual range with the non-blank data.
// METHOD      Search for real data in memory (works with TLWIN only, when special DLL add markers in reserved area)
// EXCEPTIONS  none
bool RA4E1_RA6E1::GetAreaForProcessing(DWORD end_address, DWORD min_page_size, DWORD max_page_size, DWORD &address, DWORD &area_start_address, DWORD &area_end_address)
{
	DWORD byteCnt;

	if (m_UF_filled0xFF)
	{ // gaps are filled with 0xFF -> blocks are complete
		area_start_address = address;
		area_end_address = end_address;
		address = end_address + 1;
		return true;
	}

	while (m_srcdata_bp[address + MARKER_OFFSET] == 0xFF && address <= end_address)
		address++;
	if (address > end_address)
		return false; // all blank - nothing to do

	address &= ~(min_page_size - 1);
	area_start_address = address;

	do
	{
		address += min_page_size;
		if (0 == (address % max_page_size))
			break;

		for (byteCnt = 0; byteCnt < min_page_size; byteCnt++)
		{
			if (m_srcdata_bp[address + MARKER_OFFSET + byteCnt] != 0xFF)
				break;
		}
	} while (byteCnt != min_page_size);
	area_end_address = address - 1;
	if (address % max_page_size)
		address += min_page_size;

	return true;
}

// -- Begin entry points for programmer system
/*************************************************************************
					IDCheck()
ARGUMENTS
RETURNS true if part ID == expected ID, false if not
METHOD
		Read device ID's and compare to expected
EXCEPTIONS --  none
*************************************************************************/
bool RA4E1_RA6E1::IDCheck()
{
#if (ALG_DEBUG > 0)
	PRINTF("RA4E1_RA6E1::IDCheck() (Signature + EXTRAs)\n"); // debug statements sent to command window
#endif

	// LM_Phapi::Get()->ScopePinSet (1);
	BYTE nr_of_recordable_areas = 0;
	BYTE current_DLM_state = 0;
	BYTE areaCnt;
	SOCKET_STATUS_E skt_stat;
	int skt_mask, failed_sockets;
	int nDUT;
	BYTE socket_stat;
	bool device_id_ok = true;
	BYTE param[8];

	if (false == m_UART_initialized)
		if (false == DeviceInit())
			return false;

	SendPacket(SOH, SIGNATURE_CMD);
	socket_stat = Verify_RxData(NULL, 41, SIGNATURE_CMD, TIMEOUT_100MS, true, false);
	if (CompareFailed(socket_stat))
	{
		if (!m_prg_api_p->MisCompare(DeviceOperation::IDCHECK, socket_stat, 0, STATUS_CODE_ACK))
		{
			m_prg_api_p->Write2EventLog("RA4E1_RA6E1::IDCheck() - SIGNATURE CMD timeout...");
			return false;
		}
	}

	// LOG info
	failed_sockets = 0; // saves socket mask if it fails
	for (nDUT = MAX_SOCKET_NUM - 1; nDUT >= 0; nDUT--)
	{
		// check expected data for active sockets only
		skt_stat = m_prg_api_p->SocketStatusGet(SocketNumChange(nDUT + 1));
		if (skt_stat == SOCKET_ENABLE)
		{
			skt_mask = 1 << nDUT;
			if (!m_Signature_logged)
			{
				PRINTF("Socket%d - RMB: 0x%02X,0x%02X,0x%02X,0x%02X, NOA: 0x%02X, Typ: 0x%02X, BFV: 0x%02X,0x%02X, 0x%02X \n", SocketNumChange(nDUT + 1),
					   m_uart_rcv_buffer.pBuffer[nDUT][4], m_uart_rcv_buffer.pBuffer[nDUT][5], m_uart_rcv_buffer.pBuffer[nDUT][6], m_uart_rcv_buffer.pBuffer[nDUT][7],
					   m_uart_rcv_buffer.pBuffer[nDUT][8],
					   m_uart_rcv_buffer.pBuffer[nDUT][9],
					   m_uart_rcv_buffer.pBuffer[nDUT][10], m_uart_rcv_buffer.pBuffer[nDUT][11], m_uart_rcv_buffer.pBuffer[nDUT][12]);
				PRINTF("Socket%d - DID: 0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X \n", SocketNumChange(nDUT + 1),
					   m_uart_rcv_buffer.pBuffer[nDUT][13], m_uart_rcv_buffer.pBuffer[nDUT][14], m_uart_rcv_buffer.pBuffer[nDUT][15], m_uart_rcv_buffer.pBuffer[nDUT][16],
					   m_uart_rcv_buffer.pBuffer[nDUT][17], m_uart_rcv_buffer.pBuffer[nDUT][18], m_uart_rcv_buffer.pBuffer[nDUT][19], m_uart_rcv_buffer.pBuffer[nDUT][20],
					   m_uart_rcv_buffer.pBuffer[nDUT][21], m_uart_rcv_buffer.pBuffer[nDUT][22], m_uart_rcv_buffer.pBuffer[nDUT][23], m_uart_rcv_buffer.pBuffer[nDUT][24],
					   m_uart_rcv_buffer.pBuffer[nDUT][25], m_uart_rcv_buffer.pBuffer[nDUT][26], m_uart_rcv_buffer.pBuffer[nDUT][27], m_uart_rcv_buffer.pBuffer[nDUT][28]);
				PRINTF("Socket%d - PTN: 0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X \n", SocketNumChange(nDUT + 1),
					   m_uart_rcv_buffer.pBuffer[nDUT][29], m_uart_rcv_buffer.pBuffer[nDUT][30], m_uart_rcv_buffer.pBuffer[nDUT][31], m_uart_rcv_buffer.pBuffer[nDUT][32],
					   m_uart_rcv_buffer.pBuffer[nDUT][33], m_uart_rcv_buffer.pBuffer[nDUT][34], m_uart_rcv_buffer.pBuffer[nDUT][35], m_uart_rcv_buffer.pBuffer[nDUT][36],
					   m_uart_rcv_buffer.pBuffer[nDUT][37], m_uart_rcv_buffer.pBuffer[nDUT][38], m_uart_rcv_buffer.pBuffer[nDUT][39], m_uart_rcv_buffer.pBuffer[nDUT][40],
					   m_uart_rcv_buffer.pBuffer[nDUT][41], m_uart_rcv_buffer.pBuffer[nDUT][42], m_uart_rcv_buffer.pBuffer[nDUT][43], m_uart_rcv_buffer.pBuffer[nDUT][44]);
			}
			nr_of_recordable_areas = m_uart_rcv_buffer.pBuffer[nDUT][8];
			switch (m_uart_rcv_buffer.pBuffer[nDUT][9]) // device type
			{
			case RA6E1:
				sprintf(msgbuff, "\tSocket%d device type:RA6E1_Series. FW ver.: %d.%d.%d", SocketNumChange(nDUT + 1), m_uart_rcv_buffer.pBuffer[nDUT][10], m_uart_rcv_buffer.pBuffer[nDUT][11], m_uart_rcv_buffer.pBuffer[nDUT][12]);
				break;
			default:
				sprintf(msgbuff, "\tSocket%d device type: UNKNOWN.", SocketNumChange(nDUT + 1));
				break;
			}
			m_prg_api_p->Write2EventLog(msgbuff);

			if (m_devInfo_p->signature[4] != nr_of_recordable_areas)
			{
				sprintf(msgbuff, "\tSocket%d signature check failed: expected NOA 0x%02X, actual 0x%02X\r\n", SocketNumChange(nDUT + 1), m_devInfo_p->signature[4], nr_of_recordable_areas);
				m_prg_api_p->Write2EventLog(msgbuff);
				failed_sockets |= skt_mask;
			}

			if (m_devInfo_p->signature[5] != m_uart_rcv_buffer.pBuffer[nDUT][9])
			{
				sprintf(msgbuff, "\tSocket%d signature check failed: expected device type 0x%02X, actual 0x%02X\r\n", SocketNumChange(nDUT + 1), m_devInfo_p->signature[5], m_uart_rcv_buffer.pBuffer[nDUT][9]);
				m_prg_api_p->Write2EventLog(msgbuff);
				failed_sockets |= skt_mask;
			}

		} //-- OF if (skt_stat==SOCKET_ENABLE)
	}	  //-- OF for (nDUT=MAX_SOCKET_NUM-1; nDUT>=0; nDUT--)

	// disable bad DUTs
	if (failed_sockets)
		if (!m_prg_api_p->MisCompare(DeviceOperation::IDCHECK, (BYTE)failed_sockets, 0, STATUS_CODE_ACK))
			device_id_ok = false;

	for (areaCnt = 0; areaCnt < nr_of_recordable_areas && device_id_ok; areaCnt++)
	{
		SendPacket(SOH, AREA_INFO_CMD, &areaCnt, 1);
		socket_stat = Verify_RxData(NULL, 25, AREA_INFO_CMD, TIMEOUT_100MS, true, false);
		if (CompareFailed(socket_stat))
		{
			if (!m_prg_api_p->MisCompare(DeviceOperation::IDCHECK, socket_stat, 0, STATUS_CODE_ACK))
			{
				m_prg_api_p->Write2EventLog("RA4E1_RA6E1::IDCheck() - SIGNATURE CMD timeout...");
				device_id_ok = false;
				break;
			}
		}

		if (!m_Signature_logged)
		{
			// log the info for devparam
			for (nDUT = MAX_SOCKET_NUM - 1; nDUT >= 0; nDUT--)
			{
				// check expected data for active sockets only
				skt_stat = m_prg_api_p->SocketStatusGet(SocketNumChange(nDUT + 1));
				if (skt_stat == SOCKET_ENABLE)
				{
					PRINTF("Socket%d - AREA %d: 0x%02X, 0x%02X,0x%02X,0x%02X,0x%02X, 0x%02X,0x%02X,0x%02X,0x%02X, 0x%02X,0x%02X,0x%02X,0x%02X, 0x%02X,0x%02X,0x%02X,0x%02X,  0x%02X,0x%02X,0x%02X,0x%02X, 0x%02X,0x%02X,0x%02X,0x%02X\n", SocketNumChange(nDUT + 1), areaCnt,
						   m_uart_rcv_buffer.pBuffer[nDUT][4],
						   m_uart_rcv_buffer.pBuffer[nDUT][5], m_uart_rcv_buffer.pBuffer[nDUT][6], m_uart_rcv_buffer.pBuffer[nDUT][7], m_uart_rcv_buffer.pBuffer[nDUT][8],
						   m_uart_rcv_buffer.pBuffer[nDUT][9], m_uart_rcv_buffer.pBuffer[nDUT][10], m_uart_rcv_buffer.pBuffer[nDUT][11], m_uart_rcv_buffer.pBuffer[nDUT][12],
						   m_uart_rcv_buffer.pBuffer[nDUT][13], m_uart_rcv_buffer.pBuffer[nDUT][14], m_uart_rcv_buffer.pBuffer[nDUT][15], m_uart_rcv_buffer.pBuffer[nDUT][16],
						   m_uart_rcv_buffer.pBuffer[nDUT][17], m_uart_rcv_buffer.pBuffer[nDUT][18], m_uart_rcv_buffer.pBuffer[nDUT][19], m_uart_rcv_buffer.pBuffer[nDUT][20],
						   m_uart_rcv_buffer.pBuffer[nDUT][21], m_uart_rcv_buffer.pBuffer[nDUT][22], m_uart_rcv_buffer.pBuffer[nDUT][23], m_uart_rcv_buffer.pBuffer[nDUT][24],
						   m_uart_rcv_buffer.pBuffer[nDUT][25], m_uart_rcv_buffer.pBuffer[nDUT][26], m_uart_rcv_buffer.pBuffer[nDUT][27], m_uart_rcv_buffer.pBuffer[nDUT][28]);
					break; // only for one socket
				}		   //-- OF if (skt_stat==SOCKET_ENABLE)
			}			   //-- OF for (nDUT=MAX_SOCKET_NUM-1; nDUT>=0; nDUT--)
		}				   //-- if (!m_Signature_logged)

		/*socket_stat = Verify_RxData(&m_devInfo_p->area_info[areaCnt][0], 25, AREA_INFO_CMD, TIMEOUT_100MS, true);
		if (CompareFailed(socket_stat))
		{
			if (!m_prg_api_p->MisCompare(DeviceOperation::IDCHECK, socket_stat, 0, STATUS_CODE_ACK))
			{
				sprintf (msgbuff, "RA4E1_RA6E1::IDCheck() - AREA %d info check failed!", areaCnt);
				m_prg_api_p->Write2EventLog(msgbuff);
				device_id_ok = false;
			}
		}*/
	} //-- OF for (areaCnt = 0; areaCnt < nr_of_recordable_areas && device_id_ok; areaCnt++)
	m_Signature_logged = true;

	// DLM state Request Command
	SendPacket(SOH, DLM_STATE_REQ_CMD);
	socket_stat = Verify_RxData(NULL, 1, DLM_STATE_REQ_CMD, TIMEOUT_100MS, true, false);
	if (CompareFailed(socket_stat))
	{
		if (!m_prg_api_p->MisCompare(DeviceOperation::IDCHECK, socket_stat, 0, STATUS_CODE_ACK))
		{
			m_prg_api_p->Write2EventLog("RA4E1_RA6E1::IDCheck() - SIGNATURE CMD timeout...");
			device_id_ok = false;
		}
	}

	// log the info for devparam
	for (nDUT = MAX_SOCKET_NUM - 1; nDUT >= 0; nDUT--)
	{
		// check expected data for active sockets only
		skt_stat = m_prg_api_p->SocketStatusGet(SocketNumChange(nDUT + 1));
		if (skt_stat == SOCKET_ENABLE)
		{
			PRINTF("Socket%d - DLM STATE  : 0x%02X \n", SocketNumChange(nDUT + 1),
				   m_uart_rcv_buffer.pBuffer[nDUT][4]);
			current_DLM_state = m_uart_rcv_buffer.pBuffer[nDUT][4];
			break; // only for one socket
		}		   //-- OF if (skt_stat==SOCKET_ENABLE)
	}			   //-- OF for (nDUT=MAX_SOCKET_NUM-1; nDUT>=0; nDUT--)

	if (current_DLM_state == DLM_STATE_CM) // expected state for a virgin chip , transit to SSD state
	{
		param[0] = DLM_STATE_CM;
		param[1] = DLM_STATE_SSD;
		SendPacket(SOH, DLM_STATE_TRANSIT_CMD, &param[0], 2);
		socket_stat = Verify_RxData(&STATUS_CODE_ACK, 9, DLM_STATE_TRANSIT_CMD, TIMEOUT_100MS, true,true);
		if (CompareFailed(socket_stat))
		{
			if (!m_prg_api_p->MisCompare(DeviceOperation::IDCHECK, socket_stat, 0, STATUS_CODE_ACK))
			{
				m_prg_api_p->Write2EventLog("RA4E1_RA6E1::IDCheck() - DLM_Transit failed CM->SSD");
				device_id_ok = false;
			}
		}
	}
	else // device is in different DLM state. devices in NSECSD, DPL, SSD can be intilazed back to SSD state.
	{

		if ((current_DLM_state > 0x02) & (current_DLM_state > 0x05))
		{
			param[0] = current_DLM_state;
			param[1] = DLM_STATE_SSD;
			SendPacket(SOH, INITALIZE_CMD, &param[0], 2);
			DELAY_MS(500);
			socket_stat = Verify_RxData(&STATUS_CODE_ACK, 9, INITALIZE_CMD, TIMEOUT_1S, true, true);
			if (CompareFailed(socket_stat))
			{
				if (!m_prg_api_p->MisCompare(DeviceOperation::IDCHECK, socket_stat, 0, STATUS_CODE_ACK))
				{
					m_prg_api_p->Write2EventLog("RA4E1_RA6E1::IDCheck() - DLM_Transit failed UNKNOWN->SSD");
					device_id_ok = false;
				}
			}
		}
		else // the devices in Debug lock and RMQ cannot be recovered.
		{
			if (current_DLM_state > 0x05)
			{
				m_prg_api_p->Write2EventLog("RA4E1_RA6E1::IDCheck() - Device cannot be unlocked due a DLM state");
				device_id_ok = false;
			}
		}
	}

	// LM_Phapi::Get()->ScopePinSet (0);
	return device_id_ok;
}

//*************************************************************************
// FUNCTION    Read()
// ARGUMENTS   none
// RETURNS     DEV_STAT_E - device status enumeration
// METHOD      Read device to image on a per block basis
// EXCEPTIONS --  none
//*************************************************************************
RA4E1_RA6E1::DEV_STAT_E RA4E1_RA6E1::Read()
{
#if (ALG_DEBUG > 0)
	// debug statements
	PRINTF("RA4E1_RA6E1::Read()\n");
#endif

	BYTE socket_stat;
	int nDUT, active_DUT = 0;
	DWORD start_address_in_mem, end_address_in_mem, address_in_mem;
	DWORD start_address_in_dev, end_address_in_dev, address_in_dev;
	DWORD blockSize;
	DWORD dummy; // not used
	BYTE param[8];
	DEV_STAT_E read_stat = OPERATION_OK;

	for (nDUT = MAX_SOCKET_NUM - 1; nDUT >= 0; nDUT--)
	{
		// read data from active socket
		if (SOCKET_ENABLE == m_prg_api_p->SocketStatusGet(SocketNumChange(nDUT + 1)))
		{
			active_DUT = nDUT;
			break;
		} //-- OF if (skt_stat==SOCKET_ENABLE)
	}	  //-- OF for (nDUT=MAX_SOCKET_NUM-1; nDUT>=0; nDUT--)

	WORD block = 0;
	do
	{
		// see if block has to be programmed
		if (m_prg_api_p->GetSectorFlag(SectorOp::PROGRAM_SECTOR_OP, block))
		{
			start_address_in_mem = m_devsectors_p[block].begin_address;
			end_address_in_mem = m_devsectors_p[block].end_address;
			start_address_in_dev = start_address_in_mem;
			end_address_in_dev = end_address_in_mem;
			if (block == m_DF_block_nr)
			{
				if ((m_UF_filled0xFF == false))
					continue; // data flash is undefined
				if (false == GetDeviceAreaInfo(UDF, start_address_in_dev, end_address_in_dev, dummy))
					m_prg_api_p->ThrowException(ALG_LOGIC_ERR_S, __LINE__, __file);
			}
			if (block == m_CFG_block_nr)
			{
				if (false == GetDeviceAreaInfo(CFG, start_address_in_dev, end_address_in_dev, dummy))
					m_prg_api_p->ThrowException(ALG_LOGIC_ERR_S, __LINE__, __file);
			}
			ALG_ASSERT((end_address_in_mem - start_address_in_mem + 1) == (end_address_in_dev - start_address_in_dev + 1));

			address_in_mem = start_address_in_mem;
			address_in_dev = start_address_in_dev;
			do
			{
				blockSize = end_address_in_mem - address_in_mem + 1;
				if (blockSize > m_devInfo_p->rd_page_size)
					blockSize = m_devInfo_p->rd_page_size;

				param[0] = (BYTE)(address_in_dev >> 24);
				param[1] = (BYTE)(address_in_dev >> 16);
				param[2] = (BYTE)(address_in_dev >> 8);
				param[3] = (BYTE)address_in_dev;
				param[4] = (BYTE)((address_in_dev + blockSize - 1) >> 24);
				param[5] = (BYTE)((address_in_dev + blockSize - 1) >> 16);
				param[6] = (BYTE)((address_in_dev + blockSize - 1) >> 8);
				param[7] = (BYTE)(address_in_dev + blockSize - 1);
				SendPacket(SOH, READ_CMD, &param[0], 8);
				socket_stat = Verify_RxData(NULL, (WORD)blockSize, READ_CMD, TIMEOUT_1S, true, false);
				if (CompareFailed(socket_stat))
				{ // call MisCompare here reporting which block has failed
					if (!m_prg_api_p->MisCompare(DeviceOperation::READ, socket_stat, 0, STATUS_CODE_ACK))
					{
						sprintf(msgbuff, "RA4E1_RA6E1::Read() - READ cmd failed in range 0x%X-0x%X!", address_in_mem, address_in_mem + blockSize - 1);
						m_prg_api_p->Write2EventLog(msgbuff);
						read_stat = READ_ERR;
						break;
					}
				}
				memcpy(&m_srcdata_bp[address_in_mem], &m_uart_rcv_buffer.pBuffer[active_DUT][4], blockSize);
				address_in_mem += blockSize;
				address_in_dev += blockSize;
			} while (address_in_mem <= end_address_in_mem && read_stat == OPERATION_OK);

			// check for any system events
			if (m_prg_api_p->SysEvtChk())
				return HARDWARE_ERR; // O.C. or Adapter change - return immediately
		}							 // end of if 'sector' was to be programmed
	} while (++block < m_devparms_p->sector_quantity && read_stat == OPERATION_OK);

	return read_stat;
}

//*************************************************************************
// FUNCTION    Erase()
// ARGUMENTS   none
// RETURNS     DEV_STAT_E - device status enumeration
// METHOD      BLOCK ERASE
// EXCEPTIONS --  none
//*************************************************************************
RA4E1_RA6E1::DEV_STAT_E RA4E1_RA6E1::Erase()
{
#if (ALG_DEBUG > 0)
	PRINTF("RA4E1_RA6E1::Erase()\n"); // debug statements
#endif

	BYTE socket_stat;
	int sector_quantity;
	DWORD start_address_in_dev, end_address_in_dev;
	DWORD dummy; // not used
	BYTE param[8];
	DWORD erase_timeout = TIMEOUT_1S;
	DEV_STAT_E erase_stat = OPERATION_OK;

	if (false == m_UART_initialized)
		if (false == DeviceInit())
			return BLOCK_ERASE_ERR;

	sector_quantity = m_devparms_p->sector_quantity - 1; // without CFG area
	WORD block = 0;
	do
	{
		// see if block has to be programmed
		if (m_prg_api_p->GetSectorFlag(SectorOp::ERASE_SECTOR_OP, block))
		{
			start_address_in_dev = m_devsectors_p[block].begin_address;
			end_address_in_dev = m_devsectors_p[block].end_address;
			if (block == m_DF_block_nr)
			{
				if (false == GetDeviceAreaInfo(UDF, start_address_in_dev, end_address_in_dev, dummy))
					m_prg_api_p->ThrowException(ALG_LOGIC_ERR_S, __LINE__, __file);

				erase_timeout = TIMEOUT_10S;
			}

			param[0] = (BYTE)(start_address_in_dev >> 24);
			param[1] = (BYTE)(start_address_in_dev >> 16);
			param[2] = (BYTE)(start_address_in_dev >> 8);
			param[3] = (BYTE)start_address_in_dev;
			param[4] = (BYTE)(end_address_in_dev >> 24);
			param[5] = (BYTE)(end_address_in_dev >> 16);
			param[6] = (BYTE)(end_address_in_dev >> 8);
			param[7] = (BYTE)end_address_in_dev;
			SendPacket(SOH, ERASE_CMD, &param[0], 8);
			socket_stat = Verify_RxData(&STATUS_CODE_ACK, 9, ERASE_CMD, erase_timeout, true, true);
			if (CompareFailed(socket_stat))
			{ // call MisCompare here reporting which block has failed
				if (!m_prg_api_p->MisCompare(DeviceOperation::ERASE, socket_stat, 0, STATUS_CODE_ACK))
				{
					sprintf(msgbuff, "RA4E1_RA6E1::Erase() - ERASE CMD failed in range 0x%X-0x%X!", start_address_in_dev, end_address_in_dev);
					m_prg_api_p->Write2EventLog(msgbuff);
					erase_stat = BLOCK_ERASE_ERR;
				}
			}
			// check for any system events
			if (m_prg_api_p->SysEvtChk())
				return HARDWARE_ERR; // O.C. or Adapter change - return immediately
		}							 // end of if 'sector' was to be programmed
	} while (++block < sector_quantity && erase_stat == OPERATION_OK);

	return erase_stat;
}

////////////////////////////////////////////////////////////////////////////////
//          Program ()
// Inputs:
//
// Return:
//
// METHOD:
//
//
////////////////////////////////////////////////////////////////////////////////
RA4E1_RA6E1::DEV_STAT_E RA4E1_RA6E1::Program()
{
#if (ALG_DEBUG > 0)
	PRINTF("RA4E1_RA6E1::Program()\n"); // debug statements
#endif

	DWORD start_address_in_mem, end_address_in_mem;
	DWORD start_address_in_dev, end_address_in_dev;
	DWORD wr_unit;
	DWORD address_cnt;
	int sector_quantity;
	DWORD area_start_address_in_mem = 0;
	DWORD area_end_address_in_mem = 0;
	DEV_STAT_E prog_stat = OPERATION_OK;

	if (false == m_UART_initialized)
		if (false == DeviceInit())
			return PROGRAM_ERR;

	sector_quantity = m_devparms_p->sector_quantity - 1; // without CFG area

	// get write unit for User Code Flash
	if (false == GetDeviceAreaInfo(UCF, start_address_in_dev, end_address_in_dev, wr_unit))
		m_prg_api_p->ThrowException(ALG_LOGIC_ERR_S, __LINE__, __file);

	WORD block = 0;
	do
	{
		// see if block has to be programmed
		if (m_prg_api_p->GetSectorFlag(SectorOp::PROGRAM_SECTOR_OP, block))
		{
			start_address_in_mem = m_devsectors_p[block].begin_address;
			end_address_in_mem = m_devsectors_p[block].end_address;
			start_address_in_dev = start_address_in_mem;

			if (block == m_DF_block_nr)
			{
				if (false == GetDeviceAreaInfo(UDF, start_address_in_dev, end_address_in_dev, wr_unit))
					m_prg_api_p->ThrowException(ALG_LOGIC_ERR_S, __LINE__, __file);
			}

			address_cnt = start_address_in_mem;
			do
			{
				if (GetAreaForProcessing(end_address_in_mem, wr_unit, end_address_in_mem - start_address_in_mem + 1, address_cnt, area_start_address_in_mem, area_end_address_in_mem))
					prog_stat = ProgramBlock(area_start_address_in_mem, start_address_in_dev + (area_start_address_in_mem - start_address_in_mem), area_end_address_in_mem - area_start_address_in_mem + 1);
			} while (address_cnt <= end_address_in_mem && prog_stat == OPERATION_OK);

			// check for any system events
			if (m_prg_api_p->SysEvtChk())
				return HARDWARE_ERR; // O.C. or Adapter change - return immediately
		}							 // end of if 'sector' was to be programmed
	} while (++block < sector_quantity && prog_stat == OPERATION_OK);

	return prog_stat;
} // RA4E1_RA6E1::Program()

////////////////////////////////////////////////////////////////////////////////
// FUNCTION    Verify()
// ARGUMENTS
// RETURNS     DEV_STAT_E - device status enumeration
// METHOD      verify device data towards image or checksum on a per block basis
//
// EXCEPTIONS --  none
////////////////////////////////////////////////////////////////////////////////
RA4E1_RA6E1::DEV_STAT_E RA4E1_RA6E1::Verify()
{
#if (ALG_DEBUG > 0)
	PRINTF("RA4E1_RA6E1::Verify()\n");
#endif

	DWORD start_address_in_mem, end_address_in_mem;
	DWORD start_address_in_dev, end_address_in_dev;
	DWORD wr_unit;
	DWORD address_cnt;
	int sector_quantity;
	DWORD area_start_address_in_mem = 0;
	DWORD area_end_address_in_mem = 0;
	const CURRENT_OP_STATUS *current_op_stat_p = m_prg_api_p->CurrentOpStatusGet();
	DEV_OP_E current_op_mode = current_op_stat_p->operation;
	DEV_STAT_E verify_stat = OPERATION_OK;

	if (false == m_UART_initialized)
		if (false == DeviceInit())
			return VERIFY_ERR;

	if (current_op_mode == STAND_ALONE_VERIFY || current_op_mode == READ_VERIFY)
		sector_quantity = m_devparms_p->sector_quantity;
	else
		sector_quantity = m_devparms_p->sector_quantity - 1; // without CFG area

	// get write unit for User Code Flash
	if (false == GetDeviceAreaInfo(UCF, start_address_in_dev, end_address_in_dev, wr_unit))
		m_prg_api_p->ThrowException(ALG_LOGIC_ERR_S, __LINE__, __file);

	WORD block = 0;
	do
	{
		// see if block has to be programmed
		if (m_prg_api_p->GetSectorFlag(SectorOp::PROGRAM_SECTOR_OP, block))
		{
			start_address_in_mem = m_devsectors_p[block].begin_address;
			end_address_in_mem = m_devsectors_p[block].end_address;
			start_address_in_dev = start_address_in_mem;

			if (block == m_DF_block_nr)
			{
				if ((current_op_mode == READ_VERIFY) && (m_UF_filled0xFF == false))
					continue; // data flash is undefined
				if (false == GetDeviceAreaInfo(UDF, start_address_in_dev, end_address_in_dev, wr_unit))
					m_prg_api_p->ThrowException(ALG_LOGIC_ERR_S, __LINE__, __file);
			}
			if (block == m_CFG_block_nr)
			{
				if (false == GetDeviceAreaInfo(CFG, start_address_in_dev, end_address_in_dev, wr_unit))
					m_prg_api_p->ThrowException(ALG_LOGIC_ERR_S, __LINE__, __file);
				verify_stat = VerifyBlock(start_address_in_mem, start_address_in_dev, end_address_in_mem - start_address_in_mem + 1);
				continue; // we are done
			}

			address_cnt = start_address_in_mem;
			do
			{
				if (GetAreaForProcessing(end_address_in_mem, wr_unit, end_address_in_mem - start_address_in_mem + 1, address_cnt, area_start_address_in_mem, area_end_address_in_mem))
					verify_stat = VerifyBlock(area_start_address_in_mem, start_address_in_dev + (area_start_address_in_mem - start_address_in_mem), area_end_address_in_mem - area_start_address_in_mem + 1);
			} while (address_cnt <= end_address_in_mem && verify_stat == OPERATION_OK);

			// check for any system events
			if (m_prg_api_p->SysEvtChk())
				return HARDWARE_ERR; // O.C. or Adapter change - return immediately
		}							 // end of if 'sector' was to be programmed
	} while (++block < sector_quantity && verify_stat == OPERATION_OK);

	return verify_stat;
} // RA4E1_RA6E1::Verify()

////////////////////////////////////////////////////////////////////////////////
//          RA4E1_RA6E1::Secure() ()
// Inputs:
//
// Return:
//
// METHOD:
//
////////////////////////////////////////////////////////////////////////////////
RA4E1_RA6E1::DEV_STAT_E RA4E1_RA6E1::Secure()
{
#if (ALG_DEBUG > 0)
	PRINTF("RA4E1_RA6E1::Secure()\n");
#endif

	DWORD start_address_in_mem, end_address_in_mem;
	DWORD start_address_in_dev, end_address_in_dev;
	DWORD wr_unit;
	WORD block = m_CFG_block_nr;
	DEV_STAT_E secure_stat = OPERATION_OK;
	BYTE socket_stat;
	BYTE param[8];

	if (!m_prg_api_p->GetSectorFlag(SectorOp::PROGRAM_SECTOR_OP, block))
		return secure_stat; //nothing to do

		if (false == m_UART_initialized)
			if (false == DeviceInit())
				return SECURE_ERR;

		start_address_in_mem = m_devsectors_p[block].begin_address;
		end_address_in_mem = m_devsectors_p[block].end_address;
		if (false == GetDeviceAreaInfo(CFG, start_address_in_dev, end_address_in_dev, wr_unit))
			m_prg_api_p->ThrowException(ALG_LOGIC_ERR_S, __LINE__, __file);

		secure_stat = ProgramBlock(start_address_in_mem, start_address_in_dev, end_address_in_mem - start_address_in_mem + 1);
		if (secure_stat == OPERATION_OK)
			secure_stat = VerifyBlock(start_address_in_mem, start_address_in_dev, end_address_in_mem - start_address_in_mem + 1);

		if (secure_stat != OPERATION_OK)
			secure_stat = SECURE_ERR;
	
	
	return secure_stat;
} // RA4E1_RA6E1::Secure()

//-- END RA4E1_RA6E1 definitions
