//----------------------------------------------------------------------------
// Name			:		RTC_SYNERGY.HPP
//
// Purpose	:   header with class definitions for the RTC SYNERGY (ARM Cortex-Mx w/ UART interface)  family
//              
// Functions
// included	:   -
//
//
//----------------------------------------------------------------------------
// Change History (drop down):
//		Version 1.0   : 08/18/17SK - Initial release
//            1.1   : 09/06/17SK - added dummy blank check (req. for read)
//					
//
// Copyright 2017, Data I/O Corporation
//----------------------------------------------------------------------------
#ifndef RTC_SYNERGY_HPP
#define RTC_SYNERGY_HPP

#include "StdWiggler.hpp"

typedef BYTE SOCKET_STATUS_T; 
// evaluate SOCKET_STATUS_T as defined by the RR h/w:
// Bit 0 -> DUT1    0=compare pass 1=compare failed
// Bit 1 -> DUT2    0=compare pass 1=compare failed
// Bit 2 -> DUT3    0=compare pass 1=compare failed
// Bit 3 -> DUT4    0=compare pass 1=compare failed
#define CompareFailed(socket_stat) ((socket_stat) != 0)
// returns false if all devices passed compare, 
// returns true if at least one device failed compare
#define ComparePassed(socket_stat) ((socket_stat) == 0)

#define DELAY_MS(time) {((time)<20) ? (m_prg_api_p->DelayUS((time)*1000)): delay((time));}
//---------------------------------------------------------

typedef BYTE FRAMESTART_T;
typedef BYTE FRAMEEND_T;

//device pins used in UART programming mode:
#define RES_PIN     A24
#define MD_PIN      A20
#define TXD_PIN     CE  //device RXD
#define RXD_PIN     D0  //device TXD

#define MSG_LEN  120
#define PW_LEN    16

#define FRAME_SIZE     5    //frame overhead: SOH+LNH+LNL+SUM+ETX
#define TX_BUF_SZ   1024
#define RCV_BUF_SZ  1060 		//UART-FPGA: 1kB (+ overhead) receiver buffer per socket

#define RA6E1  1


#define MAX_AREA_CNT  4

//memory types (KOA - kind of area)
#define UCF  0x00  //user code flash
#define UDF  0x10  //user data flash
#define CFG  0x20  //config area

#define TIMEOUT_1MS				200
#define TIMEOUT_10MS		 2000
#define TIMEOUT_100MS		20000
#define TIMEOUT_1S		 200000
#define TIMEOUT_10S		2000000
#define TIMEOUT_100S 20000000

#define MARKER_OFFSET  0x1080000  //TLWin-DLL saves data marker at this offset

//Just for reference: Status Code List
//Code Description
//---- -------------------------------------------------------------------------------------------------------------------------
// 00H  OK <--- EXPECTED
// C0H  Unsupported command error
// C1H  Packet error (illegal length, missing ETX, etc)
// C2H  Checksum error
// C3H  Flow error
// D0H  Address error
// D4H  Baud rate margin error
// DAH  Protection error
// DBH  ID mismatch error
// DCH  Serial programming disable error
// E1H  Erase error
// E2H  Write error
// E7H  Sequence error
//---- -------------------------------------------------------------------------------------------------------------------------

struct DEVICE_DESCRIPTOR_T
{
	BYTE signature[41];
	BYTE area_info[4][25];  //up to 4 areas
	DWORD ID_address;       //ID address in the last sector
	DWORD startup_baud;
	DWORD baud_rate;
	DWORD rd_page_size;
};

//forward declaration
class StdWiggler;   

// class declaration of RA4E1_RA6E1
// implements FLASH UART protocol used by SYNERGY family
/////////////////////////////////////////////////////////////
class RA4E1_RA6E1 : public FlashAlg2
{
	/////////////////////////////////////////////////////////////////////////////
	public: //parameter
		//frame types
		static const FRAMESTART_T SOH  = 0x01; //start of command packet
		static const FRAMESTART_T SOD  = 0x81; //start of data packet
		static const FRAMEEND_T   ETX  = 0x03; //end of packet

	protected:  //parameter
		//flash firmware command settings
		static const BYTE INQUIRY_CMD   		= 0x00;
		static const BYTE ERASE_CMD     		= 0x12;
		static const BYTE WRITE_CMD     		= 0x13;
		static const BYTE READ_CMD      		= 0x15;
		static const BYTE DLM_STATE_REQ_CMD		= 0x2C;	
		static const BYTE ID_AUTH_CMD   		= 0x30;
		static const BYTE BAUD_SET_CMD  		= 0x34;
		static const BYTE SIGNATURE_CMD 		= 0x3A;
		static const BYTE AREA_INFO_CMD 		= 0x3B;
		static const BYTE INITALIZE_CMD			= 0x50;
		static const BYTE DLM_STATE_TRANSIT_CMD	= 0x71;

		static const BYTE GENERIC_CODE  = 0x55;

		//flash response codes
		static const BYTE STATUS_CODE_ACK 	= 0x00;
		static const BYTE BOOT_CODE_ACK   	= 0xC3;
		static const BYTE STATUS_FLOW_ERR 	= 0xC3;
		static const BYTE FILL_BYTES	  	= 0xFF;
		static const BYTE BOOT_CODE_ACK_C6	= 0xC6;

		//DLM State code
		static const BYTE DLM_STATE_CM 		 	= 0x01;
		static const BYTE DLM_STATE_SSD			= 0x02;
		static const BYTE DLM_STATE_NSECSD		= 0x03;
		static const BYTE DLM_STATE_DPL			= 0x04;
		static const BYTE DLM_STATE_LCK_DBG		= 0x05;
		static const BYTE DLM_STATE_LCK_BOOT	= 0x06;
		static const BYTE DLM_STATE_RMA_REQ		= 0x07;
		static const BYTE DLM_STATE_RMA_ACK		= 0x08;


		
		//another const settings
		static const BYTE COMPARE_ALL_MASK   = 0x00;
		static const BYTE COMPARE_NOTHING    = 0xFF;
		

		StdWiggler* m_fpga_p; //access UART FPGA (F152, rev >= 6)
		DEVICE_DESCRIPTOR_T *m_devInfo_p;
		
		bool m_UF_filled0xFF;
		BYTE m_IDCODE_buffer[PW_LEN];
		BYTE* m_ID_buffer_p;
		DWORD m_write_unit;
		WORD  m_DF_block_nr;
		WORD  m_CFG_block_nr;

		WORD m_bit_time;        //length of one bit in the serial bitstream in us. It depends on the actual baudrate
		bool m_UART_initialized; //is true, if the programming interface is up and functional
		bool m_Signature_logged;
	

		UARTRcvBuffer_S m_uart_rcv_buffer; //buffer for uart receiver (for all sockets)
		
	private:  //parameter

	//methods
	public:
		//ctor 
		RA4E1_RA6E1 (DEVPARMS* dparms_p, DEVPINS* dpins_p, DEVSECTORS* dsectors_p, DEVICE_DESCRIPTOR_T* devInfo_p);
		virtual void Initialize (void); 
			
		// entry points for programmer system 
		virtual void PowerUp(void);
		virtual void PowerDown(void);

		virtual bool IDCheck();           // ID check
		virtual DEV_STAT_E  Read();       // Read device data (not all devices are supported)

		virtual DEV_STAT_E  Program();    // Program device 
		virtual DEV_STAT_E  Verify();     // Verify device data against image data
		virtual DEV_STAT_E  BlankCheck() {return BLANKCHECK_ERR;}; // Check for blank device - in fact not used
		virtual DEV_STAT_E  Erase();      // Erase the device
		virtual DEV_STAT_E  Secure();     // Secure device

	protected: // methods

		virtual void DoPowerUp (void); 
		virtual void DoPowerDown (void); 
		bool ResetToProgrammingMode(void);
		int DeviceInit(void);

		bool GetAreaForProcessing(DWORD end_address, DWORD min_page_size, DWORD max_page_size, DWORD& address, DWORD& area_start_address, DWORD& area_end_address);
		bool GetDeviceAreaInfo(BYTE areaType, DWORD& areaStart, DWORD& areaEnd, DWORD& area_write_unit);
		BYTE CalcFrameCheckSum(const BYTE* buffer_p);
		void SendPacket(const FRAMESTART_T startType, const BYTE cmd, const BYTE* param_p = NULL, WORD param_length = 0);
		BYTE Verify_RxData(const BYTE* expected_p, const WORD dataCnt, const BYTE cmd = 0xFF, DWORD timeout = TIMEOUT_10MS, bool verbose = false, bool status_check = true);
		DEV_STAT_E VerifyBlock(DWORD start_in_mem, DWORD start_in_dev, DWORD area_size);
		DEV_STAT_E ProgramBlock(DWORD start_in_mem, DWORD start_in_dev, DWORD area_size);

	private: // methods
		
};
//-- END OF class declaration of RA4E1_RA6E1

#endif RTC_SYNERGY_HPP