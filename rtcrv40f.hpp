//----------------------------------------------------------------------------
// Name     :   rtcrv40f.hpp
//
// Purpose  :   header with class definitions for Renesas SINGLE VOLTAGE Flash Micros 
//              with serial interface: 
//              SUPPORTS FLASH PROCESS RV40F 
// Functions
// included :   -
//                    MM/DD/YY
//    Version 1.0   : 05/20/14 - Initial release
//            1.1   : 09/30/15 - added blank check for data flash
//            1.2   : 11/03/15 - prepared EXTENDED_OPTION commands
//            1.3   : 03/04/16 - added VerifyDFBlock(...) to allow verify of data units smaller than 16bytes
//            2.0   : 08/19/16 - added RPI file support (in RH850DataMapper.dll) with ALL device features supported
//                             - changed PRM_T struct --> devparam has to be adapted
//            2.2   : 11/16/16 - added feature: Code flash fill up can be disabled.
//            3.0   : 11/29/16 - speedup using FPGA sequencer F172 rev. 9. Programming time reduced to a quarter.
//            4.0   : 04/14/17 - added RH850/P1x-C class
//						4.2   : 11/28/17 - added VerifyArea(...)
// 						5.4   : 08/13/18 - added get_ICU_S_RegionSize(void)
//	     6.1    : 02/08/21 - added function WaituntillDevcieReady.
//	     7.0    : 03/02/21 - added function socketsimilaritycheck for ICUS supported devices.
//----------------------------------------------------------------------------
#ifndef RTCRV40F_HPP
#define RTCRV40F_HPP

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
#define DELAY_THRESHOLD    1000

typedef BYTE FRAMESTART_T;
typedef BYTE FRAMEEND_T;

//NEC flash uC defines
#define _RESET_PIN  A24
#define FLMD0_PIN   A22
#define FLMD1_PIN   A20
#define MODE0_PIN   A21 //P0_2 - (don't care, used for boundary scan only)
#define MODE1_PIN   A23 //P0_3 - (don't care, used for boundary scan only)

#define POWERGOOD_PIN   A19 //used, if the core voltage is generated by external voltage regulator
#define JP0_4_TRST_PIN  A17
#define JP0_3_TMS_PIN   A14
#define JP0_2_SCK_PIN   OE_NOT
#define JP0_1_SO_PIN    D0
#define JP0_0_SI_PIN    A25
//GP4 is used for the second power supply on the adapter (either directly or regulator supply)
#define VDD_AUX_PIN     GP4
#define VDD_AUX_EN_PIN  A0      //enable pin of the regulator if exists
#define VREG_VS0_PIN    A1      //voltage select pin of the regulator if exists
#define VREG_VS1_PIN    A2      //voltage select pin of the regulator if exists
#define VREG_VS2_PIN    A3      //voltage select pin of the regulator if exists

#define COMMODE_CSI_SEL_CNT   3
#define STARTUP_SCI_FREQUENCY 10000     //the serial clock at startup - set it lower than 40000 to make it symmetrical (50% duty cycle), otherwise communication fails
#define DEFAULT_TIMEOUT       2000000   //~ 2s generic timeout
#define ERASE_TIMEOUT         45000000  //~ 45 erase timeout

#define LONG_DELAY  true

//Verify types
#define CHECKSUM_VERIFY  1
#define DATA_VERIFY      2

//STATUS byte counts
#define CHECK_ST1      1
#define CHECK_ST1_ST2  2

#define CMD_BUFFER_SIZE       66
#define MIN_PAGE_SIZE        256  //Code flash
#define MAX_PAGE_SIZE       1024
#define COM_BUFFER_SIZE	    1030 // 1 + 2(LN) + 1 + 1024 + 1(CS) + 1(ETX)

#define DEVICE_NAME_LENGTH    16
#define MESSAGE_LENGTH        80
#define SIGNATURE_LENGTH      58

#define DF_START_IN_IMAGE    0x02000000
#define DF_MARKER_OFFSET     0x00100000 //for RV40F data flash

#define CF_MARKER_OFFSET     0x02180000 //1 byte per 256 CF bytes -> 16MB requires 64kB marker area


//HEX-file map defines (partly based on RPI/Consolidated-HEX requirements) 
#define PROT_OFFSET        0xF00000
#define OPBT_OFFSET(index) (0xF00028 + (index)*4)
//RPI (fixed by own design)
#define LOCK_BIT_OFFSET    0xF01000
#define OTP_BIT_OFFSET     0xF01100
#define ID_OFFSET          0xF01200
#define ICU_S_OFFSET       0xF013F0
#define ID_AUTH_OFFSET     0xF013F1  //password protection
#define SPD_OFFSET         0xF013F2  //Serial Programming Disable setting
#define IDSEL_OFFSET       0xF013FF

//P1XC
#define P1XC_CFG_DATA_OFFSET  0xF00040
#define P1XC_CFG_DATA_LENGTH  0xC0
#define P1XC_RSTVCT_OFFSET    0xF00060
#define P1XC_OPBT1_OFFSET     0xF00074 //critical data, it changes the startup behaviour of the PLL

//Length in bytes
#define DEVTYPE_LENGTH   0x18
#define OPBT_LENGTH      0x20
#define PROT_LENGTH      0x01
#define LB_LENGTH        0x62 //Lock/OTP bits length
#define EXT_OPBT_LENGTH  0x14 //Extended option bytes area length (OPBT8-12)

//device options
#define LB        		0x00000001
#define OTP       		0x00000002
#define SPD       		0x00000004
#define ID_AUTH   		0x00000008
#define OPBT      		0x00000010
#define OPBTEX    		0x00000020
#define ICU_S     		0x00000040
#define ICU_M     		0x00000080
#define ID_CODE   		0x00000100
#define CFG_WRITE 		0x00000200
#define ICU_S_ERASE_PROHIBITED	0x00000400

//Security Flag formats in the Security Byte - based on spec, but not confirmed by Renesas
//Bitindex  Description                       File/device
//  0       fixed "1"                            0/0
//  1       fixed "1"                            1/1
//  2       fixed "1"                            2/2
//  3       fixed "1"                            3/3
//  4       fixed "1"                            4/4
//  5       Erase Disable                        5/5
//  6       Write Disable                        6/6
//  7       Read Disable                         7/7
#define PROT_BYTE_READPROT_MSK  0x80
#define PROT_BYTE_PROGPROT_MSK  0x40
#define PROT_BYTE_ERASPROT_MSK  0x20

#define MODE_ENTRY_DELAY_MAXCNT   5

#define UNKNOWN                 0xFE

//ID AUTHENTICATION
#define ID_AUTHENTICATION_MODE  0x00
#define COMMAND_PROTECTION_MODE 0xFF
//ICU-S
#define ICU_S_VALID             0x00 //device status from spec
#define ICU_S_INVALID           0xFF //device status from spec
// skt status after ICUS validity check
#define SOCKETS_HAVE_DIFFERENT_STATUS	-1 
#define ICU_S_RESPONSIVE_VALID  	 0
#define ICU_S_RESPONSIVE_INVALID	 1



//struct of PR5 file parameter. EXTRACT only (use PFI.wsf to get extract)
struct PRM_T 
{
	DWORD FCPU;        //fCPU frequency in MHz
	BYTE TYPE[DEVTYPE_LENGTH];  //expcted device type
	BYTE SIGNATURE[SIGNATURE_LENGTH];

	DWORD RANSET[2];   //setting of input range
	DWORD RST_H_FLMD0_DLY_LIST[MODE_ENTRY_DELAY_MAXCNT]; //programming mode entry delays for different fw versions has to be added here 
};

struct ALG_CODE
{
	DWORD destinationAddress;
	WORD code_length;
	const BYTE* const code_p;
};
struct DEVICE_DESCRIPTOR_T
{
	DWORD program_start;        // program start in RAM
	BYTE* code_p;               // pointer to algorithm code
	WORD  code_length;
	int 	app_runtime;
};

//forward declaration
class StdWiggler;   


// class declaration of C_RV40F
// implements FLASH PROCESS FLOW-E for NEC Flash uController with 3-pin serial interface (SI, SO, SCK)
// RST   - A24
// FLMD0 - A22
// FLMD1 - A20
// SCK   - OE
// SI    - A25
// SO    - D0
/////////////////////////////////////////////////////////////
class C_RV40F : public FlashAlg2
{
	/////////////////////////////////////////////////////////////////////////////
	public: //parameter
		//frame types
		static const FRAMESTART_T SOH  = 0x01;
		static const FRAMESTART_T SOD  = 0x81;
		static const FRAMEEND_T   ETB  = 0x17;
		static const FRAMEEND_T   ETX  = 0x03;

	protected:  //parameter
		//flash firmware command settings
		static const BYTE INQUIRY_CMD          	   = 0x00;
		static const BYTE BLANKCHECK_CMD       	   = 0x10;
		static const BYTE ERASE_CMD            	   = 0x12;
		static const BYTE PROGRAM_CMD          	   = 0x13;
		static const BYTE EXTENDED_READ_CMD    	   = 0x14;
		static const BYTE EXTENDED_READ_CMD_ERR	   = 0x94;
		static const BYTE READ_CMD            	   = 0x15;
		static const BYTE VERIFY_CMD           	   = 0x16;
		static const BYTE CRC_CMD              	   = 0x18;
		static const BYTE CONFIG_CLEAR_CMD     	   = 0x1C;
		static const BYTE PROTECTION_SET_CMD   	   = 0x20;
		static const BYTE PROTECTION_GET_CMD   	   = 0x21;
		static const BYTE LOCKBIT_SET_CMD      	   = 0x22;
		static const BYTE LOCKBIT_GET_CMD      	   = 0x23;
		static const BYTE OPTION_SET_CMD       	   = 0x26; //OPBT0..7
		static const BYTE OPTION_GET_CMD      	   = 0x27; //OPBT0..7
		static const BYTE ID_AUTH_SET_CMD      	   = 0x28; //Serial programming ID set
		static const BYTE SP_DISABLE_CMD       	   = 0x29; //Serial programming disable (SPD)
		static const BYTE IDCODE_SET_CMD       	   = 0x2A;
		static const BYTE IDCODE_GET_CMD       	   = 0x2B;
		static const BYTE ID_AUTH_MODE_GET_CMD 	   = 0x2C; //FF - command prot mode; 00: ID auth mode
		static const BYTE OTP_SET_CMD          	   = 0x2D;
		static const BYTE OTP_GET_CMD          	   = 0x2E;
		static const BYTE ID_AUTH_CHECK_CMD    	   = 0x30;
		static const BYTE FREQUENCY_SET_CMD    	   = 0x32;
		static const BYTE DEVICE_TYPE_GET_CMD  	   = 0x38;
		static const BYTE SIGNATURE_GET_CMD    	   = 0x3A;
		static const BYTE VERSION_GET_CMD      	   = 0x3C;
		static const BYTE BOOTSTRAP_CMD        	   = 0x3F;
		static const BYTE ICU_S_OPTION_SET_CMD	   = 0x6E;
		static const BYTE ICU_S_VALIDATE_CMD   	   = 0x70; //Intelligent Crypto Unit - Slave (last 16kB of DF becomes secure memory)
		static const BYTE ICU_S_MODE_CHECK_CMD 	   = 0x71;
		static const BYTE ICU_S_MODE_CHECK_ERR	   = 0xF1;
		static const BYTE ICU_S_STATUS_VERIFY_ERR  = 0xE3;
		static const BYTE ICU_REGION_ERASE_CMD     = 0x72; //not supported by many devices, for internal use only!
		static const BYTE EXTENDED_OPTION2_SET_CMD = 0x74; //OPBT8..11 (Caution: it can render the device useless, e.g. ICU-M validation(parts of CF and DF are secure))
		static const BYTE EXTENDED_OPTION1_SET_CMD = 0x75; //OPBT12
		//static const BYTE EXTENDED_OPTION_CHK_CMD  = 0x76; // - not supported
		//static const BYTE ICU_M_MODE_CHECK_CMD = 0x77;     // - not supported

		//another const settings
		static const BYTE COMPARE_ALL_MASK   = 0x00;
		static const BYTE COMPARE_NOTHING    = 0xFF;

		StdWiggler* m_fpga_p;     

		bool m_fpga_speedup_supported;   //true only, when the appropriate FPGA is loaded

		PRM_T* m_tRV40F_Param_p;    //pointer to specific device param struct
		ALG_CODE* m_tBoot_code_p;  //pointer to bootloader code

		int m_sector_quantity; 
		BYTE m_akt_skt_msk;
		bool m_fTarget_initialized;
		bool m_fStartupMode;
		BYTE m_VerifyType;
		bool m_fDF_filled0xFF; //for devices with DF unit = 16 the user can select if undefined addreses are filled up with 0xFF
		bool m_fCF_filled0xFF; //The user can select if undefined addreses are filled up with 0xFF in code flash

		BYTE  m_op_frequency;  //operation frequency (XTAL), unit 100kHz
		DWORD m_sys_clk;       //system frequency (XTAL x PLL)   

		DEV_OP_E m_current_op_mode;
		BYTE m_current_CMD;    //current flash firmware command. Parameter used in different flows.
		
		WORD m_option_data_block; //nr of the option block
		DWORD m_reset_H_flmd0_pulse_start_wait;

		//device special features
		DWORD m_optionSupportedByDev;
		DWORD m_optionSelectedByUser;
		
		BYTE m_IDCODE_buffer[96];  //ID code for authentication: for up to 3 IDs of 32B
		BYTE* m_ID_buffer_p;
		bool m_fCfgClearCmdReq;
		bool m_fReadProtected;
		WORD m_ICU_S_RegionSize;
		int  m_ICU_S_Status[MAX_SOCKET_NUM];
		int  m_All_skts_ICU_S_INVALID;
		int  m_ICU_S_Erase;
		
		//auxiliary parameter for data flash handling    
		DWORD m_DF_erase_unit;
		DWORD m_DF_write_unit;
		WORD  m_DF_block_nr;
		
		WORD m_id_len;
		WORD m_signature_len;

	private:  //parameter
		BYTE m_cmd_buffer[CMD_BUFFER_SIZE];
		BYTE m_comm_buffer[COM_BUFFER_SIZE];

	//methods
	public:
		//ctor 
		C_RV40F (DEVPARMS* dparms_p, DEVPINS* dpins_p, DEVSECTORS* dsectors_p,
											BYTE opfreq_parm,
											PRM_T* tdprm_p, ALG_CODE* tBoot_code_p = NULL);
			
		virtual ~C_RV40F(); 
		virtual void Initialize (void); 
			
		// entry points for programmer system 
		virtual void PowerUp(void);
		virtual void PowerDown(void);

		virtual bool IDCheck();           // ID check
		virtual DEV_STAT_E  Read();       // Read device data

		virtual DEV_STAT_E  Program();    // Program device 
		virtual DEV_STAT_E  Verify();     // Verify device data against image data
		virtual DEV_STAT_E  BlankCheck(); // Check for blank device
		virtual DEV_STAT_E  Erase();      // Erase the device
		virtual DEV_STAT_E  Secure();     // Secure device

	protected: // methods

		BYTE swapBits(const BYTE dataByte, const BYTE bit_1, const BYTE bit_2);
		BYTE* GetCmdBufferP(void) { return &m_cmd_buffer[0];};
		void WriteCmdBuffer(const BYTE offset, const BYTE value);
		void WriteCmdBuffer(const BYTE cmd, const DWORD param1, const DWORD param2);
		void WaitLoop(const DWORD delay_us);
		bool GetSockets_ICU_S_Status();

		virtual void DoPowerUp (void); 
		virtual void DoPowerDown (void); 
		virtual void ResetToProgrammingMode(void);
		virtual DWORD GetDFAdressInDevice(DWORD address);
		virtual bool DeviceInit(void);
		virtual int CheckFlashID(void);
		int SerialWrite(const BYTE* data_p);
		int SetFrequency(const BYTE chFrequency);
		int InitializeFlashFirmware(ALG_CODE* tCode_p);
		DEV_STAT_E VerifyArea(DWORD startInMem, DWORD startInDev, DWORD areaSize);
		DEV_STAT_E RV_ProtBits(void);
		virtual DEV_STAT_E RV_OPBT(void);
		virtual DEV_STAT_E RV_IDCode(void);
		virtual DEV_STAT_E Prog_IDCode(const BYTE cmd);
		virtual DEV_STAT_E Erase_DataFlash_Area(DWORD DF_startaddress_in_device, DWORD DF_endaddress_in_device);
		virtual DEV_STAT_E Program_DataFlash_Area(WORD Block, WORD ICU_S_RegionSize);
		virtual DEV_STAT_E Verify_DataFlash_Area(WORD Block, WORD ICU_S_RegionSize);
		virtual DEV_STAT_E Erase_ICU_Area();


		
		virtual DEV_STAT_E RV_DeviceConfig(void);
		DEV_STAT_E RV_BlockProtBits(BYTE blockProtCmd);
	 
		BYTE WaitUntilDeviceReady(DEV_OP_E opMode, const WORD pinLvl, DWORD timeout);
		int SendFrame(const FRAMESTART_T startType, const FRAMEEND_T endType, const BYTE* buffer_p, const WORD length, BYTE includeACK = 0x00);
		int GetDataFrame(const FRAMEEND_T endType, BYTE* buffer_p, const WORD length, const WORD check_length, BYTE includeACK = 0x00, bool fLongWait = false);
		int ReadDataFrame(BYTE* buffer_p);
		bool DF_IsAreaEmpty(DWORD startAddress, DWORD areaSize);
		int VerifyDFblock(const FRAMEEND_T endType, BYTE* buffer_p, WORD pageSZ);
		bool CF_IsAreaEmpty(DWORD startAddress, DWORD areaSize);
		bool CF_IsAreaFragmented(DWORD startAddress, DWORD areaSize);
		WORD get_ICU_S_RegionSize(void);
		bool check_ICU_S_area(void);

		inline void StoreDataToRam_8Bit (const DWORD address, const BYTE data, const DWORD ram_window)
		{
			volatile BYTE* p = (volatile BYTE*)ram_window; 
			p += address;  
			*p = data; 
		}
			
		inline BYTE GetDataFromRam_8Bit (const DWORD address, const DWORD ram_window)
		{
			volatile BYTE* p = (volatile BYTE*)ram_window; 
			p += address;
			return *p; 
		}

		inline DWORD GetDataFromRam_32Bit (const DWORD address, const DWORD ram_window)
		{
			volatile BYTE* p = (volatile BYTE*)ram_window;
			p += address;
			return *(DWORD*)p;
		}

	private: // methods
		
};
//-- END OF class declaration of C_RV40F


// class declaration of C_RV40F_P1XC
// implements the extended flash programming commands for P1x-C family
/////////////////////////////////////////////////////////////
class C_RV40F_P1XC : public C_RV40F
{
	/////////////////////////////////////////////////////////////////////////////
	protected:  //parameter
		//flash firmware command settings
		static const BYTE FLASH_ID_CHECK_CMD   = 0x78;
		static const BYTE CONFIG_WRITE_CMD     = 0x79;
		static const BYTE CONFIG_VERIFY_CMD    = 0x7A;
		
	//methods
	public:
		//ctor 
		C_RV40F_P1XC(DEVPARMS* dparms_p, DEVPINS* dpins_p, DEVSECTORS* dsectors_p,
									BYTE opfreq_parm,
									PRM_T* tdprm_p, ALG_CODE* tBoot_code_p = NULL);
			
		virtual void Initialize (void); 
			
		// entry points for programmer system 

		virtual DEV_STAT_E  Secure();     // Secure device

	protected: // methods

		virtual DWORD GetDFAdressInDevice(DWORD address);
		virtual int CheckFlashID(void);
		virtual DEV_STAT_E RV_OPBT(void);
		virtual DEV_STAT_E RV_IDCode(void);
		virtual DEV_STAT_E Prog_IDCode(const BYTE cmd);
		virtual DEV_STAT_E RV_DeviceConfig(void);
};
//-- END OF class declaration of C_RV40F_P1XC

//class declaration  of C_RV40F_Kimball
//implements the custom routine for Kimball electronics refer DSR#62314
///////////////////////////////////////////////////////////////////////////////
class C_RV40F_Kimball : public C_RV40F_P1XC
{
		//////////////////////////////////////////////////////////////
			//methods
	public:
		//ctor 
		C_RV40F_Kimball(DEVPARMS* dparms_p, DEVPINS* dpins_p, DEVSECTORS* dsectors_p,
									BYTE opfreq_parm,
									PRM_T* tdprm_p,DEVICE_DESCRIPTOR_T* descriptor_p = NULL,  ALG_CODE* tBoot_code_p = NULL);
									
		
		virtual DEV_STAT_E  Secure();
		virtual DEV_STAT_E  Erase_CF_Block();
		virtual DEV_STAT_E  Program_CF_Block();
		virtual DEV_STAT_E  Verify_CF_Block();
		
		protected:
			DEVICE_DESCRIPTOR_T *m_descriptor_p;
			WORD m_sectors2load;
};
#endif RTCRV40F_HPP