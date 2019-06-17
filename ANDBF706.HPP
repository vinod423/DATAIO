//----------------------------------------------------------------------------
// Name     :   ANDBF706.hpp
//
// Purpose  :   header with class definitions for ANDBF706
//              SPI slave mode is used for OTP programming
//              
// Functions
// included :   -
//
//
//----------------------------------------------------------------------------

#ifndef  ANDBF706_HPP
#define  ANDBF706_HPP

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

#define  MESSAGE_LENGTH     80

//pin defines
#define SYS_HWRST A24
#define JTAG_TRST A15

//SPI2 PINS
#define SPI2_CLK 	A31
#define SPI2_MISO	D0
#define SPI2_MOSI GP1
#define SPI2_SS		CE

#define APP_STATUS_PIN_D2		D2
#define PRG_STATUS_PIN_D3 	D3

#define APP_STATUS_PIN_D2_MASK		0xFFFB
#define PRG_STATUS_PIN_D3_MASK    0xFFF7
  
#define HOST_START_SINGLE_BIT_MODE 0x03  //SPI mode selected
 
 
//forward declaration
class StdWiggler;   

class C_AND_BF706xx : public FlashAlg2
{
  //parameter
  public: 

  protected:

		StdWiggler* m_fpga_p;     

  private:
    
  //methods    
  public:
    C_AND_BF706xx(DEVPARMS* dparms_p, DEVPINS* dpins_p, DEVSECTORS* dsectors_p);

    //dtor
    virtual ~C_AND_BF706xx (void);
    virtual void Initialize ();

    // programming methods
    virtual void PowerUp();
    virtual void PowerDown();

    // compare Device ID
    virtual bool IDCheck();

    // Check for blank device
    virtual DEV_STAT_E  BlankCheck();

    // Erase the device
    virtual DEV_STAT_E  Erase();

    // Program device
    virtual DEV_STAT_E  Program();

    // Verify device data against image data
    virtual DEV_STAT_E  Verify();

    // Read device data
    virtual DEV_STAT_E  Read();

    // secure device data
    virtual DEV_STAT_E  Secure();

  protected:
    // Helper Functions
    virtual void DoPowerUp();
    virtual void DoPowerDown();
    
		DWORD GetApplicationLength(void);
    
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
		
    
};  // End of C_AND_BF706xx class

#endif ANDBF706_HPP