
#include <stdint.h>
#include <string.h>

namespace SSD1306
{

inline void swap( uint8_t& a, uint8_t& b )
{
	uint8_t x = a;
	a = b;
	b = x;
}

class Base_I2C
{
public:
	Base_I2C() {}
protected:
	Base_I2C( const Base_I2C& orig __attribute__((unused)) ) {}
public:
	virtual ~Base_I2C() {}
private:
	Base_I2C& operator=( const Base_I2C& orig __attribute__((unused)) ) { return *this; }
public:
	virtual bool write( uint8_t slave_addr, uint8_t ctrl_byte,
			uint8_t* data, uint16_t length ) = 0;
};

#ifdef OPENCM3

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/i2c.h>

class OpenCM3_I2C : public Base_I2C
{
	uint32_t i2c;
public:
	// OpenCM3_I2C( I2C2 )
	// OpenCM3_I2C( uint32_t _i2c )
	OpenCM3_I2C()
	  : i2c(I2C2)
	{
	}
private:
	OpenCM3_I2C( const OpenCM3_I2C& orig ) : Base_I2C( orig ), i2c(0) {}
public:
	~OpenCM3_I2C()
	{
	}
private:
	OpenCM3_I2C& operator=( const OpenCM3_I2C& orig __attribute__((unused)) ) { return *this; }
public:
	void initialize()
	{
		/* Enable clocks for I2C2 and AFIO. */
		rcc_periph_clock_enable( RCC_I2C2 );
		rcc_periph_clock_enable( RCC_AFIO );

		/* Set alternate functions for the SCL and SDA pins of I2C2. */
		gpio_set_mode( GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
				GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN,
				GPIO_I2C2_SCL | GPIO_I2C2_SDA );

		/* Disable the I2C before changing any configuration. */
		i2c_peripheral_disable( i2c );

		/* APB1 is running at 36MHz. */
		i2c_set_clock_frequency( i2c, I2C_CR2_FREQ_36MHZ );

		/* 400KHz - I2C Fast Mode */
		i2c_set_fast_mode( i2c );


		// i2c_set_speed(); +++++

		/*
		* fclock for I2C is 36MHz APB2 -> cycle time 28ns, low time at 400kHz
		* incl trise -> Thigh = 1600ns; CCR = tlow/tcycle = 0x1C,9;
		* Datasheet suggests 0x1e.
		*/
		i2c_set_ccr( i2c, 0x1e );

		/*
		* fclock for I2C is 36MHz -> cycle time 28ns, rise time for
		* 400kHz => 300ns and 100kHz => 1000ns; 300ns/28ns = 10;
		* Incremented by 1 -> 11.
		*/
		i2c_set_trise( i2c, 0x0b );

		/*
		* This is our slave address - needed only if we want to receive from
		* other masters.
		*/
		// i2c_set_own_7bit_slave_address( i2c, 0x32 );

		/* If everything is configured -> enable the peripheral. */
		i2c_peripheral_enable( i2c );
	}
protected:
	enum I2cStatusFlags {
		I2C_SR_SB = I2C_SR1_SB,
		I2C_SR_ADDR = I2C_SR1_ADDR,
		I2C_SR_BTF = I2C_SR1_BTF,
		I2C_SR_ADD10 = I2C_SR1_ADD10,
		I2C_SR_STOPF = I2C_SR1_STOPF,
		I2C_SR_RxNE = I2C_SR1_RxNE,
		I2C_SR_TxE = I2C_SR1_TxE,
		I2C_SR_BERR = I2C_SR1_BERR,
		I2C_SR_ARLO = I2C_SR1_ARLO,
		I2C_SR_AF = I2C_SR1_AF,
		I2C_SR_OVR = I2C_SR1_OVR,
		I2C_SR_PECERR = I2C_SR1_PECERR,
		I2C_SR_TIMEOUT = I2C_SR1_TIMEOUT,
		I2C_SR_SMBALERT = I2C_SR1_SMBALERT,
		I2C_SR_MSL = I2C_SR2_MSL << 16,
		I2C_SR_BUSY = I2C_SR2_BUSY << 16,
		I2C_SR_TRA = I2C_SR2_TRA << 16,
		I2C_SR_GENCALL = I2C_SR2_GENCALL << 16,
		I2C_SR_SMBDEFAULT = I2C_SR2_SMBDEFAULT << 16,
		I2C_SR_SMBHOST = I2C_SR2_SMBHOST << 16,
		I2C_SR_DUALF = I2C_SR2_DUALF << 16
		// I2C_SR_PEC = I2C_SR2_PEC << 16
	};
	uint32_t get_sr()
	{
		uint32_t sr = I2C_SR1(i2c);
		sr |= I2C_SR2(i2c) << 16;
		return sr;
	}
	bool send_start( uint8_t addr )
	{
		uint16_t delay_counter;
		uint32_t reg32 __attribute__((unused));

		/* Send START condition. */
		i2c_send_start( i2c );

		/* Waiting for START is send and switched to master mode. */
		delay_counter = 65535;
		while( !( (I2C_SR1(i2c) & I2C_SR1_SB) & (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY)) ) )
		{
			if( delay_counter-- == 0 )
			{
				return false;
			}
		}

		/* Send destination address. */
		i2c_send_7bit_address( i2c, addr, I2C_WRITE );

		/* Waiting for address is transferred. */
		delay_counter = 65535;
		while( !(I2C_SR1(i2c) & I2C_SR1_ADDR) )
		{
			if( delay_counter-- == 0 )
			{
				return false;
			}
		}

		/* Cleaning ADDR condition sequence. */
		reg32 = I2C_SR2(i2c);

		return true;
	}
	bool send_data( uint8_t data )
	{
		uint16_t delay_counter;

		i2c_send_data( i2c, data );
		delay_counter = 65535;
		while( !(I2C_SR1(i2c) & I2C_SR1_BTF) )
		{
			if( delay_counter-- == 0 )
			{
				return false;
			}
		}

		return true;
	}
	bool send_stop()
	{
		uint16_t delay_counter;

		/* After the last byte we have to wait for TxE too. */
		delay_counter = 65535;
		while( !(I2C_SR1(i2c) & (I2C_SR1_BTF | I2C_SR1_TxE)) )
		{
			if( delay_counter-- == 0 )
			{
				return false;
			}
		}

		/* Send STOP condition. */
		i2c_send_stop(i2c);

		// +++ is here a delay/wait-for-flag needed ?

		return true;
	}
	void reset()
	{
		// +++++
	}
public:
	bool write( uint8_t slave_addr, uint8_t ctrl_byte,
			uint8_t* data, uint16_t length )
	{
		if( send_start( slave_addr ) && send_data( ctrl_byte ) )
		{
			bool ok = true;
			while( ok && length-- > 0 )
			{
				ok = send_data( *data++ );
			}
			if( ok && send_stop() )
			{
				return true;
			}
		}
		reset();
		return false;
	}
};
#endif


struct Font
{
	uint8_t height;
};

class Display
{
public:
	enum Constants
	{
		// +++++ these two should be configurable
		LCD_HEIGHT = 32,
		LCD_WIDTH = 128,
		BLOCK_COUNT = LCD_HEIGHT * LCD_WIDTH / (8*8),

		/**
		  I2C Slave Address

		  SSD1306 has to recognize the slave address before transmitting or
		  receiving any information by the I2C-bus. The device will respond
		  to the slave address following by the slave address bit ("SA0" bit)
		  and the read/write select bit ("R/W#" bit) with the following byte
		  format,

		  0 1 1 1 1 0 SA0 R/W#

		  "SA0" bit provides an extension bit for the slave address. Either
		  "0111100" or "0111101", can be selected as the slave address of
		  SSD1306. D/C# pin acts as SA0 for slave address selection.
		  "R/W#" bit is used to determine the operation mode of the I2C-bus
		  interface. R/W#=1, it is in read mode. R/W#=0, it is in write mode.
		*/
		SLAVE_ADDR_0 = 0x3C,
		SLAVE_ADDR_1 = 0x3D,

		/// data to be written to the GDDRAM is following
		CTRL_BYTE_FLAG_DATA = 1<<6,
		/// command to execute is following
		CTRL_BYTE_FLAG_CMD = 0<<6,
		/// a continuous stream of data or commands is following
		CTRL_BYTE_FLAG_STREAM = 0<<7,
		/// +++ this is just a guess: a single command or data byte is following
		CTRL_BYTE_FLAG_SINGLE = 1<<7,
		/// a stream of commands is following
		CTRL_BYTE_CMD_STREAM = CTRL_BYTE_FLAG_CMD | CTRL_BYTE_FLAG_STREAM,
		/// a stream of graphics data is following
		CTRL_BYTE_DATA_STREAM = CTRL_BYTE_FLAG_DATA | CTRL_BYTE_FLAG_STREAM,

		/**
		  Set Contrast Control 

		  81     1  0  0  0  0  0  0  1 
		  A[7:0] A7 A6 A5 A4 A3 A2 A1 A0

		  Double byte command to select 1 out of 256 contrast steps.
		  Contrast increases as the value increases.
		  (RESET = 7Fh )
		  
		*/
		SET_CONTRAST = 0x81,

		/**
		  Entire Display ON

		  A4/A5 1 0 1 0 0 1 0 X0

		  A4h, X0=0b: Resume to RAM content display (RESET). Output follows RAM content
		  A5h, X0=1b: Entire display ON.  Output ignores RAM content
		*/
		DISPLAY_ON = 0xA4,

		/**
		  Entire Display ON
		  @see SSD1306_DISPLAY_ON
		*/
		DISPLAY_ON_IGN_RAM = 0xA5,

		/**
		  Set Normal/Inverse Display

		  A6/A7 1 0 1 0 0 1 1 X0

		  A6h, X[0]=0b: Normal display (RESET)
				0 in RAM: OFF in display panel
				1 in RAM: ON in display panel
		  A7h, X[0]=1b: Inverse display
				 0 in RAM: ON in display panel
				1 in RAM: OFF in display panel
		*/
		SET_INVERSE_OFF = 0xA6,
	    /// @see SET_INVERSE_OFF
		SET_INVERSE_ON = 0xA7,

		/**
		  Set Display ON/OFF 

		  AE/AF 1 0 1 0 1 1 1 X0

		  AEh, X[0]=0b:Display OFF (sleep mode) (RESET)
		  AFh X[0]=1b:Display ON in normal mode
		*/
		SLEEP_MODE_ON = 0xAE,

		/**
		  Set Display Sleep ON/OFF
		  @see SSD1306_SLEEP_MODE_ON
		*/
		SLEEP_MODE_OFF = 0xAF,

		/**
		  Continuous Horizontal Scroll Setup

		  26/27 0  0 1 0 0 1   1 X0
		  A[7:0] 0 0 0 0 0 0  0  0
		  B[2:0] * * * * * B2 B1 B0
		  C[2:0] * * * * * C2 C1 C0
		  D[2:0] * * * * * D2 D1 D0
		  E[7:0] 0 0 0 0 0 0  0  0
		  F[7:0] 1 1 1 1 1 1  1  1

		  26h, X[0]=0, Right Horizontal Scroll
		  27h, X[0]=1, Left Horizontal Scroll
		  (Horizontal scroll by 1 column)
		  A[7:0] : Dummy byte (Set as 00h)
		  B[2:0] : Define start page address
			 000b   PAGE0 011b   PAGE3 110b   PAGE6
			 001b   PAGE1 100b   PAGE4 111b   PAGE7
			 010b   PAGE2 101b   PAGE5
		  C[2:0] : Set time interval between each scroll step in
				 terms of frame frequency
			  000b   5 frames          100b   3 frames
			  001b   64 frames         101b   4 frames
			  010b   128 frames        110b   25 frame
			  011b   256 frames        111b   2 frame
		  D[2:0] : Define end page address
			 000b   PAGE0 011b   PAGE3 110b   PAGE6
			 001b   PAGE1 100b   PAGE4 111b   PAGE7
			 010b   PAGE2 101b   PAGE5
				 The value of D[2:0] must be larger or equal
				  to B[2:0]
		  E[7:0] : Dummy byte (Set as 00h)
		  F[7:0] : Dummy byte (Set as FFh)
		  26h, X[0]=0, Right Horizontal Scroll
		  27h, X[0]=1, Left Horizontal Scroll
		  (Horizontal scroll by 1 column)
		  A[7:0] : Dummy byte (Set as 00h)
		  B[2:0] : Define start page address
			 000b | PAGE0
			 001b | PAGE1
			 010b | PAGE2
			 011b | PAGE3
			 100b | PAGE4
			 101b | PAGE5
			 110b | PAGE6
			 111b | PAGE7
		  C[2:0] : Set time interval between each scroll step in terms of frame frequency
			  000b | 5 frames  
			  001b | 64 frames 
			  010b | 128 frames
			  011b | 256 frames
			  100b | 3 frames
			  101b | 4 frames
			  110b | 25 frame
			  111b | 2 frame
		  D[2:0] : Define end page address
			 000b | PAGE0
			 001b | PAGE1
			 010b | PAGE2
			 011b | PAGE3
			 100b | PAGE4
			 101b | PAGE5
			 110b | PAGE6
			 111b | PAGE7
			 The value of D[2:0] must be larger or equal to B[2:0]
		  E[7:0] : Dummy byte (Set as 00h)
		  F[7:0] : Dummy byte (Set as FFh)
		*/
		SETUP_SCROLL_RIGHT = 0x26,

		/**
		  Continuous Horizontal Scroll Setup
		  @see SSD1306_SETUP_SCROLL_RIGHT
		*/
		SETUP_SCROLL_LEFT = 0x27,

		/**
		  Continuous Vertical and Horizontal Scroll Setup

		  29/2A  0 0 1  0  1  0  X1 X0
		  A[2:0] 0 0 0  0  0  0  0  0
		  B[2:0] * * *  *  *  B2 B1 B0
		  C[2:0] * * *  *  *  C2 C1 C0
		  D[2:0] * * *  *  *  D2 D1 D0
		  E[5:0] * * E5 E4 E3 E2 E1 E0

		  29h, X1X0=01b : Vertical and Right Horizontal Scroll
		  2Ah, X1X0=10b : Vertical and Left Horizontal Scroll
		   (Horizontal scroll by 1 column)
		  A[7:0] : Dummy byte
		  B[2:0] : Define start page address
			 000b | PAGE0
			 001b | PAGE1
			 010b | PAGE2
			 011b | PAGE3
			 100b | PAGE4
			 101b | PAGE5
			 110b | PAGE6
			 111b | PAGE7
		  C[2:0] : Set time interval between each scroll step in terms of frame frequency
			  000b | 5 frames  
			  001b | 64 frames 
			  010b | 128 frames
			  011b | 256 frames
			  100b | 3 frames
			  101b | 4 frames
			  110b | 25 frames
			  111b | 2 frame

		  D[2:0] : Define end page address
			 000b | PAGE0
			 001b | PAGE1
			 010b | PAGE2
			 011b | PAGE3
			 100b | PAGE4
			 101b | PAGE5
			 110b | PAGE6
			 111b | PAGE7
			 The value of D[2:0] must be larger or equal to B[2:0]
		  E[5:0] : Vertical scrolling offset
				 e.g. E[5:0]= 01h refer to offset =1 row
					  E[5:0] =3Fh refer to offset =63 rows
		  Note: No continuous vertical scrolling is available.
		*/
		SETUP_SCROLL_VERT_RIGHT = 0x29,

		/**
		  Continuous Vertical and Horizontal Scroll Setup
		  @see SSD1306_SETUP_SCROLL_VERT_LEFT
		*/
		SETUP_SCROLL_VERT_LEFT = 0x2A,

		/**
		  Deactivate scroll

		  2E 0 0 1 0 1 1 1 0

		  Stop scrolling that is configured by command 26h/27h/29h/2Ah.

		  Note: After sending 2Eh command to deactivate the scrolling action,
		  the ram data needs to be rewritten.

		*/
		DEACTIVATE_SCROLL = 0x2E,

		/**
		  Activate scroll

		  2F 0 0 1 0 1 1 1 1

		  Start scrolling that is configured by the scrolling setup commands:
		  26h/27h/29h/2Ah with the following valid sequences:
		  Valid command sequence 1: 26h ;2Fh.
		  Valid command sequence 2: 27h ;2Fh.
		  Valid command sequence 3: 29h ;2Fh.
		  Valid command sequence 4: 2Ah ;2Fh.
		  For example, if "26h; 2Ah; 2Fh." commands are issued, the setting in the
		  last scrolling setup command, i.e. 2Ah in this case, will be executed. In
		  other words, setting in the last scrolling setup command overwrites the
		  setting in the previous scrolling setup commands.  
		*/
		ACTIVATE_SCROLL = 0x2F,

		/**
		  Set Vertical Scroll Area

		  A3     1 0  1  0  0  0  1  1 
		  A[5:0] * * A5 A4 A3  A2 A1 A0 
		  B[6:0] * B6 B5 B4 B3 B2 B1 B0

		  A[5:0] : Set No. of rows in top fixed area. The No. of
				   rows in top fixed area is referenced to the
				   top of the GDDRAM (i.e. row 0).[RESET =
				   0]
		  B[6:0] : Set No. of rows in scroll area. This is the
				   number of rows to be used for vertical
				   scrolling. The scroll area starts in the first
				   row below the top fixed area. [RESET = 64]

		  Note
			(1) A[5:0]+B[6:0] <= MUX ratio
			(2) B[6:0] <= MUX ratio
			(3a) Vertical scrolling offset (E[5:0] in 29h/2Ah) < B[6:0]
			(3b) Set Display Start Line (X5X4X3X2X1X0 of 40h~7Fh) < B[6:0]
			(4) The last row of the scroll area shifts to the first row of the
				scroll area.
			(5) For 64d MUX display
			  A[5:0] = 0, B[6:0]=64 : whole area scrolls
			  A[5:0]= 0, B[6:0] < 64 : top area scrolls
			  A[5:0] + B[6:0] < 64 : central area scrolls
			  A[5:0] + B[6:0] = 64 : bottom area scrolls
		*/
		SET_VERT_SCROLL_AREA = 0xA3,

		/**
		  Set Lower Column Start Address for Page Addressing Mode

		  00~0F 0 0 0 0 X3 X2 X1 X0

		  Set the lower nibble of the column start address register for Page
		  Addressing Mode using X[3:0] as data bits. The initial display line
		  register is reset to 0000b after RESET.

		  Note: This command is only for page addressing mode
		*/
		SET_COL_ADDR_LOW = 0x00,

		/**
		  Set Higher Column Start Address for Page Addressing Mode

		  10~1F 0 0 0 1 X3 X2 X1 X0

		  Set the higher nibble of the column start address register for Page
		  Addressing Mode using X[3:0] as data bits. The initial display line
		  register is reset to 0000b after RESET.  

		  Note: This command is only for page addressing mode
		*/
		SET_COL_ADDR_HIGH = 0x10,

		/**
		  Set Memory Addressing Mode

		  20     0 0 1 0 0 0 0  0 
		  A[1:0] * * * * * * A1 A0 

		A[1:0] = 00b, Horizontal Addressing Mode
		A[1:0] = 01b, Vertical Addressing Mode
		A[1:0] = 10b, Page Addressing Mode (RESET)
		A[1:0] = 11b, Invalid  
		*/
		SET_ADDR_MODE = 0x20,
		/// @see SSD1306_SET_ADDR_MODE
		ADDR_MODE_HORIZ = 0x00,
		/// @see SSD1306_SET_ADDR_MODE
		ADDR_MODE_VERT = 0x01,
		/// @see SSD1306_SET_ADDR_MODE
		ADDR_MODE_PAGE = 0022,

		/**
		  Set Column Address

		  21     0 0  1  0  0  0  0  1 
		  A[6:0] * A6 A5 A4 A3 A2 A1 A0
		  B[6:0] * B6 B5 B4 B3 B2 B1 B0

		  Setup column start and end address
		  A[6:0]: Column start address, range: 0-127d, (RESET=0d)
		  B[6:0]: Column end address, range: 0-127d, (RESET =127d)

		  Note: This command is only for horizontal or vertical addressing mode.

		*/
		SET_COL_ADDR = 21,

		/**
		  Set Page Address

		  22     0 0 1 0 0 0   1  0 
		  A[2:0] * * * * * A2 A1 A0
		  B[2:0] * * * * * B2 B1 B0

		  Setup page start and end address
		  A[2:0]: Page start Address, range: 0-7d, (RESET = 0d)
		  B[2:0]: Page end Address, range: 0-7d, (RESET = 7d)

		  Note: This command is only for horizontal or vertical addressing mode.
		*/
		SET_PAGE_RANGE = 0x22,

		/**
		  Set Page Start Address for Page Addressing Mode

		  B0~B7 1 0 1 1 0 X2 X1 X0

		  Set GDDRAM Page Start Address (PAGE0~PAGE7) for Page Addressing
		  Mode using X[2:0].

		  Note: This command is only for page addressing mode
		*/
		SET_PAGE_ADDR = 0xB0,

		/**
		  Set Display Start Line

		  40~7F 0 1 X5 X4 X3 X2 X1 X0

		  Set display RAM display start line register from 0-63 using X5..0.
		  Display start line register is reset to 000000b during RESET.
		*/
		START_LINE = 0x40,

		/**
		  Set Segment Re-map

		  A0/A1 1 0 1 0 0 0 0 X0

		  A0h, X[0]=0b: column address 0 is mapped to SEG0 (RESET)
		  A1h, X[0]=1b: column address 127 is mapped to SEG0
		*/
		SET_SEG_REMAP_OFF = 0xA0,
		/// @see SSD1306_SET_SEG_REMAP_OFF
		SET_SEG_REMAP_ON = 0xA1,

		/**
		  Set Multiplex Ratio

		  A8     1 0 1  0  1  0  0  0 
		  A[5:0] * * A5 A4 A3 A2 A1 A0

		  Set MUX ratio to N+1 MUX
		  N=A[5:0] : from 16MUX to 64MUX, RESET=
					 111111b (i.e. 63d, 64MUX)
		  A[5:0] from 0 to 14 are invalid entry.
		*/
		SET_MULTIPLEX_RATIO = 0xA8,

		/**
		  Set COM Output Scan Direction

		  C0/C8 1 1 0 0 X3 0 0 0

		  C0h, X[3]=0b: normal mode (RESET) Scan from COM0 to COM[N-1]
		  C8h, X[3]=1b: remapped mode. Scan from COM[N-1] to COM0
		  Where N is the Multiplex ratio.
		*/
		SET_SCAN_DIR_UP = 0xC0,
		/// @see SSD1306_SET_SCAN_DIR_UP
		SET_SCAN_DIR_DOWN = 0xC8,

		/**
		  Set Display Offset

		  D3     1 1 0  1  0  0  1  1 
		  A[5:0] * * A5 A4 A3 A2 A1 A0

		  Set vertical shift by COM from 0d~63d
		  The value is reset to 00h after RESET.
		*/
		SET_DISP_OFFS = 0xD3,

		/**
		  Set COM Pins Hardware Configuration

		  DA     1 1 0  1  1 0 1 0
		  A[5:4] 0 0 A5 A4 0 0 1 0

		  A[4]=0b, Sequential COM pin configuration
		  A[4]=1b(RESET), Alternative COM pin configuration
		  A[5]=0b(RESET), Disable COM Left/Right remap
		  A[5]=1b, Enable COM Left/Right remap
		*/
		SET_COM_PIN_CONFIG = 0xDA,

		/**
		  Set Display Clock Divide Ratio/Oscillator Frequency

		  D5     1  1  0  1  0  1  0  1 
		  A[7:0] A7 A6 A5 A4 A3 A2 A1 A0

		  A[3:0] : Define the divide ratio (D) of the display clocks (DCLK):
				   Divide ratio= A[3:0] + 1, RESET is 0000b (divide ratio = 1)

		  A[7:4] : Set the Oscillator Frequency, FOSC. Oscillator Frequency
		           increases with the value of A[7:4] and vice versa.
				   RESET is 1000b
				   Range:0000b~1111b
				   Frequency increases as setting value increases.
		*/
		SET_CLOCK_CONFIG = 0xD5,

		/**
		  Set Pre-charge Period 

		  D9     1  1  0  1  1  0  0  1 
		  A[7:0] A7 A6 A5 A4 A3 A2 A1 A0

		  A[3:0]: Phase 1 period of up to 15 DCLK clocks,
		      0 is invalid entry (RESET=2h)
		  A[7:4]: Phase 2 period of up to 15 DCLK clocks,
		      0 is invalid entry (RESET=2h)
		*/
		SET_PRECHARGE_PERIOD = 0xD9,

		/**
		  Set VCOMH Deselect Level

		  DB     1 1  0  1  1 0 1 1
		  A[6:4] 0 A6 A5 A4 0 0 0 0

		  A[6:4]   Hex code   V COMH deselect level
		  000b     00h        ~ 0.65 x VCC
		  010b     20h        ~ 0.77 x VCC (RESET)
		  011b     30h        ~ 0.83 x VCC
		*/
		SET_VCOM_DESELECT_LEVEL = 0xDB,
		VCOM_LEVEL_0_65 = 0x00,
		VCOM_LEVEL_0_77 = 0x20,
		VCOM_LEVEL_0_83 = 0x30,

		/**
		  Charge Pump Setting

		  8D     1 0 0 0 1 1  0 1
		  A[7:0] * * 0 1 0 A2 0 0

		  A[2] = 0b, Disable charge pump(RESET)
		  A[2] = 1b, Enable charge pump during display on

		  Note: The Charge Pump must be enabled by the following command:
			8Dh; Charge Pump Setting
			14h; Enable Charge Pump
			AFh; Display ON
		*/
		CHARGE_PUMP = 0x8D,
		CHARGE_PUMP_ENABLED = 0x14,
		CHARGE_PUMP_DISABLED = 0x10,

		/**
		  NOP

		  E3 1 1 1 0 0 0 1 1

		  Command for no operation
		*/
		NOP = 0xE3
	};
	enum PixelOperation {
		SET_PIXEL = 0,
		CLEAR_PIXEL,
		INVERT_PIXEL
	};
private:
	Base_I2C& i2c;
	uint8_t i2c_addr;
	uint8_t gfx[LCD_HEIGHT*LCD_WIDTH/8];
	uint16_t changed_blocks[LCD_HEIGHT*LCD_WIDTH/(8*8*2)];
public:
	Display( Base_I2C& _i2c, bool second_addr=false )
	  : i2c(_i2c), i2c_addr( second_addr ? SLAVE_ADDR_1 : SLAVE_ADDR_0 )
	{
		const bool external_vcc = false;
		uint8_t i2c_cmd[] = {
			// i2c_addr,
			// CTRL_BYTE_CMD_STREAM,

			SLEEP_MODE_ON,
			SET_CLOCK_CONFIG, 0x80,
			SET_MULTIPLEX_RATIO, LCD_HEIGHT-1,
			SET_DISP_OFFS, 0,
			START_LINE | 0,
			CHARGE_PUMP,
				external_vcc ? CHARGE_PUMP_DISABLED : CHARGE_PUMP_ENABLED,
			SET_ADDR_MODE, ADDR_MODE_HORIZ,
			SET_PAGE_RANGE, 0, (LCD_HEIGHT/8)-1,
			SET_COL_ADDR, 0, LCD_WIDTH-1,

			// the following two rotate the display 180 degrees
			SET_SEG_REMAP_ON,
			SET_SCAN_DIR_DOWN,

			SET_COM_PIN_CONFIG, 0x08,
			SET_CONTRAST, 0x8F,
			SET_PRECHARGE_PERIOD, 0x22,
			SET_VCOM_DESELECT_LEVEL, VCOM_LEVEL_0_77, // ada uses invalid 0x40
			DISPLAY_ON,
			SET_INVERSE_OFF,
			DEACTIVATE_SCROLL,
			SLEEP_MODE_OFF
		};
		// i2c_write( i2c_cmd, sizeof(i2c_cmd) );
		i2c.write( i2c_addr, CTRL_BYTE_CMD_STREAM,
				i2c_cmd, sizeof(i2c_cmd) );
		clear();
#if 1 // ++++-- test
		gfx[0] = 0xFF;
		gfx[1] = 0x81;
		gfx[2] = 0xBD;
		gfx[3] = 0xA5;
		gfx[4] = 0xA5;
		gfx[5] = 0xBD;
		gfx[6] = 0x81;
		gfx[7] = 0xFF;
#endif
		update();
	}
protected:
	Display( const Display& orig ) : i2c(orig.i2c), i2c_addr(0) {}
public:
	~Display()
	{
	}
protected:
	Display& operator=( const Display& orig __attribute__((unused)) ) { return *this; }
private:
	/*
	void i2c_cmd( uint8_t data_cnt, uint8_t cmd, data1 )
	{
		i2c_cmd[1] = 0x00;
		i2c_cmd[2] = cmd;
	}
	*/
public:
	/*
	void sleep_mode( bool on )
	{
		i2c_cmd( 0, SSD1306_SLEEP_MODE_ON, 0 );
	}
	*/
	void clear()
	{
		memset( gfx, 0, LCD_HEIGHT*LCD_WIDTH/8 );
		// memset( changed_blocks, 0xFF, BLOCK_COUNT );
		for( uint8_t i=0; i<LCD_HEIGHT/8; i++ )
		{
			changed_blocks[i] = (1 << (LCD_WIDTH/8)) - 1;
		}
	}
	void update()
	{
		uint8_t i;
		int16_t start = -1;

		// +++++ this is not optimal for displays with less than 128 columns
		for( i=0; i<BLOCK_COUNT; i++ )
		{
			if( changed_blocks[i/8] >> (i&7) )
			{
				if( start < 0 )
				{
					start = i;
				}
			}
			else if( start >= 0 )
			{
				uint8_t j = i+1;
				if( j >= BLOCK_COUNT || (changed_blocks[j/8] >> (j&7)) == 0 )
				{
					transfer_blocks( start, i-1 );
				}
			}
		}
		if( start >= 0 )
		{
			transfer_blocks( start, i-1 );
		}
	}
private:
	void transfer_blocks( uint8_t start, uint8_t end )
	{
		uint16_t start_byte = start * 8;
		uint16_t byte_count = (end - start + 1) * 8;
		/* +++--  this is for page addressing
		uint8_t i2c_cmds[] = {
			SET_PAGE_ADDR | (start/(LCD_WIDTH/8)),
			SET_COL_ADDR_HIGH | ((start >> 1) & 0x0F),
			SET_COL_ADDR_LOW | ((start & 1) << 3)
		};
		*/
		// +++++ how to set the "cursor" ?
		uint8_t i2c_cmds[] = {
			42
		};
		i2c.write( i2c_addr, CTRL_BYTE_CMD_STREAM,
				i2c_cmds, sizeof(i2c_cmds) );
		i2c.write( i2c_addr, CTRL_BYTE_DATA_STREAM,
				gfx+start_byte, byte_count );
	}
public:
	void horiz_line( uint8_t x1, uint8_t x2, uint8_t y, PixelOperation op )
	{
		rect_filled( x1, y, x2, y, op );
	}
	void vert_line( uint8_t x, uint8_t y1, uint8_t y2, PixelOperation op )
	{
		rect_filled( x, y1, x, y2, op );
	}
	void rect( uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, PixelOperation op )
	{
		rect_filled( x1, y1, x2, y1, op );
		rect_filled( x1, y2, x2, y2, op );
		rect_filled( x1, y1, x1, y2, op );
		rect_filled( x2, y1, x2, y2, op );
	}
	void rect_filled( uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, PixelOperation op )
	{
		if( x1 >= LCD_WIDTH ) x1 = LCD_WIDTH - 1;
		if( x2 >= LCD_WIDTH ) x2 = LCD_WIDTH - 1;
		if( y1 >= LCD_HEIGHT ) y1 = LCD_HEIGHT - 1;
		if( y2 >= LCD_HEIGHT ) y2 = LCD_HEIGHT - 1;
		if( x1 > x2 ) swap( x1, x2 );
		if( y1 > y2 ) swap( y1, y2 );
		uint8_t start_page = y1 >> 3;
		uint8_t end_page = y2 >> 3;
		for( uint8_t page=start_page+1; page<end_page; page++ )
		{
			uint8_t* pgfx = gfx + page * LCD_WIDTH + x1;
			uint8_t pattern;
			if( page == start_page )
			{
				if( page == end_page )
				{
					// Y start and end are in the same page
					pattern = (1 << (y2 & 3)) - (1 << (y1 & 3));
				}
				else
				{
					// Y start is in this page, end is in a later page
					pattern = (1 << 8) - (1 << (y1 & 3));
				}
			}
			else
			{
				if( page == end_page )
				{
					// Y end is in this page but not the start
					pattern = (1 << (y2 & 3)) - 1;
				}
				else
				{
					// neither Y start nor end are in this page
					pattern = 0xFF;
				}
			}
			for( uint8_t x=x1; x<=x2; x++ )
			{
				switch( op )
				{
					case SET_PIXEL: *pgfx |= pattern; break;
					case CLEAR_PIXEL: *pgfx &= ~pattern; break;
					case INVERT_PIXEL: *pgfx ^= pattern; break;
				}
				pgfx++;
				changed_blocks[start_page] |= 1 << (x >> 3);
			}
		}
	}
private:
	void pixel_internal( uint8_t x, uint8_t y, PixelOperation op )
	{
		uint8_t* pgfx = gfx + x + (y&~3) * (LCD_WIDTH/8);
		uint8_t pattern = 1 << (y & 3);
		switch( op )
		{
			case SET_PIXEL: *pgfx |= pattern; break;
			case CLEAR_PIXEL: *pgfx &= ~pattern; break;
			case INVERT_PIXEL: *pgfx ^= pattern; break;
		}
		changed_blocks[y >> 3] |= 1 << (x >> 3);
	}
public:
	void pixel( uint8_t x, uint8_t y, PixelOperation op )
	{
		if( x < LCD_WIDTH && y < LCD_HEIGHT )
		{
			pixel_internal( x, y, op );
		}
	}
	void line( uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, PixelOperation op )
	{
		// is it h or v line?
		if( x1 == x2 || y1 == y2 )
		{
			rect_filled( x1, y1, x2, y2, op );
			return;
		}
		// hmm, does this make sense? it is maybe not what the caller wants
		// but we have to limit the values somehow
		if( x1 >= LCD_WIDTH ) x1 = LCD_WIDTH - 1;
		if( x2 >= LCD_WIDTH ) x2 = LCD_WIDTH - 1;
		if( y1 >= LCD_HEIGHT ) y1 = LCD_HEIGHT - 1;
		if( y2 >= LCD_HEIGHT ) y2 = LCD_HEIGHT - 1;
		// calculate the differences of the x and y values and their absolutes
		int16_t dx = x2 - x1;
		int16_t dy = y2 - y1;
		int16_t adx = dx >= 0 ? dx : -dx;
		int16_t ady = dy >= 0 ? dy : -dy;
		// is the line between -45 and +45 degrees?
		if( ady < adx )
		{
			// between -45 and +45 degrees, only x is incremented in every step
			// y is incremented or decremented depending on the angle
			if( dx < 0 )
			{
				// swap the coordinates, we want an increasing X coordinate
				swap( x1, x2 );
				swap( y1, y2 );
				dx = -dx;
				dy = -dy;
			}
			// set y start
			int16_t y = y1 << 8;
			// get the slope for the line
			int16_t a = ((ady << 8) + 255) / dx;
			// the offset at the end of the line, divided by two
			int16_t o = (a * dx - (ady << 8)) / 2;
			// is y increasing?
			if( dy >= 0 )
			{
				// adjust y so that after adding the offset it will increase
				// this centers the line in the pixel
				y += 256 - o;
			}
			else
			{
				// adjust y so that after subtracting the offset it will
				// decrease
				y += o;
				// slope is negative
				a = -a;
			}
			// draw the line
			for( uint8_t x=x1; x<=x2; x++ )
			{
				pixel_internal( x, y>>8, op );
				y += a;
			}
		}
		else
		{
			// between 45 and 135 degrees, only y is incremented in every step
			if( dy < 0 )
			{
				swap( x1, x2 );
				swap( y1, y2 );
				dx = -dx;
				dy = -dy;
			}
			int16_t x = x1 << 8;
			int16_t a = ((dx << 8) + 255) / dy;
			int16_t o = (a * dy - (dx << 8)) / 2;
			if( dx >= 0 )
			{
				x += 256 - o;
			}
			else
			{
				x += o;
				a = -a;
			}
			for( uint8_t y=y1; y<=y2; y++ )
			{
				pixel_internal( x>>8, y, op );
				x += a;
			}
		}
	}
	/* ugh, somebody please implement this ;)
	void circle( uint8_t x, uint8_t y, uint8_t r, PixelOperation op )
	{
	}
	*/
	void text( uint8_t x, uint8_t y, Font& font, PixelOperation op, const char* text, uint8_t length )
	{
		// ++++++++ implement
		if( !( x || y || font.height || op || text || length ) )
		{
			pixel_internal( 0, 0, INVERT_PIXEL );
			pixel_internal( 0, 0, INVERT_PIXEL );
		}
	}
	/* +++++ implement
	void icon( uint8_t x, uint8_t y, Icon& icon, PixelOperation op )
	{
	}
	*/
};

}; // namespace SSD1306

