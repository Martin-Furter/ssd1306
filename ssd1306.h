
#ifndef SSD1306_H
#define SSD1306_H

#include <stdint.h>
#include <string.h>

namespace SSD1306
{

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

struct Font
{
	// height in bytes (8 pixel)
	uint8_t height;
	// width of a character in pixels without the space
	int8_t width;
	// space between two characters
	uint8_t space;
	// value of the first character
	uint8_t first_char;
	// the number of characters in this font
	uint8_t char_count;
	// the character data
	const uint8_t* data;
};

extern const Font font_5x8;
extern const Font font_8x8;
extern const Font font_8x12;
extern const Font font_16x16;
extern const Font font_7seg_32x50;

class Display
{
public:
	// +++++ these two should be configurable
	enum {
		HEIGHT_PIX = 32,
		WIDTH_PIX = 128,
		HEIGHT_BLK = HEIGHT_PIX/8,
		WIDTH_BLK = WIDTH_PIX/8,
		BLOCK_COUNT = HEIGHT_BLK * WIDTH_BLK
	};
	/*
	const uint8_t HEIGHT_PIX = 32;
	const uint8_t WIDTH_PIX = 128;
	const uint8_t HEIGHT_BLK = HEIGHT_PIX/8;
	const uint8_t WIDTH_BLK = WIDTH_PIX/8;
	const uint8_t BLOCK_COUNT = HEIGHT_BLK * WIDTH_BLK;
	*/
	enum Constants
	{

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
	uint8_t gfx[HEIGHT_BLK*WIDTH_PIX];
	uint16_t changed_blocks[HEIGHT_BLK];
public:
	Display( Base_I2C& _i2c, bool second_addr=false );
protected:
	Display( const Display& orig );
public:
	~Display();
protected:
	Display& operator=( const Display& orig __attribute__((unused)) );
public:
	void clear();
	void update();
private:
	void transfer_blocks( uint8_t page, uint8_t start, uint8_t end );
public:
	void horiz_line( uint8_t x1, uint8_t x2, uint8_t y, PixelOperation op )
	{
		rect_filled( x1, y, x2, y, op );
	}
	void vert_line( uint8_t x, uint8_t y1, uint8_t y2, PixelOperation op )
	{
		rect_filled( x, y1, x, y2, op );
	}
	void rect( uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2,
			PixelOperation op );
	void rect_filled( uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2,
			PixelOperation op );
private:
	void pixel_internal( uint8_t x, uint8_t y, PixelOperation op );
public:
	void pixel( uint8_t x, uint8_t y, PixelOperation op )
	{
		if( x < WIDTH_PIX && y < HEIGHT_PIX )
		{
			pixel_internal( x, y, op );
		}
	}
	void line( uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, PixelOperation op );
	/* ugh, somebody please implement this ;)
	void circle( uint8_t x, uint8_t y, uint8_t r, PixelOperation op )
	{
	}
	*/
	uint8_t text( uint8_t x, uint8_t y, const Font& font, PixelOperation op,
			const char* text, uint8_t length );
	/* +++++ implement
	void icon( uint8_t x, uint8_t y, Icon& icon, PixelOperation op );
	*/
};

}; // namespace SSD1306

#endif // SSD1306_H

