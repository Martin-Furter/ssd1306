
#include "ssd1306.h"
#ifdef __unix
#include <stdio.h>
#endif

using namespace SSD1306;

namespace {

inline void swap( uint8_t& a, uint8_t& b )
{
	uint8_t x = a;
	a = b;
	b = x;
}

inline bool all_bits_set( uint32_t value, uint32_t bits )
{
	return (value & bits) == bits;
}

};

Display::Display( Base_I2C& _i2c, bool second_addr )
	  : i2c(_i2c), i2c_addr( second_addr ? SLAVE_ADDR_1 : SLAVE_ADDR_0 )
{
	const bool external_vcc = false;
	uint8_t i2c_cmd[] = {
		// i2c_addr,
		// CTRL_BYTE_CMD_STREAM,

		SLEEP_MODE_ON,
		SET_CLOCK_CONFIG, 0x80,
		SET_MULTIPLEX_RATIO, HEIGHT_PIX-1,
		SET_DISP_OFFS, 0,
		START_LINE | 0,
		CHARGE_PUMP,
			external_vcc ? CHARGE_PUMP_DISABLED : CHARGE_PUMP_ENABLED,
		SET_ADDR_MODE, ADDR_MODE_PAGE,
		SET_PAGE_RANGE, 0, HEIGHT_BLK-1,
		SET_COL_ADDR, 0, WIDTH_PIX-1,

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
#if 0 // ++++-- test
	gfx[0] = 0xFF;
	gfx[1] = 0x81;
	gfx[2] = 0xBD;
	gfx[3] = 0xA5;
	gfx[4] = 0xA5;
	gfx[5] = 0xBD;
	gfx[6] = 0x81;
	gfx[7] = 0xFF;
#endif
#ifndef __unix
	update();
#endif
}

Display::Display( const Display& orig )
  : i2c(orig.i2c), i2c_addr(0)
{
}

Display::~Display()
{
}

Display& Display::operator=( const Display& orig __attribute__((unused)) )
{
	return *this;
}

void Display::clear()
{
	memset( gfx, 0, sizeof(gfx) );
	for( uint8_t i=0; i<HEIGHT_BLK; i++ )
	{
		changed_blocks[i] = (uint32_t(1) << WIDTH_BLK) - 1;
	}
}

#ifdef __unix
void Display::update()
{
	char ulborder[] = "+--------------------------------------------------------------------------------------------------------------------------------+\n";
	char line[132];

	printf( ulborder );
	line[0] = '|';
	line[WIDTH_PIX+1] = '|';
	line[WIDTH_PIX+2] = '\n';
	line[WIDTH_PIX+3] = 0;
	for( uint8_t y=0; y<HEIGHT_PIX; y++ )
	{
		for( uint8_t x=0; x<WIDTH_PIX; x++ )
		{
			uint8_t b = gfx[x + WIDTH_PIX * (y>>3)];
			line[x+1] = (b & (1 << (y & 7))) ? '*' : ' ';
		}
		printf( line );
	}
	printf( ulborder );
}
#else
void Display::update()
{
	uint8_t page;
	uint8_t i;
	int16_t start = -1;

	for( page=0; page<HEIGHT_BLK; page++ )
	{
		for( i=0; i<WIDTH_BLK; i++ )
		{
			if( changed_blocks[page] >> i )
			{
				if( start < 0 )
				{
					start = i;
				}
			}
			else if( start >= 0 )
			{
				uint8_t j = i+1;
				if( j >= WIDTH_BLK || (changed_blocks[page] >> j) == 0 )
				{
					transfer_blocks( page, start, i-1 );
				}
			}
		}
		if( start >= 0 )
		{
			transfer_blocks( page, start, i-1 );
		}
	}
}
#endif

void Display::transfer_blocks( uint8_t page, uint8_t start, uint8_t end )
{
	uint16_t start_byte = page * WIDTH_PIX + start * 8;
	uint16_t byte_count = (end - start + 1) * 8;
	uint8_t i2c_cmds[] = {
		uint8_t(SET_PAGE_ADDR | page),
		uint8_t(SET_COL_ADDR_HIGH | ((start >> 1) & 0x0F)),
		uint8_t(SET_COL_ADDR_LOW | ((start & 1) << 3))
	};
	i2c.write( i2c_addr, CTRL_BYTE_CMD_STREAM,
			i2c_cmds, sizeof(i2c_cmds) );
	i2c.write( i2c_addr, CTRL_BYTE_DATA_STREAM,
			gfx+start_byte, byte_count );
}

void Display::rect( uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, PixelOperation op )
{
	rect_filled( x1, y1, x2, y1, op );
	rect_filled( x1, y2, x2, y2, op );
	rect_filled( x1, y1, x1, y2, op );
	rect_filled( x2, y1, x2, y2, op );
}

void Display::rect_filled( uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, PixelOperation op )
{
	if( x1 >= WIDTH_PIX ) x1 = WIDTH_PIX - 1;
	if( x2 >= WIDTH_PIX ) x2 = WIDTH_PIX - 1;
	if( y1 >= HEIGHT_PIX ) y1 = HEIGHT_PIX - 1;
	if( y2 >= HEIGHT_PIX ) y2 = HEIGHT_PIX - 1;
	if( x1 > x2 ) swap( x1, x2 );
	if( y1 > y2 ) swap( y1, y2 );
	uint8_t start_page = y1 >> 3;
	uint8_t end_page = y2 >> 3;
	for( uint8_t page=start_page+1; page<end_page; page++ )
	{
		uint8_t* pgfx = gfx + page * WIDTH_PIX + x1;
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

void Display::pixel_internal( uint8_t x, uint8_t y, PixelOperation op )
{
	uint8_t* pgfx = gfx + x + (y&~3) * (WIDTH_PIX/8);
	uint8_t pattern = 1 << (y & 3);
	switch( op )
	{
		case SET_PIXEL: *pgfx |= pattern; break;
		case CLEAR_PIXEL: *pgfx &= ~pattern; break;
		case INVERT_PIXEL: *pgfx ^= pattern; break;
	}
	changed_blocks[y >> 3] |= 1 << (x >> 3);
}

void Display::line( uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, PixelOperation op )
{
	// is it h or v line?
	if( x1 == x2 || y1 == y2 )
	{
		rect_filled( x1, y1, x2, y2, op );
		return;
	}
	// hmm, does this make sense? it is maybe not what the caller wants
	// but we have to limit the values somehow
	if( x1 >= WIDTH_PIX ) x1 = WIDTH_PIX - 1;
	if( x2 >= WIDTH_PIX ) x2 = WIDTH_PIX - 1;
	if( y1 >= HEIGHT_PIX ) y1 = HEIGHT_PIX - 1;
	if( y2 >= HEIGHT_PIX ) y2 = HEIGHT_PIX - 1;
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
void Display::circle( uint8_t x, uint8_t y, uint8_t r, PixelOperation op )
{
}
*/

uint8_t Display::text( uint8_t x, uint8_t y, const Font& font, PixelOperation op, const char* text, uint8_t length )
{
	uint8_t w = 0;
	const uint8_t* data;

	if( y >= HEIGHT_PIX )
	{
		return 0;
	}
	// uint16_t char_bytes = font.width * ((font.height + 7) >> 3);
	uint8_t pixshift = y & 7;
	uint8_t iymax = font.height - (pixshift == 0 ? 1 : 0);
	while( length > 0 && x < WIDTH_PIX )
	{
		uint8_t c = uint8_t(*text);
		text++;
		length--;
		if( c >= font.first_char && c < (font.first_char + font.char_count) )
		{
			// data = font.data + char_bytes * (c - font.first_char);
			data = font.data + font.height * font.width * (c - font.first_char);
			printf( "D %d\n", data-font.data );
			for( uint8_t ix=0; ix<font.width; ix++ )
			{
				uint8_t carry = 0;
				for( uint8_t iy=0; iy<=iymax; iy++ )
				{
					if( (y + 8*iy) > HEIGHT_PIX )
					{
						data += font.height - iy;
						break;
					}
					uint8_t pattern = iy < font.height ? *data++ : 0;
					printf( "%d > %02X %02X\n", pixshift, carry, pattern );
					if( pixshift )
					{
						uint8_t tmp = carry | (pattern << pixshift);
						carry = pattern >> (pixshift ^ 7);
						pattern = tmp;
					}
					printf( "%d < %02X %02X\n", pixshift, carry, pattern );
					uint8_t* pgfx = gfx + (x + ix) + ((y >> 3) + iy) * WIDTH_PIX;
					printf( "%03d/%02d pattern %02X\n", ix, iy, pattern );
					switch( op )
					{
						case SET_PIXEL: *pgfx |= pattern; break;
						case CLEAR_PIXEL: *pgfx &= ~pattern; break;
						case INVERT_PIXEL: *pgfx ^= pattern; break;
					}
				}
			}
		}
		x += font.width + font.space;
		w += font.width + font.space;
	}
	return w;
}

/* +++++ implement
void Display::icon( uint8_t x, uint8_t y, Icon& icon, PixelOperation op )
{
}
*/


