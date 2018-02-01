
// #include <libopencm3/cm3/common.h>
// #include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

#define OPENCM3
#include "ssd1306.h"

using namespace SSD1306;

namespace {

void gpio_setup()
{
	rcc_periph_clock_enable( RCC_GPIOC );
	gpio_set_mode( GPIOC, GPIO_MODE_OUTPUT_2_MHZ,
					GPIO_CNF_OUTPUT_PUSHPULL, GPIO13 );
	gpio_clear( GPIOC, GPIO13 );
}

void usart_setup()
{
	/* Enable clocks for GPIO port A (for GPIO_USART1_TX) and USART1. */
	rcc_periph_clock_enable( RCC_GPIOA );
	rcc_periph_clock_enable( RCC_USART1 );

	/* Setup GPIO pin GPIO_USART1_TX/GPIO9 on GPIO port A for transmit. */
	gpio_set_mode( GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
				GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX );

	/* Setup UART parameters. */
	usart_set_baudrate( USART1, 115200 );
	usart_set_databits( USART1, 8 );
	usart_set_stopbits( USART1, USART_STOPBITS_1 );
	usart_set_mode( USART1, USART_MODE_TX_RX );
	usart_set_parity( USART1, USART_PARITY_NONE );
	usart_set_flow_control( USART1, USART_FLOWCONTROL_NONE );

	/* Finally enable the USART. */
	usart_enable( USART1 );
}

void my_delay()
{
	int i = 72e6/2/6;

	while( i > 0 )
	{
		i--;
		__asm__( "nop" );
	}
}

}; // anonymous namespace

int main(void)
{
	OpenCM3_I2C i2c;

	rcc_clock_setup_in_hse_16mhz_out_72mhz();
	gpio_setup();
	usart_setup();
	// i2c_setup();
	i2c.initialize();
	gpio_toggle( GPIOC, GPIO13 );
	Display display( i2c );
	gpio_toggle( GPIOC, GPIO13 );

	/* Send a message on USART1. */
	usart_send(USART1, 's');
	usart_send(USART1, 't');
	usart_send(USART1, 'm');
	usart_send(USART1, '\r');
	usart_send(USART1, '\n');



	while( true )
	{
		my_delay();
		gpio_toggle( GPIOC, GPIO13 );
	}

	return 0;
}

