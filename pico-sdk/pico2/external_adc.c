/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * https://github.com/steve-m/hsdaoh-rp2350
 *
 * https://www.aliexpress.com/item/1005003038271519.html
 * https://www.aliexpress.com/item/1005007954380779.html
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "hardware/vreg.h"
#include "hardware/pll.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/structs/bus_ctrl.h"

#include "adc_12bit_input.pio.h"
/*
#if (PICO_PIO_USE_GPIO_BASE != 1)
#warning "PICO_PIO_USE_GPIO_BASE is not set to 1, this application will not work correctly!"
#endif
*/
// Pico W devices use a GPIO on the WIFI chip for the LED,
// so when building for Pico W, CYW43_WL_GPIO_LED_PIN will be defined
#ifdef CYW43_WL_GPIO_LED_PIN
//#include "pico/cyw43_arch.h"
#endif

#ifndef LED_DELAY_MS
#define LED_DELAY_MS 250
#endif

/* The PIO is running with sys_clk/1, and needs 4 cycles per sample,
 * so the ADC clock is sys_clk/4 */
#define SYS_CLK (160 * 1000)	// 40 MHz ADC clock
//#define SYS_CLK	192000	// 48 MHz ADC clock
//#define SYS_CLK	320000	// 80 MHz ADC clock
//#define SYS_CLK	384000	// 96 MHz ADC clock, maximum that works on my Pico2 (with overvolting)

// For alignment of 3x16 bit words in the payload, so that every line starts with word 0
#define ADC_DATA_LEN	(RBUF_SLICE_LEN - 3)
// ADC is attached to GP0 - GP11 with clock on GP20
#define PIO_INPUT_PIN_BASE 23
#define PIO_OUTPUT_CLK_PIN 20

#define DMACH_PIO_PING 0
#define DMACH_PIO_PONG 1
// stream ID, CRC word and length/metadata word, so 3 reserved words
#define MODE_H_ACTIVE_PIXELS	1920
#define NUM_RESERVED_WORDS	3
#define RBUF_DEFAULT_SLICES	16
#define RBUF_SLICE_LEN		MODE_H_ACTIVE_PIXELS
//#define RBUF_MAX_DATA_LEN	(RBUF_SLICE_LEN - NUM_RESERVED_WORDS)
#define RBUF_DEFAULT_TOTAL_LEN	(RBUF_SLICE_LEN * RBUF_DEFAULT_SLICES)


static bool pio_dma_pong = false;
uint16_t ringbuffer[RBUF_DEFAULT_TOTAL_LEN];
int ringbuf_head = 2;

typedef struct
{
	bool active;
	uint16_t *rbuf;
	uint tail;
	uint head;
	uint rbuf_slices;
	uint16_t format;
	uint32_t srate;
	uint16_t len;
	uint64_t data_cnt;
	bool overflow;
} stream_t;

#define MAX_STREAMS		4
stream_t streams[MAX_STREAMS];


// Perform initialisation
int pico_led_init(void) {
    // A device like Pico that uses a GPIO for the LED will define PICO_DEFAULT_LED_PIN
    // so we can use normal GPIO functionality to turn the led on and off
    // s/pico2.h:38:#define PICO_DEFAULT_LED_PIN 25
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    return PICO_OK;
}

// Turn the led on or off
void pico_set_led(bool led_on) {
//#if defined(PICO_DEFAULT_LED_PIN)
    // Just set the GPIO on or off
    gpio_put(PICO_DEFAULT_LED_PIN, led_on);
//#endif
}

void hsdaoh_init() //int dstrength, int slewrate)
{
	for (uint i = 0; i < MAX_STREAMS; i++)
		streams[i].active = false;
    /* give the DMA the priority over the CPU on the bus */
	bus_ctrl_hw->priority = BUSCTRL_BUS_PRIORITY_DMA_W_BITS | BUSCTRL_BUS_PRIORITY_DMA_R_BITS;


}

int hsdaoh_add_stream(uint16_t stream_id, uint16_t format, uint32_t samplerate, uint length, uint slices, uint16_t *ringbuf)
{
	if (stream_id >= MAX_STREAMS)
		return -1;

	stream_t *stream = &streams[stream_id];
	stream->rbuf = ringbuf;
	stream->format = format;
	stream->srate = samplerate;
	stream->len = length;
	stream->rbuf_slices = slices;
	stream->tail = slices-1;
	stream->head = 0;
	stream->data_cnt = 0;
	stream->active = true;

	return 0;
} 

void hsdaoh_update_head(int stream_id, int head)
{
	if (streams[stream_id].tail == head)
		streams[stream_id].overflow = true;
	else
		streams[stream_id].head = head;
}

void __scratch_y("") pio_dma_irq_handler()
{
	uint ch_num = pio_dma_pong ? DMACH_PIO_PONG : DMACH_PIO_PING;
	dma_channel_hw_t *ch = &dma_hw->ch[ch_num];
	dma_hw->intr = 1u << ch_num;
	pio_dma_pong = !pio_dma_pong;

	ringbuf_head = (ringbuf_head + 1) % RBUF_DEFAULT_SLICES;

	ch->write_addr = (uintptr_t)&ringbuffer[ringbuf_head * RBUF_SLICE_LEN];
	ch->transfer_count = ADC_DATA_LEN/2;

	hsdaoh_update_head(0, ringbuf_head);
}


void init_pio_input(void)
{
	PIO pio = pio0;

	/* move up GPIO base of PIO to access all ADC pins */
	//pio_set_gpio_base(pio, 16);

	uint offset = pio_add_program(pio, &adc_12bit_input_program);
	uint sm_data = pio_claim_unused_sm(pio, true);

	adc_12bit_input_program_init(pio, sm_data, offset, PIO_INPUT_PIN_BASE, PIO_OUTPUT_CLK_PIN);
    dma_channel_config c;
	c = dma_channel_get_default_config(DMACH_PIO_PING);
	channel_config_set_chain_to(&c, DMACH_PIO_PONG);
	channel_config_set_dreq(&c, pio_get_dreq(pio, sm_data, false));
	channel_config_set_read_increment(&c, false);
	channel_config_set_write_increment(&c, true);
	channel_config_set_transfer_data_size(&c, DMA_SIZE_16);

    dma_channel_configure(
		DMACH_PIO_PING,
		&c,
		&ringbuffer[0 * RBUF_SLICE_LEN],
		&pio->rxf[sm_data],
		ADC_DATA_LEN/2,
		false
	);

    c = dma_channel_get_default_config(DMACH_PIO_PONG);
	channel_config_set_chain_to(&c, DMACH_PIO_PING);
	channel_config_set_dreq(&c, pio_get_dreq(pio, sm_data, false));
	channel_config_set_read_increment(&c, false);
	channel_config_set_write_increment(&c, true);
	channel_config_set_transfer_data_size(&c, DMA_SIZE_16);

	dma_channel_configure(
		DMACH_PIO_PONG,
		&c,
		&ringbuffer[1 * RBUF_SLICE_LEN],
		&pio->rxf[sm_data],
		ADC_DATA_LEN/2,
		false
	);

	dma_hw->ints0 |= (1u << DMACH_PIO_PING) | (1u << DMACH_PIO_PONG);
	dma_hw->inte0 |= (1u << DMACH_PIO_PING) | (1u << DMACH_PIO_PONG);
    irq_set_exclusive_handler(DMA_IRQ_0, pio_dma_irq_handler);
	irq_set_enabled(DMA_IRQ_0, true);

	dma_channel_start(DMACH_PIO_PING);


}
int main() {
    int rc = pico_led_init();
    bool led_on = false;
#ifdef OVERVOLT
	/* set maximum 'allowed' voltage without voiding warranty */
	vreg_set_voltage(VREG_VOLTAGE_MAX);
	sleep_ms(1);
#endif

    hard_assert(rc == PICO_OK);

    set_sys_clock_khz(SYS_CLK, true);
     // The previous line automatically detached clk_peri from clk_sys, and
    // attached it to pll_usb, so that clk_peri won't be disturbed by future
    // changes to system clock or system PLL. If we need higher clk_peri
    // i examples/ detached_clk_peri.c
    //
    /* set HSTX clock to sysclk/1 */
	hw_write_masked(
		&clocks_hw->clk[clk_hstx].div,
		1 << CLOCKS_CLK_HSTX_DIV_INT_LSB,
		CLOCKS_CLK_HSTX_DIV_INT_BITS
	);

    pll_init(pll_usb, 1, 1536 * MHZ, 4, 2);

	/* set USB clock to clk_usb/4 */
	hw_write_masked(&clocks_hw->clk[clk_usb].div,
            4 << CLOCKS_CLK_USB_DIV_INT_LSB,
            CLOCKS_CLK_USB_DIV_INT_BITS);

    /* set GPOUT0 clock at gpio 21 to USB PLL/10 -> 19.2 MHz, resulting in 75 kHz ADC sample rate (19.2M/256) */
	clock_gpio_init(21, CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB, 10);

    // The serial clock won't vary from this point onward, so we can configure
    // the UART etc.
    stdio_init_all();

    hsdaoh_init();
    hsdaoh_add_stream(0, 1, (SYS_CLK/4) * 1000, ADC_DATA_LEN, RBUF_DEFAULT_SLICES, ringbuffer);

    //	hsdaoh_start(); // Only for HSTX on core1
    init_pio_input();

    while (true) {
        printf("Hello, world %d!\n", led_on);
        pico_set_led(led_on);
        led_on = !led_on;
        sleep_ms(1000);
    }
}
