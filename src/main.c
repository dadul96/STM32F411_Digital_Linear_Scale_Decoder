#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/flash.h>
#include "libopencm3/stm32/spi.h"
#include <libopencm3/stm32/f4/nvic.h>
#include <libopencm3/stm32/dma.h>
#include "delay.h"
#include "usb_serial.h"

/* preprocessor macros: */
#define SYS_CLOCK_MHZ (96U) // possible options (84U), (96U), (100U)  NOTE: 100MHz does not support USB
#define BUFFER_LEN (3U)

/* function prototypes: */
static void clock_setup(void);
static void gpio_setup(void);
static void configure_SPI1(void);
static void configure_DMA2(void);
static void wait_DMA2_disabled(void);
static void SPI1_NSS_high_between_bursts(void);

/* global variables: */
volatile uint8_t buffer[BUFFER_LEN] = {0};
volatile float pos = 0.f;
volatile uint8_t new_value = 0;

int main(void)
{
	clock_setup();
	gpio_setup();
	delay_init(SYS_CLOCK_MHZ);

	/* enable LED to indicate missing USB serial connection: */
	gpio_clear(GPIOC, GPIO13);

	/* initialize USB serial: */
	USB_Serial_init();

	/* disable LED to indicate established USB serial connection: */
	gpio_set(GPIOC, GPIO13);

	/* configure and enable SPI1: */
	configure_SPI1();

	/* setup DMA2 stream 0 interrupt: */
 	nvic_set_priority(NVIC_DMA2_STREAM0_IRQ, 0);
	nvic_enable_irq(NVIC_DMA2_STREAM0_IRQ);

	/* configure but not enable DMA2 stream 0: */
	configure_DMA2();

	/* synchronize SPI1 with incoming signal: */
	SPI_DR(SPI1);
	while (!(SPI_SR(SPI1) & SPI_SR_RXNE));
	SPI_DR(SPI1); // clear register
	spi_set_nss_high(SPI1);
	delay_ms(95); // wait until 5ms before next SPI cycle
	spi_set_nss_low(SPI1);
	SPI_DR(SPI1); // clear register once more

	/* enable DMA2 stream 0: */
	dma_enable_stream(DMA2, DMA_STREAM0);


	while (1)
	{
		/* wait for new value: */
		while (!new_value);
		new_value = 0;

		/* calculate position in mm: */
		uint32_t unsigned_pos = buffer[BUFFER_LEN-3] + (256*buffer[BUFFER_LEN-2]);
		int32_t signed_pos = unsigned_pos;
		if (buffer[BUFFER_LEN-1] == 0)
		{
			signed_pos *= -1;
		}
		pos = ((float)signed_pos/100.f);

		/* print postion to serial console: */
		USB_Serial_write_float(pos);

		/* wait for DMA2 stream 0 disabled: */
		wait_DMA2_disabled();

		/* set SPI1 NSS pin high between signal bursts to reject noise and stay in sync with signal: */
		SPI1_NSS_high_between_bursts();

		/* enable DMA2 stream 0: */
		dma_enable_stream(DMA2, DMA_STREAM0);
	}
	return 0;
}


static void clock_setup(void)
{
	#if (SYS_CLOCK_MHZ == (100U))
		const struct rcc_clock_scale rcc_hse_25mhz_3v3_100MHz = {
			.pllm = 12,
			.plln = 96,
			.pllp = 2,
			.pllq = 4,
			.pllr = 0,
			.pll_source = RCC_CFGR_PLLSRC_HSE_CLK,
			.hpre = RCC_CFGR_HPRE_DIV_NONE,
			.ppre1 = RCC_CFGR_PPRE_DIV_2,
			.ppre2 = RCC_CFGR_PPRE_DIV_NONE,
			.voltage_scale = PWR_SCALE1,
			.flash_config = FLASH_ACR_DCEN | FLASH_ACR_ICEN | FLASH_ACR_LATENCY_3WS,
			.ahb_frequency  = 100000000,
			.apb1_frequency = 50000000,
			.apb2_frequency = 100000000
		};
		/* set STM32 to 100 MHz: */
		rcc_clock_setup_pll(&rcc_hse_25mhz_3v3_100MHz);

	#elif (SYS_CLOCK_MHZ == (96U))
		const struct rcc_clock_scale rcc_hse_25mhz_3v3_96MHz = {
			.pllm = 25,
			.plln = 192,
			.pllp = 2,
			.pllq = 4,
			.pllr = 0,
			.pll_source = RCC_CFGR_PLLSRC_HSE_CLK,
			.hpre = RCC_CFGR_HPRE_DIV_NONE,
			.ppre1 = RCC_CFGR_PPRE_DIV_2,
			.ppre2 = RCC_CFGR_PPRE_DIV_NONE,
			.voltage_scale = PWR_SCALE1,
			.flash_config = FLASH_ACR_DCEN | FLASH_ACR_ICEN | FLASH_ACR_LATENCY_3WS,
			.ahb_frequency  = 96000000,
			.apb1_frequency = 48000000,
			.apb2_frequency = 96000000
		};
		/* set STM32 to 100 MHz: */
		rcc_clock_setup_pll(&rcc_hse_25mhz_3v3_96MHz);

	#elif (SYS_CLOCK_MHZ == (84U))
		/* set STM32 to 84 MHz: */
		rcc_clock_setup_pll(&rcc_hse_25mhz_3v3[RCC_CLOCK_3V3_84MHZ]);

	#else
		#error "wrong SYS_CLOCK_MHZ selected"

	#endif

	/* enable GPIO clock: */
	rcc_periph_clock_enable(RCC_GPIOC); // PORT C for LED
	rcc_periph_clock_enable(RCC_GPIOA); // PORT A for USB and SPI1
	rcc_periph_clock_enable(RCC_GPIOB); // PORT B for SPI1

	/* set clock source for systick counter: */
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);

	/* enable clock for USB: */
	rcc_periph_clock_enable(RCC_OTGFS);

	/* configure clock for SPI1: */
	rcc_periph_clock_enable(RCC_SPI1);

	/* configure clock for DMA2: */
	rcc_periph_clock_enable(RCC_DMA2);
}


static void gpio_setup(void)
{
	/* configure LED pin: */
	gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO13);
	gpio_set_output_options(GPIOC,  GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO13);

	/* configure USB pins: */
	gpio_mode_setup(GPIOA,  GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO11|GPIO12);
	gpio_set_output_options(GPIOA,  GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO11|GPIO12);
    gpio_set_af(GPIOA, GPIO_AF10, GPIO11|GPIO12);

	/* configure SPI pins: */
	gpio_mode_setup(GPIOA,  GPIO_MODE_AF, GPIO_PUPD_PULLDOWN, GPIO15);
	gpio_set_output_options(GPIOA,  GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO15);
    gpio_set_af(GPIOA, GPIO_AF5, GPIO15);
	gpio_mode_setup(GPIOB,  GPIO_MODE_AF, GPIO_PUPD_PULLDOWN, GPIO3|GPIO4|GPIO5);
	gpio_set_output_options(GPIOB,  GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO3|GPIO4|GPIO5);
	gpio_set_af(GPIOB, GPIO_AF5, GPIO3|GPIO4|GPIO5);
}


static void configure_SPI1(void)
{
	rcc_periph_reset_pulse(RST_SPI1);
	spi_disable(SPI1);
	spi_init_master(SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_2, SPI_CR1_CPOL_CLK_TO_1_WHEN_IDLE, SPI_CR1_CPHA_CLK_TRANSITION_2, SPI_CR1_DFF_8BIT, SPI_CR1_LSBFIRST);
	spi_set_slave_mode(SPI1);
	spi_enable_software_slave_management(SPI1);
	spi_set_receive_only_mode(SPI1);
	spi_set_nss_low(SPI1);
	spi_enable_rx_dma(SPI1);
	spi_enable(SPI1);
}


static void configure_DMA2(void)
{
	rcc_periph_reset_pulse(RST_DMA2);
	wait_DMA2_disabled();
	dma_set_peripheral_address(DMA2, DMA_STREAM0, (uint32_t)&SPI1_DR);
	dma_set_memory_address(DMA2, DMA_STREAM0, (uint32_t)&buffer[0]);
	dma_set_number_of_data(DMA2, DMA_STREAM0, BUFFER_LEN);
	dma_channel_select(DMA2, DMA_STREAM0, DMA_SxCR_CHSEL_3);
	dma_set_priority(DMA2, DMA_STREAM0, DMA_SxCR_PL_VERY_HIGH);
	dma_enable_direct_mode(DMA2, DMA_STREAM0);
	dma_enable_memory_increment_mode(DMA2, DMA_STREAM0);
	dma_set_peripheral_size(DMA2, DMA_STREAM0, DMA_SxCR_PSIZE_8BIT);
	dma_set_memory_size(DMA2, DMA_STREAM0, DMA_SxCR_MSIZE_8BIT);
	dma_enable_transfer_complete_interrupt(DMA2, DMA_STREAM0);
}


static void wait_DMA2_disabled(void)
{
	dma_disable_stream(DMA2, DMA_STREAM0);
	while (DMA_SCR(DMA2, DMA_STREAM0) & DMA_SxCR_EN);
	DMA_LISR(DMA2) = 0;
	DMA_HISR(DMA2) = 0;
}


static void SPI1_NSS_high_between_bursts(void)
{
	spi_set_nss_high(SPI1);
	delay_ms(95); // wait until 5ms before next SPI cycle
	spi_set_nss_low(SPI1);
	SPI_DR(SPI1); // clear register before SPI cycle
}


void dma2_stream0_isr(void)
{
	new_value = 1;
	dma_clear_interrupt_flags(DMA2, DMA_STREAM0, NVIC_DMA2_STREAM0_IRQ);
}