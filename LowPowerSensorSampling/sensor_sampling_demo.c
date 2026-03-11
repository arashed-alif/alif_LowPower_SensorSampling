
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <limits.h>

#include <pm.h>
#include <sys_clocks.h>
#include <pinconf.h>
#include <soc.h>
#include <cmsis_compiler.h>
#include <soc_dma_map.h>
#include <se_services_port.h>
#include <sys_utils.h>

#include <Driver_SPI.h>
#include <Driver_SPI_EX.h>
//#include <Driver_SPI_Private.h>
#include "LPTIMER_config.h"
#include "UTIMER_config.h"
#include "drv_counter.h"

#ifndef HW_REG32
#define HW_REG32(u,v) (*((volatile uint32_t *)(u + v)))
#endif

#if SPI0_DMA_TX_PERIPH_REQ == 20
#error "required edit in soc_dma_map.h - change SPI0_DMA_TX_PERIPH_REQ to 1"
#endif

#define SERVICES_check_response {if ((ret != 0) || (service_response != 0)) while(1) __WFI();}

#define SPI_INSTANCE            0
#define SPI_TRANSFER_COUNT      8
#define SPI_TRANSFER_MULTIPLE   20

extern ARM_DRIVER_SPI ARM_Driver_SPI_(SPI_INSTANCE);
static ARM_DRIVER_SPI *ptrSPI = &ARM_Driver_SPI_(SPI_INSTANCE);
extern uint32_t HFRC_CLK;
extern uint32_t HFXO_CLK;
extern uint32_t RTSS_HE_CLK;
extern uint32_t RTSS_HP_CLK;

uint8_t user_spi_rx_mcode[] = {
        0xBC, 0x01, 0x04, 0x50, 0x01, 0x04,
        0xBC, 0x00, 0x60, 0x30, 0x10, 0x48,
        0xBC, 0x02, 0x00, 0x00, 0x00, 0x00,
        0x22, 0x07, 0x20, 0x07, 0x35, 0x80,
        0x30, 0x80, 0x25, 0x80, 0x09, 0x38,
        0x07, 0x3C, 0x0B, 0x13, 0x34, 0x00,
        0x22, 0x07, 0x20, 0x07, 0x35, 0x80,
        0x30, 0x80, 0x25, 0x80, 0x09, 0x38,
        0x07, 0x3C, 0x0B, 0x13, 0x34, 0x00,
        0x5E, 0x00, 0xFE, 0x2C, 0x27
};

uint8_t user_spi_tx_mcode[] = {
        0xBC, 0x01, 0x05, 0x10, 0x01, 0x04,
        0xBC, 0x00, 0x00, 0x00, 0x00, 0x00,
        0xBC, 0x02, 0x60, 0x30, 0x10, 0x48,
        0x20, 0x07, 0x35, 0x40, 0x30, 0x40,
        0x25, 0x40, 0x09, 0x13, 0x38, 0x08,
        0x5C, 0xE0, 0xFF, 0x2C, 0x0F
};

volatile uint8_t spi_cb_status = 0;
volatile uint8_t buffer_select = 0;

uint32_t static spi_tx_buff[SPI_TRANSFER_COUNT] __attribute__((section(".bss.spi_dma_xfer_buffers")));
uint32_t static spi_rx_buff[SPI_TRANSFER_COUNT * SPI_TRANSFER_MULTIPLE * 2] __attribute__((section(".bss.spi_dma_xfer_buffers")));

static void configure_spi_pinmux()
{
    uint32_t padconf = PADCTRL_READ_ENABLE |
            PADCTRL_SCHMITT_TRIGGER_ENABLE |
            PADCTRL_OUTPUT_DRIVE_STRENGTH_4MA;

#if defined(ENSEMBLE_SOC_GEN2)

    /* Port 1 */
    HW_REG32(PINMUX_BASE, 0x20) = 2 | (padconf << 16);  /* P1_0 is SPI0_MISO */
    HW_REG32(PINMUX_BASE, 0x24) = 2 | (padconf << 16);  /* P1_1 is SPI0_MOSI */
    HW_REG32(PINMUX_BASE, 0x28) = 2 | (padconf << 16);  /* P1_2 is SPI0_CLK */
    HW_REG32(PINMUX_BASE, 0x2C) = 2 | (padconf << 16);  /* P1_3 is SPI0_SS0 */

#elif defined(ENSEMBLE_SOC_E1C)

    /* Port 5 */
    HW_REG32(PINMUX_BASE, 0xA0) = 3 | (padconf << 16);  /* P5_0 is SPI0_MISO */
    HW_REG32(PINMUX_BASE, 0xA4) = 3 | (padconf << 16);  /* P5_1 is SPI0_MOSI */
    HW_REG32(PINMUX_BASE, 0xA8) = 4 | (padconf << 16);  /* P5_2 is SPI0_SS0 */
    HW_REG32(PINMUX_BASE, 0xAC) = 3 | (padconf << 16);  /* P5_3 is SPI0_SCLK */

#else

    /* Port 5 */
    HW_REG32(PINMUX_BASE, 0xA0) = 4 | (padconf << 16);  /* P5_0 is SPI0_MISO */
    HW_REG32(PINMUX_BASE, 0xA4) = 4 | (padconf << 16);  /* P5_1 is SPI0_MOSI */
    HW_REG32(PINMUX_BASE, 0xA8) = 4 | (padconf << 16);  /* P5_2 is SPI0_SS0 */
    HW_REG32(PINMUX_BASE, 0xAC) = 3 | (padconf << 16);  /* P5_3 is SPI0_SCLK */

#endif

    /* DEBUG */
    HW_REG32(PINMUX_BASE, 0xB0) = padconf << 16; /* P5_4 is GPIO5_4 */
    HW_REG32(PINMUX_BASE, 0xB8) = padconf << 16; /* P5_6 is GPIO5_6 */
    HW_REG32(GPIO5_BASE,  0x00) = 0;        /* clear GPIO5 port */
    HW_REG32(GPIO5_BASE,  0x04) = 7U << 4;  /* GPIO5_4 and GPIO5_6 are outputs */
}

static void SPI_cb_func(uint32_t event)
{
    spi_cb_status |= event;
    buffer_select ^= 1;
}

void low_power_sensor_sampling_demo()
{
    int32_t ret;
    uint32_t service_response = 0;
    uint32_t spi_rx_offset;
    uint32_t spi_tx_data[SPI_TRANSFER_COUNT] = {0x111111, 0x222222, 0x333333, 0x444444, 0x555555, 0x666666, 0x777777, 0x888888};
    uint32_t application_buffer[SPI_TRANSFER_COUNT * SPI_TRANSFER_MULTIPLE] = {0};
    uint32_t spi_freq   = 4800000;  // actual spi_freq is 5 MHz on PLL and 4.8 MHz on HFXO
    uint32_t spi_config = ARM_SPI_MODE_MASTER |
            ARM_SPI_SS_MASTER_HW_OUTPUT |
            ARM_SPI_CPOL0_CPHA0 |
            ARM_SPI_DATA_BITS(24);

    /* place the outgoing data into the spi_tx_buff */
    for (uint32_t i = 0; i < SPI_TRANSFER_COUNT; i++) {
        spi_tx_buff[i] = spi_tx_data[i];
    }

    refclk_cntr_init();
    configure_spi_pinmux();

    ret = ptrSPI->Initialize(SPI_cb_func);
    while(ret != ARM_DRIVER_OK);

    ret = ptrSPI->PowerControl(ARM_POWER_FULL);
    while(ret != ARM_DRIVER_OK);

    ret = ptrSPI->Control(spi_config, spi_freq);
    while(ret != ARM_DRIVER_OK);

    ret = ptrSPI->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_ACTIVE);
    while(ret != ARM_DRIVER_OK);

    /* modify the user_rx_mcode with spi_rx_buff address */
    uint32_t spi_rx_buff_global = LocalToGlobal(spi_rx_buff);
    uint32_t spi_rx_dma_increment = 65536 - (SPI_TRANSFER_COUNT * SPI_TRANSFER_MULTIPLE * sizeof(uint32_t) * 2);
    user_spi_rx_mcode[14] = (uint8_t) ((spi_rx_buff_global >>  0) & 0xFF);
    user_spi_rx_mcode[15] = (uint8_t) ((spi_rx_buff_global >>  8) & 0xFF);
    user_spi_rx_mcode[16] = (uint8_t) ((spi_rx_buff_global >> 16) & 0xFF);
    user_spi_rx_mcode[17] = (uint8_t) ((spi_rx_buff_global >> 24) & 0xFF);
    user_spi_rx_mcode[19] = (uint8_t) (SPI_TRANSFER_MULTIPLE - 1);
    user_spi_rx_mcode[37] = (uint8_t) (SPI_TRANSFER_MULTIPLE - 1);
    user_spi_rx_mcode[55] = (uint8_t) (spi_rx_dma_increment >> 0) & 0xFF;
    user_spi_rx_mcode[56] = (uint8_t) (spi_rx_dma_increment >> 8) & 0xFF;

    /* modify the user_tx_mcode with spi_tx_buff address */
    uint32_t spi_tx_buff_global = LocalToGlobal(spi_tx_buff);
    user_spi_tx_mcode[8]  = (uint8_t) ((spi_tx_buff_global >>  0) & 0xFF);
    user_spi_tx_mcode[9]  = (uint8_t) ((spi_tx_buff_global >>  8) & 0xFF);
    user_spi_tx_mcode[10] = (uint8_t) ((spi_tx_buff_global >> 16) & 0xFF);
    user_spi_tx_mcode[11] = (uint8_t) ((spi_tx_buff_global >> 24) & 0xFF);
    user_spi_tx_mcode[21] = (uint8_t) (SPI0_DMA_TX_PERIPH_REQ << 3);
    user_spi_tx_mcode[23] = (uint8_t) (SPI0_DMA_TX_PERIPH_REQ << 3);
    user_spi_tx_mcode[25] = (uint8_t) (SPI0_DMA_TX_PERIPH_REQ << 3);

    spi_cb_status = 0;
    buffer_select = 1;

    ret = ptrSPI->Control(ARM_SPI_USE_CUSTOM_DMA_MCODE_RX, (uint32_t) &user_spi_rx_mcode[0]);
    while(ret != ARM_DRIVER_OK);


    ret = ptrSPI->Control(ARM_SPI_USE_CUSTOM_DMA_MCODE_TX, (uint32_t) &user_spi_tx_mcode[0]);
    while(ret != ARM_DRIVER_OK);

    do {
        ret = ptrSPI->Transfer(&spi_tx_buff[0], &spi_rx_buff[0], SPI_TRANSFER_COUNT * SPI_TRANSFER_MULTIPLE);
    }
    while(ret == ARM_DRIVER_ERROR_BUSY);
    while(ret != ARM_DRIVER_OK);

    UTIMER_config(GetSystemAXIClock());
    LPTIMER_config();

    while(1) {
        while(spi_cb_status == 0) pm_core_enter_normal_sleep();
        spi_cb_status = 0;

        HW_REG32(GPIO5_BASE, 0x00) = 1U << 4;   // clock config start

        /* service call to turn on PLL */
        ret = SERVICES_pll_clkpll_start(se_services_s_handle, 1, INT32_MAX, &service_response);
        SERVICES_check_response;

        UTIMER_sync();                  // sync barrier: make sure there is no ongoing SPI xfer

        /* switch clocks to PLL */
#if defined (CORE_M55_HP)
        CGU->PLL_CLK_SEL |= (1U << 16);         // switch RTSS-HP from HFXO to PLL
        RTSS_HP_CLK=400000000;             // RTSS_HP_CLK is 400M
#endif
#if defined (CORE_M55_HE)
        CGU->PLL_CLK_SEL |= (1U << 20);         // switch RTSS-HE from HFXO to PLL
        RTSS_HE_CLK=120000000;             // RTSS_HE_CLK is 120M
#endif
        CGU->PLL_CLK_SEL |= (1U << 4);          // switch SYSPLL from HFXO to PLL
        AON->SYSTOP_CLK_DIV = 0x102;            // HCLK = ACLK/2 and PCLK = ACLK/4
        HW_REG32(CLKCTL_SYS_BASE, 0x820) = 2;   // ACLK source set to SYSPLL

        SystemAXIClock = 100000000;               // SYST_ACLK is SYSPLL div by 4
        SystemAHBClock = 50000000;               // SYST_HCLK is ACLK div by 2
        SystemAPBClock = 25000000;               // SYST_PCLK is ACLK div by 4

        UTIMER_config(GetSystemAXIClock());
        ptrSPI->Control(ARM_SPI_SET_BUS_SPEED, spi_freq);
        HW_REG32(GPIO5_BASE, 0x00) = 0;         // clock config end

        spi_rx_offset = buffer_select ? SPI_TRANSFER_COUNT * SPI_TRANSFER_MULTIPLE : 0;

        /* Application Code Starts Here */

        /* Application has ~9.3 milliseconds to use spi_rx_buff directly or
         * copy the data to a local buffer as shown, before it is overwritten by the DMA */
        memcpy(application_buffer, spi_rx_buff + spi_rx_offset, sizeof(application_buffer));

        /* Code to use application_buffer here */
        delay_us_refclk(1250);

        /* Application Code Ends Here */

        /* switch clocks to HFXO */
        HW_REG32(GPIO5_BASE,  0x00) = 1U << 4;  // clock config start
        /* set the clock variables */
        SystemAXIClock = (HFXO_CLK);          // SYST_ACLK is SYSPLL div by 4
        SystemAHBClock = (HFXO_CLK);          // SYST_HCLK is ACLK div by 1
        SystemAPBClock = (HFXO_CLK>>1);       // SYST_PCLK is ACLK div by 2
#if defined (CORE_M55_HP)
        RTSS_HP_CLK=HFXO_CLK;
        CGU->PLL_CLK_SEL &= ~(1U << 16);        // switch RTSS-HP from PLL to HFXO
#endif
#if defined (CORE_M55_HE)
        RTSS_HE_CLK=HFXO_CLK;
        CGU->PLL_CLK_SEL &= ~(1U << 20);        // switch RTSS-HE from PLL to HFXO
#endif

        UTIMER_sync();

        UTIMER_config(GetSystemAXIClock());
        ptrSPI->Control(ARM_SPI_SET_BUS_SPEED, spi_freq);

        HW_REG32(CLKCTL_SYS_BASE, 0x820) = 1;   // ACLK source set to REFCLK
        AON->SYSTOP_CLK_DIV = 0x001;            // HCLK = AXI and PCLK = AXI/2
        CGU->PLL_CLK_SEL &= ~(1U << 4);         // switch SYSPLL from PLL to HFXO

        /* service call to turn off PLL */
        ret = SERVICES_pll_clkpll_stop(se_services_s_handle, &service_response);
        SERVICES_check_response;

        HW_REG32(GPIO5_BASE, 0x00) = 0;         // clock config end
    }

    ret = ptrSPI->PowerControl(ARM_POWER_OFF);
    while(ret != ARM_DRIVER_OK);

    ret = ptrSPI->Uninitialize();
    while(ret != ARM_DRIVER_OK);
}
