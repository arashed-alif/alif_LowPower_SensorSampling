#include <stdint.h>
#include <stdlib.h>

#include <sys_clocks.h>
#include <sys_utils.h>
#include <soc.h>
#include <soc_dma_map.h>
#include <pinconf.h>
#include <utimer.h>

#ifndef HW_REG32
#define HW_REG32(u,v) (*((volatile uint32_t *)(u + v)))
#endif

#define UT_USE_LPTIMER  1

#define UT_CHANNEL_MASK 3U
#define UT_DRIVER_MASK  0xFFFFF8UL
#define UT_COMPARE_CFG0 0x919   /* 1 at start of cycle, 0 on compare match */
#define UT_COMPARE_CFG1 0x906   /* 0 at start of cycle, 1 on compare match */
#define UT_FALLING_EDGE 2U
#define UT_RISING_EDGE  1U
#define UT_TRIGGER_SRC(u) (u * 2)

#define UTIMER0_PERIOD  20
#define UTIMER0_PWIDTH  2
#define UTIMER1_PERIOD  5000
#define UTIMER1_PWIDTH  159

static void configure_utimer_pinmux()
{
    uint32_t padconf = PADCTRL_READ_ENABLE |
            PADCTRL_OUTPUT_DRIVE_STRENGTH_4MA;

    /* Port 0
     * Note: these 3 pins do not need to be pinmuxed for the demo to work,
     * they are only made available for observation and debug
     */
    HW_REG32(PINMUX_BASE, 0x00) = 4 | (padconf << 16);  /* P0_0 is UT0_T0 */
    HW_REG32(PINMUX_BASE, 0x04) = 4 | (padconf << 16);  /* P0_1 is UT0_T1 */
    HW_REG32(PINMUX_BASE, 0x08) = 4 | (padconf << 16);  /* P0_2 is UT1_T0 */

    padconf |= PADCTRL_SCHMITT_TRIGGER_ENABLE |
            PADCTRL_DRIVER_DISABLED_PULL_DOWN;
    /* Port 4
     * Note: this pin must be pinmuxed and connected to the LPTIMER on P15_0
     */
    HW_REG32(PINMUX_BASE, 0x80) = 0 | (padconf << 16);  /* P4_0 is GPIO4_0 */
}

static void configure_utimer_in_event_router()
{
#if defined(ENSEMBLE_SOC_E1C)
    /* enable clock to event router */
    HW_REG32(M55HE_CFG_BASE, 0xC) |= (1U << 4);

    HW_REG32(EVTRTR2_BASE, 0x08) = 0x12;    /* EVTRTR2_IN[2] is UT1_T0 */
    HW_REG32(EVTRTR2_BASE, 0x20) = 0x13;    /* EVTRTR2_IN[8] is GPIO4_0 */
#else
    /* enable clock to event router */
    HW_REG32(CLKCTL_PER_MST_BASE, 0xC) |= (1U << 4);

    HW_REG32(EVTRTR0_BASE, 0x08) = 0x12;    /* EVTRTR0_IN[2] is UT1_T0 */
    HW_REG32(EVTRTR0_BASE, 0x20) = 0x13;    /* EVTRTR0_IN[8] is GPIO4_0 */
#endif
}

void UTIMER_config(uint32_t UTIMER_CLK)
{
    static uint32_t initialized = 0;
    UTIMER_Type *utimer = (UTIMER_Type *) UTIMER_BASE;

    if (initialized == 0) {
        /* UTIMER clock enable for UT[0] and UT[1] */
        /* UTIMER driver enable for UT0_T0, UT0_T1, and UT1_T0 */
        utimer->UTIMER_GLB_CLOCK_ENABLE = UT_CHANNEL_MASK;
        utimer->UTIMER_GLB_DRIVER_OEN = UT_DRIVER_MASK;

        configure_utimer_pinmux();
        configure_utimer_in_event_router();
    }

    /* UTIMER0 */
    UTIMER_UTIMER_CHANNEL_CFG_Type *ut0 = (UTIMER_UTIMER_CHANNEL_CFG_Type *) &utimer->UTIMER_CHANNEL_CFG[0];
    ut0->UTIMER_CNTR_CTRL = 0;  /* disable timer channel */
    ut0->UTIMER_CNTR      = 0;  /* clear timer counter */
    if (initialized == 0) {
        ut0->UTIMER_START_0_SRC = UT_RISING_EDGE << UT_TRIGGER_SRC(8);  /* rising edge of trigger 8 causes counter to start */
        ut0->UTIMER_STOP_0_SRC  = UT_FALLING_EDGE << UT_TRIGGER_SRC(2); /* falling edge of trigger 2 causes counter to stop */
        ut0->UTIMER_CLEAR_0_SRC = UT_FALLING_EDGE << UT_TRIGGER_SRC(2); /* falling edge of trigger 2 causes counter to clear */
        ut0->UTIMER_COMPARE_CTRL_A = UT_COMPARE_CFG0;
        ut0->UTIMER_COMPARE_CTRL_B = UT_COMPARE_CFG1;
        ut0->UTIMER_BUF_OP_CTRL = 0;
    }
    ut0->UTIMER_CNTR_PTR  = (UTIMER_CLK * ((float)UTIMER0_PERIOD / 1000000)) - 1;
    ut0->UTIMER_COMPARE_A = (UTIMER_CLK * ((float)UTIMER0_PWIDTH / 1000000)) - 1;
    ut0->UTIMER_COMPARE_B = (ut0->UTIMER_CNTR_PTR) - (UTIMER_CLK * ((float)UTIMER0_PWIDTH / 1000000)) - 1;
    ut0->UTIMER_CNTR_CTRL = 1;  /* enable timer channel */

    /* UTIMER1 */
    UTIMER_UTIMER_CHANNEL_CFG_Type *ut1 = (UTIMER_UTIMER_CHANNEL_CFG_Type *) &utimer->UTIMER_CHANNEL_CFG[1];
    ut1->UTIMER_CNTR_CTRL = 0;  /* disable timer channel */
    ut1->UTIMER_CNTR      = 0;  /* clear timer counter */
    if (initialized == 0) {
        ut1->UTIMER_START_0_SRC = UT_RISING_EDGE << UT_TRIGGER_SRC(8);  /* rising edge of trigger 8 causes counter to start */
        ut1->UTIMER_CLEAR_0_SRC = UT_RISING_EDGE << UT_TRIGGER_SRC(8);  /* rising edge of trigger 8 causes counter to clear */
        ut1->UTIMER_COMPARE_CTRL_A = UT_COMPARE_CFG0;
        ut1->UTIMER_BUF_OP_CTRL = 0;
        initialized = 1;
    }
    ut1->UTIMER_CNTR_PTR  = (UTIMER_CLK * ((float)UTIMER1_PERIOD / 1000000)) - 1;
    ut1->UTIMER_COMPARE_A = (UTIMER_CLK * ((float)UTIMER1_PWIDTH / 1000000)) - 1;
    ut1->UTIMER_CNTR_CTRL = 1;  /* enable timer channel */

    utimer->UTIMER_GLB_CNTR_START = 1;
}

void UTIMER_sync() {
    HW_REG32(GPIO5_BASE, 0x00) = 5U << 4;

    UTIMER_Type *utimer = (UTIMER_Type *) UTIMER_BASE;
    UTIMER_UTIMER_CHANNEL_CFG_Type *ut1 = (UTIMER_UTIMER_CHANNEL_CFG_Type *) &utimer->UTIMER_CHANNEL_CFG[1];

    /* This sync barrier will hold the CPU until it is safe to change bus clocks.
     *
     * compare_val_lo = 200us (160 us for SPI xfers + 40 us extra)
     * compare_val_hi = 840us (136 us until next SPI xfer)
     */
    uint32_t compare_val = ut1->UTIMER_COMPARE_A;
    uint32_t compare_val_lo = (compare_val >> 2) + compare_val;     // = compare_val * 1.25
    uint32_t compare_val_hi = (compare_val << 2) + compare_val_lo;  // = compare_val * 5.25

    /* hold the CPU while outside of the 320 us window to change bus clocks */
    while(1) {
        uint32_t counter_val = ut1->UTIMER_CNTR;
        if ((counter_val > compare_val_lo) &&
                (counter_val < compare_val_hi))
            break;
    }

    HW_REG32(GPIO5_BASE, 0x00) = 1U << 4;
}
