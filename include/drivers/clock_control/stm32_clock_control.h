/*
 * Copyright (c) 2016 Open-RnD Sp. z o.o.
 * Copyright (c) 2016 BayLibre, SAS
 * Copyright (c) 2017 Linaro Limited.
 * Copyright (c) 2017 RnDity Sp. z o.o.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_STM32_CLOCK_CONTROL_H_
#define ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_STM32_CLOCK_CONTROL_H_

#include <drivers/clock_control.h>
#include <dt-bindings/clock/stm32_clock.h>

/* common clock control device node for all STM32 chips */
#define STM32_CLOCK_CONTROL_NODE DT_NODELABEL(rcc)

/*
 * Kconfig to device tree transition for clocks on STM32 targets:
 *
 * Following definitions are provided to allow a smooth transition
 * between Kconfig based to dts based clocks configuration.
 * These symbols allow to have both configuration schemes used simultaneoulsy
 * while giving precedence to dts based configuration once available on a
 * target.
 * Finally, once all in-tree users are converted to dts based configuration,
 * we'll be able to generate deprecation warnings for out of tree users of
 * Kconfig related symbols.
 */

#if DT_NODE_HAS_PROP(DT_INST(0, st_stm32_rcc), ahb_prescaler)
#define STM32_AHB_PRESCALER	DT_PROP(DT_NODELABEL(rcc), ahb_prescaler)
#else
#define STM32_AHB_PRESCALER	CONFIG_CLOCK_STM32_AHB_PRESCALER
#endif

#if DT_NODE_HAS_PROP(DT_INST(0, st_stm32_rcc), apb1_prescaler)
#define STM32_APB1_PRESCALER	DT_PROP(DT_NODELABEL(rcc), apb1_prescaler)
#else
#define STM32_APB1_PRESCALER	CONFIG_CLOCK_STM32_APB1_PRESCALER
#endif

#if DT_NODE_HAS_PROP(DT_INST(0, st_stm32_rcc), apb2_prescaler)
#define STM32_APB2_PRESCALER	DT_PROP(DT_NODELABEL(rcc), apb2_prescaler)
#else
#define STM32_APB2_PRESCALER	CONFIG_CLOCK_STM32_APB2_PRESCALER
#endif

#define STM32_AHB3_PRESCALER	CONFIG_CLOCK_STM32_AHB3_PRESCALER

#if DT_NODE_HAS_PROP(DT_INST(0, st_stm32wb_rcc), ahb4_prescaler)
#define STM32_AHB4_PRESCALER	DT_PROP(DT_NODELABEL(rcc), ahb4_prescaler)
#else
#define STM32_AHB4_PRESCALER	CONFIG_CLOCK_STM32_AHB4_PRESCALER
#endif

#if DT_NODE_HAS_PROP(DT_INST(0, st_stm32wb_rcc), cpu1_prescaler)
#define STM32_CPU1_PRESCALER	DT_PROP(DT_NODELABEL(rcc), cpu1_prescaler)
#else
#define STM32_CPU1_PRESCALER	CONFIG_CLOCK_STM32_CPU1_PRESCALER
#endif

#if DT_NODE_HAS_PROP(DT_INST(0, st_stm32wb_rcc), cpu2_prescaler)
#define STM32_CPU2_PRESCALER	DT_PROP(DT_NODELABEL(rcc), cpu2_prescaler)
#else
#define STM32_CPU2_PRESCALER	CONFIG_CLOCK_STM32_CPU2_PRESCALER
#endif

#if DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(pll), st_stm32l4_pll_clock, okay) || \
	DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(pll), st_stm32wb_pll_clock, okay) || \
	DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(pll), st_stm32f4_pll_clock, okay)
#define STM32_PLL_M_DIVISOR	DT_PROP(DT_NODELABEL(pll), div_m)
#define STM32_PLL_N_MULTIPLIER	DT_PROP(DT_NODELABEL(pll), mul_n)
#define STM32_PLL_P_DIVISOR	DT_PROP(DT_NODELABEL(pll), div_p)
#define STM32_PLL_Q_DIVISOR	DT_PROP(DT_NODELABEL(pll), div_q)
#define STM32_PLL_R_DIVISOR	DT_PROP(DT_NODELABEL(pll), div_r)
#else
#define STM32_PLL_M_DIVISOR	CONFIG_CLOCK_STM32_PLL_M_DIVISOR
#define STM32_PLL_N_MULTIPLIER	CONFIG_CLOCK_STM32_PLL_N_MULTIPLIER
#define STM32_PLL_P_DIVISOR	CONFIG_CLOCK_STM32_PLL_P_DIVISOR
#define STM32_PLL_Q_DIVISOR	CONFIG_CLOCK_STM32_PLL_Q_DIVISOR
#define STM32_PLL_R_DIVISOR	CONFIG_CLOCK_STM32_PLL_R_DIVISOR
#endif

#define DT_RCC_CLOCKS_CTRL	DT_CLOCKS_CTLR(DT_NODELABEL(rcc))

#if DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(rcc), st_stm32_rcc, okay)
#define STM32_SYSCLK_SRC_PLL	DT_NODE_HAS_PROP(DT_NODELABEL(rcc), clocks)  && \
				DT_SAME_NODE(DT_RCC_CLOCKS_CTRL, DT_NODELABEL(pll))
#define STM32_SYSCLK_SRC_HSI	DT_NODE_HAS_PROP(DT_NODELABEL(rcc), clocks)  && \
				DT_SAME_NODE(DT_RCC_CLOCKS_CTRL, DT_NODELABEL(clk_hsi))
#define STM32_SYSCLK_SRC_HSE	DT_NODE_HAS_PROP(DT_NODELABEL(rcc), clocks)  && \
				DT_SAME_NODE(DT_RCC_CLOCKS_CTRL, DT_NODELABEL(clk_hse))
#define STM32_SYSCLK_SRC_MSI	DT_NODE_HAS_PROP(DT_NODELABEL(rcc), clocks)  && \
				DT_SAME_NODE(DT_RCC_CLOCKS_CTRL, DT_NODELABEL(clk_msi))
#else
#define STM32_SYSCLK_SRC_PLL	CONFIG_CLOCK_STM32_SYSCLK_SRC_PLL
#define STM32_SYSCLK_SRC_HSI	CONFIG_CLOCK_STM32_SYSCLK_SRC_HSI
#define STM32_SYSCLK_SRC_HSE	CONFIG_CLOCK_STM32_SYSCLK_SRC_HSE
#define STM32_SYSCLK_SRC_MSI	CONFIG_CLOCK_STM32_SYSCLK_SRC_MSI
#endif

#define DT_PLL_CLOCKS_CTRL	DT_CLOCKS_CTLR(DT_NODELABEL(pll))

#if DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(pll), st_stm32l4_pll_clock, okay) || \
	DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(pll), st_stm32wb_pll_clock, okay) || \
	DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(pll), st_stm32f4_pll_clock, okay)
#define STM32_PLL_SRC_MSI	DT_SAME_NODE(DT_PLL_CLOCKS_CTRL, DT_NODELABEL(clk_msi))
#define STM32_PLL_SRC_HSI	DT_SAME_NODE(DT_PLL_CLOCKS_CTRL, DT_NODELABEL(clk_hsi))
#define STM32_PLL_SRC_HSE	DT_SAME_NODE(DT_PLL_CLOCKS_CTRL, DT_NODELABEL(clk_hse))
#else
#define STM32_PLL_SRC_MSI	CONFIG_CLOCK_STM32_PLL_SRC_MSI
#define STM32_PLL_SRC_HSI	CONFIG_CLOCK_STM32_PLL_SRC_HSI
#define STM32_PLL_SRC_HSE	CONFIG_CLOCK_STM32_PLL_SRC_HSE
#endif

#if DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(pll), st_stm32l4_pll_clock, okay)
#define STM32_PLL_SRC_PLL2	DT_SAME_NODE(DT_CLOCKS_CTLR(DT_NODELABEL(pll)), DT_NODELABEL(pll2))
#else
#define STM32_PLL_SRC_PLL2	CONFIG_CLOCK_STM32_PLL_SRC_PLL2
#endif

#if DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(clk_lse), fixed_clock, okay)
#define STM32_LSE_CLOCK		DT_PROP(DT_NODELABEL(clk_hse), clock_frequency)
#else
#define STM32_LSE_CLOCK		CONFIG_CLOCK_STM32_LSE
#endif

#if DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(clk_msi), st_stm32_msi_clock, okay)
#define STM32_MSI_RANGE		DT_PROP(DT_NODELABEL(clk_msi), msi_range)
#else
#define STM32_MSI_RANGE		CONFIG_CLOCK_STM32_MSI_RANGE
#endif

#if DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(clk_msi), st_stm32_msi_clock, okay)
#define STM32_MSI_PLL_MODE	DT_PROP(DT_NODELABEL(clk_msi), msi_pll_mode)
#else
#define STM32_MSI_PLL_MODE	CONFIG_CLOCK_STM32_MSI_PLL_MODE
#endif

#if DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(clk_hse), st_stm32_hse_clock, okay)
#define STM32_HSE_BYPASS	DT_PROP(DT_NODELABEL(clk_hse), hse_bypass)
#else
#define STM32_HSE_BYPASS	CONFIG_CLOCK_STM32_HSE_BYPASS
#endif

struct stm32_pclken {
	uint32_t bus;
	uint32_t enr;
};

#endif /* ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_STM32_CLOCK_CONTROL_H_ */
