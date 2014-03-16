/* Board support file for Google Steelhead Board.
 *
 * Copyright (C) 2011 Google, Inc.
 * Copyright (C) 2010 Texas Instruments
 *
 * Based on mach-omap2/board-tuna.c
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/ion.h>
#include <linux/leds.h>
#include <linux/gpio.h>
#include <linux/memblock.h>
#include <linux/moduleparam.h>
#include <linux/omap_ion.h>
#include <linux/usb/otg.h>
#include <linux/i2c/twl.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/reboot.h>

#include <mach/hardware.h>
#include <mach/omap4-common.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <video/omapdss.h>
#include <linux/omapfb.h>
#include <video/omap-panel-generic-dpi.h>
#include <plat/vram.h>

#include <plat/board.h>
#include <plat/common.h>
#include <plat/dma.h>
#include <plat/usb.h>
#include <plat/mmc.h>
#include <plat/remoteproc.h>
#include <mach/omap_fiq_debugger.h>
#include <mach/id.h>
#include <mach/dmm.h>
#include "timer-gp.h"

#include "hsmmc.h"
#include "control.h"
#include "mux.h"
#include "board-steelhead.h"
#include "resetreason.h"
#include "common-board-devices.h"
#include "pm.h"

#include <linux/i2c.h>
#include <plat/i2c.h>
#include <plat/mcasp.h>
#include <plat/omap-pm.h>
#include <linux/platform_data/ram_console.h>
#include <linux/tas5713.h>
#include <linux/steelhead_avr.h>
#include <linux/aah_localtime.h>
#include <linux/pn544.h>
#include <sound/pcm.h>

#include <video/omap-panel-generic-dpi.h>


#define TPS62361_GPIO		7

#define STEELHEAD_RAMCONSOLE_START	(PLAT_PHYS_OFFSET + SZ_512M)
#define STEELHEAD_RAMCONSOLE_SIZE	SZ_2M

#define STEELHEAD_ATTR_RO(_type, _name, _show) \
	struct kobj_attribute steelhead_##_type##_prop_attr_##_name = \
		__ATTR(_name, S_IRUGO, _show, NULL)

static struct resource ramconsole_resources[] = {
	{
		.flags  = IORESOURCE_MEM,
		.start	= STEELHEAD_RAMCONSOLE_START,
		.end	= (STEELHEAD_RAMCONSOLE_START +
			   STEELHEAD_RAMCONSOLE_SIZE - 1),
	},
};

static struct ram_console_platform_data ramconsole_pdata;

static struct platform_device ramconsole_device = {
	.name           = "ram_console",
	.id             = -1,
	.num_resources  = ARRAY_SIZE(ramconsole_resources),
	.resource       = ramconsole_resources,
	.dev            = {
		.platform_data = &ramconsole_pdata,
	},

};

int steelhead_hw_rev;
module_param(steelhead_hw_rev, int, S_IRUGO);
MODULE_PARM_DESC(steelhead_hw_rev, "hardware revision");

static bool enable_sr = true;
module_param(enable_sr, bool, S_IRUSR | S_IRGRP | S_IROTH);

static const char const *omap4_steelhead_hw_name[] = {
	[STEELHEAD_REV_ALPHA]  = "Steelhead ALPHA",
	[STEELHEAD_REV_EVT]    = "Steelhead EVT",
	[STEELHEAD_REV_EVT2]   = "Steelhead EVT2",
	[STEELHEAD_REV_DVT]    = "Steelhead DVT",
	[STEELHEAD_REV_DVT1_5] = "Steelhead DVT1.5",
	[STEELHEAD_REV_DVT2]   = "Steelhead DVT2",
	[STEELHEAD_REV_DVT3]   = "Steelhead DVT3",
	[STEELHEAD_REV_DVT4]   = "Steelhead DVT4",
	[STEELHEAD_REV_DVT5]   = "Steelhead DVT5",
	[STEELHEAD_REV_DVT6]   = "Steelhead DVT6",
	[STEELHEAD_REV_PROD]   = "Steelhead PROD",
	[STEELHEAD_REV_PROD1]  = "Steelhead PROD1",
	[STEELHEAD_REV_PROD2]  = "Steelhead PROD2",
	[STEELHEAD_REV_PROD3]  = "Steelhead PROD3",
};

static const char *omap4_steelhead_hw_rev_name(void)
{
	int num = ARRAY_SIZE(omap4_steelhead_hw_name);

	if (steelhead_hw_rev >= num ||
	    !omap4_steelhead_hw_name[steelhead_hw_rev])
		return "Steelhead unknown version";

	return omap4_steelhead_hw_name[steelhead_hw_rev];
}

static void __init omap4_steelhead_init_hw_rev(void)
{
	pr_info("Steelhead HW revision: %02x (%s), cpu %s\n", steelhead_hw_rev,
		omap4_steelhead_hw_rev_name(),
		cpu_is_omap443x() ? "OMAP4430" : "OMAP4460");
}

#define PHYS_ADDR_SMC_SIZE				(SZ_1M * 3)
#define PHYS_ADDR_DUCATI_SIZE				(SZ_1M * 105)
#define OMAP_STEELHEAD_ION_HEAP_SECURE_INPUT_SIZE	(SZ_1M * 90)
#define OMAP_STEELHEAD_ION_HEAP_TILER_SIZE		(SZ_1M * 81)
#define OMAP_STEELHEAD_ION_HEAP_NONSECURE_TILER_SIZE	(SZ_1M * 32)

#define PHYS_ADDR_SMC_MEM	(0x80000000 + SZ_1G - PHYS_ADDR_SMC_SIZE)
#define PHYS_ADDR_DUCATI_MEM	(PHYS_ADDR_SMC_MEM - \
				 PHYS_ADDR_DUCATI_SIZE - \
				 OMAP_STEELHEAD_ION_HEAP_SECURE_INPUT_SIZE)

static struct ion_platform_data sh_ion_data = {
	.nr = 3,
	.heaps = {
		{
			.type = ION_HEAP_TYPE_CARVEOUT,
			.id = OMAP_ION_HEAP_SECURE_INPUT,
			.name = "secure_input",
			.base = PHYS_ADDR_SMC_MEM -
				OMAP_STEELHEAD_ION_HEAP_SECURE_INPUT_SIZE,
			.size = OMAP_STEELHEAD_ION_HEAP_SECURE_INPUT_SIZE,
		},
		{	.type = OMAP_ION_HEAP_TYPE_TILER,
			.id = OMAP_ION_HEAP_TILER,
			.name = "tiler",
			.base = PHYS_ADDR_DUCATI_MEM -
					OMAP_STEELHEAD_ION_HEAP_TILER_SIZE,
			.size = OMAP_STEELHEAD_ION_HEAP_TILER_SIZE,
		},
		{	.type = OMAP_ION_HEAP_TYPE_TILER,
			.id = OMAP_ION_HEAP_NONSECURE_TILER,
			.name = "nonsecure_tiler",
			.base = PHYS_ADDR_DUCATI_MEM -
				OMAP_STEELHEAD_ION_HEAP_TILER_SIZE -
				OMAP_STEELHEAD_ION_HEAP_NONSECURE_TILER_SIZE,
			.size = OMAP_STEELHEAD_ION_HEAP_NONSECURE_TILER_SIZE,
		},
	},
};

static struct platform_device steelhead_ion_device = {
	.name = "ion-omap4",
	.id = -1,
	.dev = {
		.platform_data = &sh_ion_data,
	},
};

static struct platform_device *steelhead_devices[] __initdata = {
	&ramconsole_device,
	&steelhead_ion_device,
};

static void __init steelhead_init_early(void)
{
	omap2_init_common_infrastructure();
	omap2_init_common_devices(NULL, NULL);
}

static struct omap_musb_board_data musb_board_data = {
	.interface_type		= MUSB_INTERFACE_UTMI,
#if defined(CONFIG_USB_MUSB_PERIPHERAL)
	.mode			= MUSB_PERIPHERAL,
#elif defined(CONFIG_USB_MUSB_OTG)
	.mode			= MUSB_OTG,
#elif defined(CONFIG_USB_MUSB_HOST)
	.mode			= MUSB_HOST,
#endif
	.power			= 500,
};

static struct twl4030_usb_data omap4_usbphy_data = {
	.phy_init	= omap4430_phy_init,
	.phy_exit	= omap4430_phy_exit,
	.phy_power	= omap4430_phy_power,
	.phy_set_clock	= omap4430_phy_set_clk,
	.phy_suspend	= omap4430_phy_suspend,
};

static struct omap2_hsmmc_info mmc[] = {
	{
		.mmc		= 1,
		.nonremovable	= true,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA,
		.ocr_mask	= MMC_VDD_29_30	| MMC_VDD_30_31,
		.gpio_wp	= -EINVAL,
		.gpio_cd	= -EINVAL,
	},
	{
		.name		= "omap_wlan",
		.mmc		= 5,
		.caps		= MMC_CAP_4_BIT_DATA,
		.gpio_wp	= -EINVAL,
		.gpio_cd	= -EINVAL,
		.ocr_mask	= MMC_VDD_27_28 |
				  MMC_VDD_28_29 |
				  MMC_VDD_29_30 |
				  MMC_VDD_30_31 |
				  MMC_VDD_31_32 |
				  MMC_VDD_32_33 |
				  MMC_VDD_33_34 |
				  MMC_VDD_34_35 |
				  MMC_VDD_35_36,
		.nonremovable	= false,
		.mmc_data	= &steelhead_wifi_data,
	},
	{}	/* Terminator */
};

static struct regulator_consumer_supply steelhead_vmmc_supply[] = {
	{
		.supply = "vmmc",
		.dev_name = "omap_hsmmc.0",
	},
};

static int __init omap4_twl6030_hsmmc_init(struct omap2_hsmmc_info *controllers)
{
	omap2_hsmmc_init(controllers);
	return 0;
}

/* In DVT2, this provides power to eMMC core.  In earlier devices,
 * this is really a NOP because the supply line to VAUX1 is not
 * connected so the PMIC won't generate anything even though we
 * ask it to, so we don't bother doing any runtime config.
 */
static struct regulator_init_data steelhead_vaux1 = {
	.constraints = {
		.min_uV			= 3000000,
		.max_uV			= 3000000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.always_on	= true,
	},
};

static struct regulator_init_data steelhead_vaux2 = {
	.constraints = {
		.min_uV			= 1200000,
		.max_uV			= 2800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled	= true,
		},
		.initial_state		= PM_SUSPEND_MEM,
	},
};

static struct regulator_init_data steelhead_vaux3 = {
	.constraints = {
		.min_uV			= 1000000,
		.max_uV			= 3000000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
};

/* VMMC1 for MMC1 card */
static struct regulator_init_data steelhead_vmmc = {
	.constraints = {
		.min_uV			= 3000000,
		.max_uV			= 3000000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = steelhead_vmmc_supply,
};

static struct regulator_init_data steelhead_vpp = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 2500000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled	= true,
		},
		.initial_state		= PM_SUSPEND_MEM,
	},
};

static struct regulator_init_data steelhead_vusim = {
	.constraints = {
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,
	},
};

static struct regulator_init_data steelhead_vana = {
	.constraints = {
		.min_uV			= 2100000,
		.max_uV			= 2100000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.always_on	= true,
	},
};

static struct regulator_consumer_supply steelhead_vcxio_supply[] = {
	REGULATOR_SUPPLY("vdds_dsi", "omapdss_dss"),
	REGULATOR_SUPPLY("vdds_dsi", "omapdss_dsi1"),
};

/* always on for Tungsten because it provides power to a number of CPU
 * inputs in VDDA_DPLL_CORE_AUDIO, VDDA_DPLL_MPU, VDDA_DPLL_IVA_PER,
 * VDDS_DV_BANK7, VDDA_USBA0OTG_1P8V as well as VDDA_DSI1, VDDA_DSI2,
 * VDDA_CSI21, VDDA_CSI22.
 */
static struct regulator_init_data steelhead_vcxio = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.always_on		= true,
	},
	.num_consumer_supplies	= ARRAY_SIZE(steelhead_vcxio_supply),
	.consumer_supplies	= steelhead_vcxio_supply,
};

static struct regulator_consumer_supply steelhead_vdac_supply[] = {
	{
		.supply = "hdmi_vref",
	},
};

static struct regulator_init_data steelhead_vdac = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= ARRAY_SIZE(steelhead_vdac_supply),
	.consumer_supplies	= steelhead_vdac_supply,
};

static struct regulator_init_data steelhead_vusb = {
	.constraints = {
		.min_uV			= 3300000,
		.max_uV			= 3300000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 =	REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled	= true,
		},
		.initial_state		= PM_SUSPEND_MEM,
	},
};

/* clk32kg is a twl6030 32khz clock modeled as a regulator, used by wifi */
static struct regulator_consumer_supply steelhead_clk32kg_supply[] = {
	{
		/* we use the same name as tuna though
		 * we're not really using the 6030's clk32kaudio
		 * output.  that way we can share
		 * board-tuna-bluetooth.c
		 */
		.supply = "clk32kaudio",
	},
};

static struct regulator_init_data steelhead_clk32kg = {
	.constraints = {
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
		.boot_on		= true,
	},
	.num_consumer_supplies  = ARRAY_SIZE(steelhead_clk32kg_supply),
	.consumer_supplies      = steelhead_clk32kg_supply,
};

/* define a regulator so on 4460 this can be turned off because
 * it's not needed.
 */
static struct regulator_init_data steelhead_vdd3 = {
	.constraints = {
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled	= true,
		},
		.initial_state		= PM_SUSPEND_MEM,
	},
};

/* define a regulator so on 4460 this can be turned off because
 * it's not needed.
 */
static struct regulator_init_data steelhead_vmem = {
	.constraints = {
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled	= true,
		},
		.initial_state		= PM_SUSPEND_MEM,
	},
};

static struct regulator_init_data steelhead_v2v1 = {
	.constraints = {
		.min_uV			= 2100000,
		.max_uV			= 2100000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.always_on		= true,
	},
};


static struct twl4030_platform_data steelhead_twldata = {
	.irq_base	= TWL6030_IRQ_BASE,
	.irq_end	= TWL6030_IRQ_END,

	/* Regulators */
	.vmmc		= &steelhead_vmmc,
	.vpp		= &steelhead_vpp,
	.vusim		= &steelhead_vusim,
	.vana		= &steelhead_vana,
	.vcxio		= &steelhead_vcxio,
	.vdac		= &steelhead_vdac,
	.vusb		= &steelhead_vusb,
	.vaux1		= &steelhead_vaux1,
	.vaux2		= &steelhead_vaux2,
	.vaux3		= &steelhead_vaux3,
	.clk32kg	= &steelhead_clk32kg,
	.usb		= &omap4_usbphy_data,

	/* SMPS */
	.vdd3		= &steelhead_vdd3,
	.vmem		= &steelhead_vmem,
	.v2v1		= &steelhead_v2v1,
};

static struct omap_device_pad serial3_pads[] __initdata = {
	{
		.name   = "uart3_tx_irtx.uart3_tx_irtx",
		.enable = OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
	},
	{
		.name   = "uart3_rx_irrx.uart3_rx_irrx",
		.flags  = OMAP_DEVICE_PAD_REMUX | OMAP_DEVICE_PAD_WAKEUP,
		.enable = OMAP_PIN_INPUT | OMAP_MUX_MODE0,
		.idle   = OMAP_PIN_INPUT | OMAP_MUX_MODE0,
	},
};

static struct omap_device_pad serial4_pads[] __initdata = {
	{
		.name   = "uart4_rx.uart4_rx",
		.flags  = OMAP_DEVICE_PAD_REMUX | OMAP_DEVICE_PAD_WAKEUP,
		.enable = OMAP_PIN_INPUT | OMAP_MUX_MODE0,
		.idle   = OMAP_PIN_INPUT | OMAP_MUX_MODE0,
	},
	{
		.name   = "uart4_tx.uart4_tx",
		.enable = OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
	},
};

static inline void __init board_serial_init(void)
{
	/* uart1 is unused */
	omap_serial_init_port_pads(0, NULL, 0, NULL);
	/* uart2 for bluetooth is done in board-steelhead-bluetooth.c */

	/* uart3 is for FIQ debugger and is initialized in
	   board_serial_debug_init() */

	/* uart4 is used for AVR programming */
	omap_serial_init_port_pads(3, serial4_pads, ARRAY_SIZE(serial4_pads),
				   NULL);
}

/* fiq_debugger initializes really early but OMAP resource mgmt
 * is not yet ready @ arch_init, so init the serial debugger later */
static int __init board_serial_debug_init(void)
{
	return omap_serial_debug_init(2, false, true, serial3_pads,
				      ARRAY_SIZE(serial3_pads));
}
device_initcall(board_serial_debug_init);


int __init steelhead_reserve_gpios(struct steelhead_gpio_reservation *data,
				   int count,
				   const char *log_tag,
				   int do_gpio_request)
{
	int i;
	for (i = 0; i < count; ++i) {
		int status;
		struct steelhead_gpio_reservation *g = (data + i);

		status = omap_mux_init_signal(g->mux_name, g->pin_mode);
		if (status) {
			pr_err("%s: failed to set gpio pin mux for gpio \"%s\""
					" to mode %d.  (status %d)\n",
					log_tag, g->mux_name,
					g->pin_mode, status);
			return status;
		}

		if (!do_gpio_request)
			continue;

		status = gpio_request_one(
				g->gpio_id, g->init_state, g->gpio_name);

		if (status) {
			pr_err("%s: failed to reserve gpio \"%s\""
				       "(status %d)\n",
					log_tag, g->gpio_name, status);
			return status;
		}
	}

	return 0;
}

/******************************************************************************
 *                                                                            *
 *         Underflow counters for health monitoring of audio outputs          *
 *                                                                            *
 ******************************************************************************/
static u32 steelhead_mcasp_dai_underflows;
static u32 steelhead_mcbsp_dai_underflows;
static ssize_t steelhead_audio_underflows_show(struct kobject *kobj,
				    struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf,
			"spdif_dai_underflows=%u,"
			"tas5713_dai_underflows=%u\n",
			steelhead_mcasp_dai_underflows,
			steelhead_mcbsp_dai_underflows);
}

void steelhead_log_mcasp_underflow(int dev_id) {
	steelhead_mcasp_dai_underflows++;
}

void steelhead_log_mcbsp_underflow(int dev_id) {
	/* At 1.0 ship, steelhead used only one of the 4 McBSPs (#2) in order to
	 * drive the TAS5713 amp.  McBSP #1, however, is tied to the BT chip for
	 * BT audio scenarios.  If we ever turn this on in SW, we will want to
	 * revisit this and add another counter for BT audio I2S underflows
	 * coming from a McBSP FIFO underflow.  For now, it should be safe to
	 * assert that this underflow is coming from McBSP #2
	 */
	BUG_ON(dev_id != 2);
	steelhead_mcbsp_dai_underflows++;
}

static STEELHEAD_ATTR_RO(audio, underflows, steelhead_audio_underflows_show);

static struct attribute *steelhead_audio_prop_attrs[] = {
	&steelhead_audio_prop_attr_underflows.attr,
	NULL,
};

static struct attribute_group steelhead_audio_prop_attr_group = {
	.attrs = steelhead_audio_prop_attrs,
};


/******************************************************************************
 *                                                                            *
 *              TAS5713 Class-D Audio Amplifier Initialization                *
 *                                                                            *
 ******************************************************************************/

#define TAS5713_INTERFACE_EN_GPIO_ID 40
#define TAS5713_RESET_GPIO_ID 42
#define TAS5713_PDN_GPIO_ID 44

static void steelhead_tas5713_before_power_up(
		struct tas5713_platform_data* pdata)
{
	if (NULL != pdata) {
		int gpio_num = (int)(pdata->board_data);
		gpio_set_value(gpio_num, 1);
	}
}

static void steelhead_tas5713_after_power_down(
		struct tas5713_platform_data* pdata)
{
	if (NULL != pdata) {
		int gpio_num = (int)(pdata->board_data);
		gpio_set_value(gpio_num, 1);
	}
}

static struct tas5713_platform_data tas5713_pdata = {
	/* Reset and Power Down GPIO configuration */
	.reset_gpio = TAS5713_RESET_GPIO_ID,
	.pdn_gpio = TAS5713_PDN_GPIO_ID,

	/* MCLK */
	.mclk_out = NULL,

	/* Callbacks */
	.before_power_up = steelhead_tas5713_before_power_up,
	.after_power_down = steelhead_tas5713_after_power_down,
	.board_data = (void*)TAS5713_INTERFACE_EN_GPIO_ID,
};
static const unsigned long tas5713_mclk_rate = 12288000;

static void steelhead_platform_init_tas5713_audio(void)
{
	struct clk *m3x2_clk = NULL;
	struct clk *mclk_out = NULL;
	struct clk *mclk_src = NULL;
	struct clk *mcbsp_internal_clk =  NULL;
	struct clk *abe_24m_clk = NULL;
	int res;
	unsigned long tgt_rate = (tas5713_mclk_rate * 5);

	static struct steelhead_gpio_reservation tas5713_iface_en_gpio[] = {
		{
			.gpio_id = TAS5713_INTERFACE_EN_GPIO_ID,
			.gpio_name = "tas5713_interface_en",
			.mux_name = "gpmc_a16.gpio_40",
			.pin_mode = OMAP_PIN_OUTPUT,
			.init_state = GPIOF_OUT_INIT_LOW,
		},
	};

	omap_mux_init_gpio(TAS5713_RESET_GPIO_ID, OMAP_PIN_OUTPUT);
	omap_mux_init_gpio(TAS5713_PDN_GPIO_ID, OMAP_PIN_OUTPUT);

	/* Reserve the level translator enable GPIO and set it up to disable the
	 * LT.  The TAS5713 driver will enable and disable the LT as needed
	 * using the callbacks provided in the pdata.
	 */
	omap_mux_init_gpio(TAS5713_INTERFACE_EN_GPIO_ID, OMAP_PIN_OUTPUT);
	if (steelhead_reserve_gpios(tas5713_iface_en_gpio,
				ARRAY_SIZE(tas5713_iface_en_gpio),
				"tas5713", true))
		return;

	/* Turn on the pins for McBSP2.  We will be using it to deliver I2S to
	 * the TAS5713
	 */
	omap_mux_init_signal("abe_mcbsp2_clkx.abe_mcbsp2_clkx",
			OMAP_PIN_OUTPUT);
	omap_mux_init_signal("abe_mcbsp2_dx.abe_mcbsp2_dx", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("abe_mcbsp2_fsx.abe_mcbsp2_fsx", OMAP_PIN_OUTPUT);

	/* Make sure that McBSP2's internal clock selection is set to the output
	 * of the ABE DPLL and not the PER DPLL.
	 */
	mcbsp_internal_clk = clk_get(NULL, "mcbsp2_sync_mux_ck");
	if (IS_ERR_OR_NULL(mcbsp_internal_clk)) {
		pr_err("tas5713: failed to fetch mcbsp2_sync_mux_ck\n");
		goto err;
	}

	abe_24m_clk = clk_get(NULL, "abe_24m_fclk");
	if (IS_ERR_OR_NULL(abe_24m_clk)) {
		pr_err("tas5713: failed to fetch abe_24m_clk\n");
		goto err;
	}

	res = clk_set_parent(mcbsp_internal_clk, abe_24m_clk);
	if (res < 0) {
		pr_err("tas5713: failed to set reference clock"
				" for McBSP2 internal clk (res = %d)  "
				"driver will not load.\n", res);
		goto err;
	}

	/* We use fref_clk1_out to drive MCLK on steelhead.  It should be
	 * configured to run from the M3 X2 output of DPLL_PER.  With a system
	 * clock of 38.4MHz, and default multiplier of 40 as the reference for
	 * the M3X2 divider, we should end up setting the M3 divider to 25 and
	 * the fref_clk1_out divider to 5 to generate a 12.288MHz MCLK which
	 * should be 256 * 48kHz.
	 */
	m3x2_clk = clk_get(NULL, "dpll_per_m3x2_ck");
	if (IS_ERR_OR_NULL(m3x2_clk)) {
		pr_err("tas5713: failed to dpll_per_m3x2_ck\n");
		goto err;
	}

	mclk_out = clk_get(NULL, "auxclk1_ck");
	if (IS_ERR_OR_NULL(mclk_out)) {
		pr_err("tas5713: failed to auxclk1_ck\n");
		goto err;
	}

	res = clk_set_rate(m3x2_clk, tgt_rate);
	if (res < 0) {
		pr_err("tas5713: failed to set m3x2 clk"
			       "rate to %lu (res = %d).  "
			       "driver will not load.\n",
			       tgt_rate, res);
		goto err;
	}

	mclk_src = clk_get(NULL, "auxclk1_src_ck");
	if (IS_ERR_OR_NULL(mclk_out)) {
		pr_err("tas5713: failed to auxclk1_ck\n");
		goto err;
	}

	res = clk_set_parent(mclk_src, m3x2_clk);
	if (res < 0) {
		pr_err("tas5713: failed to set reference clock"
				" for fref_clk1_out (res = %d)  "
				"driver will not load.\n", res);
		goto err;
	}

	res = clk_set_rate(mclk_out, tas5713_mclk_rate);
	if (res < 0) {
		pr_err("tas5713: failed to set mclk_out"
			       "rate to %lu (res = %d).  "
			       "driver will not load.\n",
			       tas5713_mclk_rate, res);
		goto err;
	}

	/* Stash the clock and enable the pin. */
	tas5713_pdata.mclk_out = mclk_out;
	omap_mux_init_signal("fref_clk1_out", OMAP_PIN_OUTPUT);

err:
	if (!IS_ERR_OR_NULL(m3x2_clk))
		clk_put(m3x2_clk);
	if (!IS_ERR_OR_NULL(mclk_src))
		clk_put(mclk_src);
	if (!IS_ERR_OR_NULL(mcbsp_internal_clk))
		clk_put(mcbsp_internal_clk);
	if (!IS_ERR_OR_NULL(abe_24m_clk))
		clk_put(abe_24m_clk);
	/* TODO: release gpios, but we're fatal anyway. */
	return;
}

/******************************************************************************
 *                                                                            *
 *                     McASP S/PDIF Audio Initialization                      *
 *                                                                            *
 ******************************************************************************/

static struct platform_device steelhead_spdif_dit_device = {
	.name		= "spdif-dit",
	.id		= 0,
};

static void __init steelhead_platform_init_mcasp_audio(void)
{
	int res;

	res = omap_mux_init_signal("abe_mcbsp2_dr.abe_mcasp_axr",
				   OMAP_PIN_OUTPUT);
	if (res) {
		pr_err("omap-mcasp: failed to enable MCASP_AXR pin mux, S/PDIF"
				"will be unavailable. (res = %d)\n", res);
		return;
	}

	platform_device_register(&steelhead_spdif_dit_device);
	return;
};

/******************************************************************************
 *                                                                            *
 *           AVR Front Panel Controls and LED Initialization                  *
 *                                                                            *
 ******************************************************************************/

#define AVR_RESET_GPIO_ID 48
#define AVR_INT_GPIO_ID   49

static struct steelhead_avr_platform_data steelhead_avr_pdata = {
	.interrupt_gpio = AVR_INT_GPIO_ID,
	.reset_gpio = AVR_RESET_GPIO_ID,
};

static void steelhead_platform_init_avr(void)
{
	/* reset should already have been released by bootloader */
	static struct steelhead_gpio_reservation avr_gpios[] = {
		{
			.gpio_id = AVR_RESET_GPIO_ID,
			.gpio_name = "avr_reset",
			.mux_name = "gpmc_a24.gpio_48",
			.pin_mode = OMAP_PIN_OUTPUT,
			.init_state = GPIOF_OUT_INIT_HIGH
		},
		{
			.gpio_id = AVR_INT_GPIO_ID,
			.gpio_name = "avr_int",
			.mux_name = "gpmc_a25.gpio_49",
			.pin_mode = OMAP_PIN_INPUT_PULLUP | OMAP_WAKEUP_EN,
			.init_state = GPIOF_IN,

		},
	};

	if (steelhead_reserve_gpios(avr_gpios, ARRAY_SIZE(avr_gpios),
				    "steelhead-avr", false))
		return;
}

/******************************************************************************
 *                                                                            *
 *           PN544 NFC                                                        *
 *                                                                            *
 ******************************************************************************/
#define GPIO_NFC_FW  162
#define GPIO_NFC_EN  163
#define GPIO_NFC_IRQ 164

static struct pn544_i2c_platform_data pn544_pdata = {
	.irq_gpio = GPIO_NFC_IRQ,
	.ven_gpio = GPIO_NFC_EN,
	.firm_gpio = GPIO_NFC_FW,
};

void __init omap4_steelhead_nfc_init(void)
{
	static struct steelhead_gpio_reservation nfc_gpios[] = {
		{
			.gpio_id = GPIO_NFC_FW,
			.gpio_name = "nfc_fw",
			.mux_name = "usbb2_ulpitll_dat1.gpio_162",
			.pin_mode = OMAP_PIN_OUTPUT,
			.init_state = GPIOF_OUT_INIT_LOW,
		},
		{
			.gpio_id = GPIO_NFC_EN,
			.gpio_name = "nfc_en",
			.mux_name = "usbb2_ulpitll_dat2.gpio_163",
			.pin_mode = OMAP_PIN_OUTPUT,
			.init_state = GPIOF_OUT_INIT_LOW,
		},
		{
			.gpio_id = GPIO_NFC_IRQ,
			.gpio_name = "nfc_irq",
			.mux_name = "usbb2_ulpitll_dat3.gpio_164",
			.pin_mode = OMAP_PIN_INPUT_PULLUP,
			.init_state = GPIOF_IN,
		},
	};

	if (steelhead_reserve_gpios(nfc_gpios, ARRAY_SIZE(nfc_gpios),
				    "steelhead-nfc", false))
		return;

}

/******************************************************************************
 *                                                                            *
 *                            I2C Bus setup                                   *
 *                                                                            *
 ******************************************************************************/

static struct i2c_board_info __initdata steelhead_i2c_bus2[] = {
	{
		I2C_BOARD_INFO("steelhead-avr", (0x20)),
		.platform_data = &steelhead_avr_pdata,
	},
	{
		I2C_BOARD_INFO("tmp101", (0x48)),
	},
};

static struct i2c_board_info __initdata steelhead_i2c_bus3[] = {
	{
		I2C_BOARD_INFO("pn544", (0x28)),
		.irq = OMAP_GPIO_IRQ(GPIO_NFC_IRQ),
		.platform_data = &pn544_pdata,
	},
};

static struct i2c_board_info __initdata steelhead_i2c_bus4[] = {
	{
		I2C_BOARD_INFO("tas5713", (0x36 >> 1)),
		.platform_data = &tas5713_pdata,
	},
};

static int __init steelhead_i2c_init(void)
{
	omap_mux_init_signal("sys_nirq1", OMAP_PIN_INPUT_PULLUP |
			     OMAP_WAKEUP_EN);
	omap_mux_init_signal("i2c1_scl.i2c1_scl", OMAP_PIN_INPUT);
	omap_mux_init_signal("i2c1_sda.i2c1_sda", OMAP_PIN_INPUT);
	omap_mux_init_signal("i2c2_scl.i2c2_scl", OMAP_PIN_INPUT);
	omap_mux_init_signal("i2c2_sda.i2c2_sda", OMAP_PIN_INPUT);
	omap_mux_init_signal("i2c3_scl.i2c3_scl", OMAP_PIN_INPUT);
	omap_mux_init_signal("i2c3_sda.i2c3_sda", OMAP_PIN_INPUT);
	omap_mux_init_signal("i2c4_scl.i2c4_scl", OMAP_PIN_INPUT);
	omap_mux_init_signal("i2c4_sda.i2c4_sda", OMAP_PIN_INPUT);

	/*
	 * This will allow unused regulator to be shutdown. This flag
	 * should be set in the board file. Before regulators are registered.
	 */
	regulator_has_full_constraints();

	/* i2c1 - PMIC */
	omap4_pmic_init("twl6030", &steelhead_twldata);
	/* i2c2 - AVR */
	omap_register_i2c_bus(2, 400, steelhead_i2c_bus2,
			      ARRAY_SIZE(steelhead_i2c_bus2));
	/* i2c3 - NFC */
	omap_register_i2c_bus(3, 400, steelhead_i2c_bus3,
			      ARRAY_SIZE(steelhead_i2c_bus3));
	/* i2c4 - TAS5713 */
	omap_register_i2c_bus(4, 400, steelhead_i2c_bus4,
			      ARRAY_SIZE(steelhead_i2c_bus4));

	/*
	 * Drive MSECURE high for TWL6030 write access.
	 */
	omap_mux_init_signal("fref_clk0_out.gpio_wk6", OMAP_PIN_OUTPUT);
	gpio_request(6, "msecure");
	gpio_direction_output(6, 1);

	return 0;
}

#define GPIO_HDMI_HPD		63

/* control signals going to TPD12S015A level converter for HDMI */
#define HDMI_GPIO_CT_CP_HPD	60	/* Enable for HDMI HPD and 5V output */
#define HDMI_GPIO_LS_OE		41	/* Enable for HDMI level converter */

static void omap4_steelhead_hdmi_mux_init(void)
{
	u32 reg;
	int err;

	/* PAD0_HDMI_HPD_PAD1_HDMI_CEC */
	omap_mux_init_signal("hdmi_cec.hdmi_cec",
			OMAP_PIN_INPUT_PULLUP);

	/* PAD0_HDMI_DDC_SCL_PAD1_HDMI_DDC_SDA */
	omap_mux_init_signal("hdmi_ddc_scl.hdmi_ddc_scl",
			OMAP_PIN_INPUT);
	omap_mux_init_signal("hdmi_ddc_sda.hdmi_ddc_sda",
			OMAP_PIN_INPUT);

	reg = omap4_ctrl_pad_readl(OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_I2C_1);
	reg |= (OMAP4_HDMI_DDC_SDA_PULLUPRESX_MASK | OMAP4_HDMI_DDC_SCL_PULLUPRESX_MASK);
	omap4_ctrl_pad_writel(reg, OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_I2C_1);

	/* do gpio setup of HDMI_HPD.  the hdmi driver uses gpio
	 * interrupt to detect hot plug and notify Android userland
	 * of connect/disconnect.
	 */
	err = omap_mux_init_gpio(GPIO_HDMI_HPD, OMAP_PIN_INPUT_PULLDOWN);
	if (err)
		pr_err("omap_mux_init_gpio() of HDMI_HPD returned %d\n", err);
	err = gpio_request_one(GPIO_HDMI_HPD, GPIOF_IN, "hdmi_hpd");
	if (err)
		pr_err("gpio_request_one() of hdmi_hpd returned %d\n", err);

	/* enable HPD detect for HDMI in TPD12S015A */
	err = omap_mux_init_gpio(HDMI_GPIO_CT_CP_HPD, OMAP_PIN_OUTPUT);
	if (err)
		pr_err("omap_mux_init_gpio() of HDMI_CT_CP_HPD returned %d\n",
		       err);
	err = gpio_request_one(HDMI_GPIO_CT_CP_HPD, GPIOF_OUT_INIT_HIGH,
			       "hdmi_ct_cp_hpd");
	if (err)
		pr_err("gpio_request_one() of hdmi_ct_cp_hpd returned %d\n",
		       err);

	/* enable the HDMI level shifer output enable pin, and set it to be
	 * powered up.  In theory, we can save power by only powering the level
	 * converter when someone calls enable_hdmi_lc (in response to HPD), but
	 * in practice the HDMI compliance folks want to see DDC and CEC pulled
	 * up at all times, even when HPD is not asserted.
	 */
	err = omap_mux_init_gpio(HDMI_GPIO_LS_OE, OMAP_PIN_OUTPUT);
	if (err)
		pr_err("omap_mux_init_gpio() of HDMI_LS_OE returned %d\n", err);

	err = gpio_request_one(HDMI_GPIO_LS_OE, GPIOF_OUT_INIT_HIGH,
				  "hdmi_gpio_ls_oe");
	if (err)
		pr_err("gpio_request_one() of hdmi_gpio_ls_oe returned %d\n",
		       err);
}

/* enable level converter */
static int omap4_steelhead_enable_hdmi_lc(struct omap_dss_device *dssdev)
{
	pr_info("%s:\n", __func__);

	/* NOTE : Even though we want to fix this line to be on at all times
	 * (See the commend in omap4_steelhead_hdmi_mux_init for details) we
	 * still take the time to explicitly enable the gpio in enable_hdmi_lc,
	 * operating under the theory that if some freakish bug turns off the
	 * level converter, it would not hurt to leave in the code which might
	 * someday turn it back on again.
	 */
	gpio_set_value(HDMI_GPIO_LS_OE, 1);
	return 0;
}

/* disable level converter */
static void omap4_steelhead_disable_hdmi_lc(struct omap_dss_device *dssdev)
{
	pr_info("%s:\n", __func__);
	/* NOTE : deliberately do nothing here for compliance reasons.  See
	 * the comment in omap4_steelhead_hdmi_mux_init for details.
	 */
	/* gpio_set_value(HDMI_GPIO_LS_OE, 0); */
}

static struct omap_dss_device  omap4_steelhead_hdmi_device = {
	.name = "hdmi",
	.driver_name = "hdmi_panel",
	.type = OMAP_DISPLAY_TYPE_HDMI,
	.platform_enable = omap4_steelhead_enable_hdmi_lc,
	.platform_disable = omap4_steelhead_disable_hdmi_lc,
	.clocks	= {
		.dispc	= {
			.dispc_fclk_src	= OMAP_DSS_CLK_SRC_FCK,
		},
		.hdmi	= {
			.regn	= 15,
			.regm2	= 1,
			.max_pixclk_khz = 165000,
		},
	},
	.panel = {
		.width_in_um	= CONFIG_OMAP2_LARGE_FB_WIDTH * 100,
		.height_in_um	= CONFIG_OMAP2_LARGE_FB_HEIGHT * 100,
		.fb_width_in_pixels = CONFIG_OMAP2_LARGE_FB_WIDTH,
		.fb_height_in_pixels = CONFIG_OMAP2_LARGE_FB_HEIGHT,
		/* try to use 1080p@60Hz by default when hpd is detected */
		.hdmi_default_cea_code = 16,
	},
	.hpd_gpio = GPIO_HDMI_HPD,
	.channel = OMAP_DSS_CHANNEL_DIGIT,
};

static struct omap_dss_device *omap4_steelhead_dss_devices[] = {
	&omap4_steelhead_hdmi_device,
};

static struct omap_dss_board_info omap4_steelhead_dss_data = {
	.num_devices	= ARRAY_SIZE(omap4_steelhead_dss_devices),
	.devices	= omap4_steelhead_dss_devices,
	.default_device	= &omap4_steelhead_hdmi_device,
};

#define STEELHEAD_FB_RAM_SIZE		SZ_16M /* ~1920*1080*4 * 2 */

static struct omapfb_platform_data steelhead_fb_pdata = {
	.mem_desc = {
		.region_cnt = 1,
		.region = {
			[0] = {
				.size = STEELHEAD_FB_RAM_SIZE,
			},
		},
	},
};

void omap4_steelhead_display_init(void)
{
	/* set the clock source for gfx to DPLL_PER, not sure why it's not
	 * already that way because the OPP definition for gpu_fck
	 * says to use DPLL_PER, but the default source is DPLL_CORE and
	 * nothing else appears to set the source.
	 */
	struct clk *dpll_per_m7x2_clk;
	struct clk *gpu_fck_clk;

	dpll_per_m7x2_clk = clk_get(NULL, "dpll_per_m7x2_ck");
	if (IS_ERR_OR_NULL(dpll_per_m7x2_clk)) {
		pr_err("%s: failed to fetch dpll_per_m7x2_ck\n",
		       __func__);
		goto err_get_dpll_per_m7x2_ck;
	}
	gpu_fck_clk = clk_get(NULL, "gpu_fck");
	if (IS_ERR_OR_NULL(gpu_fck_clk)) {
		pr_err("%s: failed to fetch gpu_fck\n", __func__);
		goto err_get_gpu_fck;
	}
	if (clk_set_parent(gpu_fck_clk, dpll_per_m7x2_clk) < 0) {
		pr_err("%s: failed to set gpu_fck to dpll_per_m7x2_ck\n",
		       __func__);
	}
	clk_put(gpu_fck_clk);
err_get_gpu_fck:
	clk_put(dpll_per_m7x2_clk);
err_get_dpll_per_m7x2_ck:

	omap_vram_set_sdram_vram(STEELHEAD_FB_RAM_SIZE, 0);
	omapfb_set_platform_data(&steelhead_fb_pdata);
	omap4_steelhead_hdmi_mux_init();
	omap_display_init(&omap4_steelhead_dss_data);
}

#define PUBLIC_SAR_RAM_1_FREE_OFFSET 0xA0C

static int steelhead_reboot_notifier_handler(struct notifier_block *this,
					     unsigned long code, void *_cmd)
{
	void __iomem *sar_free_p = omap4_get_sar_ram_base();

	if (!sar_free_p)
		return notifier_from_errno(-ENOMEM);

	sar_free_p += PUBLIC_SAR_RAM_1_FREE_OFFSET;
	memset(sar_free_p, 0, 32);
	if (code == SYS_RESTART) {
		/* set default for normal reboot */
		strcpy(sar_free_p, "normal");
		if (_cmd) {
			/* do special reboot requests if needed */
			if (!strcmp(_cmd, "recovery"))
				strcpy(sar_free_p, "recovery");
			else if (!strcmp(_cmd, "recovery:wipe_data"))
				strcpy(sar_free_p, "recovery:wipe_data");
			else if (!strcmp(_cmd, "bootloader"))
				strcpy(sar_free_p, "bootloader");
		}
	}

	return NOTIFY_DONE;
}

static struct notifier_block steelhead_reboot_notifier = {
	.notifier_call = steelhead_reboot_notifier_handler,
};

static ssize_t steelhead_soc_family_show(struct kobject *kobj,
				    struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "OMAP%04x\n", GET_OMAP_TYPE);
}

static ssize_t steelhead_soc_revision_show(struct kobject *kobj,
				 struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "ES%d.%d\n", (GET_OMAP_REVISION() >> 4) & 0xf,
		       GET_OMAP_REVISION() & 0xf);
}

static ssize_t steelhead_soc_die_id_show(struct kobject *kobj,
				 struct kobj_attribute *attr, char *buf)
{
	struct omap_die_id oid;
	omap_get_die_id(&oid);
	return sprintf(buf, "%08X-%08X-%08X-%08X\n", oid.id_3, oid.id_2,
			oid.id_1, oid.id_0);
}

static ssize_t steelhead_soc_prod_id_show(struct kobject *kobj,
				 struct kobj_attribute *attr, char *buf)
{
	struct omap_die_id oid;
	omap_get_production_id(&oid);
	return sprintf(buf, "%08X-%08X\n", oid.id_1, oid.id_0);
}

static ssize_t steelhead_soc_msv_show(struct kobject *kobj,
				      struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%08X\n", omap_ctrl_readl(0x013c));
}

static const char * const omap_types[] = {
	[OMAP2_DEVICE_TYPE_TEST]	= "TST",
	[OMAP2_DEVICE_TYPE_EMU]		= "EMU",
	[OMAP2_DEVICE_TYPE_SEC]		= "HS",
	[OMAP2_DEVICE_TYPE_GP]		= "GP",
	[OMAP2_DEVICE_TYPE_BAD]		= "BAD",
};

static ssize_t steelhead_soc_type_show(struct kobject *kobj,
				 struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", omap_types[omap_type()]);
}

static STEELHEAD_ATTR_RO(soc, family, steelhead_soc_family_show);
static STEELHEAD_ATTR_RO(soc, revision, steelhead_soc_revision_show);
static STEELHEAD_ATTR_RO(soc, type, steelhead_soc_type_show);
static STEELHEAD_ATTR_RO(soc, die_id, steelhead_soc_die_id_show);
static STEELHEAD_ATTR_RO(soc, production_id, steelhead_soc_prod_id_show);
static STEELHEAD_ATTR_RO(soc, msv, steelhead_soc_msv_show);

static struct attribute *steelhead_soc_prop_attrs[] = {
	&steelhead_soc_prop_attr_family.attr,
	&steelhead_soc_prop_attr_revision.attr,
	&steelhead_soc_prop_attr_type.attr,
	&steelhead_soc_prop_attr_die_id.attr,
	&steelhead_soc_prop_attr_production_id.attr,
	&steelhead_soc_prop_attr_msv.attr,
	NULL,
};

static struct attribute_group steelhead_soc_prop_attr_group = {
	.attrs = steelhead_soc_prop_attrs,
};

static ssize_t steelhead_board_revision_show(struct kobject *kobj,
	 struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%s (0x%02x)\n", omap4_steelhead_hw_rev_name(),
		steelhead_hw_rev);
}

static STEELHEAD_ATTR_RO(board, revision, steelhead_board_revision_show);
static struct attribute *steelhead_board_prop_attrs[] = {
	&steelhead_board_prop_attr_revision.attr,
	NULL,
};

static struct attribute_group steelhead_board_prop_attr_group = {
	.attrs = steelhead_board_prop_attrs,
};

static void __init omap4_steelhead_create_board_props(void)
{
	struct kobject *board_props_kobj;
	struct kobject *soc_kobj;
	struct kobject *audio_kobj;
	int ret = 0;

	board_props_kobj = kobject_create_and_add("board_properties", NULL);
	if (!board_props_kobj)
		goto err_board_obj;

	soc_kobj = kobject_create_and_add("soc", board_props_kobj);
	if (!soc_kobj)
		goto err_soc_obj;

	audio_kobj = kobject_create_and_add("audio", board_props_kobj);
	if (!audio_kobj)
		goto err_audio_obj;

	ret = sysfs_create_group(board_props_kobj,
				 &steelhead_board_prop_attr_group);
	if (ret)
		goto err_board_sysfs_create;

	ret = sysfs_create_group(soc_kobj, &steelhead_soc_prop_attr_group);
	if (ret)
		goto err_soc_sysfs_create;

	ret = sysfs_create_group(audio_kobj, &steelhead_audio_prop_attr_group);
	if (ret)
		goto err_audio_sysfs_create;

	return;

err_audio_sysfs_create:
	sysfs_remove_group(soc_kobj, &steelhead_soc_prop_attr_group);
err_soc_sysfs_create:
	sysfs_remove_group(board_props_kobj, &steelhead_board_prop_attr_group);
err_board_sysfs_create:
	kobject_put(audio_kobj);
err_audio_obj:
	kobject_put(soc_kobj);
err_soc_obj:
	kobject_put(board_props_kobj);
err_board_obj:
	if (!board_props_kobj || !soc_kobj || ret)
		pr_err("failed to create board_properties\n");
}

/* At 1080p FB, we need L3 to run at OPP100 to avoid DSS underflows.
 * Be careful how many of such bus constraints are held because they
 * are all added up and in omap-pm-helper.c and there is no overflow
 * check at this time.  5 callers at 800000 KBit/sec should be okay.
 * McASP currently holds one such constraint.
 */
static void steelhead_platform_reserve_l3_bus_bw(void) {
	static struct device dummy_bw_reserve_dev = {
		.init_name = "steelhead_l3_bw_reserve_dev",
	};

	omap_pm_set_min_bus_tput(&dummy_bw_reserve_dev,
				OCP_INITIATOR_AGENT,
				200 * 1000 * 4);
}


static void __init steelhead_init(void)
{
	int package = OMAP_PACKAGE_CBS;
	int status;

	if (omap_rev() == OMAP4430_REV_ES1_0)
		package = OMAP_PACKAGE_CBL;

	omap4_mux_init(NULL, NULL, package);

	omap4_steelhead_init_hw_rev();
	omap4_steelhead_emif_init();

	register_reboot_notifier(&steelhead_reboot_notifier);

	steelhead_platform_reserve_l3_bus_bw();

	steelhead_platform_init_avr();
	steelhead_platform_init_tas5713_audio();
#if defined(CONFIG_SND_OMAP_SOC_MCASP)
	steelhead_platform_init_mcasp_audio();
#endif

	steelhead_i2c_init();
	ramconsole_pdata.bootinfo = omap4_get_resetreason();
	platform_add_devices(steelhead_devices, ARRAY_SIZE(steelhead_devices));
	board_serial_init();
	omap4_twl6030_hsmmc_init(mmc);
	omap4_steelhead_ehci_init();
	usb_musb_init(&musb_board_data);
	omap4_steelhead_create_board_props();
	omap_dmm_init();
	omap4_steelhead_display_init();
	omap4_steelhead_nfc_init();

	/* Vsel0 = gpio, vsel1 = gnd */
	status = omap_tps6236x_board_setup(true, TPS62361_GPIO, -1,
					   OMAP_PIN_OFF_OUTPUT_HIGH,
					   -1);
	if (status)
		pr_err("TPS62361 initialization failed: %d\n", status);

	/*
	 * Some devices have a 4430 chip on a 4460 board, manually
	 * tweak the power tree to the 4460 style with the TPS
	 * regulator.
	 */
	if (cpu_is_omap443x()) {
		/* Disable 4430 mapping */
		omap_twl_pmic_update("mpu", CHIP_IS_OMAP443X, 0x0);
		omap_twl_pmic_update("core", CHIP_IS_OMAP443X, 0x0);
		/* make 4460 map usable for 4430 */
		omap_twl_pmic_update("core", CHIP_IS_OMAP446X,
				     CHIP_IS_OMAP443X);
		omap_tps6236x_update("mpu", CHIP_IS_OMAP446X,
				     CHIP_IS_OMAP443X);
	}
	if (enable_sr)
		omap_enable_smartreflex_on_init();
	steelhead_init_wlan();
	steelhead_init_bluetooth();
}

static int __init steelhead_init_late(void)
{
	steelhead_platform_init_counter();
	return 0;
}

/*
 * This is needed because the platform counter is dependent on omap_dm_timers
 * which get initialized at device_init time
 */
late_initcall(steelhead_init_late);

static void __init steelhead_map_io(void)
{
	omap2_set_globals_443x();
	omap44xx_map_common_io();
}

static void __init steelhead_reserve(void)
{
	int i;
	int ret;

	/* do the static reservations first */
	memblock_remove(STEELHEAD_RAMCONSOLE_START, STEELHEAD_RAMCONSOLE_SIZE);

	memblock_remove(PHYS_ADDR_SMC_MEM, PHYS_ADDR_SMC_SIZE);
	memblock_remove(PHYS_ADDR_DUCATI_MEM, PHYS_ADDR_DUCATI_SIZE);

	for (i = 0; i < sh_ion_data.nr; i++) {
		if (sh_ion_data.heaps[i].type == ION_HEAP_TYPE_CARVEOUT ||
		    sh_ion_data.heaps[i].type == OMAP_ION_HEAP_TYPE_TILER) {
			ret = memblock_remove(sh_ion_data.heaps[i].base,
					      sh_ion_data.heaps[i].size);
			if (ret)
				pr_err("memblock remove of %x@%lx failed\n",
				       sh_ion_data.heaps[i].size,
				       sh_ion_data.heaps[i].base);
		}
	}
	/* ipu needs to recognize secure input buffer area as well */
	omap_ipu_set_static_mempool(PHYS_ADDR_DUCATI_MEM,
				    PHYS_ADDR_DUCATI_SIZE +
				    OMAP_STEELHEAD_ION_HEAP_SECURE_INPUT_SIZE);

	omap_reserve();
}

MACHINE_START(STEELHEAD, "Steelhead")
	/* Maintainer: Google, Inc */
	.boot_params	= 0x80000100,
	.reserve	= steelhead_reserve,
	.map_io		= steelhead_map_io,
	.init_early	= steelhead_init_early,
	.init_irq	= gic_init_irq,
	.init_machine	= steelhead_init,
	.timer		= &omap_timer,
MACHINE_END
