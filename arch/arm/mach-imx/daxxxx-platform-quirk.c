/*
 * Platform suspend/resume/poweroff quirk example
 *
 * Copyright (C) 2015 Dialog Semiconductor Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/mfd/da9063/registers.h>
#include <linux/pm.h>
#include <linux/suspend.h>
#include <linux/reboot.h>
#include <linux/delay.h>

static struct i2c_client *da9063_client;

static int daxxxx_suspend_pm_cb(struct notifier_block *nb,
				unsigned long action, void *ptr)
{
	switch (action) {
	case PM_SUSPEND_PREPARE:
	case PM_HIBERNATION_PREPARE:
		/*
		 * E.G. ADJUST PMIC SEQUENCER FOR SUSPEND
		 * E.G. MANIPULATE CONTROL LINE USAGE
		 * i2c_smbus_write_byte_data(daxxxx_client, <REGISTER>, <VALUE>);
		 */
		break;
	case PM_POST_SUSPEND:
	case PM_POST_HIBERNATION:
		/*
		 * E.G. RESTORE PMIC SEQUENCER / CONTROL LINES
		 * i2c_smbus_write_byte_data(daxxxx_client, <REGISTER>, <VALUE>);
		 */
		break;
	default:
		return NOTIFY_DONE;
	}
	return NOTIFY_OK;
}

static void daxxxx_poweroff_quirk(void)
{
	/*
	 * Do nothing - this must be assigned as pm_power_off callback, or
	 * otherwise /kernel/reboot.c : SYSCALL_DEFINE4(reboot, ... reduces
	 * LINUX_REBOOT_CMD_POWER_OFF to LINUX_REBOOT_CMD_HALT
	 * and so the pm_power_off_prepare callback would never be used!
	 *
	 * This callback is now apparently too late in the power off process
	 * for daxxxx I2C work, as it caused a stack dump with the message:
	 *      WARNING: CPU: 0 PID: 50 at kernel/workqueue.c:1958
	 *      process_one_work+0x3bc/0x424()
	 */
}
static void da9063_poweroff_prepare_quirk(void)
{
	/* E.G. SET PMIC MODE AND POWER OFF */
	u8 val = 0;

	/* set RTC_MODE_PD in register CONTROL_E */
	val = i2c_smbus_read_byte_data(da9063_client, DA9063_REG_CONTROL_E);
	val |= DA9063_RTC_MODE_PD;
	i2c_smbus_write_byte_data(da9063_client, DA9063_REG_CONTROL_E, val);

	msleep(250);

	printk(KERN_ALERT "Poweroff DA9063\n");

	/* clear SYSTEM_EN in register CONTROL_A */
	val = i2c_smbus_read_byte_data(da9063_client, DA9063_REG_CONTROL_A);
	val &= ~DA9063_SYSTEM_EN;
	i2c_smbus_write_byte_data(da9063_client, DA9063_REG_CONTROL_A, val);

	while (1);

	return;
}

static int daxxxx_reboot_notify(struct notifier_block *nb,
				unsigned long action, void *data)
{
	switch (action) {
	case SYS_POWER_OFF:
		break;
	case SYS_HALT:
	case SYS_RESTART:
		/*
		 * E.G. RESTORE PMIC SEQUENCER
		 * E.G. MODIFY GPIO TO RESET SLAVE DEVICE
		 * i2c_smbus_write_byte_data(daxxxx_client, <REGISTER>, <VALUE>);
		 */
		break;
	default:
		break;
	}
	return 0;
}

static struct notifier_block daxxxx_reboot_nb = {
	.notifier_call = daxxxx_reboot_notify
};

static int platform_i2c_bus_notify(struct notifier_block *nb,
				   unsigned long action, void *data)
{
	struct device *dev = data;
	struct i2c_client *client;

	if ((action != BUS_NOTIFY_ADD_DEVICE) ||
	    (dev->type == &i2c_adapter_type))
		 return 0;

	client = to_i2c_client(dev);

	if ((client->addr == 0x58 && !strcmp(client->name, "da9063"))) {
		da9063_client = client;

		/*
		 * E.G. SET IRQ MASKS
		 * i2c_smbus_write_byte_data(daxxxx_client, <REGISTER>, <VALUE>);
		 */

		/*
		 * Register PM notifier for suspend/resume switchovers
		 * of control
		 */
		pm_notifier(daxxxx_suspend_pm_cb, 0);

		/* Register reboot notifier */
		register_reboot_notifier(&daxxxx_reboot_nb);

		/* Establish poweroff callback */
		printk(KERN_INFO "Installing DA9063 poweroff control\n");
		pm_power_off_prepare = da9063_poweroff_prepare_quirk;
		pm_power_off = daxxxx_poweroff_quirk;

		/* Get rid of this notification */
		bus_unregister_notifier(&i2c_bus_type, nb);
	}

	return 0;
}

static struct notifier_block platform_i2c_bus_nb = {
	.notifier_call = platform_i2c_bus_notify
};

static int __init platform_quirk(void)
{
	da9063_client = NULL;
	bus_register_notifier(&i2c_bus_type, &platform_i2c_bus_nb);
	return 0;
}

arch_initcall(platform_quirk);

