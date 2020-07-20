/*
 * da9063-hwmon.c - Hardware monitor support for DA9063
 * Copyright (C) 2014-2015  Dialog Semiconductor Ltd.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/platform_device.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/regmap.h>
#include <linux/of.h>
#include <linux/mfd/da9063/core.h>
#include <linux/mfd/da9063/pdata.h>

#define DA9063_ADC_RES	(1 << (DA9063_ADC_RES_L_BITS + DA9063_ADC_RES_M_BITS))
#define DA9063_ADC_MAX	(DA9063_ADC_RES - 1)
#define DA9063_2V5	2500
#define DA9063_5V0	5000
#define DA9063_5V5	5500
#define DA9063_TJUNC_M	-408
#define DA9063_TJUNC_C	-829
#define DA9063_VBBAT_M	2048

enum da9063_adc {
	DA9063_CHAN_VSYS = DA9063_ADC_MUX_VSYS,
	DA9063_CHAN_ADCIN1 = DA9063_ADC_MUX_ADCIN1,
	DA9063_CHAN_ADCIN2 = DA9063_ADC_MUX_ADCIN2,
	DA9063_CHAN_ADCIN3 = DA9063_ADC_MUX_ADCIN3,
	DA9063_CHAN_TJUNC = DA9063_ADC_MUX_T_SENSE,
	DA9063_CHAN_VBBAT = DA9063_ADC_MUX_VBBAT,
	DA9063_CHAN_LDO_G1 = DA9063_ADC_MUX_LDO_G1,
	DA9063_CHAN_LDO_G2 = DA9063_ADC_MUX_LDO_G2,
	DA9063_CHAN_LDO_G3 = DA9063_ADC_MUX_LDO_G3
};

struct da9063_hwmon {
	struct da9063 *da9063;
	struct device *classdev;
	struct mutex hwmon_mutex;
	struct completion adc_ready;
	signed char tjunc_offset;
	int irq;
};

static int da9063_adc_convert(struct da9063_hwmon *hwmon, int channel,
			      int *value)
{
	int val = *value;
	int ret = 0;

	switch (channel) {
	case DA9063_CHAN_ADCIN1:
	case DA9063_CHAN_ADCIN2:
	case DA9063_CHAN_ADCIN3:
		val = (DA9063_2V5 * val) / DA9063_ADC_MAX;
		break;
	case DA9063_CHAN_VSYS:
		val = ((DA9063_5V5 - DA9063_2V5) * val) / DA9063_ADC_MAX +
			DA9063_2V5;
		break;
	case DA9063_CHAN_TJUNC:
		val -= hwmon->tjunc_offset;
		val = (DA9063_TJUNC_M * (val + DA9063_TJUNC_C)) >> 10;
		break;
	case DA9063_CHAN_VBBAT:
		val = (DA9063_5V0 * val) / DA9063_ADC_MAX;
		break;
	default:
		ret = -EINVAL;
		goto err_convert;
	}

	*value = val;
err_convert:
	return ret;
}

static int da9063_adc_current_switch(struct da9063_hwmon *hwmon, int channel,
				     bool on)
{
	int ret;
	unsigned int val;
	unsigned int mask;

	switch (channel) {
	case DA9063_CHAN_ADCIN1:
		mask = DA9063_ADC_AD1_ISRC_EN;
		break;
	case DA9063_CHAN_ADCIN2:
		mask = DA9063_ADC_AD2_ISRC_EN;
		break;
	case DA9063_CHAN_ADCIN3:
		mask = DA9063_ADC_AD3_ISRC_EN;
		break;
	default:
		ret = -EINVAL;
		goto err_switch;
	}

	if (on)
		val = mask;
	else
		val = ~mask;

	ret = regmap_update_bits(hwmon->da9063->regmap, DA9063_REG_ADC_CONT,
				 mask, val);
err_switch:
	return ret;

}

static int da9063_adc_manual_read(struct da9063_hwmon *hwmon, int channel)
{
	int ret;
	unsigned char val;
	unsigned char data[2];
	int adc_man;

	mutex_lock(&hwmon->hwmon_mutex);

	init_completion(&hwmon->adc_ready);

	val = (channel & DA9063_ADC_MUX_MASK) | DA9063_ADC_MAN;
	ret = regmap_update_bits(hwmon->da9063->regmap, DA9063_REG_ADC_MAN,
				 DA9063_ADC_MUX_MASK | DA9063_ADC_MAN, val);
	if (ret < 0)
		goto err_mread;

	ret = wait_for_completion_timeout(&hwmon->adc_ready,
					  msecs_to_jiffies(1000));
	if (ret == 0) {
		ret = -ETIMEDOUT;
		goto err_mread;
	}

	ret = regmap_read(hwmon->da9063->regmap, DA9063_REG_ADC_MAN, &adc_man);
	if (ret < 0)
		goto err_mread;

	/* data value is not ready */
	if ((adc_man & DA9063_ADC_MAN) != 0) {
		msleep(10);

		ret = regmap_read(hwmon->da9063->regmap, DA9063_REG_ADC_MAN,
				  &adc_man);
		if (ret < 0)
			goto err_mread;

		if ((adc_man & DA9063_ADC_MAN) != 0) {
			ret = -EINVAL;
			goto err_mread;
		}
	}

	ret = regmap_bulk_read(hwmon->da9063->regmap,
			       DA9063_REG_ADC_RES_L, data, 2);
	if (ret < 0)
		goto err_mread;

	ret = (data[0] & DA9063_ADC_RES_L_MASK) >> DA9063_ADC_RES_L_SHIFT;
	ret |= data[1] << DA9063_ADC_RES_L_BITS;
err_mread:
	mutex_unlock(&hwmon->hwmon_mutex);
	return ret;
}

static irqreturn_t da9063_hwmon_irq_handler(int irq, void *irq_data)
{
	struct da9063_hwmon *hwmon = irq_data;
	complete(&hwmon->adc_ready);
	return IRQ_HANDLED;
}

static ssize_t da9063_adc_read(struct device *dev,
			       struct device_attribute *devattr, char *buf)
{
	struct da9063_hwmon *hwmon = dev_get_drvdata(dev);
	int channel = to_sensor_dev_attr(devattr)->index;
	int val;
	int ret;

	switch (channel) {
	case DA9063_CHAN_ADCIN1:
	case DA9063_CHAN_ADCIN2:
	case DA9063_CHAN_ADCIN3:
		/* fallthrough for ADC measures */
		ret = da9063_adc_current_switch(hwmon, channel, true);
		if (ret < 0)
			goto err_read;

		val = da9063_adc_manual_read(hwmon, channel);
		if (val < 0) {
			ret = val;
			if (ret == -EINVAL)
				dev_err(dev, "Conversion was not completed\n");
			else
				dev_err(dev, "ADC read error %d\n", ret);
			goto err_read;
		}

		ret = da9063_adc_current_switch(hwmon, channel, false);
		if (ret < 0) {
			dev_err(dev, "Could not switch current\n");
			goto err_read;
		}
		break;

	case DA9063_CHAN_VSYS:
	case DA9063_CHAN_TJUNC:
	case DA9063_CHAN_VBBAT:
		/* fallthrough for internal measures */
		val = da9063_adc_manual_read(hwmon, channel);
		if (val < 0) {
			dev_err(dev, "ADC read error %d\n", val);
			return val;
		}
		break;

	default:
		/* error case */
		ret = -EINVAL;
		goto err_read;
	}

	ret = da9063_adc_convert(hwmon, channel, &val);
	if (ret < 0) {
		dev_err(dev, "Failed to convert ADC value %d\n", ret);
		goto err_read;
	}

	return sprintf(buf, "%d\n", val);
err_read:
	return ret;
}

static ssize_t da9063_show_name(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sprintf(buf, DA9063_DRVNAME_HWMON "\n");
}

//====================================================
//fj_ltls add 
struct da9063 *ltls_da9063;
static ssize_t ltls_show_reg(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int val,ret;
#if 0
	ret = regmap_read(ltls_da9063->regmap, DA9063_REG_ADC_CFG, &val);
	if (ret < 0) {
		dev_err(&ltls_da9063->regmap,
			"Failed to read read the register\n");
		return -EIO;
		}
	
	return sprintf(buf, "%d\n", val);
#endif
ret = regmap_read(ltls_da9063->regmap, DA9063_REG_BUCK_ILIM_C, &val);
	if (ret < 0) {
		dev_err(dev,"Failed to read read the register\n");
		return -EIO;
		}
//return sprintf(buf, "%d\n", val);
return sprintf(buf,"%s: fj_ltls regmap_read (%x,DA9063_REG_BUCK_ILIM_C=%x,val=%x)\n",__func__,ltls_da9063->regmap,DA9063_REG_BUCK_ILIM_C,val);
}
//====================================================


static ssize_t da9063_show_label(struct device *dev,
				 struct device_attribute *devattr, char *buf)
{
	int channel = to_sensor_dev_attr(devattr)->index;
	char *label;

	switch (channel) {
	case DA9063_CHAN_VSYS:
		label = "VSYS";
		break;
	case DA9063_CHAN_ADCIN1:
		label = "ADCIN1";
		break;
	case DA9063_CHAN_ADCIN2:
		label = "ADCIN2";
		break;
	case DA9063_CHAN_ADCIN3:
		label = "ADCIN3";
		break;
	case DA9063_CHAN_TJUNC:
		label = "TJUNC";
		break;
	case DA9063_CHAN_VBBAT:
		label = "VBBAT";
		break;
	default:
		label = "UNKNOWN";
	}

	return sprintf(buf, "%s\n", label);
}

static SENSOR_DEVICE_ATTR(in0_input, S_IRUGO,
			  da9063_adc_read, NULL, DA9063_CHAN_VSYS);
static SENSOR_DEVICE_ATTR(in0_label, S_IRUGO,
			  da9063_show_label, NULL, DA9063_CHAN_VSYS);

static SENSOR_DEVICE_ATTR(in1_input, S_IRUGO,
			  da9063_adc_read, NULL, DA9063_CHAN_ADCIN1);
static SENSOR_DEVICE_ATTR(in1_label, S_IRUGO,
			  da9063_show_label, NULL, DA9063_CHAN_ADCIN1);

static SENSOR_DEVICE_ATTR(in2_input, S_IRUGO,
			  da9063_adc_read, NULL, DA9063_CHAN_ADCIN2);
static SENSOR_DEVICE_ATTR(in2_label, S_IRUGO,
			  da9063_show_label, NULL, DA9063_CHAN_ADCIN2);

static SENSOR_DEVICE_ATTR(in3_input, S_IRUGO,
			  da9063_adc_read, NULL, DA9063_CHAN_ADCIN3);
static SENSOR_DEVICE_ATTR(in3_label, S_IRUGO,
			  da9063_show_label, NULL, DA9063_CHAN_ADCIN3);

static SENSOR_DEVICE_ATTR(in4_input, S_IRUGO,
			  da9063_adc_read, NULL, DA9063_CHAN_VBBAT);
static SENSOR_DEVICE_ATTR(in4_label, S_IRUGO,
			  da9063_show_label, NULL, DA9063_CHAN_VBBAT);

static SENSOR_DEVICE_ATTR(temp1_input, S_IRUGO,
			  da9063_adc_read, NULL, DA9063_CHAN_TJUNC);

static SENSOR_DEVICE_ATTR(temp1_label, S_IRUGO,
			  da9063_show_label, NULL, DA9063_CHAN_TJUNC);

static DEVICE_ATTR(name, S_IRUGO, da9063_show_name, NULL);
static DEVICE_ATTR(ltls, S_IRUGO, ltls_show_reg, NULL);

static struct attribute *da9063_attributes[] = {
	&dev_attr_name.attr,
	&sensor_dev_attr_in0_input.dev_attr.attr,
	&sensor_dev_attr_in0_label.dev_attr.attr,
	&sensor_dev_attr_in1_input.dev_attr.attr,
	&sensor_dev_attr_in1_label.dev_attr.attr,
	&sensor_dev_attr_in2_input.dev_attr.attr,
	&sensor_dev_attr_in2_label.dev_attr.attr,
	&sensor_dev_attr_in3_input.dev_attr.attr,
	&sensor_dev_attr_in3_label.dev_attr.attr,
	&sensor_dev_attr_in4_input.dev_attr.attr,
	&sensor_dev_attr_in4_label.dev_attr.attr,
	&sensor_dev_attr_temp1_input.dev_attr.attr,
	&sensor_dev_attr_temp1_label.dev_attr.attr,
	&dev_attr_ltls.attr,
	NULL
};

static const struct attribute_group da9063_attr_group = {
	.attrs = da9063_attributes,
};

#if 1
//====================================================
//fj_ltls 20160703 for adc battery
struct da9063_hwmon *da9063_adc_bat;
int da9063_adc_read_batt(int channel)
{
	int val;
	if (da9063_adc_bat != NULL){
		val = da9063_adc_manual_read(da9063_adc_bat, channel);
		da9063_adc_convert(da9063_adc_bat, channel, &val);
		return val;
	}
	return -1;
}
EXPORT_SYMBOL_GPL(da9063_adc_read_batt);
#endif
static int da9063_hwmon_probe(struct platform_device *pdev)
{
	struct da9063 *da9063 = dev_get_drvdata(pdev->dev.parent);
	struct da9063_pdata *pdata = da9063->dev->platform_data;
	struct da9063_hwmon *hwmon;
	int ret;
	unsigned int val;

	hwmon = devm_kzalloc(&pdev->dev, sizeof(struct da9063_hwmon),
			     GFP_KERNEL);
	if (!hwmon)
		return -ENOMEM;

	mutex_init(&hwmon->hwmon_mutex);
	init_completion(&hwmon->adc_ready);
	hwmon->da9063 = da9063;

	/* enable the ADC functions for GPIO 0,1,2 */
	val = (DA9063_GPIO0_PIN_ADCIN1 << DA9063_GPIO0_PIN_MASK_SHIFT |
	       DA9063_GPIO1_PIN_ADCIN2_COMP << DA9063_GPIO1_PIN_MASK_SHIFT);
	ret = regmap_update_bits(hwmon->da9063->regmap, DA9063_REG_GPIO_0_1,
				 (DA9063_GPIO0_PIN_MASK |
				  DA9063_GPIO1_PIN_MASK), val);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"Failed to alter the ADCIN 1,2 bits for the GPIO 0,1 register\n");
		return -EIO;
	}

	val = DA9063_GPIO2_PIN_ADCIN3 << DA9063_GPIO2_PIN_MASK_SHIFT;
	ret = regmap_update_bits(hwmon->da9063->regmap, DA9063_REG_GPIO_2_3,
				 DA9063_GPIO2_PIN_MASK, val);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"Failed to alter the ADCIN 3 bits for the GPIO 2,3 register\n");
		return -EIO;
	}

	/* debounce ADC I settings */
	val = (DA9063_ADCIN1_DEB_ON << DA9063_REG_ADCIN1_DEB_SHIFT |
	       DA9063_ADCIN2_DEB_ON << DA9063_REG_ADCIN2_DEB_SHIFT |
	       DA9063_ADCIN3_DEB_ON << DA9063_REG_ADCIN3_DEB_SHIFT);
	ret = regmap_update_bits(hwmon->da9063->regmap, DA9063_REG_ADC_CFG,
				 (DA9063_REG_ADCIN1_DEB_MASK |
				  DA9063_REG_ADCIN2_DEB_MASK |
				  DA9063_REG_ADCIN3_DEB_MASK), val);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"Failed to alter the ADC configuration register\n");
		return -EIO;
	}

	/* set up the current configurations */
	if (pdata)
		val = (pdata->hwmon_pdata->adcin1_cur << DA9063_REG_ADCIN1_CUR_SHIFT |
		       pdata->hwmon_pdata->adcin2_cur << DA9063_REG_ADCIN2_CUR_SHIFT |
		       pdata->hwmon_pdata->adcin3_cur << DA9063_REG_ADCIN3_CUR_SHIFT);
	else {
		unsigned char tmpc1;
		unsigned char tmpc2;
		unsigned char tmpc3;
		unsigned char *exists;

		/* fj_ltls del
		exists = (unsigned char *)of_get_property((&pdev->dev)->of_node,
					 "dlg,adcin1_cur", NULL);
		if (!exists) {
			dev_err(&pdev->dev,
				"Device tree dlg,adcin1_cur does not exist\n");
			return -EINVAL;
		}
		exists = (unsigned char *)of_get_property((&pdev->dev)->of_node,
					 "dlg,adcin2_cur", NULL);
		if (!exists) {
			dev_err(&pdev->dev,
				"Device tree dlg,adcin2_cur does not exist\n");
			return -EINVAL;
		}
		exists = (unsigned char *)of_get_property((&pdev->dev)->of_node,
					 "dlg,adcin3_cur", NULL);
		if (!exists) {
			dev_err(&pdev->dev,
				"Device tree dlg,adcin3_cur does not exist\n");
			return -EINVAL;
		}

		ret = of_property_read_u8((&pdev->dev)->of_node,
					  "dlg,adcin1_cur", &tmpc1);
		if (ret < 0) {
			dev_err(&pdev->dev,
				"Reading device tree ADCIN1 value\n");
			return -EINVAL;
		}
		if (tmpc1 < DA9063_SET_ADCIN1_CUR_1UA ||
		    tmpc1 > DA9063_SET_ADCIN1_CUR_40UA) {
			dev_err(&pdev->dev,
				"Device tree ADCIN1 current is not valid\n");
			return -EINVAL;
		}

		ret = of_property_read_u8((&pdev->dev)->of_node,
					  "dlg,adcin2_cur", &tmpc2);
		if (ret < 0) {
			dev_err(&pdev->dev,
				"Reading device tree ADCIN2 value\n");
			return -EINVAL;
		}
		if (tmpc2 < DA9063_SET_ADCIN2_CUR_1UA ||
		    tmpc2 > DA9063_SET_ADCIN2_CUR_40UA) {
			dev_err(&pdev->dev,
				"Device tree ADCIN2 current is not valid\n");
			return -EINVAL;
		}

		ret = of_property_read_u8((&pdev->dev)->of_node,
					  "dlg,adcin3_cur", &tmpc3);
		if (ret < 0) {
			dev_err(&pdev->dev,
				"Reading device tree ADCIN3 value\n");
			return -EINVAL;
		}
		if (tmpc3 != DA9063_SET_ADCIN3_CUR_10UA) {
			dev_err(&pdev->dev,
				"Device tree ADCIN3 value is not valid\n");
			return -EINVAL;
		}
*/
		tmpc1=0x02;
		tmpc2=0x02;
		tmpc3=0x00;

		val = (tmpc1 << DA9063_REG_ADCIN1_CUR_SHIFT |
		       tmpc2 << DA9063_REG_ADCIN2_CUR_SHIFT |
		       tmpc3 << DA9063_REG_ADCIN3_CUR_SHIFT);
	}

	ret = regmap_update_bits(hwmon->da9063->regmap, DA9063_REG_ADC_CFG,
				 (DA9063_REG_ADCIN1_CUR_MASK |
				  DA9063_REG_ADCIN2_CUR_MASK |
				  DA9063_REG_ADCIN3_CUR_MASK), val);

	ret = regmap_read(da9063->regmap, DA9063_REG_ADC_CFG, &val);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"Failed to read read the ADC configuration register\n");
		return -EIO;
	}

	hwmon->irq = platform_get_irq_byname(pdev, DA9063_DRVNAME_HWMON);
	if (hwmon->irq < 0)
		return hwmon->irq;

	ret = devm_request_threaded_irq(&pdev->dev, hwmon->irq, NULL,
					da9063_hwmon_irq_handler,
					IRQF_TRIGGER_LOW | IRQF_ONESHOT,
					"HWMON", hwmon);
	if (ret) {
		dev_err(&pdev->dev, "Failed to request IRQ.\n");
		return ret;
	}

	platform_set_drvdata(pdev, hwmon);

	/* set trim temperature offset to value read at startup */
	hwmon->tjunc_offset = (signed char)hwmon->da9063->t_offset;

	ret = sysfs_create_group(&pdev->dev.kobj, &da9063_attr_group);
	if (ret)
		return ret;

	hwmon->classdev = hwmon_device_register(&pdev->dev);
	if (IS_ERR(hwmon->classdev)) {
		sysfs_remove_group(&pdev->dev.kobj, &da9063_attr_group);
		return PTR_ERR(hwmon->classdev);
	}
da9063_adc_bat = hwmon; //fj_ltls 20160706
	ltls_da9063=da9063;//fj_ltls
//printk("david*************************************************\n");
	return 0;
}

static int da9063_hwmon_remove(struct platform_device *pdev)
{
	struct da9063_hwmon *hwmon = platform_get_drvdata(pdev);

	hwmon_device_unregister(hwmon->classdev);
	sysfs_remove_group(&pdev->dev.kobj, &da9063_attr_group);

	return 0;
}

static struct platform_driver da9063_hwmon_driver = {
	.probe = da9063_hwmon_probe,
	.remove = da9063_hwmon_remove,
	.driver = {
		   .name = DA9063_DRVNAME_HWMON,
		   .owner = THIS_MODULE,
		   },
};

module_platform_driver(da9063_hwmon_driver);

MODULE_DESCRIPTION("Hardware monitor support device driver for Dialog DA9063");
MODULE_AUTHOR("S Twiss <stwiss.opensource@diasemi.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DA9063_DRVNAME_HWMON);
