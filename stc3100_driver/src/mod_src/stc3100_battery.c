/* drivers/power/stc3100_battery
 * STC31000 battery driver
 *
 * Copyright (C) 2009 Rockchip Corporation.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */
#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <asm/unaligned.h>
#define DRIVER_VERSION			"1.0.0"

#define BATTERY_CAPACITY_MAH   2500   	///  define battery capacity mah
#define RSENSE_RESISTANCE		(10)     ///  Rsense resistance m��

#define STC3100_REG_MODE		0x00 
#define STC3100_REG_CTRL		0x01 
#define STC3100_REG_RSOCL		0x02 /* Relative State-of-Charge */
#define STC3100_REG_RSOCH		0x03
#define STC3100_REG_COUNTL		0x04
#define STC3100_REG_COUNTH		0x05
#define STC3100_REG_AIL			0x06
#define STC3100_REG_AIH			0x07
#define STC3100_REG_VOLTL		0x08
#define STC3100_REG_VOLTH		0x09
#define STC3100_REG_TEMPL		0x0A
#define STC3100_REG_TEMPH		0x0B

struct stc3100_device_info {
	struct device 		*dev;
	struct power_supply	*bat;

	struct power_supply_desc ps_desc;
	struct power_supply_config ps_config;

	struct delayed_work work;
	unsigned int interval;
	struct i2c_client	*client;

	u16 capacity;
};

static enum power_supply_property stc3100_battery_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_STATUS
};


static int stc3100_write_reg(struct i2c_client *client, u8 reg, u8 const buf)
{
	int ret; 
	//ret = i2c_smbus_write_byte_data(client, reg, buf);
	u8 data[] = { reg, buf };
	ret = i2c_master_send(client, (const char*)data, 2);
	return ret;
}
/*
 * Common code for STC3100 devices read
 */
static int stc3100_read_reg_word(struct i2c_client *client, u8 reg, u16 *data)
{
	int ret;
	//ret = i2c_smbus_read_word_data(client, reg);
	ret = i2c_master_send(client, (const char*)&reg, 1);
	if(ret < 0) return ret;
	ret = i2c_master_recv(client, (char*)data, 2);
	return ret;
}

static int stc3100_read_reg_byte(struct i2c_client *client, u8 reg, u8 *data)
{
	int ret;
	//ret = i2c_smbus_read_byte_data(client, reg);
	ret = i2c_master_send(client, (const char*)&reg, 1);
	if(ret < 0) return ret;
	ret = i2c_master_recv(client, (char*)data, 1);
	return ret;
}

/*
 * Return the battery temperature in Celsius degrees
 * Or < 0 if something fails.
 */
static int stc3100_battery_temperature(struct stc3100_device_info *di)
{
	int ret, val, sign;
	u16 data;
	short dataSigned;

	ret = stc3100_read_reg_word(di->client,STC3100_REG_TEMPL, &data);
	dev_dbg(di->dev, "%s battery temperature 0x%x\n", __func__, (u16)ret);
	if (ret<0) 
	{
		dev_err(di->dev, "error reading temperature\n");
		return ret;
	}

	sign = !!(data & (1<<11));
	dataSigned = (data & ~(1<<11)) | (sign | 15);

	val = (dataSigned * 125) / 100; // convert to 10th of degree
	dev_dbg(di->dev, "%s battery temperature val = %d\n", __func__, val);

	return val;  ///temperature (��C) = Temperature_code * 0.125.
}

/*
 * Return the battery Voltage in milivolts
 * Or < 0 if something fails.
 */
static int stc3100_battery_voltage(struct stc3100_device_info *di)
{
	int ret, val;
	short data;

	ret = stc3100_read_reg_word(di->client,STC3100_REG_VOLTL, (u16*)&data);
	dev_dbg(di->dev, "%s battery voltage 0x%x\n", __func__, (u16)ret);
	if (ret<0) 
	{
		dev_err(di->dev, "error reading voltage\n");
		return ret;
	}

	data = (data << 4) >> 4;

	val = (data * 244) * 10; // convert to uV
	dev_dbg(di->dev, "%s battery voltage val = %d\n", __func__, val);

	return val;   //voltage (mV) = Voltage_code * 2.44.
}

/*
 * Return the battery average current
 * Note that current can be negative signed as well
 * Or 0 if something fails.
 */
static int stc3100_battery_current(struct stc3100_device_info *di)
{
	int ret, val;
	short data;

	ret = stc3100_read_reg_word(di->client,STC3100_REG_AIL, (u16*)&data);
	dev_dbg(di->dev, "%s battery current 0x%x\n", __func__, (u16)ret);
	if (ret<0) 
	{
		dev_err(di->dev, "error reading current\n");
		return 0;
	}

	data = (data << 2) >> 2;

	val = ((data * 1177) / 100) * RSENSE_RESISTANCE;
	dev_dbg(di->dev, "%s battery current val = %d\n", __func__, val);
	
	return val;   ///current (mA) = current_code* 11.77 / Rsense (m��)
}

/*
 * Return the battery Relative State-of-Charge
 * Or < 0 if something fails.
 */
static int stc3100_battery_rsoc(struct stc3100_device_info *di)
{
	int ret, val;
	short data, unused;

	ret = stc3100_read_reg_word(di->client,STC3100_REG_RSOCL, (u16*)&data);
	stc3100_read_reg_word(di->client, STC3100_REG_COUNTL, (u16*)&unused);
	dev_dbg(di->dev, "%s battery rsoc 0x%x\n", __func__, (u16)ret);
	if (ret<0) 
	{
		dev_err(di->dev, "error reading relative State-of-Charge\n");
		return ret;
	}

	val = (((data * 67) / 10) * RSENSE_RESISTANCE);
	dev_dbg(di->dev, "%s battery rsoc val = %d\n", __func__, val);

	return val;    ////charge data (mA.h) = 6.70 * charge_code / Rsense (m��).
}

static int dc_charge_status(struct stc3100_device_info *di)
{
	int ret;
	u8 data;
	ret = stc3100_read_reg_byte(di->client, STC3100_REG_CTRL, &data);
	dev_info(di->dev, "%s dc charge status %d, 0x%x\n", __func__, (u8)data, (u8)data);
	if (ret<0) 
	{
		dev_err(di->dev, "error reading CTRL register\n");
		return ret;
	}

	if(data & 1)
		return POWER_SUPPLY_STATUS_CHARGING;
	else
		return POWER_SUPPLY_STATUS_NOT_CHARGING;
}

static int stc3100_battery_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct stc3100_device_info *di = power_supply_get_drvdata(psy);

	switch (psp) 
	{
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = dc_charge_status(di);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = stc3100_battery_voltage(di);
		if (psp == POWER_SUPPLY_PROP_PRESENT)
			val->intval = val->intval <= 0 ? 0 : 1;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = stc3100_battery_current(di);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = stc3100_battery_rsoc(di);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = stc3100_battery_temperature(di);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static void stc3100_powersupply_init(struct stc3100_device_info *di)
{
	memset(&di->ps_desc, 0, sizeof di->ps_desc);

	di->ps_desc.get_property = stc3100_battery_get_property;
	di->ps_desc.external_power_changed = NULL;

	di->ps_desc.properties = stc3100_battery_props;
	di->ps_desc.num_properties = ARRAY_SIZE(stc3100_battery_props);
	di->ps_desc.type = POWER_SUPPLY_TYPE_BATTERY;

	di->ps_desc.name = "stc3100-battery";
}

static void stc3100_battery_update_status(struct stc3100_device_info *di)
{
	power_supply_changed(di->bat);
}

static void stc3100_battery_work(struct work_struct *work)
{
	struct stc3100_device_info *di = container_of(work, struct stc3100_device_info, work.work); 

	stc3100_battery_update_status(di);
	/* reschedule for the next time */
	schedule_delayed_work(&di->work, di->interval);
}

static int stc3100_battery_probe(struct i2c_client *client)
{
	struct stc3100_device_info *di;
	int retval = 0;
	u16 capacity;

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		dev_err(&client->dev, "failed to allocate device info data\n");
		retval = -ENOMEM;
		goto batt_failed_2;
	}

	i2c_set_clientdata(client, di);
	di->dev = &client->dev;
	di->client = client;
	retval = stc3100_write_reg(client, STC3100_REG_MODE, 1<<4);
	if(retval < 0)
	{
		dev_err(&client->dev, "failed initialize stc3100\n");
		goto batt_failed_3;
	}
	/* 4 seconds between monotor runs interval */
	di->interval = msecs_to_jiffies(4 * 1000);
	stc3100_powersupply_init(di);

	memset(&di->ps_config, 0, sizeof di->ps_config);
	di->ps_config.drv_data = di;
	di->ps_config.of_node = client->dev.of_node;

	di->bat = power_supply_register(&client->dev, &di->ps_desc, &di->ps_config);
	if (IS_ERR(di->bat)) {
		retval = PTR_ERR(di->bat);
		dev_err(&client->dev, "failed to register battery\n");
		goto batt_failed_3;
	}
	
	INIT_DELAYED_WORK(&di->work, stc3100_battery_work);
	schedule_delayed_work(&di->work, di->interval);
	
	/*
	if(device_property_read_u16(&client->dev, "capacity", &capacity))
	{
		dev_warn(&client->dev, "capacity not specified. using default: %d\n", BATTERY_CAPACITY_MAH);
		capacity = BATTERY_CAPACITY_MAH;
	}
	*/

	di->capacity = BATTERY_CAPACITY_MAH;

	dev_info(&client->dev, "support ver. %s enabled\n", DRIVER_VERSION);
	dev_info(&client->dev, "using capacity of %d\n", di->capacity);

	return 0;

batt_failed_3:
	kfree(di);
batt_failed_2:
	return retval;
}

static void stc3100_battery_remove(struct i2c_client *client)
{
	struct stc3100_device_info *di = i2c_get_clientdata(client);

	cancel_delayed_work_sync(&di->work);
	power_supply_unregister(di->bat);

	//kfree(di->bat.name);

	kfree(di);
}

/*
 * Module stuff
 */

static const struct i2c_device_id stc3100_i2c_device_id[] = {
	{ "stc3100", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, stc3100_i2c_device_id);

static const struct of_device_id stc3100_of_device_id[] = {
	{ .compatible = "st,stc3100", },
	{ }
};
MODULE_DEVICE_TABLE(of, stc3100_of_device_id);

static struct i2c_driver stc3100_battery_driver = {
	.driver = {
		.name = "stc3100",
		.of_match_table = stc3100_of_device_id,
	},
	.probe = stc3100_battery_probe,
	.remove = stc3100_battery_remove,
	.id_table = stc3100_i2c_device_id,
};

static int __init stc3100_battery_init(void)
{
	int ret;

	ret = i2c_add_driver(&stc3100_battery_driver);
	if (ret)
		printk(KERN_ERR "Unable to register stc3100 driver\n");

	return ret;
}
module_init(stc3100_battery_init);

static void __exit stc3100_battery_exit(void)
{
	i2c_del_driver(&stc3100_battery_driver);
}
module_exit(stc3100_battery_exit);

MODULE_AUTHOR("novalis");
MODULE_DESCRIPTION("stc3100 battery monitor driver");
MODULE_LICENSE("GPL");
