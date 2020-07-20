/*
 *  Driver for Goodix Touchscreens
 *
 *  Copyright (c) 2014 Red Hat Inc.
 *
 *  This code is based on gt9xx.c authored by andrew@goodix.com:
 *
 *  2010 - 2012 Goodix Technology.
 */

/*
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; version 2 of the License.
 */

#include <linux/kernel.h>
#include <linux/dmi.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/acpi.h>
#include <linux/of.h>
#include <asm/unaligned.h>

//==================================================================================
//fj_ltls add 20160408
#include <linux/gpio.h>
#include <linux/of_gpio.h>

#define PC7106   0
#define PC9715   1

#define MACHIPTYPE PC9715




//#define GOODIX_DBG(fmt, arg...) printk(KERN_INFO "goodix: " fmt "\n" , ## arg)
#define GOODIX_DBG(fmt, arg...)



//#define GTP_ERROR(fmt, arg...) printk(KERN_INFO "goodix: " fmt "\n" , ## arg)
#define GTP_ERROR(fmt, arg...)

#define GOODIX_ICS_SLOT_REPORT  0// linux 0  android 1
int wakegpio,restgpio;
//==================================================================================

struct goodix_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	int abs_x_max;
	int abs_y_max;
	unsigned int max_touch_num;
	unsigned int int_trigger_type;
	bool rotated_screen;
	u8  gtp_cfg_len;
};
#if (MACHIPTYPE==PC7106)
//800x480 pc7106
#define GOODIX_MAX_HEIGHT		800
#define GOODIX_MAX_WIDTH		480
#else
//1024x768 pc9715
#define GOODIX_MAX_HEIGHT		1024
#define GOODIX_MAX_WIDTH		768
#endif

#define GOODIX_INT_TRIGGER		1
#define GOODIX_CONTACT_SIZE		8
#define GOODIX_MAX_CONTACTS		10

#define GOODIX_CONFIG_MAX_LENGTH	240

/* Register defines */
#define GOODIX_READ_COOR_ADDR		0x814E
#define GOODIX_REG_CONFIG_DATA		0x8047
#define GOODIX_REG_ID			0x8140

//#define GTP_READ_COOR_ADDR    0x814E
//#define GTP_REG_SLEEP         0x8040
#define GTP_REG_SENSOR_ID     0x814A
#define GTP_REG_CONFIG_DATA   0x8047
//#define GTP_REG_VERSION       0x8140



#define RESOLUTION_LOC		1
#define MAX_CONTACTS_LOC	5
#define TRIGGER_LOC		6



//==================================================================================
//fj_ltls add 20160504

#define GTP_DRIVER_SEND_CFG 1
#define GTP_ADDR_LENGTH       2
#define GTP_CONFIG_MAX_LENGTH 240

#define CFG_GROUP_LEN(p_cfg_grp)  (sizeof(p_cfg_grp) / sizeof(p_cfg_grp[0]))

#if (MACHIPTYPE==PC7106)
//800x480 pc7106
#define CTP_CFG_GROUP1 { \
	0x00,0x20,0x03,0xE0,0x01,0x05,0x38,0x00,0x02,0x3F,\
	0x19,0x0E,0x50,0x3C,0x03,0x05,0x00,0x00,0x00,0x00,\
	0x00,0x00,0x00,0x18,0x1A,0x1E,0x14,0x8C,0x2B,0x0E,\
	0x32,0x37,0x98,0x15,0x00,0x00,0x00,0x00,0x03,0x35,\
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
	0x00,0x00,0x1C,0x1A,0x18,0x16,0x14,0x12,0x10,0x0E,\
	0x0C,0x0A,0x08,0x06,0x04,0x02,0xFF,0xFF,0xFF,0xFF,\
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,\
	0xFF,0xFF,0x26,0x24,0x22,0x21,0x20,0x1F,0x1E,0x1D,\
	0x1C,0x18,0x16,0x14,0x13,0x12,0x10,0x0F,0x0C,0x0A,\
	0x08,0x06,0x04,0x02,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,\
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,\
	0xFF,0xFF,0xFF,0xFF,0xCA,0x01\
}
#else

//1024x768 pc9715
#define CTP_CFG_GROUP1 { \
	0x63,0x00,0x04,0x00,0x03,0x02,0x0C,0x00,0x01,0x08,\
	0x28,0x08,0xC8,0x96,0x03,0x05,0x00,0x00,0x00,0x00,\
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x8B,0x2B,0x88,\
	0x1E,0x20,0x0C,0x08,0x00,0x00,0x00,0x03,0x02,0x2D,\
	0x00,0x00,0x00,0x00,0x00,0x03,0x00,0x00,0x00,0x00,\
	0x00,0x0F,0x2C,0x94,0x45,0x02,0x07,0x00,0x00,0x04,\
	0xE2,0x10,0x00,0xBB,0x14,0x00,0x97,0x19,0x00,0x78,\
	0x20,0x00,0x64,0x27,0x00,0x64,0x00,0x00,0x00,0x00,\
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
	0x00,0x00,0x00,0x00,0x03,0x00,0x00,0x00,0x00,0x00,\
	0x00,0x00,0x15,0x14,0x11,0x10,0x0F,0x0E,0x0D,0x0C,\
	0x09,0x08,0x07,0x06,0x05,0x04,0x01,0x00,0xFF,0xFF,\
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,0x00,0x00,0x00,\
	0x00,0x00,0x25,0x24,0x23,0x22,0x21,0x20,0x1F,0x1E,\
	0x1C,0x1B,0x19,0x12,0x11,0x10,0x0F,0x0A,0x08,0x07,\
	0x06,0x04,0x02,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,\
	0xFF,0xFF,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,\
	0x00,0x00,0x00,0x00,0xAF,0x01\
}
#if 0
//1024x768 pc9715
#define CTP_CFG_GROUP1 { \ 
        0x00,0x00,0x04,0x00,0x03,0x0A,0x08,0x00,0x02,0x08,\
	0x19,0x05,0x32,0x28,0x03,0x05,0x00,0x00,0x00,0x00,\
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x8B,0x2B,0x88,\
	0x1A,0x15,0x7C,0x15,0x00,0x00,0x01,0x01,0x03,0x2D,\
	0x00,0x00,0x00,0x00,0x00,0x03,0x64,0x32,0x00,0x00,\
	0x00,0x32,0xC8,0x94,0x05,0x02,0x07,0x00,0x00,0x31,\
	0x0E,0x1F,0xB5,0x11,0x21,0x9D,0x15,0x24,0x59,0x18,\
	0x2A,0x4D,0x1A,0x34,0x00,0x00,0x00,0x00,0x00,0x00,\
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
	0x00,0x00,0x00,0x01,0x04,0x05,0x06,0x07,0x08,0x09,\
	0x0C,0x0D,0x0E,0x0F,0x10,0x11,0x14,0x15,0xFF,0xFF,\
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,\
	0xFF,0xFF,0x00,0x02,0x04,0x06,0x07,0x08,0x0A,0x0F,\
	0x10,0x11,0x12,0x19,0x1B,0x1C,0x1E,0x1F,0x20,0x21,\
	0x22,0x23,0x24,0x25,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,\
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,\
	0xFF,0xFF,0xFF,0xFF,0x5E,0x01\
}
#endif
#endif

// TODO: define your config for Sensor_ID == 1 here, if needed
#define CTP_CFG_GROUP2 {\
    }

// TODO: define your config for Sensor_ID == 2 here, if needed
#define CTP_CFG_GROUP3 {\
    }

// TODO: define your config for Sensor_ID == 3 here, if needed
#define CTP_CFG_GROUP4 {\
    }

// TODO: define your config for Sensor_ID == 4 here, if needed
#define CTP_CFG_GROUP5 {\
    }

// TODO: define your config for Sensor_ID == 5 here, if needed
#define CTP_CFG_GROUP6 {\
    }


u8 config[GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH]
                = {GTP_REG_CONFIG_DATA >> 8, GTP_REG_CONFIG_DATA & 0xff};

//==================================================================================





static const unsigned long goodix_irq_flags[] = {
	IRQ_TYPE_EDGE_RISING,
	IRQ_TYPE_EDGE_FALLING,
	IRQ_TYPE_LEVEL_LOW,
	IRQ_TYPE_LEVEL_HIGH,
};

/*
 * Those tablets have their coordinates origin at the bottom right
 * of the tablet, as if rotated 180 degrees
 */
static const struct dmi_system_id rotated_screen[] = {
#if defined(CONFIG_DMI) && defined(CONFIG_X86)
	{
		.ident = "WinBook TW100",
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "WinBook"),
			DMI_MATCH(DMI_PRODUCT_NAME, "TW100")
		}
	},
	{
		.ident = "WinBook TW700",
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "WinBook"),
			DMI_MATCH(DMI_PRODUCT_NAME, "TW700")
		},
	},
#endif
	{}
};


//==================================================================================
//fj_ltls add 20160408


/*******************************************************
Function:
    Synchronization.
Input:
    ms: synchronization time in millionsecond.
Output:
    None.
*******************************************************/
void goodix_int_sync(s32 ms)
{
    gpio_direction_output(wakegpio, 0);
    msleep(ms);
    gpio_direction_input(wakegpio);
}

/*******************************************************
Function:
    Reset chip.
Input:
    ms: reset time in millionsecond
Output:
    None.
*******************************************************/
void goodix_reset_guitar(struct i2c_client *client, s32 ms)
{
    

    gpio_direction_output(restgpio, 0);   // begin select I2C slave addr
    msleep(ms);                         // T2: > 10ms
    // HIGH: 0x28/0x29, LOW: 0xBA/0xBB
    gpio_direction_output(wakegpio, client->addr == 0x14);

    msleep(2);                          // T3: > 100us
    gpio_direction_output(restgpio, 1);
    
    msleep(6);                          // T4: > 5ms

    gpio_direction_input(restgpio);    // end select I2C slave addr

    goodix_int_sync(50);                
    

}




/* wake up controller by an falling edge of interrupt gpio.  */
static int goodix_wake_up_device(struct i2c_client *client)
{
	struct device_node *np = client->dev.of_node;
	//int wakegpio,restgpio;
	int ret;

	if (!np)
		return -ENODEV;

	wakegpio = of_get_named_gpio(np, "wakeup-gpios", 0);
	if (!gpio_is_valid(wakegpio))
		return -ENODEV;

	ret = gpio_request(wakegpio, "goodix_irq");
	if (ret < 0) {
		dev_err(&client->dev,
			"request gpio failed, cannot wake up controller: %d\n",
			ret);
		return ret;
	}

	restgpio = of_get_named_gpio(np, "rest-gpios", 0);
	if (!gpio_is_valid(restgpio))
		return -ENODEV;

	ret = gpio_request(restgpio, "rest_gpios");
	if (ret < 0) {
		dev_err(&client->dev,
			"request gpio failed, cannot restgpio controller: %d\n",
			ret);
		return ret;
	}

        

   #if 0

	/* wake up controller via an falling edge on IRQ gpio. */
	gpio_direction_output(wakegpio, 0);
	gpio_direction_output(restgpio, 0);
	msleep(1);
	gpio_set_value(restgpio, 1);	
	gpio_set_value(wakegpio, 1);
	msleep(1);
         

	/* controller should be waken up, return irq.  */
	gpio_direction_input(wakegpio);
	gpio_free(wakegpio);

	
   #else
	goodix_reset_guitar(client, 20);
   #endif
	return 0;
}


//==================================================================================

/**
 * goodix_i2c_read - read data from a register of the i2c slave device.
 *
 * @client: i2c device.
 * @reg: the register to read from.
 * @buf: raw write data buffer.
 * @len: length of the buffer to write
 */
static int goodix_i2c_read(struct i2c_client *client,
			   u16 reg, u8 *buf, int len)
{
	struct i2c_msg msgs[2];
	u16 wbuf = cpu_to_be16(reg);
	int ret;

	msgs[0].flags = 0;
	msgs[0].addr  = client->addr;
	msgs[0].len   = 2;
	msgs[0].buf   = (u8 *)&wbuf;

	msgs[1].flags = I2C_M_RD;
	msgs[1].addr  = client->addr;
	msgs[1].len   = len;
	msgs[1].buf   = buf;

	ret = i2c_transfer(client->adapter, msgs, 2);
	return ret < 0 ? ret : (ret != ARRAY_SIZE(msgs) ? -EIO : 0);
}



//================================================================================
// fj_ltls 20150504

/*******************************************************
Function:
    Write data to the i2c slave device.
Input:
    client:     i2c device.
    buf[0~1]:   write start address.
    buf[2~len-1]:   data buffer
    len:    GTP_ADDR_LENGTH + write bytes count
Output:
    numbers of i2c_msgs to transfer: 
        1: succeed, otherwise: failed
*********************************************************/
s32 gtp_i2c_write(struct i2c_client *client,u8 *buf,s32 len)
{
    struct i2c_msg msg;
    s32 ret = -1;
    s32 retries = 0;

   // GOODIX_DBG_FUNC();

    msg.flags = !I2C_M_RD;
    msg.addr  = client->addr;
    msg.len   = len;
    msg.buf   = buf;

    while(retries < 5)
    {
        ret = i2c_transfer(client->adapter, &msg, 1);
        if (ret == 1)break;
        retries++;
    }
    if((retries >= 5))
    {

        GOODIX_DBG("Write I2C communication timeout, resetting chip...");
       // gtp_reset_guitar(client, 10);
    }
    return ret;
}


/*******************************************************
Function:
    Send config.
Input:
    client: i2c device.
Output:
    result of i2c write operation. 
        1: succeed, otherwise: failed
*********************************************************/
s32 gtp_send_cfg(struct i2c_client *client)
{
    s32 ret = 2;
    
#if GTP_DRIVER_SEND_CFG
    s32 retry = 0;

    for (retry = 0; retry < 5; retry++)
    {
        ret = gtp_i2c_write(client, config , GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH);
        if (ret > 0)
        {
            break;
        }
    }
#endif

    return ret;
}
//===================================================================================================================



static int goodix_ts_read_input_report(struct goodix_ts_data *ts, u8 *data)
{
	int touch_num;
	int error;

	error = goodix_i2c_read(ts->client, GOODIX_READ_COOR_ADDR, data,
				GOODIX_CONTACT_SIZE + 1);
	if (error) {
		dev_err(&ts->client->dev, "I2C transfer error: %d\n", error);
		return error;
	}

	if (!(data[0] & 0x80))
		return -EAGAIN;

	touch_num = data[0] & 0x0f;
	if (touch_num > ts->max_touch_num)
		return -EPROTO;

	if (touch_num > 1) {
		data += 1 + GOODIX_CONTACT_SIZE;
		error = goodix_i2c_read(ts->client,
					GOODIX_READ_COOR_ADDR +
						1 + GOODIX_CONTACT_SIZE,
					data,
					GOODIX_CONTACT_SIZE * (touch_num - 1));
		if (error)
			return error;
	}

	return touch_num;
}

static void goodix_ts_report_touch(struct goodix_ts_data *ts, u8 *coor_data)
{
	int id = coor_data[0] & 0x0F;
	int input_x = get_unaligned_le16(&coor_data[1]);
	int input_y = get_unaligned_le16(&coor_data[3]);
	int input_w = get_unaligned_le16(&coor_data[5]);

	if (ts->rotated_screen) {
		input_x = ts->abs_x_max - input_x;
		input_y = ts->abs_y_max - input_y;
	}
//==================================================================================
//fj_ltls add 20160408
	GOODIX_DBG("%s: id:%d x:%d y:%d w:", __func__, id,input_x,input_y,input_w);

#if GOODIX_ICS_SLOT_REPORT
	input_mt_slot(ts->input_dev, id);
	input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, true);
	#if (MACHIPTYPE==PC7106)
	//pc7106
	input_report_abs(ts->input_dev, ABS_MT_POSITION_X, input_x);
	input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, input_y);
	#else
	//pc9715
	input_report_abs(ts->input_dev, ABS_MT_POSITION_X, 1024-input_x);
	input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, 768-input_y);
#endif

	input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, input_w);
	input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, input_w);
#else
	#if (MACHIPTYPE==PC7106)
	//pc7106
        input_report_abs(ts->input_dev, ABS_X, input_x);
        input_report_abs(ts->input_dev, ABS_Y, input_y);
	#else
	//pc9715
        input_report_abs(ts->input_dev, ABS_X, 1024-input_x);
        input_report_abs(ts->input_dev, ABS_Y, 768-input_y);
	#endif

	//input_report_abs(ts->input_dev, ABS_X, x);
       // input_report_abs(ts->input_dev, ABS_Y, y);
        input_report_abs(ts->input_dev, ABS_PRESSURE, input_w);
	input_sync(ts->input_dev);//important
	GOODIX_DBG("%s: 2 id:%d x:%d y:%d", __func__, id,1024-input_x,768-input_y);
#endif
//====================================================================================
}

/**
 * goodix_process_events - Process incoming events
 *
 * @ts: our goodix_ts_data pointer
 *
 * Called when the IRQ is triggered. Read the current device state, and push
 * the input events to the user space.
 */
static void goodix_process_events(struct goodix_ts_data *ts)
{
	u8  point_data[1 + GOODIX_CONTACT_SIZE * GOODIX_MAX_CONTACTS];
	int touch_num;
	int i;

	touch_num = goodix_ts_read_input_report(ts, point_data);
	if (touch_num < 0)
		return;

	for (i = 0; i < touch_num; i++)
		goodix_ts_report_touch(ts,
				&point_data[1 + GOODIX_CONTACT_SIZE * i]);
//====================================================================================
      //fj_ltls add 20160412
#if !GOODIX_ICS_SLOT_REPORT
        input_report_key(ts->input_dev, BTN_TOUCH, touch_num); //important    
	GOODIX_DBG("GOODIX_ICS_SLOT_REPORT= %d !=%d \n",GOODIX_ICS_SLOT_REPORT,!GOODIX_ICS_SLOT_REPORT);
#endif  
//====================================================================================
	input_mt_sync_frame(ts->input_dev);
	input_sync(ts->input_dev);
}

/**
 * goodix_ts_irq_handler - The IRQ handler
 *
 * @irq: interrupt number.
 * @dev_id: private data pointer.
 */
static irqreturn_t goodix_ts_irq_handler(int irq, void *dev_id)
{
	static const u8 end_cmd[] = {
		GOODIX_READ_COOR_ADDR >> 8,
		GOODIX_READ_COOR_ADDR & 0xff,
		0
	};
	struct goodix_ts_data *ts = dev_id;

	goodix_process_events(ts);

	if (i2c_master_send(ts->client, end_cmd, sizeof(end_cmd)) < 0)
		dev_err(&ts->client->dev, "I2C write end_cmd error\n");

	return IRQ_HANDLED;
}

/**
 * goodix_read_config - Read the embedded configuration of the panel
 *
 * @ts: our goodix_ts_data pointer
 *
 * Must be called during probe
 */
static void goodix_read_config(struct goodix_ts_data *ts)
{
	u8 config[GOODIX_CONFIG_MAX_LENGTH];
	int error;

	error = goodix_i2c_read(ts->client, GOODIX_REG_CONFIG_DATA,
				config,
				GOODIX_CONFIG_MAX_LENGTH);
	if (error) {
		dev_warn(&ts->client->dev,
			 "Error reading config (%d), using defaults\n",
			 error);
		ts->abs_x_max = GOODIX_MAX_WIDTH;
		ts->abs_y_max = GOODIX_MAX_HEIGHT;
		ts->int_trigger_type = GOODIX_INT_TRIGGER;
		ts->max_touch_num = GOODIX_MAX_CONTACTS;
		return;
	}

	ts->abs_x_max = get_unaligned_le16(&config[RESOLUTION_LOC]);
	ts->abs_y_max = get_unaligned_le16(&config[RESOLUTION_LOC + 2]);
	ts->int_trigger_type = config[TRIGGER_LOC] & 0x03;
	ts->max_touch_num = config[MAX_CONTACTS_LOC] & 0x0f;
	if (!ts->abs_x_max || !ts->abs_y_max || !ts->max_touch_num) {
		dev_err(&ts->client->dev,
			"Invalid config, using defaults\n");
		ts->abs_x_max = GOODIX_MAX_WIDTH;
		ts->abs_y_max = GOODIX_MAX_HEIGHT;
		ts->max_touch_num = GOODIX_MAX_CONTACTS;
	}

	ts->rotated_screen = dmi_check_system(rotated_screen);
	if (ts->rotated_screen)
		dev_dbg(&ts->client->dev,
			 "Applying '180 degrees rotated screen' quirk\n");
}

/**
 * goodix_read_version - Read goodix touchscreen version
 *
 * @client: the i2c client
 * @version: output buffer containing the version on success
 * @id: output buffer containing the id on success
 */
static int goodix_read_version(struct i2c_client *client, u16 *version, u16 *id)
{
	int error;
	u8 buf[6];
	char id_str[5];

	error = goodix_i2c_read(client, GOODIX_REG_ID, buf, sizeof(buf));
	if (error) {
		dev_err(&client->dev, "read version failed: %d\n", error);
		return error;
	}

	memcpy(id_str, buf, 4);
	id_str[4] = 0;
	if (kstrtou16(id_str, 10, id))
		*id = 0x1001;

	*version = get_unaligned_le16(&buf[4]);

	dev_info(&client->dev, "ID %d, version: %04x\n", *id, *version);

	return 0;
}

/**
 * goodix_i2c_test - I2C test function to check if the device answers.
 *
 * @client: the i2c client
 */
static int goodix_i2c_test(struct i2c_client *client)
{
	int retry = 0;
	int error;
	u8 test;

	while (retry++ < 2) {
		error = goodix_i2c_read(client, GOODIX_REG_CONFIG_DATA,
					&test, 1);
		if (!error)
			return 0;

		dev_err(&client->dev, "i2c test failed attempt %d: %d\n",
			retry, error);
		msleep(20);
	}

	return error;
}

/**
 * goodix_request_input_dev - Allocate, populate and register the input device
 *
 * @ts: our goodix_ts_data pointer
 * @version: device firmware version
 * @id: device ID
 *
 * Must be called during probe
 */
static int goodix_request_input_dev(struct goodix_ts_data *ts, u16 version,
				    u16 id)
{
	int error;

	ts->input_dev = devm_input_allocate_device(&ts->client->dev);
	if (!ts->input_dev) {
		dev_err(&ts->client->dev, "Failed to allocate input device.");
		return -ENOMEM;
	}


//============================================================================================	
//fj_ltls add for 20160411 
#if GOODIX_ICS_SLOT_REPORT
	__set_bit(EV_ABS, ts->input_dev->evbit);//important
	__set_bit(EV_KEY, ts->input_dev->evbit);//important
	__set_bit(BTN_TOUCH, ts->input_dev->keybit);//important
	input_set_abs_params(ts->input_dev, ABS_X, 0, ts->abs_x_max, 0, 0);
    	input_set_abs_params(ts->input_dev, ABS_Y, 0, ts->abs_y_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, ts->abs_x_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, ts->abs_y_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_mt_init_slots(ts->input_dev, ts->max_touch_num, INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
#else 
   //fj_ltls add for 20160411  
          ts->input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) ;  //important
          ts->input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);  //important
          ts->input_dev->absbit[0] = BIT(ABS_X) | BIT(ABS_Y) | BIT(ABS_PRESSURE);   //important

	input_set_abs_params(ts->input_dev, ABS_X, 0, ts->abs_x_max, 0, 0);
    	input_set_abs_params(ts->input_dev, ABS_Y, 0, ts->abs_y_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_PRESSURE, 0, 255, 0, 0); 
#endif
//===============================================================================================
	

	ts->input_dev->name = "Goodix Capacitive TouchScreen";
	ts->input_dev->phys = "input/ts";
	ts->input_dev->id.bustype = BUS_I2C;
	ts->input_dev->id.vendor = 0x0416;
	ts->input_dev->id.product = id;
	ts->input_dev->id.version = version;

	error = input_register_device(ts->input_dev);
	if (error) {
		dev_err(&ts->client->dev,
			"Failed to register input device: %d", error);
		return error;
	}

	return 0;
}





//=====================================================================================
//fj_ltls 20150503

/*******************************************************
Function:
    Initialize gtp.
Input:
    ts: goodix private data
Output:
    Executive outcomes.
        0: succeed, otherwise: failed
*******************************************************/
static s32 gtp_init_panel(struct goodix_ts_data *ts)
{
    int ret = -1;

#if GTP_DRIVER_SEND_CFG
    s32 i;
    u8 check_sum = 0;
    u8 opr_buf[16];
    u8 sensor_id = 0;

    u8 cfg_info_group1[] = CTP_CFG_GROUP1;
    u8 cfg_info_group2[] = CTP_CFG_GROUP2;
    u8 cfg_info_group3[] = CTP_CFG_GROUP3;
    u8 cfg_info_group4[] = CTP_CFG_GROUP4;
    u8 cfg_info_group5[] = CTP_CFG_GROUP5;
    u8 cfg_info_group6[] = CTP_CFG_GROUP6;
    u8 *send_cfg_buf[] = {cfg_info_group1, cfg_info_group2, cfg_info_group3,
                        cfg_info_group4, cfg_info_group5, cfg_info_group6};
    u8 cfg_info_len[] = { CFG_GROUP_LEN(cfg_info_group1), 
                          CFG_GROUP_LEN(cfg_info_group2),
                          CFG_GROUP_LEN(cfg_info_group3),
                          CFG_GROUP_LEN(cfg_info_group4), 
                          CFG_GROUP_LEN(cfg_info_group5),
                          CFG_GROUP_LEN(cfg_info_group6)};

    GOODIX_DBG("Config Groups\' Lengths: %d, %d, %d, %d, %d, %d", 
        cfg_info_len[0], cfg_info_len[1], cfg_info_len[2], cfg_info_len[3],
        cfg_info_len[4], cfg_info_len[5]);


    if ((!cfg_info_len[1]) && (!cfg_info_len[2]) && 
        (!cfg_info_len[3]) && (!cfg_info_len[4]) && 
        (!cfg_info_len[5]))
    {
        sensor_id = 0; 
    }
    else
    {
       // opr_buf[0] = (u8)(GTP_REG_SENSOR_ID >> 8);
       // opr_buf[1] = (u8)(GTP_REG_SENSOR_ID & 0xff);
       //ret = goodix_i2c_read(ts->client, opr_buf, 3);
	ret=goodix_i2c_read(ts->client,GTP_REG_SENSOR_ID,opr_buf,sizeof(opr_buf));
        if (ret < 0)
        {
            GTP_ERROR("Failed to read Sensor_ID, using DEFAULT config!");
            sensor_id = 0;
            if (cfg_info_len[0] != 0)
            {
                send_cfg_buf[0][0] = 0x00;      // RESET Config Version
            }
        }
        else
        {
            sensor_id = opr_buf[0] & 0x07;
        }
    }
    GOODIX_DBG("Sensor_ID: %d", sensor_id);
    
    ts->gtp_cfg_len = cfg_info_len[sensor_id];
    
    if (ts->gtp_cfg_len == 0)
    {
        GTP_ERROR("Sensor_ID(%d) matches with NULL CONFIG GROUP!NO Config Send! You need to check you header file CFG_GROUP section!", sensor_id);
        return -1;
    }
    
    memset(&config[GTP_ADDR_LENGTH], 0, GTP_CONFIG_MAX_LENGTH);
    memcpy(&config[GTP_ADDR_LENGTH], send_cfg_buf[sensor_id], ts->gtp_cfg_len);
    
    check_sum = 0;
    for (i = GTP_ADDR_LENGTH; i < ts->gtp_cfg_len; i++)
    {
        check_sum += config[i];
    }
    config[ts->gtp_cfg_len] = (~check_sum) + 1;
    
#else // DRIVER NOT SEND CONFIG
    ts->gtp_cfg_len = GTP_CONFIG_MAX_LENGTH;
    ret = goodix_i2c_read(ts->client, config, ts->gtp_cfg_len + GTP_ADDR_LENGTH);
    if (ret < 0)
    {
        GTP_ERROR("Read Config Failed, Using DEFAULT Resolution & INT Trigger!");
        ts->abs_x_max = GTP_MAX_WIDTH;
        ts->abs_y_max = GTP_MAX_HEIGHT;
        ts->int_trigger_type = GTP_INT_TRIGGER;
    }
#endif // GTP_DRIVER_SEND_CFG

  //  GOODIX_DBG_FUNC();
    if ((ts->abs_x_max == 0) && (ts->abs_y_max == 0))
    {
        ts->abs_x_max = (config[RESOLUTION_LOC + 1] << 8) + config[RESOLUTION_LOC];
        ts->abs_y_max = (config[RESOLUTION_LOC + 3] << 8) + config[RESOLUTION_LOC + 2];
        ts->int_trigger_type = (config[TRIGGER_LOC]) & 0x03; 
    }
    ret = gtp_send_cfg(ts->client);
    if (ret < 0)
    {
        GTP_ERROR("Send config error.");
    }
    GOODIX_DBG("X_MAX = %d, Y_MAX = %d, TRIGGER = 0x%02x",
        ts->abs_x_max,ts->abs_y_max,ts->int_trigger_type);


    msleep(10);
    return 0;
}

//=====================================================================================
static int goodix_ts_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct goodix_ts_data *ts;
	unsigned long irq_flags;
	int error;
	u16 version_info, id_info;

	dev_dbg(&client->dev, "I2C Address: 0x%02x\n", client->addr);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "I2C check functionality failed.\n");
		return -ENXIO;
	}

	ts = devm_kzalloc(&client->dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	ts->client = client;
	i2c_set_clientdata(client, ts);

//==================================================================================
//fj_ltls add 20160408
#if 1
	/* controller may be in sleep, wake it up. */
	error = goodix_wake_up_device(client);  //important
	if (error) {
		dev_err(&client->dev, "Failed to wake up the controller\n");
		return error;
	}
#endif
//==================================================================================

	error = goodix_i2c_test(client);
	if (error) {
		dev_err(&client->dev, "I2C communication failure: %d\n", error);
		return error;
	}

	error = goodix_read_version(client, &version_info, &id_info);
	if (error) {
		dev_err(&client->dev, "Read version failed.\n");
		return error;
	}

	goodix_read_config(ts);

//==================================================================================
//fj_ltls add 20160505
      gtp_init_panel(ts); 
//==================================================================================


	error = goodix_request_input_dev(ts, version_info, id_info);
	if (error)
		return error;

	irq_flags = goodix_irq_flags[ts->int_trigger_type] | IRQF_ONESHOT;
	error = devm_request_threaded_irq(&ts->client->dev, client->irq,
					  NULL, goodix_ts_irq_handler,
					  irq_flags, client->name, ts);
	if (error) {
		dev_err(&client->dev, "request IRQ failed: %d\n", error);
		return error;
	}

	return 0;
}

static const struct i2c_device_id goodix_ts_id[] = {
	{ "GDIX1001:00", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, goodix_ts_id);

#ifdef CONFIG_ACPI
static const struct acpi_device_id goodix_acpi_match[] = {
	{ "GDIX1001", 0 },
	{ }
};
MODULE_DEVICE_TABLE(acpi, goodix_acpi_match);
#endif

#ifdef CONFIG_OF
static const struct of_device_id goodix_of_match[] = {
	{ .compatible = "goodix,gt911" },
	{ .compatible = "goodix,gt9110" },
	{ .compatible = "goodix,gt912" },
	{ .compatible = "goodix,gt927" },
	{ .compatible = "goodix,gt9271" },
	{ .compatible = "goodix,gt928" },
	{ .compatible = "goodix,gt967" },
	{ }
};
MODULE_DEVICE_TABLE(of, goodix_of_match);
#endif

static struct i2c_driver goodix_ts_driver = {
	.probe = goodix_ts_probe,
	.id_table = goodix_ts_id,
	.driver = {
		.name = "Goodix-TS",
		.acpi_match_table = ACPI_PTR(goodix_acpi_match),
		.of_match_table = of_match_ptr(goodix_of_match),
	},
};
module_i2c_driver(goodix_ts_driver);

MODULE_AUTHOR("Benjamin Tissoires <benjamin.tissoires@gmail.com>");
MODULE_AUTHOR("Bastien Nocera <hadess@hadess.net>");
MODULE_DESCRIPTION("Goodix touchscreen driver");
MODULE_LICENSE("GPL v2");

