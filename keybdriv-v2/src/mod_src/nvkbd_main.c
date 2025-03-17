// SPDX-License-Identifier: GPL-2.0-only

#include "nvkbd_main.h"

static void nvkbd_read_fifo(struct nvkbd_data *nvkbd_data)
{
	struct i2c_client *i2c_client = nvkbd_data->i2c_client;
	uint8_t fifo_data[2];
	uint8_t count;
	uint8_t pos;
	int returnValue;

	returnValue = nvkbd_read(i2c_client, REG_KEY, &count, sizeof(uint8_t));
	if (returnValue != 0) {
		dev_err(&i2c_client->dev, "%s Could not read REG_KEY, Error: %d\n", __func__, returnValue);
		return;
	}
	count = count & REG_KEY_KEYCOUNT_MASK;
	nvkbd_data->fifoCount = count;
	pos = 0;
	while (count) {
		returnValue = nvkbd_read(i2c_client, REG_FIF, fifo_data, 2*sizeof(uint8_t));
		if (returnValue != 0) {
			dev_err(&i2c_client->dev, "%s Could not read REG_FIF, Error: %d\n", __func__, returnValue);
			return;
		}
		nvkbd_data->fifoData[pos][0] = fifo_data[0] & 3;
		nvkbd_data->fifoData[pos][1] = fifo_data[1];
#if (DEBUG_LEVEL & DEBUG_LEVEL_LD)
		dev_info(&i2c_client->dev, "%s Filled Data: KeyState:%d SCANCODE:%d at Pos: %d Count: %d\n",
			__func__, nvkbd_data->fifoData[pos][0], nvkbd_data->fifoData[pos][1], pos, count);
#endif
		++pos;
		--count;
	}
}

static unsigned int nvkbd_scancode_with_offset(struct nvkbd_data *data, unsigned int scancode, unsigned int pressed)
{
	ktime_t currentTime = ktime_get();
	uint8_t offset;
	u64 cur, pre;
	if(scancode == MODIFIER_SCANCODE)
	{
		if(pressed && data->layer_lock)
		{
			data->layer_lock = false;
		}
		else if(pressed)
		{
			cur = ktime_to_ms(currentTime);
			pre = ktime_to_ms(data->last_release_time);
			if(data->last_released_scancode == MODIFIER_SCANCODE && (cur - pre) < 1000)
			{
				data->layer_lock = true;
			}
		}
		data->layer_switch = pressed;

		if(!pressed)
		{
			data->last_released_scancode = scancode;
			data->last_release_time = currentTime;
		}

		return 0;
	}

	if(scancode >= CONTROL_START_SCANCODE && scancode <= CONTROL_END_SCANCODE)
	{
		return scancode;
	}

	if(!pressed)
	{
		data->last_released_scancode = scancode;
	}

	offset = data->layer_lock ? 1 : data->layer_switch;

	return scancode + LAYER_OFFSET(offset);
}

static void nvkbd_work_fnc(struct work_struct *work_struct_ptr)
{
	struct nvkbd_data *nvkbd_data;
	struct input_dev *input_dev;
	struct i2c_client *i2c_client;
	unsigned int keycode, scancode, pressed;

	uint8_t pos = 0;
	uint8_t registerValue = 0x00;
	int returnValue = 0;

	nvkbd_data = container_of(work_struct_ptr, struct nvkbd_data, work_struct);
	input_dev = nvkbd_data->input_dev;
	i2c_client = nvkbd_data->i2c_client;

	while (nvkbd_data->fifoCount > 0) {
		pressed = nvkbd_data->fifoData[pos][0] == KEY_PRESSED_STATE;
		scancode = nvkbd_data->fifoData[pos][1];
		input_event(input_dev, EV_MSC, MSC_SCAN, scancode);

		scancode = nvkbd_scancode_with_offset(nvkbd_data, scancode, pressed);
		if(scancode != 0)
		{
			keycode = nvkbd_data->keycode[scancode];
#if (DEBUG_LEVEL & DEBUG_LEVEL_LD)
			dev_info(&i2c_client->dev, "%s scancode: %d(%c) keycode: %d State: %d\n", __func__, nvkbd_data->fifoData[pos][1], nvkbd_data->fifoData[pos][1], keycode, nvkbd_data->fifoData[pos][0]);
#endif
			input_report_key(input_dev, keycode, pressed);
		}

		++pos;
		--nvkbd_data->fifoCount;
	}

	input_sync(input_dev);
	registerValue = 0x00;
	returnValue = nvkbd_write(i2c_client, REG_INT, &registerValue, sizeof(uint8_t));
	if (returnValue < 0)
		dev_err(&i2c_client->dev, "%s Could not clear REG_INT. Error: %d\n", __func__, returnValue);
}

static irqreturn_t nvkbd_irq_handler(int irq, void *dev_id)
{
	struct nvkbd_data *nvkbd_data = dev_id;
	struct i2c_client *client = nvkbd_data->i2c_client;
	int returnValue;
	uint8_t registerValue;

#if (DEBUG_LEVEL & DEBUG_LEVEL_LD)
	dev_info(&client->dev, "%s Interrupt Fired. IRQ: %d\n", __func__, irq);
#endif

	returnValue = nvkbd_read(client, REG_INT, &registerValue, sizeof(uint8_t));
	if (returnValue < 0) {
		dev_err(&client->dev, "%s: Could not read REG_INT. Error: %d\n", __func__, returnValue);
		return IRQ_NONE;
	}

#if (DEBUG_LEVEL & DEBUG_LEVEL_LD)
	dev_info(&client->dev, "%s Interrupt: 0x%02x\n", __func__, registerValue);
#endif

	if (registerValue == 0x00)
		return IRQ_NONE;

	if (registerValue & REG_INT_OVERFLOW)
		dev_warn(&client->dev, "%s overflow occurred.\n", __func__);

	if (registerValue & REG_INT_KEY) {
		nvkbd_read_fifo(nvkbd_data);
		schedule_work(&nvkbd_data->work_struct);
	}

	return IRQ_HANDLED;
}

static int nvkbd_update_backlight(struct backlight_device *dev)
{
	struct nvkbd_data *nvkbd = bl_get_data(dev);
	struct i2c_client *client = nvkbd->i2c_client;
	uint8_t brightness;
	int returnValue;

	brightness = backlight_get_brightness(dev);

#if (DEBUG_LEVEL & DEBUG_LEVEL_LD)
	dev_info(&client->dev, "%s Backlight set brightness: 0x%02x\n", __func__, brightness);
#endif

	returnValue = nvkbd_write(client, REG_BKL, &brightness, sizeof(uint8_t));
	if (returnValue != 0) {
		dev_err(&client->dev, "%s Could not write to NVKBD Screen Backlight. Error: %d\n", __func__, returnValue);
		return returnValue;
	}

	return 0;
}

static int nvkbd_backlight_get_brightness(struct backlight_device *dev)
{
	struct nvkbd_data *nvkbd = bl_get_data(dev);
	struct i2c_client *client = nvkbd->i2c_client;
	uint8_t registerValue;
	int returnValue;

	returnValue = nvkbd_read(client, REG_BKL, &registerValue, sizeof(uint8_t));
	if (returnValue != 0) {
		dev_err(&client->dev, "%s Could not read from NVKBD Screen Backlight. Error: %d\n", __func__, returnValue);
		return returnValue;
	}

#if (DEBUG_LEVEL & DEBUG_LEVEL_LD)
	dev_info(&client->dev, "%s Backlight get brightness: 0x%02x\n", __func__, registerValue);
#endif

	return registerValue;
}

static int nvstatus_open(struct inode *inode, struct file *file)
{
	struct nvkbd_data *nvkbd_data = container_of(inode->i_cdev, struct nvkbd_data, status_dev);
	file->private_data = nvkbd_data;
	try_module_get(THIS_MODULE);
	return 0;
}

static int nvstatus_close(struct inode *inode, struct file *file)
{
	module_put(THIS_MODULE);
	return 0;
}

static ssize_t nvstatus_read(struct file *file, char *buffer, size_t length, loff_t *offset)
{
	struct nvkbd_data *data = file->private_data;
	const char *res = (data->layer_lock ? "1" : "0");

	if(length < 2)
		return -EINVAL;

	if(*offset >= 2)
		return 0;

	if(copy_to_user(buffer, res, 2))
		return -EFAULT;

	*offset += 2;

	return 2;
}

static ssize_t nvstatus_write(struct file *file, const char *buffer, size_t length, loff_t *offset)
{
	return -EINVAL;
}

static struct file_operations nvstatus_fops = 
{
	.open = nvstatus_open,
	.release = nvstatus_close,
	.read = nvstatus_read,
	.write = nvstatus_write
};

static char* status_devnode(const struct device *dev, umode_t *mode)
{
	if(!mode)
		return NULL;
	*mode = 0666;
	return NULL;
}

static int nvkbd_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct nvkbd_data *nvkbd_data;
	struct input_dev *input;
	struct backlight_device *backlight;
	struct class *class;
	struct device *device;
	int returnValue;
	int i;
	uint8_t registerValue = 0x00;
	dev_t statusDev;

#if (DEBUG_LEVEL & DEBUG_LEVEL_FE)
	dev_info(&client->dev, "%s Probing NVKBD.\n", __func__);
#endif
	nvkbd_data = devm_kzalloc(dev, sizeof(*nvkbd_data), GFP_KERNEL);
	if (!nvkbd_data)
		return -ENOMEM;
	nvkbd_data->i2c_client = client;
	i2c_set_clientdata(client, nvkbd_data);
	memcpy(nvkbd_data->keycode, keycodes, sizeof(nvkbd_data->keycode));
	
	nvkbd_data->layer_switch = 0;

	returnValue = nvkbd_write(client, REG_RST, &registerValue, sizeof(uint8_t));
	if (returnValue) 
	{
		dev_err(&client->dev, "%s Could not Reset NVKBD. Error: %d\n", __func__, returnValue);
		return -ENODEV;
	}
	msleep(400);

	returnValue = nvkbd_read(client, REG_VER, &registerValue, sizeof(uint8_t));
	if (returnValue != 0) 
	{
		dev_err(&client->dev, "%s Could not Read Version NVKBD. Error: %d\n", __func__, returnValue);
		return -ENODEV;
	}
	dev_info(&client->dev, "%s NVKBD Software version: 0x%02X\n", __func__, registerValue);
	nvkbd_data->version_number = registerValue;

	registerValue = REG_CFG_DEFAULT_SETTING;
	returnValue = nvkbd_write(client, REG_CFG, &registerValue, sizeof(uint8_t));
	if (returnValue != 0) 
	{
		dev_err(&client->dev, "%s Could not write configuration to NVKBD. Error: %d\n", __func__, returnValue);
		return -ENODEV;
	}
#if (DEBUG_LEVEL & DEBUG_LEVEL_LD)
	returnValue = nvkbd_read(client, REG_CFG, &registerValue, sizeof(uint8_t));
	if (returnValue != 0) 
	{
		dev_err(&client->dev, "%s Could not read REG_CFG. Error: %d\n", __func__, returnValue);
		return returnValue;
	}
	dev_info(&client->dev, "%s Configuration Register Value: 0x%02X\n", __func__, registerValue);
#endif

	input = devm_input_allocate_device(dev);
	if (!input) 
	{
		dev_err(&client->dev, "%s Could not devm_input_allocate_device NVKBD. Error: %d\n", __func__, returnValue);
		return -ENOMEM;
	}
	nvkbd_data->input_dev = input;

	input->name = client->name;
	input->id.bustype = NVKBD_BUS_TYPE;
	input->id.vendor  = NVKBD_VENDOR_ID;
	input->id.product = NVKBD_PRODUCT_ID;
	input->id.version = NVKBD_VERSION_ID;
	input->keycode = nvkbd_data->keycode;
	input->keycodesize = sizeof(nvkbd_data->keycode[0]);
	input->keycodemax = ARRAY_SIZE(nvkbd_data->keycode);

	for (i = 0; i < NUM_KEYCODES; i++)
		__set_bit(nvkbd_data->keycode[i], input->keybit);

	__clear_bit(KEY_RESERVED, input->keybit);

	__set_bit(EV_REP, input->evbit);
	__set_bit(EV_KEY, input->evbit);

	input_set_capability(input, EV_MSC, MSC_SCAN);
		
	returnValue = devm_request_threaded_irq(dev, client->irq,
										NULL, nvkbd_irq_handler,
										IRQF_SHARED | IRQF_ONESHOT,
										client->name, nvkbd_data);

	if (returnValue != 0) 
	{
		dev_err(&client->dev, "Could not claim IRQ %d; error %d\n", client->irq, returnValue);
		return returnValue;
	}
	INIT_WORK(&nvkbd_data->work_struct, nvkbd_work_fnc);

	returnValue = input_register_device(input);
	if (returnValue != 0) 
	{
		dev_err(dev, "Failed to register input device, error: %d\n", returnValue);
		return returnValue;
	}

	memset(&nvkbd_data->backlight_props, 0, sizeof nvkbd_data->backlight_props);
	nvkbd_data->backlight_props.max_brightness = 255;
	nvkbd_data->backlight_props.brightness = 255;
	nvkbd_data->backlight_props.type = BACKLIGHT_FIRMWARE;
	nvkbd_data->backlight_props.scale = BACKLIGHT_SCALE_LINEAR;

	memset(&nvkbd_data->backlight_ops, 0, sizeof nvkbd_data->backlight_ops);
	nvkbd_data->backlight_ops.update_status = nvkbd_update_backlight;
	nvkbd_data->backlight_ops.get_brightness = nvkbd_backlight_get_brightness;

	backlight = devm_backlight_device_register(dev, BACKLIGHT_NAME, dev, nvkbd_data, &nvkbd_data->backlight_ops, &nvkbd_data->backlight_props);
	if(IS_ERR(backlight))
	{
		dev_err(dev, "Failed to register backlight device, error: %d\n", (int)PTR_ERR(backlight));
		return (int)PTR_ERR(backlight);
	}

	nvkbd_data->backlight_dev = backlight;

	returnValue = alloc_chrdev_region(&statusDev, 0, 1, NV_CHAR_NAME);
	if(returnValue != 0)
	{
		dev_err(dev, "Failed to allocate character device, error: %d\n", returnValue);
		return returnValue;
	}

	nvkbd_data->status_dev.owner = THIS_MODULE;
	cdev_init(&nvkbd_data->status_dev, &nvstatus_fops);
	returnValue = cdev_add(&nvkbd_data->status_dev, statusDev, 1);
	if(returnValue != 0)
	{
		unregister_chrdev_region(MKDEV(statusDev, 0), 1);
		dev_err(dev, "Failed to add character device, error: %d\n", returnValue);
		return returnValue;
	}
	nvkbd_data->status_major = MAJOR(statusDev);

	class = class_create(NV_CLASS_NAME);
	if(IS_ERR(class))
	{
		cdev_del(&nvkbd_data->status_dev);
		unregister_chrdev_region(MKDEV(statusDev, 0), 1);
		dev_err(dev, "Failed to create class, error: %d\n", (int)PTR_ERR(class));
		return (int)PTR_ERR(class);
	}
	class->devnode = status_devnode;

	device = device_create(class, dev, statusDev, NULL, "nvstatus");
	if(IS_ERR(device))
	{
		class_unregister(class);
		class_destroy(class);
		cdev_del(&nvkbd_data->status_dev);
		unregister_chrdev_region(MKDEV(statusDev, 0), 1);
		dev_err(dev, "Failed to create device, error: %d\n", (int)PTR_ERR(device));
		return (int)PTR_ERR(device);
	}

	nvkbd_data->status_class = class;
	nvkbd_data->status_device = device;

	dev_info(&client->dev, "%s NVKBD Initialized char device. Major:%d\n", __func__, nvkbd_data->status_major);

	return 0;
}

static void nvkbd_remove(struct i2c_client *client)
{
	struct nvkbd_data *nvkbd_data = i2c_get_clientdata(client);
	device_destroy(nvkbd_data->status_class, MKDEV(nvkbd_data->status_major, 0));
	class_unregister(nvkbd_data->status_class);
	class_destroy(nvkbd_data->status_class);
	cdev_del(&nvkbd_data->status_dev);
	unregister_chrdev_region(MKDEV(nvkbd_data->status_major, 0), 1);
}

static void nvkbd_shutdown(struct i2c_client *client)
{
	int returnValue;
	uint8_t registerValue = 0x00;
#if (DEBUG_LEVEL & DEBUG_LEVEL_FE)
	dev_info(&client->dev, "%s Shutting Down Keyboard And Screen Backlight.\n", __func__);
#endif
	returnValue = nvkbd_write(client, REG_BKL, &registerValue, sizeof(uint8_t));
	if (returnValue != 0) 
	{
		dev_err(&client->dev, "%s Could not write to NVKBD Screen Backlight. Error: %d\n", __func__, returnValue);
		return;
	}

	returnValue = nvkbd_read(client, REG_VER, &registerValue, sizeof(uint8_t));
	if (returnValue != 0) 
	{
		dev_err(&client->dev, "%s Could not read NVKBD Software Version. Error: %d\n", __func__, returnValue);
		return;
	}
}

static struct i2c_driver nvkbd_driver = 
{
	.driver = 
	{
		.name = DEVICE_NAME,
		.of_match_table = nvkbd_of_device_id,
	},
	.probe		= nvkbd_probe,
	.shutdown	= nvkbd_shutdown,
	.remove		= nvkbd_remove,
	.id_table	= nvkbd_i2c_device_id,
};

static int __init nvkbd_init(void)
{
	int returnValue;

	returnValue = i2c_add_driver(&nvkbd_driver);
	if (returnValue != 0) {
		pr_err("%s Could not initialise NVKBD driver! Error: %d\n", __func__, returnValue);
		return returnValue;
	}
	pr_info("%s Initalised NVKBD.\n", __func__);
	return returnValue;
}
module_init(nvkbd_init);

static void __exit nvkbd_exit(void)
{
	pr_info("%s Exiting NVKBD.\n", __func__);
	i2c_del_driver(&nvkbd_driver);
}
module_exit(nvkbd_exit);
