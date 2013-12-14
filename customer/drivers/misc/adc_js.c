/*
 * drivers/amlogic/input/adc_keypad/adc_keypad.c
 *
 * ADC Keypad Driver
 *
 * Copyright (C) 2010 Amlogic, Inc.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 * author :   Robin Zhu
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/errno.h>
#include <asm/irq.h>
#include <asm/io.h>

#include <mach/am_regs.h>
#include <mach/pinmux.h>
#include <linux/major.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <linux/saradc.h>
#include <linux/adc_keypad.h>
#include <mach/gpio.h>
#include <mach/gpio_data.h>

#define ADC_KEY 250

struct joy {
    struct input_dev *input;
    //LX, LY, RX, RY, a, b, x, y, l, r, l2, r2, start, select, volup, voldn
    unsigned int buttons[16];
    int jstickkeys[8];

    //int jstickaxis[4];
    int chan[SARADC_CHAN_NUM];
    struct timer_list timer;
    int chan_num;
    struct work_struct work_update;
    int config_major;
    char config_name[20];
    struct class *config_class;
    struct device *config_dev;
};

int analog[2];
static int analogvalues[4];

static void joystick_key(struct joy *joy) {
    int value, i;

    for (i = 0; i < joy->chan_num; i++) {
        value = get_adc_sample(joy->chan[i]);
        if (value < 0) {
            ;
        } else {
	/*
        	if(i<2){
            if (value <= 0 + ADC_KEY) {
                if (joy->buttons[i]) {
                    input_report_key(joy->input, joy->jstickkeys[i * 2], 1);
                }
                joy->buttons[i]++;
            } else if (value >= 1023 - ADC_KEY) {
                if (joy->buttons[i]) {
                    input_report_key(joy->input, joy->jstickkeys[(i * 2) + 1],
                            1);
                }
                joy->buttons[i]++;
            } else {

                if (joy->buttons[i] > 0) {
                    joy->buttons[i] = 0;
                    input_report_key(joy->input, joy->jstickkeys[i * 2], 0);
                    input_report_key(joy->input, joy->jstickkeys[(i * 2) + 1],
                            0);
                }
            }
        	}
	*/
            analogvalues[i]=value;
            
             if (i == 0)
             input_report_abs(joy->input, ABS_RY, value);
             else if (i == 1)
             input_report_abs(joy->input, ABS_RX, 1023 - value); 
             if (i == 2)
             input_report_abs(joy->input, ABS_X, 1023 - value);
             else if (i == 3)
             input_report_abs(joy->input, ABS_Y, 1023 - value);

        }
    }

    value = get_adc_sample(joy->chan[4]);
    if (value < 0) {
        ;
    } else {
        if (value >= 0 && value <= (9 + 40)) {
            if (joy->buttons[13]) {
                input_report_key(joy->input, BTN_SELECT, 1);
            }
            joy->buttons[13]++;
        } else if (value >= (392 - 40) && value <= (392 + 40)) {
            if (joy->buttons[12]) {
                input_report_key(joy->input, KEY_ENTER, 1);
            }
            joy->buttons[12]++;
        } else if (value >= (150 - 40) && value <= (150 + 40)) {
            if (joy->buttons[15]) {
                input_report_key(joy->input, KEY_VOLUMEDOWN, 1);
            }
            joy->buttons[15]++;
        } else if (value >= (275 - 40) && value <= (275 + 40)) {
            if (joy->buttons[14]) {
                input_report_key(joy->input, KEY_VOLUMEUP, 1);
            }
            joy->buttons[14]++;
        } else {
            if (joy->buttons[12] > 0) {
                input_report_key(joy->input, KEY_ENTER, 0);
                joy->buttons[12] = 0;
            }
            if (joy->buttons[13] > 0) {
                input_report_key(joy->input, BTN_SELECT, 0);
                joy->buttons[13] = 0;
            }
            if (joy->buttons[14] > 0) {
                input_report_key(joy->input, KEY_VOLUMEUP, 0);
                joy->buttons[14] = 0;
            }
            if (joy->buttons[15] > 0) {
                input_report_key(joy->input, KEY_VOLUMEDOWN, 0);
                joy->buttons[15] = 0;
            }
        }

    }

    return 0;
}

static void gpio_keys_init(void) {
    WRITE_CBUS_REG_BITS(PAD_PULL_UP_REG0, 1, 3, 1);
    WRITE_CBUS_REG_BITS(PAD_PULL_UP_REG0, 1, 4, 1);
    WRITE_CBUS_REG_BITS(PAD_PULL_UP_REG0, 1, 5, 1);
    WRITE_CBUS_REG_BITS(PAD_PULL_UP_REG0, 1, 6, 1);
    WRITE_CBUS_REG_BITS(PAD_PULL_UP_REG0, 1, 7, 1);
    WRITE_CBUS_REG_BITS(PAD_PULL_UP_REG0, 1, 8, 1);
    WRITE_CBUS_REG_BITS(PAD_PULL_UP_REG0, 1, 9, 1);
    WRITE_CBUS_REG_BITS(PAD_PULL_UP_REG0, 1, 10, 1);

    WRITE_CBUS_REG_BITS(PERIPHS_PIN_MUX_0, 0, 6, 1);
    WRITE_CBUS_REG_BITS(PERIPHS_PIN_MUX_3, 0, 0, 3);
    WRITE_CBUS_REG_BITS(PERIPHS_PIN_MUX_3, 0, 5, 1);
    WRITE_CBUS_REG_BITS(PERIPHS_PIN_MUX_6, 0, 19, 5);

    set_gpio_mode(GPIOA_bank_bit0_27(3), GPIOA_bit_bit0_27(3), GPIO_INPUT_MODE);
    set_gpio_mode(GPIOA_bank_bit0_27(4), GPIOA_bit_bit0_27(4), GPIO_INPUT_MODE);
    set_gpio_mode(GPIOA_bank_bit0_27(5), GPIOA_bit_bit0_27(5), GPIO_INPUT_MODE);
    set_gpio_mode(GPIOA_bank_bit0_27(6), GPIOA_bit_bit0_27(6), GPIO_INPUT_MODE);

    set_gpio_mode(GPIOA_bank_bit0_27(7), GPIOA_bit_bit0_27(7), GPIO_INPUT_MODE);
    set_gpio_mode(GPIOA_bank_bit0_27(8), GPIOA_bit_bit0_27(8), GPIO_INPUT_MODE);
    set_gpio_mode(GPIOA_bank_bit0_27(9), GPIOA_bit_bit0_27(9), GPIO_INPUT_MODE);
    set_gpio_mode(GPIOA_bank_bit0_27(10), GPIOA_bit_bit0_27(10),
            GPIO_INPUT_MODE);
}

static struct joy *gp_joy = NULL;

static int keya, keyb, keyx, keyy, keyl, keyr, keyl2, keyr2;
//static int keya_old, keyb_old, keyx_old, keyy_old, keyl_old, keyr_old,
//        keyl2_old, keyr2_old;
static void scan_keys(struct joy *joy) {
    struct input_dev *input = joy->input;

    keyl2 = get_gpio_val(GPIOA_bank_bit0_27(3), GPIOA_bit_bit0_27(3));
    keyl = get_gpio_val(GPIOA_bank_bit0_27(4), GPIOA_bit_bit0_27(4));
    keyr2 = get_gpio_val(GPIOA_bank_bit0_27(5), GPIOA_bit_bit0_27(5));
    keyr = get_gpio_val(GPIOA_bank_bit0_27(6), GPIOA_bit_bit0_27(6));
    keyx = get_gpio_val(GPIOA_bank_bit0_27(7), GPIOA_bit_bit0_27(7));
    keya = get_gpio_val(GPIOA_bank_bit0_27(8), GPIOA_bit_bit0_27(8));
    keyb = get_gpio_val(GPIOA_bank_bit0_27(9), GPIOA_bit_bit0_27(9));
    keyy = get_gpio_val(GPIOA_bank_bit0_27(10), GPIOA_bit_bit0_27(10));

    //if key not pressed
    if (!keya) {
        if (joy->buttons[4] > 0) {
            input_report_key(input, BTN_A, 0);
            joy->buttons[4] = 0;
            printk("KEY A RELEASED %d \n", keya);
        }
    } else {
        if (joy->buttons[4] == 0) {
            input_report_key(input, BTN_A, 1);
            printk("KEY A PRESSED %d \n", keya);
        }
        joy->buttons[4]++;
    }
    if (!keyb) {
        if (joy->buttons[5] > 0) {
            input_report_key(input, BTN_B, 0);
            joy->buttons[5] = 0;
            printk("KEY B RELEASED \n");
        }
    } else {
        if (joy->buttons[5] == 0) {
            input_report_key(input, BTN_B, 1);
            printk("KEY B PRESSED \n");
        }
        joy->buttons[5]++;
    }
    if (!keyx) {
        if (joy->buttons[6] > 0) {
            input_report_key(input, BTN_X, 0);
            joy->buttons[6] = 0;
            printk("KEY X RELEASED \n");
        }
    } else {
        if (joy->buttons[6] == 0) {
            input_report_key(input, BTN_X, 1);
            printk("KEY X PRESSED \n");
        }
        joy->buttons[6]++;
    }
    if (!keyy) {
        if (joy->buttons[7] > 0) {
            input_report_key(input, BTN_Y, 0);
            joy->buttons[7] = 0;
            printk("KEY Y RELEASED \n");
        }
    } else {
        if (joy->buttons[7] == 0) {
            input_report_key(input, BTN_Y, 1);
            printk("KEY Y PRESSED \n");
        }
        joy->buttons[7]++;
    }
    if (!keyl) {
        if (joy->buttons[8] > 0) {
            input_report_key(input, BTN_TL, 0);
            joy->buttons[8] = 0;
            printk("KEY L RELEASED \n");
        }
    } else {
        if (joy->buttons[8] == 0) {
            input_report_key(input, BTN_TL, 1);
            printk("KEY L PRESSED \n");
        }
        joy->buttons[8]++;
    }
    if (!keyr) {
        if (joy->buttons[9] > 0) {
            input_report_key(input, BTN_TR, 0);
            joy->buttons[9] = 0;
            printk("KEY R RELEASED \n");
        }
    } else {
        if (joy->buttons[9] == 0) {
            input_report_key(input, BTN_TR, 1);
            printk("KEY R PRESSED \n");
        }
        joy->buttons[9]++;
    }
    if (!keyl2) {
        if (joy->buttons[10] > 0) {
            input_report_key(input, BTN_TL2, 0);
            joy->buttons[10] = 0;
            printk("KEY L2 RELEASED \n");
        }
    } else {
        if (joy->buttons[10] == 0) {
            input_report_key(input, BTN_TL2, 1);
            printk("KEY L2 PRESSED \n");
        }
        joy->buttons[10]++;
    }
    if (!keyr2) {
        if (joy->buttons[11] > 0) {
            input_report_key(input, BTN_TR2, 0);
            joy->buttons[11] = 0;
            printk("KEY R2 RELEASED \n");
        }
    } else {
        if (joy->buttons[11] == 0) {
            input_report_key(input, BTN_TR2, 1);
            printk("KEY R2 PRESSED \n");
        }
        joy->buttons[11]++;
    }

}

static ssize_t analog_read(struct device *dev, struct device_attribute *attr,
        char *buf) {
    char i;
    for (i = 0; i < 2; i++) {
        printk("analog[%d] = %d \n", i, analog[i]);
    }
    return 0;
}

static ssize_t analog_write(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count) {
    sscanf(buf, "%ld %ld", &analog[0], &analog[1]);
    return count;
}

static ssize_t values_read(struct device *dev, struct device_attribute *attr, const char *buf){
    return sprintf(buf,"%d %d %d %d", analogvalues[0], analogvalues[1], analogvalues[2], analogvalues[3]);
}

static DEVICE_ATTR( analog, S_IRWXUGO, analog_read, analog_write);

static DEVICE_ATTR( values, S_IRUGO, values_read, NULL);

static struct attribute *joy_attr[] = {
        &dev_attr_analog.attr,
        &dev_attr_values.attr,
        NULL
};
static struct attribute_group joy_attr_group =
        { .name = NULL, .attrs = joy_attr, };

static void joy_work(struct joy *joy) {
    scan_keys(joy);
    joystick_key(joy);
    input_sync(joy->input);
}

static void update_work_func(struct work_struct *work) {
    struct joy
    *joy_data = container_of(work, struct joy, work_update);

    joy_work(joy_data);
}

static void joy_timer_sr(unsigned long data) {
    struct joy *joy_data = (struct joy *) data;
    schedule_work(&(joy_data->work_update));
    mod_timer(&joy_data->timer, jiffies + msecs_to_jiffies(5));
}

static int adckpd_config_open(struct inode *inode, struct file *file) {
    file->private_data = gp_joy;
    return 0;
}

static int adckpd_config_release(struct inode *inode, struct file *file) {
    file->private_data = NULL;
    return 0;
}

static const struct file_operations keypad_fops = { .owner = THIS_MODULE,
        .open = adckpd_config_open, .release = adckpd_config_release, };

static int register_keypad_dev(struct joy *joy) {
    int ret = 0;
    strcpy(joy->config_name, "am_adc_js");
    ret = register_chrdev(0, joy->config_name, &keypad_fops);
    if (ret <= 0) {
        printk("register char device error\r\n");
        return ret;
    }
    joy->config_major = ret;
    printk("adc keypad major:%d\r\n", ret);
    joy->config_class = class_create(THIS_MODULE, joy->config_name);
    joy->config_dev = device_create(joy->config_class, NULL,
            MKDEV(joy->config_major, 0), NULL, joy->config_name);

    return ret;
}

static int __devinit joy_probe(struct platform_device *pdev)
{
    struct joy *joy;
    struct input_dev *input_dev;
    int i, ret;
    struct adc_joy_platform_data *pdata = pdev->dev.platform_data;
    s8 phys[32];

    if (!pdata) {
        dev_err(&pdev->dev, "platform data is required!\n");
        return -EINVAL;
    }

    joy = kzalloc(sizeof(struct joy), GFP_KERNEL);
    input_dev = input_allocate_device();
    if (!joy || !input_dev) {
        kfree(joy);
        input_free_device(input_dev);

        return -ENOMEM;
    }
    gp_joy=joy;

    platform_set_drvdata(pdev, joy);
    joy->input = input_dev;
    /*for (i=0; i<SARADC_CHAN_NUM; i++) {
        joy->cur_keycode[i] = 0;
        joy->cur_keycode_status[i] = 0;
        joy->tmp_code[i] = 0;
        joy->count[i] = 0;
    }*/

    for(i=0; i<10;i++){
        joy->buttons[i]=0;
    }/*
    for(i=0;i<joy->chan_num;i++){
        joy->analogvalues[i]=0;
    }*/

    INIT_WORK(&(joy->work_update), update_work_func);

    setup_timer(&joy->timer, joy_timer_sr, joy) ;
    mod_timer(&joy->timer, jiffies+msecs_to_jiffies(100));

    /* setup input device */
    set_bit(BTN_SELECT, input_dev->keybit);
    set_bit(KEY_ENTER, input_dev->keybit);
    set_bit(
        KEY_VOLUMEDOWN, input_dev->keybit);
    set_bit(KEY_VOLUMEUP,
        input_dev->keybit);
    set_bit(KEY_UP, input_dev->keybit);
    set_bit(KEY_DOWN,
        input_dev->keybit);
    set_bit(KEY_LEFT, input_dev->keybit);
    set_bit(KEY_RIGHT,
        input_dev->keybit);
    set_bit(KEY_I, input_dev->keybit);
    set_bit(KEY_J,
        input_dev->keybit);
    set_bit(KEY_K, input_dev->keybit);
    set_bit(KEY_L,
        input_dev->keybit);
    set_bit(BTN_A, input_dev->keybit);
    set_bit(BTN_B,
        input_dev->keybit);
    set_bit(BTN_X, input_dev->keybit);
    set_bit(BTN_Y,
        input_dev->keybit);
    set_bit(BTN_TL, input_dev->keybit);
    set_bit(BTN_TR,
        input_dev->keybit);
    set_bit(BTN_TL2, input_dev->keybit);
    set_bit(BTN_TR2,
        input_dev->keybit);

    set_bit(EV_REP, input_dev->evbit);
    set_bit(EV_KEY,
        input_dev->evbit);

    set_bit(EV_ABS, input_dev->evbit);
input_set_abs_params(input_dev, ABS_X, 0, 1023, 5, 0);
input_set_abs_params(input_dev, ABS_Y, 0, 1023, 5, 0);
input_set_abs_params(input_dev, ABS_RX, 0, 1023, 5, 0);
input_set_abs_params(input_dev, ABS_RY, 0, 1023, 5, 0);


joy->chan_num = 4;
joy->chan[0] = CHAN_0;
joy->chan[1] = CHAN_1;
joy->chan[2] = CHAN_2; //LEFT, RIGHT
joy->chan[3] = CHAN_3;//UP, DOWN
joy->chan[4] = CHAN_4;//KEY_SPACE,KEY_ENTER,KEY_VOLUMEDOWN,KEY_VOLUMEUP

joy->jstickkeys[0] = KEY_I;
joy->jstickkeys[1] = KEY_K;
joy->jstickkeys[2] = KEY_L;
joy->jstickkeys[3] = KEY_J;
joy->jstickkeys[4] = KEY_RIGHT;
joy->jstickkeys[5] = KEY_LEFT;
joy->jstickkeys[6] = KEY_DOWN;
joy->jstickkeys[7] = KEY_UP;

/*
 joy->jstickaxis[0] = ABS_RY;
 joy->jstickaxis[1] = ABS_RX;
 joy->jstickaxis[2] = ABS_X;
 joy->jstickaxis[3] = ABS_Y;
 */

sprintf(phys, "input/ts");
input_dev->name = "adc joystick";
input_dev->phys = phys;
input_dev->dev.parent = &pdev->dev;

input_dev->id.bustype = BUS_ISA;
input_dev->id.vendor = 0x0001;
input_dev->id.product = 0x0001;
input_dev->id.version = 0x100;

input_dev->rep[REP_DELAY]=0xffffffff;
input_dev->rep[REP_PERIOD]=0xffffffff;

input_dev->keycodesize = sizeof(unsigned short);
input_dev->keycodemax = 0x1ff;

ret = input_register_device(joy->input);
if (ret < 0) {
    printk(KERN_ERR "Unable to register keypad input device.\n");
    kfree(joy);
    input_free_device(input_dev);
    return -EINVAL;
}
printk("adc joystick register input device completed.\r\n");
register_keypad_dev(gp_joy);

gpio_keys_init();

struct device *dev = &pdev->dev;
sysfs_create_group(&dev->kobj, &joy_attr_group);

return 0;
}

static int joy_remove(struct platform_device *pdev) {
struct joy *joy = platform_get_drvdata(pdev);

input_unregister_device(joy->input);
input_free_device(joy->input);
unregister_chrdev(joy->config_major, joy->config_name);
if (joy->config_class) {
    if (joy->config_dev)
        device_destroy(joy->config_class, MKDEV(joy->config_major, 0));
    class_destroy(joy->config_class);
}
kfree(joy);
gp_joy = NULL;
return 0;
}

static struct platform_driver joy_driver = { .probe = joy_probe, .remove =
    joy_remove, .suspend = NULL, .resume = NULL, .driver =
    { .name = "mx-adcjs", }, };

static int __devinit joy_init() {
printk(KERN_INFO "ADC_JOY_INIT\n");
return platform_driver_register(&joy_driver);
}

static void __exit joy_exit() {
printk(KERN_INFO "ADC_JOY_EXIT\n");
platform_driver_unregister(&joy_driver);
return;
}

module_init( joy_init);
module_exit( joy_exit);

MODULE_AUTHOR("Alessandro Mangone");
MODULE_DESCRIPTION("ADC Joystick Driver");
MODULE_LICENSE("GPL");