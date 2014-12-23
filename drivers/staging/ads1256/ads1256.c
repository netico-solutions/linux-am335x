/*
 * Texas Instruments AD1256 ADC driver
 *
 * Author: Nenad Radulovic <nenad.b.radulovic@gmail.com>
 *
 * Licensed under GPL-v2
 */

#include <linux/spi/spi.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/miscdevice.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <asm/uaccess.h>

#include "ads1256.h"


struct ads1256_state {
        struct ads125x_multi    multi;
        bool                    is_busy;
};

static int ads1256_all_set_mux(struct ads125x_multi * multi, uint32_t positive, 
                uint32_t negative);
static int ads1256_open(struct inode * inode, struct file * fd);
static int ads1256_release(struct inode * inode, struct file * fd);
static long ads1256_ioctl(struct file * fd, unsigned int , unsigned long);
static ssize_t ads1256_read(struct file * fd, char __user *, size_t, loff_t *);


/*--  Module parameters  ----------------------------------------------------*/
static int g_bus_id = 0;
module_param(g_bus_id, int, S_IRUGO);
MODULE_PARM_DESC(g_bus_is, "SPI bus ID");

static int g_en_mask = 0;
module_param(g_en_mask, int, S_IRUGO);
MODULE_PARM_DESC(g_en_mask, "Enabled chip mask");

static int g_cs0;
module_param(g_cs0, int, S_IRUGO);
MODULE_PARM_DESC(g_cs0, "chip 0 CS gpio");
static int g_drdy0;
module_param(g_drdy0, int, S_IRUGO);
MODULE_PARM_DESC(g_drdy0, "chip 0 DRDY gpio");

#if (ADS125X_CONFIG_SUPPORTED_CHIPS >= 2)
static int g_cs1;
module_param(g_cs1, int, S_IRUGO);
MODULE_PARM_DESC(g_cs1, "chip 1 CS gpio");
static int g_drdy1;
module_param(g_drdy1, int, S_IRUGO);
MODULE_PARM_DESC(g_drdy1, "chip 1 DRDY gpio");
#endif

#if (ADS125X_CONFIG_SUPPORTED_CHIPS >= 3)
static int g_cs2;
module_param(g_cs2, int, S_IRUGO);
MODULE_PARM_DESC(g_cs2, "chip 2 CS gpio");
static int g_drdy2;
module_param(g_drdy2, int, S_IRUGO);
MODULE_PARM_DESC(g_drdy2, "chip 2 DRDY gpio");
#endif

#if (ADS125X_CONFIG_SUPPORTED_CHIPS >= 4)
static int g_cs3;
module_param(g_cs3, int, S_IRUGO);
MODULE_PARM_DESC(g_cs3, "chip 3 CS gpio");
static int g_drdy3;
module_param(g_drdy3, int, S_IRUGO);
MODULE_PARM_DESC(g_drdy3, "chip 3 DRDY gpio");
#endif

static const struct file_operations g_ads1256_fops = {
        .owner          = THIS_MODULE,
        .open           = ads1256_open,
        .release        = ads1256_release,
        .unlocked_ioctl = ads1256_ioctl,
        .read           = ads1256_read,
};

static struct miscdevice g_ads1256_miscdev = {
        MISC_DYNAMIC_MINOR, 
        ADS125X_NAME,
        &g_ads1256_fops
};

static struct ads1256_state g_ads1256_state;

static int ads1256_cs_gpio(int id)
{
        static int * const cs[] = {
                &g_cs0,
#if (ADS125X_CONFIG_SUPPORTED_CHIPS >= 2)
                &g_cs1,
#endif
#if (ADS125X_CONFIG_SUPPORTED_CHIPS >= 3)
                &g_cs2,
#endif
#if (ADS125X_CONFIG_SUPPORTED_CHIPS >= 4)
                &g_cs3
#endif
        };
        return (*cs[id]);
}



static int ads1256_drdy_gpio(int id)
{
        static int * const drdy[] = {
                &g_drdy0,
#if (ADS125X_CONFIG_SUPPORTED_CHIPS >= 2)
                &g_drdy1,
#endif
#if (ADS125X_CONFIG_SUPPORTED_CHIPS >= 3)
                &g_drdy2,
#endif
#if (ADS125X_CONFIG_SUPPORTED_CHIPS >= 4)
                &g_drdy3
#endif
        };
        return (*drdy[id]);
}
    


static int ads1256_all_set_mux(struct ads125x_multi * multi, uint32_t positive, 
                uint32_t negative)
{
        int                     ret;
        unsigned int            chip_id;

        for (chip_id = 0; chip_id < ADS125X_CONFIG_SUPPORTED_CHIPS; chip_id++) {
                if (!ads125x_is_chip_enabled(multi, chip_id)) {
                        ADS125X_NOT("chip %d: set_mux() skipping\n",
                                        chip_id);
                        continue;
                }
                ADS125X_NOT("chip %d: set_mux() = %d, %d\n", chip_id, 
                        positive, negative);
                ret = ads125x_set_mux(multi->chip[chip_id], positive, negative);

                if (ret) {
                        ADS125X_ERR("chip %d: set_mux() = %d, %d failed, err: %d\n",
                                        chip_id, positive, negative, ret);

                        return (ret);
                }
        }

        return (0);
}



static int ads1256_all_buffer_enable(struct ads125x_multi * multi)
{
        int                     ret;
        unsigned int            chip_id;
        unsigned int            enabled_id;
        
        ret = ads125x_multi_lock(multi);

        if (ret) {
                return (ret);
        }
        enabled_id = 0;

        for (chip_id = 0; chip_id < ADS125X_CONFIG_SUPPORTED_CHIPS; chip_id++) {
                if (!ads125x_is_chip_enabled(multi, chip_id)) {
                        ADS125X_NOT("chip %d: buffer_enable() skipping\n",
                                        chip_id);
                        continue;
                }
                ADS125X_NOT("chip %d: buffer_enable()\n", chip_id);
                ret = ads125x_buffer_enable(multi->chip[chip_id]);

                if (ret) {
                        ADS125X_ERR("chip %d: buffer_enable() failed, err: %d\n",
                                        chip_id, ret);

                        goto fail_buffer_enable;
                }
                enabled_id |= (0x1u << chip_id);
        }
        return (0);
fail_buffer_enable:
        for (chip_id = 0; chip_id < ADS125X_CONFIG_SUPPORTED_CHIPS; chip_id++) {
                if (!(enabled_id & (0x1u << chip_id))) {
                        continue;
                }
                ads125x_buffer_disable(multi->chip[chip_id]);
        }
        ads125x_multi_unlock(multi);

        return (ret);
}



static int ads1256_all_buffer_disable(struct ads125x_multi * multi)
{
        int                     ret;
        unsigned int            chip_id;

        if (!ads125x_multi_is_locked(multi)) {
                return (-EBUSY);
        }

        for (chip_id = 0; chip_id < ADS125X_CONFIG_SUPPORTED_CHIPS; chip_id++) {
                if (!ads125x_is_chip_enabled(multi, chip_id)) {
                        ADS125X_NOT("chip %d: buffer_disable() skipping\n",
                                        chip_id);
                        continue;
                }
                ADS125X_NOT("chip %d: buffer_disable()\n", chip_id);
                ret = ads125x_buffer_disable(multi->chip[chip_id]);

                if (ret) {
                        ADS125X_ERR("chip %d: buffer_disable() failed, err: %d\n",
                                        chip_id, ret);
                }
        }
        ads125x_multi_unlock(multi);

        return (0);
}



static int ads1256_all_self_calibrate(struct ads125x_multi * multi)
{
        int                     ret;
        unsigned int            chip_id;

        if (ads125x_multi_is_locked(multi)) {
                return (-EBUSY);
        }

        for (chip_id = 0; chip_id < ADS125X_CONFIG_SUPPORTED_CHIPS; chip_id++) {
                if (!ads125x_is_chip_enabled(multi, chip_id)) {
                        ADS125X_NOT("chip %d: self_calibrate() skipping\n",
                                        chip_id);
                        continue;
                }
                ADS125X_NOT("chip %d: self_calibrate()\n", chip_id);
                ret = ads125x_self_calibrate(multi->chip[chip_id]);

                if (ret) {
                        ADS125X_ERR("chip %d: self_calibrate() failed, err: %d\n",
                                        chip_id, ret);
                }
        }

        return (0);
}



static int ads1256_open(struct inode * inode, struct file * fd)
{
        ADS125X_NOT("open(): %d:%d\n", current->group_leader->pid, current->pid);

        if (g_ads1256_state.is_busy) {
                return (-EBUSY);
        }
        g_ads1256_state.is_busy = true;
        fd->private_data = &g_ads1256_state.multi;

        return (0);
}



static int ads1256_release(struct inode * inode, struct file * fd)
{
        ADS125X_NOT("close(): %d:%d\n", current->group_leader->pid, 
                        current->pid);

        if (!g_ads1256_state.is_busy) {
                return (-EINVAL);
        }
        g_ads1256_state.is_busy = false;

        return (0);
}



static long ads1256_ioctl(struct file * fd, unsigned int cmd, unsigned long arg)
{
        int                     ret;
        struct ads125x_multi *  multi = fd->private_data;
        
        switch (cmd) {
                case ADS125X_SET_MUX: {
                        struct ads125x_mux mux;
                        
                        ret = copy_from_user(&mux, (const void __user *)arg, 
                                        sizeof(struct ads125x_mux));

                        if (ret) {
                                return (-EACCES);
                        }
                        ret = ads1256_all_set_mux(multi, mux.positive, 
                                        mux.negative);

                        return (ret);
                }
                case ADS125X_SET_BUF_SIZE: {
                        int     buf_size;

                        ret = copy_from_user(&buf_size, (const void __user*)arg,
                                        sizeof(int));

                        if (ret) {
                                return (-EACCES);
                        }
                        ret = ads125x_multi_ring_set_size(multi, buf_size);

                        return (ret);
                }
                case ADS125X_SELF_CALIBRATE: {

                        return (ads1256_all_self_calibrate(multi));
                }
                case ADS125X_START_SAMPLING: {
                        
                        return (ads1256_all_buffer_enable(multi));
                }
                case ADS125X_STOP_SAMPLING: {

                        return (ads1256_all_buffer_disable(multi));
                }
                default : {

                        return (-EINVAL);
                }
        }
}



static ssize_t ads1256_read(struct file * fd, char __user * buff, size_t count, 
                loff_t * off)
{
        struct ads125x_multi *  multi = fd->private_data;
        ssize_t                 ret;

        ret = ads125x_multi_ring_get_items(multi, buff, count, HZ);

        if (ret > 0) {
                *off += ret;
        }
        
        return (ret);
}



static int __init ads1256_init(void)
{
        int                     ret;
        unsigned int            chip_id;
        struct spi_master *     master;
        struct spi_device *     spi;
        struct ads125x_multi *  multi;
        struct spi_board_info   ads1256_device_info = {
                .modalias     = ADS125X_NAME,
                .max_speed_hz = 10000000ul,
                .bus_num      = g_bus_id,
        };

        ADS125X_NOT("registering ADS1256 device driver\n");

        if (!g_en_mask) {
                ADS125X_ERR("no chip enabled: set module parameters\n");

                return (-ENODEV);
        }
        master = spi_busnum_to_master(g_bus_id);

        if (!master) {
                ADS125X_ERR("invalid SPI bus id: %d\n", g_bus_id);

                return (-ENODEV);
        }
        spi = spi_new_device(master, &ads1256_device_info);

        if (!spi) {
                ADS125X_ERR("could not create SPI device\n");

                return (-ENODEV);
        }
        g_ads1256_state.is_busy = false;
        multi = &g_ads1256_state.multi;
        ret = ads125x_init_multi(multi, spi, g_en_mask);
        
        if (ret) {
                ADS125X_ERR("failed to init multi, err: %d\n", ret);

                goto fail_init_multi;
        }

        for (chip_id = 0; chip_id < ADS125X_CONFIG_SUPPORTED_CHIPS; chip_id++) {
                struct ads125x_chip * chip;

                if (!ads125x_is_chip_enabled(multi, chip_id)) {
                        ADS125X_NOT("chip %d: skipping\n", chip_id);

                        continue;
                }
                chip = kzalloc(sizeof(*chip), GFP_KERNEL);

                if (!chip) {
                        ADS125X_ERR("chip %d: failed to allocate chip\n",
                                        chip_id);
                        ret = -ENOMEM;

                        goto fail_for_chip;
                }
                ret = ads125x_init_chip(chip, multi, chip_id, 
                                ads1256_cs_gpio(chip_id), 
                                ads1256_drdy_gpio(chip_id));

                if (ret) {
                        ADS125X_ERR("chip %d: failed to init chip, err: %d\n", 
                                        chip_id, ret);
                        kfree(chip);
                        
                        goto fail_for_chip;
                }
                ret = ads125x_probe_trigger(chip);

                if (ret) {
                        ADS125X_ERR("chip %d: failed to init trigger, err: %d\n", 
                                        chip_id, ret);
                        ads125x_term_chip(chip);
                        kfree(chip);

                        goto fail_for_chip;
                }
                ret = ads125x_init_hw(chip);
                
                if (ret) {
                        ADS125X_ERR("chip %d: failed to init hardware, err: %d\n", 
                                        chip_id, ret);
                        ads125x_remove_trigger(chip);
                        ads125x_term_chip(chip);
                        kfree(chip);
                }
                multi->chip[chip_id] = chip;
                ADS125X_NOT("chip %d: cs gpio %d, drdy gpio %d\n", chip_id, 
                                ads1256_cs_gpio(chip_id),
                                ads1256_drdy_gpio(chip_id));
        }
        ret = misc_register(&g_ads1256_miscdev);

        return (ret);
fail_for_chip:
        for (chip_id = 0; chip_id < ADS125X_CONFIG_SUPPORTED_CHIPS; chip_id++) {
                struct ads125x_chip * chip;

                chip = multi->chip[chip_id];

                if (chip) {
                        ADS125X_NOT("chip %d: cleaning up\n", chip_id);
                        ads125x_term_hw(chip);
                        ads125x_remove_trigger(chip);
                        ads125x_term_chip(chip);
                        kfree(chip);
                        multi->chip[chip_id] = NULL;
                }
        }
        ads125x_term_multi(multi);
        
fail_init_multi:
        spi_unregister_device(spi);

        return (ret);
}



static void __exit ads1256_exit(void)
{
        unsigned int            chip_id;
        struct ads125x_multi *  multi;
        struct spi_device *     spi;

        ADS125X_NOT("deregistering ADS1256 device driver\n");
        multi = &g_ads1256_state.multi;

        for (chip_id = 0; chip_id < ADS125X_CONFIG_SUPPORTED_CHIPS; chip_id++) {
                struct ads125x_chip * chip;

                chip = multi->chip[chip_id];

                if (chip) {
                        ADS125X_INF("chip %d: cleaning up\n", chip_id);
                        ads125x_term_hw(chip);
                        ads125x_remove_trigger(chip);
                        ads125x_term_chip(chip);
                        kfree(chip);
                        multi->chip[chip_id] = NULL;
                }
        }
        spi = multi_to_spi(multi);
        ads125x_term_multi(multi);
        spi_unregister_device(spi);
        misc_deregister(&g_ads1256_miscdev);
}

module_init(ads1256_init);
module_exit(ads1256_exit);
MODULE_AUTHOR("Nenad Radulovic <nenad.b.radulovic@gmail.com>");
MODULE_DESCRIPTION("Texas Instruments customized AD1256 ADC driver");
MODULE_LICENSE("GPL v2");

