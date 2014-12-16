/*
 * Texas Instruments AD1256 ADC driver
 *
 * Author: Nenad Radulovic <nenad.b.radulovic@gmail.com>
 *
 * Licensed under GPL-v2
 */

#include <linux/spi/spi.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/slab.h>

#include "ads1256.h"

#define CONFIG_NUM_OF_CHANNELS          8



enum ads1256_id_no {
    ADS1256,
};



static int ads1256_probe(struct spi_device * spi)
{
        struct ads1256_chip *       chip;
        struct ads1256_platform_data * pdata;
        int                         retval;

        if (spi->irq == NULL) {
                dev_err(&spi->dev, " missing SPI IRQ handler\n");

                return (-ENXIO);
        }
        pdata = &spi->dev.platform_data;

        if (!pdata) { 
                dev_err(&spi->dev, " missing platform data\n");

                return (-ENODEV);
        }
        chip = kzalloc(sizeof(*chip), GFP_KERNEL);

        if (!chip) {
                dev_err(&spi->dev, " could not allocate SPI device chip\n");

                return (-ENOMEM);
        }
        spi_set_drvdata(spi, chip);
        dev_info(&spi->dev, " driver probe starting.\n");
        ret = ti_sd_init(chip, spi);

        return (ret);
}


    
static int ads1256_remove(struct spi_device * spi)
{

    return (0);
}



static const struct spi_device_id ads1256_id[] = {
    { "ads1256", ADS1256 },
    {}
};
MODULE_DEVICE_TABLE(spi, ads1256_id);



static struct spi_driver ads1256_driver = {
    .driver = {
        .name   = "ads1256",
        .owner  = THIS_MODULE,
    },
    .probe      = ads1256_probe,
    .remove     = ads1256_remove,
    .id_table   = ads1256_id,
};
module_spi_driver(ads1256_driver);

MODULE_AUTHOR("Nenad Radulovic <nenad.b.radulovic@gmail.com>");
MODULE_DESCRIPTION("Texas Instruments AD1256 ADC driver");
MODULE_LICENSE("GPL v2");

