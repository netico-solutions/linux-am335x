/*
 * Texas Instruments AD1256 ADC driver
 *
 * Author: Nenad Radulovic <nenad.b.radulovic@gmail.com>
 *
 * Licensed under GPL-v2
 */

#include <linux/spi/spi.h>
#include <linux/module.h>
#include <linux/iio/iio.h>
#include <linux/printk.h>
#include <linux/device.h>

#include "ads1256.h"

#define CONFIG_NUM_OF_CHANNELS          8

enum ads1256_id_no {
    ADS1256,
};

struct ads1256_chip_info {
    struct iio_chan_spec        channel[CONFIG_NUM_OF_CHANNELS];
};

struct ads1256_state {
    struct ti_sigma_delta       sd;
    struct regulator *          regulator;
};

static int ads1256_probe(struct spi_device * spi)
{
    struct iio_dev *            indio_dev;
    struct ads1256_state *      state;
    int                         retval;

    if (spi->irq == NULL) {
        dev_err(&spi->dev, " missing SPI IRQ handler\n");

        return (-ENXIO);
    }
    indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*state));

    if (indio_dev == NULL) {
        dev_err(&spi->dev, " could not allocate IIO device\n");

        return (-ENOMEM);
    }
    state = iio_priv(indio_dev);
    /* Enable SPI power */
    state->regulator = devm_regulator_get(&spi->dev, "refin");

    if (IS_ERR(state->regulator)) {
        return (PTR_ERR(state->regulator));
    }
    retval = regulator_enable(state->regulator);

    if (retval) {
        return (retval);
    }
    ti_sd_init(&state->sd, indio_dev, spi);
    spi_set_drvdata(spi, indio_dev);
    indio_dev->dev.parent   = &spi->dev;
    indio_dev->name         = spi_get_device_id(spi)->name;
    indio_dev->modes        = INDIO_BUFFER_TRIGGERED;

    return (0);
}
    
static int ads1256_remove(struct spi_device * spi)
{
    struct iio_dev *            indio_dev;

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

