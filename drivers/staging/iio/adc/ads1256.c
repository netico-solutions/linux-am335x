
#include <linux/spi/spi.h>
#include <linux/module.h>
#include <linux/iio/iio.h>

#include "ads1256.h"

/* ========================================================================== *
 * Platform driver for ADS1256 ADC
 * ========================================================================== */

#define CONFIG_NUM_OF_CHANNELS          8

enum ads1256_id_no {
    ADS1256,
};

struct ads1256_chip_info {
    struct iio_chan_spec        channel[CONFIG_NUM_OF_CHANNELS];
};

struct ads1256_state {
    struct spi_device *         spi;
    const struct ads1256_chip_info * chip_info;
};

static int ads1256_probe(struct spi_device * spi)
{
    struct iio_dev *            indio_dev;

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

MODULE_AUTHOR("Nenad Radulovic <nenad.b.radulovic@netico-group.com");
MODULE_DESCRIPTION("Platform driver TI AD1256 ADC");
MODULE_LICENSE("GPL v2");

