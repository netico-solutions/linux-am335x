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
#include <linux/kernel.h>

#include "ads1256.h"

struct ads1256_state {
        struct ads1256_chip     chip;
};

static struct ads1256_state * state_from_chip(struct ads1256_chip * chip);

static struct ads1256_state * state_from_chip(struct ads1256_chip * chip)
{
        return (container_of(chip, struct ads1256_state, chip));
}



static int ads1256_probe(struct spi_device * spi)
{
        struct ads1256_chip *   chip;
        int                     ret;

        chip = kzalloc(sizeof(*chip), GFP_KERNEL);

        if (!chip) {
                dev_err(&spi->dev, "could not allocate ads1256 chip\n");

                return (-ENOMEM);
        }

        if (!spi->dev.of_node) {
                dev_err(&spi->dev, "missing device tree\n");
                ret = -ENODEV;

                goto fail_of_node;
        }

        if (!spi->irq) {
                dev_err(&spi->dev, "missing SPI IRQ handler\n");
                ret = -ENODEV;

                goto fail_spi_irq;
        }
        spi_set_drvdata(spi, chip);
        dev_info(&spi->dev, "evaluating DTS data\n");
        ret = ti_sd_probe_of(spi);

        if (ret) {
                dev_err(&spi->dev, "failed to parse DTS data\n");

                goto fail_dts_parse;
        }
        dev_info(&spi->dev, "initializing chip info\n");
        ret = ti_sd_init_chip(chip, spi);

        if (ret) {
                dev_err(&spi->dev, "failed to initialize chip info\n");

                goto fail_chip_init;
        }
        ret = ti_sd_probe_trigger(chip);

        if (ret) {
                dev_err(&spi->dev, "failed to probe trigger\n");

                goto fail_trigg_setup;
        }
        ret = ti_sd_init_hw(chip);

        if (ret) {
                dev_err(&spi->dev, "failed to initialize ADC\n");

                goto fail_adc_init;
        }

        return (ret);
fail_adc_init:
fail_trigg_setup:
        ti_sd_term_chip(chip);
fail_chip_init:
fail_dts_parse:
fail_spi_irq:
fail_of_node:
        kfree(chip);
        spi_set_drvdata(spi, NULL);

        return (ret);
}


    
static int ads1256_remove(struct spi_device * spi)
{
        int                     ret;
        struct ads1256_state *  state;
        struct ads1256_chip  *  chip = spi_get_drvdata(spi);

        ret = 0;

        if (chip) {
                state = state_from_chip(chip);

                ti_sd_term_hw(chip);
                ti_sd_remove_trigger(chip);
                ti_sd_term_chip(chip);
                kfree(chip);
        }

        return (ret);
}


static const struct of_device_id ads1256_of_match[] = {
    {.compatible = "ti,ads1256"},
    { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ads1256_of_match);

static const struct spi_device_id ads1256_id[] = {
        { "ads1256", 0},
        { }
};
MODULE_DEVICE_TABLE(spi, ads1256_id);

static struct spi_driver ads1256_driver = {
        .driver = {
                .name           = "ads1256",
                .owner          = THIS_MODULE,
                .of_match_table = ads1256_of_match
        },
        .probe      = ads1256_probe,
        .remove     = ads1256_remove,
        .id_table   = ads1256_id,
};
module_spi_driver(ads1256_driver);

MODULE_AUTHOR("Nenad Radulovic <nenad.b.radulovic@gmail.com>");
MODULE_DESCRIPTION("Texas Instruments AD1256 ADC driver");
MODULE_LICENSE("GPL v2");

