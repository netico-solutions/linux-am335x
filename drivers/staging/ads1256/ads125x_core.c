/*
 * Support code for Texas Instruments Sigma-delta ADCs
 *
 * Author: Nenad Radulovic <nenad.b.radulovic@gmail.com>
 * 
 * Licensed under GPL-v2
 */

#include <linux/spi/spi.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/of_gpio.h>

#include "ads1256.h"

#define CONFIG_FIFO_SIZE                1024

#define SPI_CS_ACTIVE                   0
#define SPI_CS_INACTIVE                 1


struct ti_sd_sample {
        uint32_t                raw_value;
};

static int start_chip_spi(struct ads1256_chip * chip,
                struct spi_message * message);
static void stop_chip_spi(struct ads1256_chip * chip);
static void message_complete(void * arg);
static irqreturn_t ti_sd_trigger_ready_handler(int irq, void * p);
static int ti_sd_buffer_enable(struct ads1256_chip * chip);
static int ti_sd_buffer_disable(struct ads1256_chip * chip);
static int ti_sd_read_data(struct ads1256_chip * chip);
static int ti_sd_set_mode(struct ads1256_chip * chip, uint32_t mode);

static int start_chip_spi(struct ads1256_chip * chip, 
                struct spi_message * message)
{
        message->complete = message_complete;
        message->context  = chip;
        gpio_set_value(chip->cs_gpio, SPI_CS_ACTIVE);

        return (spi_async_locked(chip->spi, message));
}



static void stop_chip_spi(struct ads1256_chip * chip)
{
        gpio_set_value(chip->cs_gpio, SPI_CS_INACTIVE);
}



static void message_complete(void * context)
{
        stop_chip_spi((struct ads1256_chip *)context);
}



static irqreturn_t ti_sd_trigger_ready_handler(int irq, void * p)
{
        struct ads1256_chip *   chip = p;
        int                     ret;

        disable_irq_nosync(irq);
        ret = ti_sd_read_data(chip);
        
        if (ret == 0) {
                /* push to buffers */
        }
        enable_irq(irq);

        return (IRQ_HANDLED);
}



static int ti_sd_read_data_begin(struct ads1256_chip * chip)
{
        memset(&chip->irq_transfer, 0, sizeof(chip->irq_transfer));
        chip->irq_transfer.rx_buf = chip->irq_data;
        chip->irq_transfer.len    = 3;             /* Samples are 24 bit wide */

        spi_message_init(&chip->irq_message);
        spi_message_add_tail(&chip->irq_transfer, &chip->irq_message);
        chip->message.complete = &ti_sd_read_data_finish;
        chip->message.complete = chip;
        gpio_set_value(chip->cs_gpio, SPI_CS_ACTIVE);

        return (start_chip_spi(chip, &chip->irq_message));
}



static void ti_sd_read_data_finish(void * arg)
{
        struct ads1256_chip *   chip;

        gpio_set_value(chip->cs_gpio, SPI_CS_INACTIVE);

        /* push to buffers here */

        complete(&chip->completion);
}


static int ti_sd_buffer_enable(struct ads1256_chip * chip)
{
        int                     ret;

        spi_bus_lock(chip->spi->master);
        chip->is_bus_locked = true;
        init_completion(&chip->completion);

        ret = ti_sd_set_mode(chip, ADS125X_MODE_CONTINUOUS);

        if (ret) {
                goto fail_set_mode;
        }
        chip->is_irq_enabled = true;
        enable_irq(gpio_to_irq(chip->drdy_gpio));

        return (0);

fail_set_mode:
        spi_bus_unlock(chip->spi->master);
    
        return (ret);
}



static int ti_sd_buffer_disable(struct ads1256_chip * chip)
{
        int                     ret;

        INIT_COMPLETION(chip->completion);
        wait_for_completion_timeout(&chip->completion, HZ);
    
        disable_irq_nosync(chip->spi->irq);
        chip->is_irq_enabled = false;
        ret = ti_sd_set_mode(chip, ADS125X_MODE_IDLE);

        spi_bus_unlock(chip->spi->master);
        chip->is_bus_locked = false;

        return (ret);
}



static int ti_sd_read_data(struct ads1256_chip * chip)
{
        memset(&chip->irq_transfer, 0, sizeof(chip->irq_transfer));
        chip->irq_transfer.rx_buf = chip->irq_data;
        chip->irq_transfer.len    = 3;             /* Samples are 24 bit wide */

        spi_message_init(&chip->irq_message);
        spi_message_add_tail(&chip->irq_transfer, &chip->irq_message);

        return (start_chip_spi(chip, &chip->irq_message));
}



static int ti_sd_set_mode(struct ads1256_chip * chip, uint32_t mode)
{
        if (chip->is_irq_enabled) {
                return (-EBUSY);
        }
        memset(&chip->transfer[0], 0, sizeof(chip->transfer));
        chip->transfer[0].tx_buf = chip->transfer_data;
        chip->transfer[0].len    = 1;
        chip->transfer[1].rx_buf = chip->transfer_data;
        chip->transfer[1].len    = 3;

        spi_message_init(&chip->message);

        switch (mode) {
                case ADS125X_MODE_CONTINUOUS:
                        chip->transfer_data[0] = ADS125X_CMD_RDATAC; 
                        spi_message_add_tail(&chip->transfer[0], 
                                             &chip->message);
                                        /* Get and dump the first measurement */
                        spi_message_add_tail(&chip->transfer[1], 
                                             &chip->message); 
                        break;
                case ADS125X_MODE_IDLE:
                        chip->transfer_data[0] = ADS125X_CMD_SDATAC;
                        spi_message_add_tail(&chip->transfer[0], 
                                             &chip->message);
                        break;
                default:
                        return (-EINVAL);
        }

        return (start_chip_spi(chip, &chip->message));
}

/*--  PUBLIC METHODS  --------------------------------------------------------*/


/**
 * ti_sd_probe_of() - setup chip data from DTS
 * spi: SPI device
 *
 * Returns 0 on success, an error code otherwise
 */
int ti_sd_probe_of(struct spi_device * spi)
{
        struct device_node *    node = spi->dev.of_node;
        struct ads1256_chip *   chip = spi_get_drvdata(spi);
        int                     gpio;
        u8                      id;

        /* Get device id */
        if (of_property_read_u8(node, "dev-id", &id)) {
                dev_err(&spi->dev, "failed to parse device ID property\n");

                return (-EINVAL);
        }
        chip->id = id;
        /* DRDY gpio pin */
        gpio = of_get_gpio(node, 0);

        if (!gpio_is_valid(gpio)) {
                dev_err(&spi->dev, "failed to parse DRDY gpio pin property\n");

                return (gpio);
        }
        chip->drdy_gpio = gpio;
        /* CS gpio pin */
        gpio = of_get_gpio(node, 1);

        if (!gpio_is_valid(gpio)) {
                dev_err(&spi->dev, "failed to parse CS gpio pin property\n");

                return (gpio);
        }
        chip->cs_gpio = gpio;

        return (0);
}
EXPORT_SYMBOL_GPL(ti_sd_probe_of);



/**
 * ti_sd_probe_trigger()
 * @chip: chip device
 *
 * Returns 0 on success, an error code otherwise
 */
int ti_sd_probe_trigger(struct ads1256_chip * chip)
{
        int                     ret;
        char                    label[16];

        ret = gpio_to_irq(chip->drdy_gpio);

        if (ret < 0) {
                dev_err(&chip->spi->dev, " trigger setup failed.\n");

                goto fail_gpio_irq;
        }
        init_completion(&chip->completion);
        sprintf(label, ADS125X_NAME "-%d-drdy-irq", chip->id);
        ret = request_irq(ret, &ti_sd_trigger_ready_handler,
                IRQF_TRIGGER_FALLING, label, chip);

        if (ret) {
                goto fail_irq_request;
        }

        return (0);
fail_irq_request:
        free_irq(gpio_to_irq(chip->drdy_gpio), chip);
fail_gpio_irq:

        return (ret);
}
EXPORT_SYMBOL_GPL(ti_sd_probe_trigger);



/**
 * ti_sd_init()
 * @chip: chip device
 * @spi: SPI device
 *
 * Returns 0 on success, an error code otherwise
 */
int ti_sd_init_chip(struct ads1256_chip * chip, struct spi_device * spi)
{
        int                     ret;
        char                    label[16];

        chip->spi            = spi;
        chip->is_bus_locked  = false;
        chip->is_irq_enabled = false;

        ret = kfifo_alloc(&chip->fifo, CONFIG_FIFO_SIZE, GFP_KERNEL);

        if (ret) {
                dev_err(&spi->dev, "error kfifo alloc\n");

                goto fail_fifo_alloc;
        }
        sprintf(label, ADS125X_NAME "-%d-drdy", chip->id);
        ret = gpio_request_one(chip->drdy_gpio, GPIOF_DIR_IN, label);

        if (ret) {
                dev_err(&spi->dev, "DRDY gpio request failed\n");

                goto fail_drdy_request;
        }
        sprintf(label, ADS125X_NAME "-%d-cs", chip->id);
        ret = gpio_request_one(chip->cs_gpio, GPIOF_INIT_HIGH, label);

        if (ret) {
                dev_err(&spi->dev, "CS gpio request failed\n");

                goto fail_cs_request; 
        }
        spi->bits_per_word  = 8;
        spi->mode           = SPI_MODE_0;
        spi->max_speed_hz   = 10000000ul;

        ret = spi_setup(spi);

        if (ret) {
                dev_err(&spi->dev, "SPI setup failed\n");

                goto fail_spi_setup; 
        }

        return (ret);
fail_spi_setup:
        gpio_free(chip->cs_gpio);
fail_cs_request:
        gpio_free(chip->drdy_gpio);
fail_drdy_request:
        kfifo_free(&chip->fifo);
fail_fifo_alloc:

        return (ret);
}
EXPORT_SYMBOL_GPL(ti_sd_init_chip);



/**
 * ti_sd_init_hw()
 * @chip: chip device
 *
 * Returns 0 on success, an error code otherwise
 */
int ti_sd_init_hw(struct ads1256_chip * chip)
{
        int                     ret;

        ret = ti_sd_write_reg(chip, ADS125X_REG_STATUS, ADS125X_STATUS_ACAL);

        if (ret) {
                goto fail_write;
        }
        ret = ti_sd_write_reg(chip, ADS125X_REG_ADCON,  0);

        if (ret) {
                goto fail_write;
        }
        ret = ti_sd_write_reg(chip, ADS125X_REG_DRATE,  ADS125X_DRATE_10);

        if (ret) {
                goto fail_write;
        }
        ret = ti_sd_write_reg(chip, ADS125X_REG_IO,     0);

        if (ret) {
                goto fail_write;
        }
        ret = ti_sd_set_channel(chip, 2, 3);

fail_write:
        return (ret);
}
EXPORT_SYMBOL_GPL(ti_sd_init_hw);



/**
 * ti_sd_term()
 * @chip: chip device
 *
 * Returns 0 on success, an error code otherwise
 */
int ti_sd_term_chip(struct ads1256_chip * chip)
{
        /* TODO: terminate SPI here */
        kfifo_free(&chip->fifo);
        gpio_free(chip->cs_gpio);
        gpio_free(chip->drdy_gpio);
        kfree(chip);

        return (0);
}
EXPORT_SYMBOL_GPL(ti_sd_term_chip);



void ti_sd_term_hw(struct ads1256_chip * chip)
{
        /* TODO: stop and terminate HW here */
}
EXPORT_SYMBOL_GPL(ti_sd_term_hw);



/**
 * ti_sd_cleanup_buffer_and_trigger()
 * @chip: chip device
 */
void ti_sd_remove_trigger(struct ads1256_chip * chip)
{
        disable_irq(gpio_to_irq(chip->drdy_gpio));
        free_irq(gpio_to_irq(chip->drdy_gpio), chip);
        /* TODO: iio_triggered_buffer_cleanup() */
}
EXPORT_SYMBOL_GPL(ti_sd_remove_trigger);



/**
 * ti_sd_write_reg() - Write a register
 *
 * @chip: The sigma delta device
 * @reg: Address of the registers
 * @val: Value to write to the register
 *
 * Returns 0 on success, an error code otherwise
 */
int ti_sd_write_reg(struct ads1256_chip * chip, uint32_t reg, uint32_t val)
{
        if (chip->is_irq_enabled) {
                return (-EBUSY);;
        }
        memset(&chip->transfer[0], 0, sizeof(chip->transfer));
        chip->transfer[0].tx_buf = chip->transfer_data;
        chip->transfer[0].len    = 3;     /* Add for 1st and 2nd command byte */

        chip->transfer_data[0]   = ADS125X_CMD_WREG(reg);       /* command id */
        chip->transfer_data[1]   = 0;              /* byte counter, 0 = 1 reg */
        chip->transfer_data[2]   = (uint8_t)val;

        spi_message_init(&chip->message);
        spi_message_add_tail(&chip->transfer[0], &chip->message);

        return (start_chip_spi(chip, &chip->message));
}
EXPORT_SYMBOL_GPL(ti_sd_write_reg);



/**
 * ti_sd_read_reg()
 * @chip: The sigma delta device
 * @reg: Address of the register
 * @val: Pointer to a buffer
 *
 * Returns 0 on success, an error code otherwise.
 */
int ti_sd_read_reg(struct ads1256_chip * chip, uint32_t reg, uint32_t * val)
{
        if (chip->is_irq_enabled) {
                return (-EBUSY);
        }
        memset(&chip->transfer[0], 0, sizeof(chip->transfer));
        chip->transfer[0].tx_buf = chip->transfer_data;
        chip->transfer[0].len    = 2;             /* 1st and 2nd command byte */
        chip->transfer[1].rx_buf = val;
        chip->transfer[1].len    = 1;

        chip->transfer_data[0]   = ADS125X_CMD_RREG(reg);       /* command id */
        chip->transfer_data[1]   = 0;             /* byte counter, 0 = 1 byte */
        spi_message_init(&chip->message);
        spi_message_add_tail(&chip->transfer[0], &chip->message);
        spi_message_add_tail(&chip->transfer[1], &chip->message);

        return (start_chip_spi(chip, &chip->message));
}
EXPORT_SYMBOL_GPL(ti_sd_read_reg);



/**
 * ti_sd_self_calibrate()
 * @chip: The sigma delta device
 *
 * Returns 0 on success, an error code otherwise.
 */
int ti_sd_self_calibrate(struct ads1256_chip * chip)
{
        if (chip->is_irq_enabled) {
                return (-EBUSY);
        }

        return (-ENOSYS);
}
EXPORT_SYMBOL_GPL(ti_sd_self_calibrate);



/**
 * ti_sd_set_channel()
 * @chip: The sigma delta device
 * @positive: positive input channel
 * @negative: negative input channel
 *
 * Returns 0 on success, an error code otherwise.
 */
int ti_sd_set_channel(struct ads1256_chip * chip, uint8_t positive, 
                uint8_t negative)
{
        uint32_t                reg_val;
        int                     ret;

        reg_val = (uint8_t)(positive << 4u) | (uint8_t)(negative & 0x0fu);

        ret = ti_sd_write_reg(chip, ADS125X_REG_MUX, reg_val);

        return (ret);
}
EXPORT_SYMBOL_GPL(ti_sd_set_channel);

MODULE_AUTHOR("Nenad Radulovic <nenad.b.radulovic@gmail.com>");
MODULE_DESCRIPTION("Texas Instruments ADS125x ADC core");
MODULE_LICENSE("GPL v2");
