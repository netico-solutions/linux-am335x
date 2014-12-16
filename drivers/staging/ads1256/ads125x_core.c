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

#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>

#include "ads1256.h"




static irqreturn_t ti_sd_trigger_handler(int irq, void * p)
{
    struct ads1256_chip *     sigma_delta;
    uint8_t                     data[16];
    int                         ret;

    pf          = p;
    ret = ti_sd_read_data(chip, &data[0]);

    if (ret == 0) {
        iio_push_to_buffers(indio_dev, data);
        iio_trigger_notify_done(indio_dev->trig);
    }
    chip->is_irq_dis = false;
    enable_irq(chip->spi->irq);

    return (IRQ_HANDLED);
}



static irqreturn_t ti_sd_trigger_ready_handler(int irq, void * p)
{
    struct ads1256_chip *     sigma_delta;

    sigma_delta   = p;

    complete(&chip->completion);
    disable_irq_nosync(irq);
    chip->is_irq_dis = true;
    iio_trigger_poll(chip->trigger, iio_get_time_ns());

    return (IRQ_HANDLED);
}



static int ti_sd_buffer_postenable(struct iio_dev * indio_dev)
{
    struct ads1256_chip *     sigma_delta;
    uint32_t                    channel;
    int                         ret;

    ret = iio_triggered_buffer_postenable(indio_dev);
    sigma_delta = iio_device_get_drvdata(indio_dev);

    if (ret < 0) {
        return (ret);
    }
    channel = find_first_bit(indio_dev->active_scan_mask, 
            indio_dev->masklength); 
    ret  = ti_sd_set_channel(chip, 
            indio_dev->channels[channel].address);

    if (ret) {
        goto FAIL_SET_CHANNEL;
    }
    spi_bus_lock(chip->spi->master);
    chip->is_bus_locked = true;
    ret  = ti_sd_set_mode(chip, ADS125X_MODE_CONTINUOUS);

    if (ret) {
        goto FAIL_SET_MODE;
    }
    chip->is_irq_dis = false;
    enable_irq(chip->spi->irq);

    return (0);

FAIL_SET_MODE:
    spi_bus_unlock(chip->spi->master);
FAIL_SET_CHANNEL:
    
    return (ret);
}



static int ti_sd_buffer_postdisable(struct iio_dev * indio_dev)
{
    struct ads1256_chip *     sigma_delta;
    int                         ret;

    sigma_delta = iio_device_get_drvdata(indio_dev);

    INIT_COMPLETION(chip->completion);
    wait_for_completion_timeout(&chip->completion, HZ);
    
    if (!chip->is_irq_dis) {
        disable_irq_nosync(chip->spi->irq);
        chip->is_irq_dis = true;
    }
    ret = ti_sd_set_mode(chip, ADS125X_MODE_IDLE);

    if (ret) {
        goto FAIL_SET_MODE;
    }
    ret = spi_bus_unlock(chip->spi->master);
    chip->is_bus_locked = false;

    return (ret);

FAIL_SET_MODE:
    spi_bus_unlock(chip->spi->master);
    chip->is_bus_locked = false;

    return (ret);
}



static int ti_sd_probe_trigger(struct ads1256_chip * chip)
{
        int                     ret;

        ret = gpio_to_irq(chip->drdy_gpio);

        if (ret < 0) {
                dev_err(&chip->spi->dev, " trigger setup failed.\n");

                return (ret);
        }
        init_completion(&chip->completion);

        ret = request_irq(ret, &ti_sd_trigger_ready_handler,
                IRQF_TRIGGER_FALLING, label, chip);

    ret = request_irq(chip->spi->irq, &ti_sd_trigger_ready_handler,
            IRQF_TRIGGER_LOW, indio_dev->name, sigma_delta);

    if (ret) {
        goto FAIL_IRQ_REQUEST;
    }

    if (!chip->is_irq_dis) {
        chip->is_irq_dis = true;
        disable_irq_nosync(chip->spi->irq);
    }
    chip->trigger->dev.parent = &chip->spi->dev;
    iio_trigger_set_drvdata(chip->trigger, sigma_delta);
    ret = iio_trigger_register(chip->trigger);

    if (ret) {
        goto FAIL_TRIGGER_REGISTER;
    }
    /* select default trigger */
    indio_dev->trig = chip->trigger;

    return (0);
FAIL_TRIGGER_REGISTER:
    free_irq(chip->spi->irq, sigma_delta);
FAIL_IRQ_REQUEST:
    iio_trigger_free(chip->trigger);
FAIL_TRIGG_ALLOC:

    return (ret);
}



static int ti_sd_read_data(struct ads1256_chip * chip, uint8_t * val)
{
    int                         ret;
    uint8_t *                   transfer_data;
    struct spi_transfer         transfer;
    struct spi_message          message;

    memset(&transfer, 0, sizeof(transfer));
    transfer_data       = chip->transfer_data;
    transfer.rx_buf     = transfer_data;
    transfer.len        = 3;                       /* Samples are 24 bit wide */
    transfer.cs_change  = chip->is_bus_locked;

    spi_message_init(&message);
    spi_message_add_tail(&transfer, &message);

    if (chip->is_bus_locked) {
        ret = spi_sync_locked(chip->spi, &message);
    } else {
        ret = spi_sync(chip->spi, &message);
    }

    return (ret);
}

/*--  PUBLIC METHODS  --------------------------------------------------------*/



/**
 * ti_sd_init_sigma_delta()
 * @sigma_delta: The sigma delta device
 * @indio_dev: IIO device
 * @spi: SPI device
 */
int ti_sd_init(struct ads1256_chip * chip, struct spi_device * spi, 
                struct ads1256_platform_data * pdata)
{
        int                     ret;

        strcpy(chip->label, "ads1256");
        chip->spi       = spi;
        chip->cs_gpio   = pdata->cs_gpio;
        chip->drdy_gpio = pdata->drdy_gpio;
        chip->id        = pdata->id;
        sprintf(label, "%s-%d-drdy", chip->label, chip->id);
        ret = gpio_request_one(chip->drdy_gpio, GPIOF_DIR_IN, label);

        if (ret) {
                dev_err(&spi->dev, " DRDY gpio request failed.\n");

                return (ret);
        }
        sprintf(label, "%s-%d-cs", chip->label, chip->id);
        ret = gpio_request_one(chip->cs_gpio, GPIOF_INIT_HIGH, label);

        if (ret) {
                dev_err(&spi->dev, " CS gpio request failed.\n");

                return (ret);
        }
        spi->bits_per_word = 8;
        spi->mode          = SPI_MODE_0;
        spi->max_speed_hz  = 10000000ul;
        ret = spi_setup(spi);

        if (ret) {
                dev_err(&spi->dev, " SPI setup failed.\n");

                return (ret);
        }

        return (ret);
}



int ti_sd_term(struct ads1256_chip * chip)
{
        gpio_free(chip->spi->cs_gpio);
}

/**
 * ti_sd_setup_buffer_and_trigger()
 * @indio_dev - IIO device
 */
int ti_sd_setup_buffer_and_trigger(struct ads1256_chip * chip)
{
        int                         ret;

        /* TODO: setup circular buffer here (iio_triggered_buffer_setupp()) */

        if (ret) {
                return (ret);
        }
        ret = ti_sd_probe_trigger(sigma_delta);

        if (ret) {
                /* If failed: iio_triggered_buffer_cleanup() */

                return (ret);
        }

        return (0);
}
EXPORT_SYMBOL_GPL(ti_sd_setup_buffer_and_trigger);



/**
 * ti_sd_cleanup_buffer_and_trigger()
 * @indio_dev - IIO device
 */
void ti_sd_cleanup_buffer_and_trigger(struct ads1256_chip * chip)
{
        free_irq(chip->spi->irq, sigma_delta);
        /* TODO: iio_triggered_buffer_cleanup() */
}
EXPORT_SYMBOL_GPL(ti_sd_cleanup_buffer_and_trigger);



/**
 * ti_sd_write_reg() - Write a register
 *
 * @sigma_delta: The sigma delta device
 * @reg: Address of the registers
 * @size: Size of the register (1 - 4)
 * @val: Value to write to the register
 *
 * Returns 0 on success, an error code otherwise
 */
int ti_sd_write_reg(struct ads1256_chip * chip, uint32_t reg,
        uint32_t val)
{
        int                         ret;
        uint8_t *                   transfer_data;
        struct spi_transfer         transfer;
        struct spi_message          message;

        memset(&transfer, 0, sizeof(transfer));
        transfer_data       = chip->transfer_data;
        transfer.tx_buf     = transfer_data;
        transfer.len        = 3;          /* Add for 1st and 2nd command byte */
        transfer.cs_change  = chip->is_bus_locked;

        transfer_data[0] = ADS125X_CMD_WREG(reg);               /* command id */
        transfer_data[1] = 0;                     /* byte counter, 0 = 1 byte */
        transfer_data[2] = (uint8_t)val;
        spi_message_init(&message);
        spi_message_add_tail(&transfer, &message);

        if (chip->is_bus_locked) {
                ret = spi_sync_locked(chip->spi, &message);
        } else {
                ret = spi_sync(chip->spi, &message);
        }

        return (ret);
}
EXPORT_SYMBOL_GPL(ti_sd_write_reg);



/**
 * ti_sd_read_reg()
 * @sigma_delta: The sigma delta device
 * @reg: Address of the register
 * @val: Pointer to a buffer
 *
 * Returns 0 on success, an error code otherwise.
 */
int ti_sd_read_reg(struct ads1256_chip * chip, uint32_t reg,
    uint32_t * val)
{
    int                         ret;
    uint8_t *                   transfer_data;
    struct spi_transfer         transfer[2];
    struct spi_message          message;

    memset(transfer, 0, sizeof(transfer));
    transfer_data           = chip->transfer_data;
    transfer[0].tx_buf      = transfer_data;
    transfer[0].len         = 2;                  /* 1st and 2nd command byte */
    transfer[1].rx_buf      = val;
    transfer[1].len         = 1;
    transfer[1].cs_change   = chip->is_bus_locked;

    transfer_data[0] = ADS125X_CMD_RREG(reg);                   /* command id */
    transfer_data[1] = 0;                         /* byte counter, 0 = 1 byte */
    spi_message_init(&message);
    spi_message_add_tail(&transfer[0], &message);
    spi_message_add_tail(&transfer[1], &message);

    if (chip->is_bus_locked) {
        ret = spi_sync_locked(chip->spi, &message);
    } else {
        ret = spi_sync(chip->spi, &message);
    }

    return (ret);
}
EXPORT_SYMBOL_GPL(ti_sd_read_reg);



/**
 * ti_sd_self_calibrate()
 * @sigma_delta: The sigma delta device
 *
 * Returns 0 on success, an error code otherwise.
 */
int ti_sd_self_calibrate(struct ads1256_chip * chip)
{
    return (-ENOSYS);
}
EXPORT_SYMBOL_GPL(ti_sd_self_calibrate);



/**
 * ti_sd_set_mode()
 * @sigma_delta: The sigma delta device
 * @mode: Set mode to ADS125x_MODE_CONTINUOUS or ADS125x_MODE_IDLE
 *
 * Returns 0 on success, an error code otherwise.
 */
int ti_sd_set_mode(struct ads1256_chip * chip, uint32_t mode)
{
    int                         ret;
    uint32_t                    command;
    struct spi_transfer         transfer[2]; 
    struct spi_message          message;

    memset(transfer, 0, sizeof(transfer));
    transfer[0].tx_buf      = &command;
    transfer[0].len         = 1;
    transfer[0].cs_change   = chip->is_bus_locked;
    transfer[1].rx_buf      = chip->transfer_data;
    transfer[1].len         = 3;
    transfer[1].cs_change   = chip->is_bus_locked;

    spi_message_init(&message);

    switch (mode) {
        case ADS125X_MODE_CONTINUOUS:
            command = ADS125X_CMD_RDATAC; 
            spi_message_add_tail(&transfer[0], &message);
                /* Get and dump the first measurement */
            spi_message_add_tail(&transfer[1], &message); 
            break;
        case ADS125X_MODE_IDLE:
            command = ADS125X_CMD_SDATAC;
            spi_message_add_tail(&transfer[0], &message);
            break;
        default:
            return (-EINVAL);
    }

    if (chip->is_bus_locked) {
        ret = spi_sync_locked(chip->spi, &message);
    } else {
        ret = spi_sync(chip->spi, &message);
    }

    return (ret);
}
EXPORT_SYMBOL_GPL(ti_sd_set_mode);



/**
 * ti_sd_set_channel()
 * @sigma_delta: The sigma delta device
 * @channel: 0 - 7 channel id
 *
 * Returns 0 on success, an error code otherwise.
 */
int ti_sd_set_channel(struct ads1256_chip * chip, uint32_t channel)
{
    uint32_t                    reg_val;
    int                         ret;

    reg_val = (uint8_t)(channel << 4u) | (uint8_t)(channel & 0x0fu);

    ret = ti_sd_write_reg(chip, ADS125X_REG_MUX, reg_val);

    return (ret);
}
EXPORT_SYMBOL_GPL(ti_sd_set_channel);

MODULE_AUTHOR("Nenad Radulovic <nenad.b.radulovic@gmail.com>");
MODULE_DESCRIPTION("Texas Instruments ADS125x ADC core");
MODULE_LICENSE("GPL v2");
