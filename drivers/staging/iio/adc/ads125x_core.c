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

static int ti_sd_buffer_postenable(struct iio_dev *);
static int ti_sd_buffer_postdisable(struct iio_dev *);
static int ti_sd_read_data(struct ti_sigma_delta *, uint8_t *);

static const struct iio_buffer_setup_ops ti_sd_buffer_setup_ops =
{
    .preenable          = &iio_sw_buffer_preenable,
    .postenable         = &ti_sd_buffer_postenable,
    .predisable         = &iio_triggered_buffer_predisable,
    .postdisable        = &ti_sd_buffer_postdisable,
    .validate_scan_mask = &iio_validate_scan_mask_onehot
};

static const struct iio_trigger_ops ti_sd_trigger_ops =
{
    .owner              = THIS_MODULE,
};

static irqreturn_t ti_sd_trigger_handler(int irq, void * p)
{
    struct iio_poll_func *      pf;
    struct iio_dev *            indio_dev;
    struct ti_sigma_delta *     sigma_delta;
    uint8_t                     data[16];
    int                         retval;

    pf          = p;
    indio_dev   = pf->indio_dev;
    sigma_delta = iio_device_get_drvdata(indio_dev);

    if (indio_dev->scan_timestamp) {
        /* Guaranteed to be aligned with 8 byte boundary */
        ((int64_t *)data)[1] = pf->timestamp;
    }
    retval = ti_sd_read_data(sigma_delta, &data[0]);

    if (retval == 0) {
        iio_push_to_buffers(indio_dev, data);
        iio_trigger_notify_done(indio_dev->trig);
    }
    sigma_delta->is_irq_dis = false;
    enable_irq(sigma_delta->spi->irq);

    return (IRQ_HANDLED);
}

static irqreturn_t ti_sd_trigger_ready_handler(int irq, void * p)
{
    struct ti_sigma_delta *     sigma_delta;

    sigma_delta   = p;

    complete(&sigma_delta->completion);
    disable_irq_nosync(irq);
    sigma_delta->is_irq_dis = true;
    iio_trigger_poll(sigma_delta->trigger, iio_get_time_ns());

    return (IRQ_HANDLED);
}

static int ti_sd_buffer_postenable(struct iio_dev * indio_dev)
{
    struct ti_sigma_delta *     sigma_delta;
    uint32_t                    channel;
    int                         retval;

    retval = iio_triggered_buffer_postenable(indio_dev);
    sigma_delta = iio_device_get_drvdata(indio_dev);

    if (retval < 0) {
        return (retval);
    }
    channel = find_first_bit(indio_dev->active_scan_mask, 
            indio_dev->masklength); 
    retval  = ti_sd_set_channel(sigma_delta, 
            indio_dev->channels[channel].address);

    if (retval) {
        goto FAIL_SET_CHANNEL;
    }
    spi_bus_lock(sigma_delta->spi->master);
    sigma_delta->is_bus_locked = true;
    retval  = ti_sd_set_mode(sigma_delta, ADS125X_MODE_CONTINUOUS);

    if (retval) {
        goto FAIL_SET_MODE;
    }
    sigma_delta->is_irq_dis = false;
    enable_irq(sigma_delta->spi->irq);

    return (0);

FAIL_SET_MODE:
    spi_bus_unlock(sigma_delta->spi->master);
FAIL_SET_CHANNEL:
    
    return (retval);
}

static int ti_sd_buffer_postdisable(struct iio_dev * indio_dev)
{
    struct ti_sigma_delta *     sigma_delta;
    int                         retval;

    sigma_delta = iio_device_get_drvdata(indio_dev);

    INIT_COMPLETION(sigma_delta->completion);
    wait_for_completion_timeout(&sigma_delta->completion, HZ);
    
    if (!sigma_delta->is_irq_dis) {
        disable_irq_nosync(sigma_delta->spi->irq);
        sigma_delta->is_irq_dis = true;
    }
    retval = ti_sd_set_mode(sigma_delta, ADS125X_MODE_IDLE);

    if (retval) {
        goto FAIL_SET_MODE;
    }
    retval = spi_bus_unlock(sigma_delta->spi->master);
    sigma_delta->is_bus_locked = false;

    return (retval);

FAIL_SET_MODE:
    spi_bus_unlock(sigma_delta->spi->master);
    sigma_delta->is_bus_locked = false;

    return (retval);
}

static int ti_sd_probe_trigger(struct iio_dev * indio_dev)
{
   struct ti_sigma_delta *      sigma_delta;
   int                          retval;

   sigma_delta = iio_device_get_drvdata(indio_dev);

   sigma_delta->trigger = iio_trigger_alloc("%s-dev%d", indio_dev->name,
           indio_dev->id);

    if (sigma_delta->trigger == NULL) {
        retval = -ENOMEM;

        goto FAIL_TRIGG_ALLOC;
    }
    sigma_delta->trigger->ops = &ti_sd_trigger_ops;
    init_completion(&sigma_delta->completion);
    retval = request_irq(sigma_delta->spi->irq, &ti_sd_trigger_ready_handler,
            IRQF_TRIGGER_LOW, indio_dev->name, sigma_delta);

    if (retval) {
        goto FAIL_IRQ_REQUEST;
    }

    if (!sigma_delta->is_irq_dis) {
        sigma_delta->is_irq_dis = true;
        disable_irq_nosync(sigma_delta->spi->irq);
    }
    sigma_delta->trigger->dev.parent = &sigma_delta->spi->dev;
    iio_trigger_set_drvdata(sigma_delta->trigger, sigma_delta);
    retval = iio_trigger_register(sigma_delta->trigger);

    if (retval) {
        goto FAIL_TRIGGER_REGISTER;
    }
    /* select default trigger */
    indio_dev->trig = sigma_delta->trigger;

    return (0);
FAIL_TRIGGER_REGISTER:
    free_irq(sigma_delta->spi->irq, sigma_delta);
FAIL_IRQ_REQUEST:
    iio_trigger_free(sigma_delta->trigger);
FAIL_TRIGG_ALLOC:

    return (retval);
}

static int ti_sd_read_data(struct ti_sigma_delta * sigma_delta, uint8_t * val)
{
    int                         retval;
    uint8_t *                   transfer_data;
    struct spi_transfer         transfer;
    struct spi_message          message;

    memset(&transfer, 0, sizeof(transfer));
    transfer_data       = sigma_delta->transfer_data;
    transfer.rx_buf     = transfer_data;
    transfer.len        = 3;                       /* Samples are 24 bit wide */
    transfer.cs_change  = sigma_delta->is_bus_locked;

    spi_message_init(&message);
    spi_message_add_tail(&transfer, &message);

    if (sigma_delta->is_bus_locked) {
        retval = spi_sync_locked(sigma_delta->spi, &message);
    } else {
        retval = spi_sync(sigma_delta->spi, &message);
    }

    return (retval);
}

/*--  PUBLIC METHODS  --------------------------------------------------------*/

/**
 * ti_sd_init_sigma_delta()
 * @sigma_delta: The sigma delta device
 * @indio_dev: IIO device
 * @spi: SPI device
 */
void ti_sd_init(struct ti_sigma_delta * sigma_delta, 
        struct iio_dev * indio_dev, struct spi_device * spi)
{
    sigma_delta->spi = spi;
    iio_device_set_drvdata(indio_dev, sigma_delta);
}

/**
 * ti_sd_setup_buffer_and_trigger()
 * @indio_dev - IIO device
 */
int ti_sd_setup_buffer_and_trigger(struct iio_dev * indio_dev)
{
    int                         retval;

    retval = iio_triggered_buffer_setup(indio_dev, &iio_pollfunc_store_time,
            &ti_sd_trigger_handler, &ti_sd_buffer_setup_ops);

    if (retval) {
        return (retval);
    }
    retval = ti_sd_probe_trigger(indio_dev);

    if (retval) {
        iio_triggered_buffer_cleanup(indio_dev);

        return (retval);
    }

    return (0);
}
EXPORT_SYMBOL_GPL(ti_sd_setup_buffer_and_trigger);

/**
 * ti_sd_cleanup_buffer_and_trigger()
 * @indio_dev - IIO device
 */
void ti_sd_cleanup_buffer_and_trigger(struct iio_dev * indio_dev)
{
    struct ti_sigma_delta *     sigma_delta;

    sigma_delta = iio_device_get_drvdata(indio_dev);
    iio_trigger_unregister(sigma_delta->trigger);
    free_irq(sigma_delta->spi->irq, sigma_delta);
    iio_trigger_free(sigma_delta->trigger);
    iio_triggered_buffer_cleanup(indio_dev);
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
int ti_sd_write_reg(struct ti_sigma_delta * sigma_delta, uint32_t reg,
    uint32_t val)
{
    int                         retval;
    uint8_t *                   transfer_data;
    struct spi_transfer         transfer;
    struct spi_message          message;

    memset(&transfer, 0, sizeof(transfer));
    transfer_data       = sigma_delta->transfer_data;
    transfer.tx_buf     = transfer_data;
    transfer.len        = 3;              /* Add for 1st and 2nd command byte */
    transfer.cs_change  = sigma_delta->is_bus_locked;

    transfer_data[0] = ADS125X_CMD_WREG(reg);                   /* command id */
    transfer_data[1] = 0;                         /* byte counter, 0 = 1 byte */
    transfer_data[2] = (uint8_t)val;
    spi_message_init(&message);
    spi_message_add_tail(&transfer, &message);

    if (sigma_delta->is_bus_locked) {
        retval = spi_sync_locked(sigma_delta->spi, &message);
    } else {
        retval = spi_sync(sigma_delta->spi, &message);
    }

    return (retval);
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
int ti_sd_read_reg(struct ti_sigma_delta * sigma_delta, uint32_t reg,
    uint32_t * val)
{
    int                         retval;
    uint8_t *                   transfer_data;
    struct spi_transfer         transfer[2];
    struct spi_message          message;

    memset(transfer, 0, sizeof(transfer));
    transfer_data           = sigma_delta->transfer_data;
    transfer[0].tx_buf      = transfer_data;
    transfer[0].len         = 2;                  /* 1st and 2nd command byte */
    transfer[1].rx_buf      = val;
    transfer[1].len         = 1;
    transfer[1].cs_change   = sigma_delta->is_bus_locked;

    transfer_data[0] = ADS125X_CMD_RREG(reg);                   /* command id */
    transfer_data[1] = 0;                         /* byte counter, 0 = 1 byte */
    spi_message_init(&message);
    spi_message_add_tail(&transfer[0], &message);
    spi_message_add_tail(&transfer[1], &message);

    if (sigma_delta->is_bus_locked) {
        retval = spi_sync_locked(sigma_delta->spi, &message);
    } else {
        retval = spi_sync(sigma_delta->spi, &message);
    }

    return (retval);
}
EXPORT_SYMBOL_GPL(ti_sd_read_reg);

/**
 * ti_sd_self_calibrate()
 * @sigma_delta: The sigma delta device
 *
 * Returns 0 on success, an error code otherwise.
 */
int ti_sd_self_calibrate(struct ti_sigma_delta * sigma_delta)
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
int ti_sd_set_mode(struct ti_sigma_delta * sigma_delta, uint32_t mode)
{
    int                         retval;
    uint32_t                    command;
    struct spi_transfer         transfer[2]; 
    struct spi_message          message;

    memset(transfer, 0, sizeof(transfer));
    transfer[0].tx_buf      = &command;
    transfer[0].len         = 1;
    transfer[0].cs_change   = sigma_delta->is_bus_locked;
    transfer[1].rx_buf      = sigma_delta->transfer_data;
    transfer[1].len         = 3;
    transfer[1].cs_change   = sigma_delta->is_bus_locked;

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

    if (sigma_delta->is_bus_locked) {
        retval = spi_sync_locked(sigma_delta->spi, &message);
    } else {
        retval = spi_sync(sigma_delta->spi, &message);
    }

    return (retval);
}
EXPORT_SYMBOL_GPL(ti_sd_set_mode);

/**
 * ti_sd_set_channel()
 * @sigma_delta: The sigma delta device
 * @channel: 0 - 7 channel id
 *
 * Returns 0 on success, an error code otherwise.
 */
int ti_sd_set_channel(struct ti_sigma_delta * sigma_delta, uint32_t channel)
{
    uint32_t                    reg_val;
    int                         retval;

    reg_val = (uint8_t)(channel << 4u) | (uint8_t)(channel & 0x0fu);

    retval = ti_sd_write_reg(sigma_delta, ADS125X_REG_MUX, reg_val);

    return (retval);
}
EXPORT_SYMBOL_GPL(ti_sd_set_channel);

MODULE_AUTHOR("Nenad Radulovic <nenad.b.radulovic@gmail.com>");
MODULE_DESCRIPTION("Texas Instruments ADS125x ADC core");
MODULE_LICENSE("GPL v2");
