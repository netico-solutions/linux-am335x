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


struct ads125x_sample {
        uint32_t                raw_value;
};

static int chip_exchange_message_bl(struct ads125x_chip * chip, 
                struct spi_message * message);
static int chip_exchange_message_bu(struct ads125x_chip * chip, 
                struct spi_message * message);
static irqreturn_t chip_trigger_ready_handler(int irq, void * p);
static int chip_read_data_begin_al(struct ads125x_chip * chip);
static void chip_read_data_finish_al(void * arg);
static int chip_set_mode_bl(struct ads125x_chip * chip, uint32_t mode);


static int chip_exchange_message_bl(struct ads125x_chip * chip, 
                struct spi_message * message)
{
        int                     ret;
        gpio_set_value(chip->cs_gpio, SPI_CS_ACTIVE);
        ret = spi_sync_locked(chip->multi->spi, message);
        gpio_set_value(chip->cs_gpio, SPI_CS_INACTIVE);

        return (ret);
}



static int chip_exchange_message_bu(struct ads125x_chip * chip, 
                struct spi_message * message)
{
        int                     ret;
        gpio_set_value(chip->cs_gpio, SPI_CS_ACTIVE);
        ret = spi_sync(chip->multi->spi, message);
        gpio_set_value(chip->cs_gpio, SPI_CS_INACTIVE);

        return (ret);
}



static irqreturn_t chip_trigger_ready_handler(int irq, void * p)
{
        struct ads125x_chip *   chip = p;
        int                     ret;

        disable_irq_nosync(irq);
        ret = chip_read_data_begin_al(chip);
        enable_irq(irq);

        return (IRQ_HANDLED);
}



static int chip_read_data_begin_al(struct ads125x_chip * chip)
{
        memset(&chip->irq_transfer, 0, sizeof(chip->irq_transfer));
        chip->irq_transfer.rx_buf = chip->irq_data;
        chip->irq_transfer.len    = 3;             /* Samples are 24 bit wide */

        spi_message_init(&chip->irq_message);
        spi_message_add_tail(&chip->irq_transfer, &chip->irq_message);
        chip->irq_message.complete = &chip_read_data_finish_al;
        chip->irq_message.context  = chip;
        gpio_set_value(chip->cs_gpio, SPI_CS_ACTIVE);

        return (spi_async_locked(chip->multi->spi, &chip->irq_message));
}



static void chip_read_data_finish_al(void * arg)
{
        struct ads125x_chip *   chip = arg;

        gpio_set_value(chip->cs_gpio, SPI_CS_INACTIVE);

        /* push to buffers here */

        complete(&chip->completion);
}





static int chip_set_mode_bl(struct ads125x_chip * chip, uint32_t mode)
{
        struct spi_transfer     transfer[2];
        struct spi_message      message;
        uint8_t                 tx_buf[8];
        uint8_t                 rx_buf[8];

        memset(&transfer[0], 0, sizeof(transfer));
        transfer[0].tx_buf = tx_buf;
        transfer[0].len    = 1;
        transfer[1].rx_buf = rx_buf;
        transfer[1].len    = 3;

        spi_message_init(&message);

        switch (mode) {
                case ADS125X_MODE_CONTINUOUS:
                        tx_buf[0] = ADS125X_CMD_RDATAC; 
                        spi_message_add_tail(&transfer[0], &message);
                                        /* Get and dump the first measurement */
                        spi_message_add_tail(&transfer[1], &message); 
                        break;
                case ADS125X_MODE_IDLE:
                        tx_buf[0] = ADS125X_CMD_SDATAC;
                        spi_message_add_tail(&transfer[0], &message);
                        break;
                default:
                        return (-EINVAL);
        }

        return (chip_exchange_message_bl(chip, &message));
}

/*--  PUBLIC METHODS  --------------------------------------------------------*/


/**
 * ads125x_probe_trigger()
 * @chip: chip device
 *
 * Returns 0 on success, an error code otherwise
 */
int ads125x_probe_trigger(struct ads125x_chip * chip)
{
        int                     ret;
        char                    label[16];

        ret = gpio_to_irq(chip->drdy_gpio);

        if (ret < 0) {
                ADS125X_ERR(" trigger setup failed.\n");

                goto fail_gpio_irq;
        }
        init_completion(&chip->completion);
        sprintf(label, ADS125X_NAME "-%d-drdy-irq", chip->id);
        ret = request_irq(ret, &chip_trigger_ready_handler,
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
EXPORT_SYMBOL_GPL(ads125x_probe_trigger);



int ads125x_init_multi(struct ads125x_multi * multi, struct spi_device * spi,
                uint32_t enabled_chip_mask)
{
        int                     ret;

        spi->bits_per_word  = 8;
        spi->mode           = SPI_MODE_0;
        spi->max_speed_hz   = 10000000ul;

        ret = spi_setup(multi->spi);

        if (ret) {
                ADS125X_ERR("SPI setup failed\n");

                goto fail_spi_setup; 
        }
        /* setup fifo/ring buffer here */
        if (ret) {
                ADS125X_ERR("FIFO setup failed\n");

                goto fail_fifo_setup;
        }
        multi->spi           = spi;
        multi->is_bus_locked = false;
        multi->enabled       = enabled_chip_mask;

        return (0);
fail_fifo_setup:

fail_spi_setup:
        return (ret);
}
EXPORT_SYMBOL_GPL(ads125x_init_multi);


/**
 * ads125x_init_chip()
 * @chip: chip device
 *
 * Returns 0 on success, an error code otherwise
 */
int ads125x_init_chip(struct ads125x_chip * chip, struct ads125x_multi * multi,
                int id, int cs_gpio, int drdy_gpio)
{
        int                     ret;
        char                    label[16];

        chip->multi          = multi;
        chip->id             = id;
        chip->cs_gpio        = cs_gpio;
        chip->drdy_gpio      = drdy_gpio;
        chip->is_irq_enabled = false;
        sprintf(label, ADS125X_NAME "-%d-drdy", chip->id);
        ret = gpio_request_one(chip->drdy_gpio, GPIOF_DIR_IN, label);

        if (ret) {
                ADS125X_ERR("chip %d: DRDY gpio %d request failed\n",
                                chip->id, chip->drdy_gpio);

                goto fail_drdy_request;
        }
        ADS125X_INF("chip %d: DRDY gpio: %s\n", chip->id, label);
        sprintf(label, ADS125X_NAME "-%d-cs", chip->id);
        ret = gpio_request_one(chip->cs_gpio, GPIOF_INIT_HIGH, label);

        if (ret) {
                ADS125X_ERR("chip %d: CS gpio %d request failed\n",
                                chip->id, chip->cs_gpio);

                goto fail_cs_request; 
        }
        ADS125X_INF("chip %d: CS gpio: %s\n", chip->id, label);

        return (ret);
fail_cs_request:
        gpio_free(chip->drdy_gpio);
fail_drdy_request:

        return (ret);
}
EXPORT_SYMBOL_GPL(ads125x_init_chip);



/**
 * ads125x_init_hw()
 * @chip: chip device
 *
 * Returns 0 on success, an error code otherwise
 */
int ads125x_init_hw(struct ads125x_chip * chip)
{
        int                     ret;

        ret = ads125x_write_reg(chip, ADS125X_REG_STATUS, ADS125X_STATUS_ACAL);

        if (ret) {
                goto fail_write;
        }
        ret = ads125x_write_reg(chip, ADS125X_REG_ADCON,  0);

        if (ret) {
                goto fail_write;
        }
        ret = ads125x_write_reg(chip, ADS125X_REG_DRATE,  ADS125X_DRATE_10);

        if (ret) {
                goto fail_write;
        }
        ret = ads125x_write_reg(chip, ADS125X_REG_IO,     0);

        if (ret) {
                goto fail_write;
        }
        ret = ads125x_set_channel(chip, 2, 3);

fail_write:
        return (ret);
}
EXPORT_SYMBOL_GPL(ads125x_init_hw);



void ads125x_term_multi(struct ads125x_multi * multi)
{
        if (multi->spi) {
                multi->spi = NULL;
                kfifo_free(&multi->fifo);
        }
}
EXPORT_SYMBOL_GPL(ads125x_term_multi);



/**
 * ads125x_term_chip()
 * @chip: chip device
 *
 * Returns 0 on success, an error code otherwise
 */
int ads125x_term_chip(struct ads125x_chip * chip)
{
        gpio_free(chip->cs_gpio);
        gpio_free(chip->drdy_gpio);

        return (0);
}
EXPORT_SYMBOL_GPL(ads125x_term_chip);



void ads125x_term_hw(struct ads125x_chip * chip)
{
        /* TODO: stop and terminate HW here */
}
EXPORT_SYMBOL_GPL(ads125x_term_hw);



/**
 * ads125x_cleanup_buffer_and_trigger()
 * @chip: chip device
 */
void ads125x_remove_trigger(struct ads125x_chip * chip)
{
        disable_irq(gpio_to_irq(chip->drdy_gpio));
        free_irq(gpio_to_irq(chip->drdy_gpio), chip);
        /* TODO: iio_triggered_buffer_cleanup() */
}
EXPORT_SYMBOL_GPL(ads125x_remove_trigger);



/**
 * ads125x_write_reg() - Write a register
 *
 * @chip: The sigma delta device
 * @reg: Address of the registers
 * @val: Value to write to the register
 *
 * Returns 0 on success, an error code otherwise
 */
int ads125x_write_reg(struct ads125x_chip * chip, uint32_t reg, uint32_t val)
{
        struct spi_transfer     transfer;
        struct spi_message      message;
        uint8_t                 tx_buf[8];

        if (chip->multi->is_bus_locked) {
                return (-EBUSY);;
        }
        memset(&transfer, 0, sizeof(transfer));
        transfer.tx_buf = tx_buf;
        transfer.len    = 3;     /* Add for 1st and 2nd command byte */

        tx_buf[0] = ADS125X_CMD_WREG(reg);       /* command id */
        tx_buf[1] = 0;              /* byte counter, 0 = 1 reg */
        tx_buf[2] = (uint8_t)val;

        spi_message_init(&message);
        spi_message_add_tail(&transfer, &message);

        return (chip_exchange_message_bu(chip, &message));
}
EXPORT_SYMBOL_GPL(ads125x_write_reg);



/**
 * ads125x_read_reg()
 * @chip: The sigma delta device
 * @reg: Address of the register
 * @val: Pointer to a buffer
 *
 * Returns 0 on success, an error code otherwise.
 */
int ads125x_read_reg(struct ads125x_chip * chip, uint32_t reg, uint32_t * val)
{
        struct spi_transfer     transfer[2];
        struct spi_message      message;
        uint8_t                 tx_buf[8];

        if (chip->multi->is_bus_locked) {
                return (-EBUSY);
        }
        memset(&transfer[0], 0, sizeof(transfer));
        transfer[0].tx_buf = tx_buf;
        transfer[0].len    = 2;             /* 1st and 2nd command byte */
        transfer[1].rx_buf = val;
        transfer[1].len    = 1;

        tx_buf[0] = ADS125X_CMD_RREG(reg);       /* command id */
        tx_buf[1] = 0;             /* byte counter, 0 = 1 byte */
        spi_message_init(&message);
        spi_message_add_tail(&transfer[0], &message);
        spi_message_add_tail(&transfer[1], &message);

        return (chip_exchange_message_bu(chip, &message));
}
EXPORT_SYMBOL_GPL(ads125x_read_reg);



/**
 * ads125x_self_calibrate()
 * @chip: The sigma delta device
 *
 * Returns 0 on success, an error code otherwise.
 */
int ads125x_self_calibrate(struct ads125x_chip * chip)
{
        if (chip->multi->is_bus_locked) {
                return (-EBUSY);
        }

        return (-ENOSYS);
}
EXPORT_SYMBOL_GPL(ads125x_self_calibrate);



/**
 * ads125x_set_channel()
 * @chip: The sigma delta device
 * @positive: positive input channel
 * @negative: negative input channel
 *
 * Returns 0 on success, an error code otherwise.
 */
int ads125x_set_channel(struct ads125x_chip * chip, uint8_t positive, 
                uint8_t negative)
{
        uint32_t                reg_val;
        int                     ret;

        reg_val = (uint8_t)(positive << 4u) | (uint8_t)(negative & 0x0fu);

        ret = ads125x_write_reg(chip, ADS125X_REG_MUX, reg_val);

        return (ret);
}
EXPORT_SYMBOL_GPL(ads125x_set_channel);



int ads125x_buffer_enable(struct ads125x_chip * chip)
{
        int                     ret;

        spi_bus_lock(chip->multi->spi->master);
        chip->multi->is_bus_locked = true;
        init_completion(&chip->completion);

        ret = chip_set_mode_bl(chip, ADS125X_MODE_CONTINUOUS);

        if (ret) {
                goto fail_set_mode;
        }
        enable_irq(gpio_to_irq(chip->drdy_gpio));

        return (0);

fail_set_mode:
        spi_bus_unlock(chip->multi->spi->master);
    
        return (ret);
}
EXPORT_SYMBOL_GPL(ads125x_buffer_enable);



int ads125x_buffer_disable(struct ads125x_chip * chip)
{
        int                     ret;

        INIT_COMPLETION(chip->completion);
        wait_for_completion_timeout(&chip->completion, HZ);
    
        disable_irq_nosync(gpio_to_irq(chip->drdy_gpio));
        ret = chip_set_mode_bl(chip, ADS125X_MODE_IDLE);

        spi_bus_unlock(chip->multi->spi->master);
        chip->multi->is_bus_locked = false;

        return (ret);
}
EXPORT_SYMBOL_GPL(ads125x_buffer_disable);



MODULE_AUTHOR("Nenad Radulovic <nenad.b.radulovic@gmail.com>");
MODULE_DESCRIPTION("Texas Instruments ADS125x ADC core");
MODULE_LICENSE("GPL v2");
