/*
 * Support code for Texas Instruments Sigma-delta ADCs
 *
 * Author: Nenad Radulovic <nenad.b.radulovic@gmail.com>
 * 
 * Licensed under GPL-v2
 */

#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include <linux/log2.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/err.h>
#include <asm/bitops.h>
#include <asm/uaccess.h>

#include "ads1256.h"

#define CONFIG_FIFO_SIZE                1024

#define ADS125X_REG_STATUS                      0x00u
#define ADS125X_REG_MUX                         0x01u
#define ADS125X_REG_ADCON                       0x02u
#define ADS125X_REG_DRATE                       0x03u
#define ADS125X_REG_IO                          0x04u
#define ADS125X_REG_OFC0                        0x05u
#define ADS125X_REG_OFC1                        0x06u
#define ADS125X_REG_OFC2                        0x07u
#define ADS125X_REG_FSC0                        0x08u
#define ADS125X_REG_FSC1                        0x09u
#define ADS125X_REG_FSC2                        0x0au

#define ADS125X_STATUS_ID_Pos                   (5)
#define ADS125X_STATUS_ID_Msk                   (0x7u << ADS125X_STATUS_ID_Pos)
#define ADS125X_STATUS_ORDER                    (0x1u << 3)
#define ADS125X_STATUS_ACAL                     (0x1u << 2)
#define ADS125X_STATUS_BUFEN                    (0x1u << 1)
#define ADS125X_STATUS_DRDY                     (0x1u << 0)

#define ADS125X_MUX_PSEL_Pos                    (4)
#define ADS125X_MUX_PSEL_Msk                    (0xfu << ADS125X_MUX_PSEL_Pos)
#define ADS125X_MUX_NSEL_Pos                    (0)
#define ADS125X_MUX_NSEL_Msk                    (0xfu << ADS125X_MUX_NSEL_Pos)

#define ADS125X_ADCON_CLK_Pos                   (5)
#define ADS125X_ADCON_CLK_Msk                   (0x3u << ADS125X_ADCON_CLK_Pos)
#define ADS125X_ADCON_SDCS_Pos                  (3)
#define ADS125X_ADCON_SDCS_Msk                  (0x3u << ADS125X_ADCON_SDCS_Pos)
#define ADS125X_ADCON_PGA_Pos                   (0)
#define ADS125X_ADCON_PGA_Msk                   (0x7u << ADS125X_ADCON_PGA_Pos)

#define ADS125X_GPIO_DIO_Pos                    (0)
#define ADS125X_GPIO_DIO_Msk                    (0xfu << ADS125X_GPIO_DIO_Pos)
#define ADS125X_GPIO_DIR_Pos                    (4)
#define ADS125X_GPIO_DIR_Msk                    (0xfu << ADS125X_GPIO_DIR_Pos)

#define ADS125X_DRATE_10                        (0x23)

#define ADS125X_CHANNEL_0                       0x00u
#define ADS125X_CHANNEL_1                       0x01u
#define ADS125X_CHANNEL_2                       0x02u
#define ADS125X_CHANNEL_3                       0x03u
#define ADS125X_CHANNEL_4                       0x04u
#define ADS125X_CHANNEL_5                       0x05u
#define ADS125X_CHANNEL_6                       0x06u
#define ADS125X_CHANNEL_7                       0x07u
#define ADS125X_CHANNEL_AINCOM                  0x08u

#define ADS125X_CMD_WAKEUP                      0x00u
#define ADS125X_CMD_RDATA                       0x01u
#define ADS125X_CMD_RDATAC                      0x03u
#define ADS125X_CMD_SDATAC                      0x0fu
#define ADS125X_CMD_RREG(reg)                   (0x10 | (reg))
#define ADS125X_CMD_WREG(reg)                   (0x50 | (reg))
#define ADS125X_CMD_SELFCAL                     0xf0u
#define ADS125X_CMD_SELFOCAL                    0xf1u
#define ADS125X_CMD_SELFFCAL                    0xf2u
#define ADS125X_CMD_SYSOCAL                     0xf3u
#define ADS125X_CMD_SYSGCAL                     0xf4u
#define ADS125X_CMD_SYNC                        0xfcu
#define ADS125X_CMD_STANDBY                     0xfdu
#define ADS125X_CMD_RESET                       0xfeu

#define ADS125X_MODE_CONTINUOUS                 0
#define ADS125X_MODE_IDLE                       1

#define SPI_CS_ACTIVE                           0
#define SPI_CS_INACTIVE                         1


static int ring_init(struct ads125x_ring * ring, unsigned int elements);

static int chip_exchange_blocking(struct ads125x_chip * chip, 
                struct spi_message * message);
static irqreturn_t chip_trigger_ready_handler(int irq, void * p);
static int chip_read_data_begin_al(struct ads125x_chip * chip);
static void chip_read_data_finish_al(void * arg);
static int chip_set_mode_bl(struct ads125x_chip * chip, uint32_t mode);

int g_log_level = 5;
EXPORT_SYMBOL_GPL(g_log_level);

/*--  RING  -----------------------------------------------------------------*/


static int ring_init(struct ads125x_ring * ring, unsigned int elements)
{
        if (!is_power_of_2(elements)) {
                ADS125X_ERR("ring_init(): elements is not power of 2: %d",
                                elements);

                return (-EINVAL);
        }
        ring->head = 0;
        ring->tail = 0;
        ring->mask = elements - 1;
        ring->free = elements - 1;
        ring->buff = kzalloc(sizeof(*ring->buff) * elements, GFP_KERNEL);

        if (!ring->buff) {
                return (-ENOMEM);
        }

        return (0);
}



static void ring_term(struct ads125x_ring * ring)
{
        kfree(ring->buff);
        ring->free = 0;
        ring->mask = 0;
        ring->buff = NULL;
}



static void ring_restart(struct ads125x_ring * ring)
{
        ring->head = 0;
        ring->tail = 0;
        ring->free = ring->mask;
}



static unsigned int ring_occupied(const struct ads125x_ring * ring)
{
        return (ring->mask - ring->free);
}



static unsigned int ring_size(const struct ads125x_ring * ring)
{
        return (ring->mask);
}



static struct ads125x_sample * ring_current_item(
                const struct ads125x_ring * ring)
{
        return (&ring->buff[ring->tail]);
}



static void ring_put_item(struct ads125x_ring * ring)
{
        ring->tail++;
        ring->tail &= ring->mask;

        if (ring->free) {
                ring->free--;
        } else {
                /* overwrite last entry */
                ring->head++;
                ring->head &= ring->mask;
        }
}



static struct ads125x_sample * ring_get_item(struct ads125x_ring * ring)
{
        struct ads125x_sample * sample;

        sample = &ring->buff[ring->head++];
        ring->head &= ring->mask;
        ring->free++;

        return (sample);
}

/*--  PPBUF  ----------------------------------------------------------------*/


static int ppbuf_init(struct ads125x_ppbuf * buff, unsigned int elements)
{
        int                     ret;

        init_completion(&buff->completion);
        buff->completion_level = 0;
        buff->producer         = &buff->ring[0];
        buff->consumer         = &buff->ring[1];

        ret = ring_init(buff->producer, elements);

        if (ret) {
                return (ret);
        }
        ret = ring_init(buff->consumer, elements);

        if (ret) {
                ring_term(buff->producer);
        }

        return (ret);
}



static void ppbuf_term(struct ads125x_ppbuf * buff)
{
        ring_term(buff->producer);
        ring_term(buff->consumer);
}



static unsigned int ppbuf_consumer_occupied(const struct ads125x_ppbuf * buff)
{
        return (ring_occupied(buff->consumer));
}



static unsigned int ppbuf_size(const struct ads125x_ppbuf * buff)
{
        return (ring_size(buff->producer) * 2u);
}



static void ppbuf_swap_i(struct ads125x_ppbuf * buff)
{
        struct ads125x_ring *   tmp;

        tmp            = buff->consumer;
        buff->consumer = buff->producer;
        buff->producer = tmp;

        ring_restart(buff->producer);
}



static struct ads125x_sample * ppbuf_current_item_i(struct ads125x_ppbuf * buff)
{
        return (ring_current_item(buff->producer));
}



/**
 * Put item to producer queue
 *
 * If there is a completion_level set it will notify when _producer_ queue size
 * is greater or equal to it.
 */
static void ppbuf_put_item_i(struct ads125x_ppbuf * buff)
{
        ring_put_item(buff->producer);

        if (buff->completion_level) {
                if (buff->completion_level >= ring_occupied(buff->producer)) {
                        buff->completion_level = 0;
                        ppbuf_swap_i(buff);
                        complete(&buff->completion);
                }
        }
}



static struct ads125x_sample * ppbuf_get_item(struct ads125x_ppbuf * buff)
{
        return (ring_get_item(buff->consumer));
}



static void ppbuf_set_completion_i(struct ads125x_ppbuf * buff, 
                unsigned int completion_level)
{
        INIT_COMPLETION(buff->completion);
        buff->completion_level = completion_level;
}



static int ppbuf_wait_complete_timed(struct ads125x_ppbuf * buff, 
                unsigned long timeout)
{
        return (wait_for_completion_timeout(&buff->completion, timeout));
}


/*--  SPI scheduler  --------------------------------------------------------*/


static void spi_scheduler_init(struct ads125x_multi * multi)
{
        multi->sched.ready   = 0;
        multi->sched.fired   = 0;
        multi->sched.is_busy = false;
}



static unsigned int spi_reschedule_i(struct ads125x_multi * multi)
{
        unsigned int            ready;
        unsigned int            ret = 0;

        if (!multi->sched.is_busy) {
                ready = multi->sched.ready & ~multi->sched.fired;
                ret   = ffs(ready);

                if (ret) {
                        multi->sched.fired  |= 0x1u << (ret - 1u);
                        multi->sched.is_busy = true;
                }
        }

        return (ret);
}



static unsigned int spi_schedule_i(struct ads125x_chip * chip)
{
        struct ads125x_multi *  multi = chip->multi;
        unsigned int            chip_id_no;

        multi->sched.ready |= 0x1u << chip->id;
        chip_id_no          = spi_reschedule_i(multi);

        return (chip_id_no);
}



static unsigned int spi_schedule_block_i(struct ads125x_chip * chip)
{
        struct ads125x_multi *  multi = chip->multi;
        unsigned int            chip_id_no;

        multi->sched.ready  &= ~(0x1u << chip->id);
        multi->sched.is_busy = false;
        chip_id_no           = spi_reschedule_i(multi);

        return (chip_id_no);
}


static void spi_scheduler_restart(struct ads125x_multi * multi)
{
        multi->sched.fired = 0;
}

/*--  SPI  ------------------------------------------------------------------*/


static int chip_exchange_blocking(struct ads125x_chip * chip, 
                struct spi_message * message)
{
        int                     ret;
        gpio_set_value(chip->cs_gpio, SPI_CS_ACTIVE);

        if (chip->multi->is_bus_locked) {
                ret = spi_sync_locked(chip->multi->spi, message);
        } else {
                ret = spi_sync(chip->multi->spi, message);
        }
        gpio_set_value(chip->cs_gpio, SPI_CS_INACTIVE);

        return (ret);
}



static irqreturn_t chip_trigger_ready_handler(int irq, void * p)
{
        struct ads125x_chip *   chip = p;
        struct ads125x_multi *  multi = chip->multi;
        unsigned long           flags;
        unsigned int            chip_id_no;

        disable_irq_nosync(irq);
        spin_lock_irqsave(&multi->lock, flags);
        chip_id_no = spi_schedule_i(chip);
        spin_unlock_irqrestore(&multi->lock, flags);

        if (chip_id_no) {
                chip_read_data_begin_al(chip->multi->chip[chip_id_no - 1u]);
        }
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
        struct ads125x_multi *  multi = chip->multi;
        struct ads125x_ppbuf *  buff  = &multi->buff;
        struct ads125x_sample * current_sample;
        unsigned long           flags;
        unsigned int            chip_id_no;

        gpio_set_value(chip->cs_gpio, SPI_CS_INACTIVE);

        spin_lock_irqsave(&multi->lock, flags);
        current_sample = ppbuf_current_item_i(buff);
        current_sample->info.completed_chip |= (0x1u << chip->id);
        current_sample->raw[chip->id]        = (chip->irq_data[2] << 16) | 
                                               (chip->irq_data[1] <<  8) |
                                               (chip->irq_data[0] <<  0);

        if (current_sample->info.completed_chip == multi->enabled_chips) {
                /* At this point all relevant chips were sampled */
                ppbuf_put_item_i(buff);
                ppbuf_current_item_i(buff)->info.completed_chip = 0;
                spi_scheduler_restart(multi);
        }
        chip_id_no = spi_schedule_block_i(chip);
        spin_unlock_irqrestore(&multi->lock, flags);

        if (chip_id_no) {
                chip_read_data_begin_al(chip->multi->chip[chip_id_no - 1u]);
        }
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

        return (chip_exchange_blocking(chip, &message));
}



static int ads125x_write_reg(struct ads125x_chip * chip, uint32_t reg, 
                uint32_t val)
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

        return (chip_exchange_blocking(chip, &message));
}



static int ads125x_read_reg(struct ads125x_chip * chip, uint32_t reg, 
                uint32_t * val)
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

        return (chip_exchange_blocking(chip, &message));
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
        char                    label[64];

        ret = gpio_to_irq(chip->drdy_gpio);

        if (ret < 0) {
                ADS125X_ERR("trigger setup failed, err: %d.\n", ret);

                goto fail_gpio_irq;
        }
        init_completion(&chip->completion);
        sprintf(label, ADS125X_NAME "-%d-drdy-irq", chip->id);
        ADS125X_INF("chip %d: probe_trigger(): requesting trigger IRQ: %d, name: %s\n",
                        chip->id, ret, label);
        ret = request_irq(ret, &chip_trigger_ready_handler,
                IRQF_TRIGGER_FALLING, label, chip);

        if (ret) {
                ADS125X_ERR("trigger IRQ setup failed, err: %d.\n", ret);

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
                uint32_t enabled_chips_mask)
{
        int                     ret;

        spi->bits_per_word  = 8;
        spi->mode           = SPI_MODE_0;
        spi->max_speed_hz   = 10000000ul;
        ADS125X_INF("multi: setup spi\n");
        ret = spi_setup(spi);

        if (ret) {
                ADS125X_ERR("SPI setup failed\n");

                goto fail_spi_setup; 
        }
        ADS125X_INF("multi: setup buffers, size: 2x%d samples\n", 
                        ADS125X_CONFIG_BUFFER_SIZE);
        ret = ppbuf_init(&multi->buff, ADS125X_CONFIG_BUFFER_SIZE);

        if (ret) {
                ADS125X_ERR("buffer setup failed\n");

                goto fail_fifo_setup;
        }
        multi->spi           = spi;
        multi->is_bus_locked = false;
        multi->enabled_chips = enabled_chips_mask;
        ADS125X_INF("initializing SPI scheduler\n");
        spi_scheduler_init(multi);

        return (ret);
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

        ADS125X_INF("chip %d: enable autocalibration\n", chip->id);
        ret = ads125x_write_reg(chip, ADS125X_REG_STATUS, ADS125X_STATUS_ACAL);

        if (ret) {
                goto fail_write;
        }
        ret = ads125x_write_reg(chip, ADS125X_REG_ADCON,  0);

        if (ret) {
                goto fail_write;
        }
        ADS125X_INF("chip %d: setup refresh rate\n", chip->id);
        ret = ads125x_write_reg(chip, ADS125X_REG_DRATE,  ADS125X_DRATE_10);

        if (ret) {
                goto fail_write;
        }
        ADS125X_INF("chip %d: initialize on chip GPIO\n", chip->id);
        ret = ads125x_write_reg(chip, ADS125X_REG_IO,     0);

        if (ret) {
                goto fail_write;
        }

fail_write:
        return (ret);
}
EXPORT_SYMBOL_GPL(ads125x_init_hw);



void ads125x_term_multi(struct ads125x_multi * multi)
{
        if (multi->spi) {
                multi->spi = NULL;
                ppbuf_term(&multi->buff);
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
}
EXPORT_SYMBOL_GPL(ads125x_remove_trigger);



int ads125x_set_log_level(int level)
{
        if (level > 5) {
                return (-EINVAL);
        }

        if (level < 0) {
                return (-EINVAL);
        }
        g_log_level = level;

        return (0);
}
EXPORT_SYMBOL_GPL(ads125x_set_log_level);



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
int ads125x_set_mux(struct ads125x_chip * chip, uint8_t positive, 
                uint8_t negative)
{
        uint32_t                reg_val;
        int                     ret;

        reg_val = (uint8_t)(positive << 4u) | (uint8_t)(negative & 0x0fu);

        ret = ads125x_write_reg(chip, ADS125X_REG_MUX, reg_val);

        return (ret);
}
EXPORT_SYMBOL_GPL(ads125x_set_mux);



int ads125x_multi_lock(struct ads125x_multi* multi)
{
        if (multi->is_bus_locked) {
                return (-EBUSY);
        }
        spi_bus_lock(multi->spi->master);
        multi->is_bus_locked = true;

        return (0);
}
EXPORT_SYMBOL_GPL(ads125x_multi_lock);



int ads125x_multi_unlock(struct ads125x_multi * multi)
{
        if (!multi->is_bus_locked) {
                return (-EBUSY);
        }
        spi_bus_unlock(multi->spi->master);
        multi->is_bus_locked = false;

        return (0);
}
EXPORT_SYMBOL_GPL(ads125x_multi_unlock);



int ads125x_multi_ring_set_size(struct ads125x_multi * multi, unsigned int size)
{
        int                     ret;

        if (!multi->is_bus_locked) {
                return (-EBUSY);
        }
        ppbuf_term(&multi->buff);
        ADS125X_INF("multi set buffer size: setting new size to 2x%d samples\n",
                        size);
        ret = ppbuf_init(&multi->buff, size);

        if (ret) {
                ADS125X_ERR("multi set buffer size: failed, err: %d\n",
                                ret);
        }

        return (ret);
}
EXPORT_SYMBOL_GPL(ads125x_multi_ring_set_size);



size_t ads125x_multi_ring_get_size(struct ads125x_multi * multi)
{
        return (ppbuf_size(&multi->buff));
}
EXPORT_SYMBOL_GPL(ads125x_multi_ring_get_size);



ssize_t ads125x_multi_ring_get_items(struct ads125x_multi * multi,
                char * buff, size_t count, unsigned long timeout)
{
        struct ads125x_sample * sample_buff = (struct ads125x_sample *)buff;
        unsigned int            transfer_pending;
        unsigned int            transfer;
        unsigned int            transfer_count;
        unsigned int            transferred;

        if ((count % (sizeof(struct ads125x_sample)) != 0)) {
                ADS125X_ERR("size of user buffer not aligned with sample" 
                                "structure\n");
                return (-EINVAL);
        }
        transfer_pending = count / sizeof(struct ads125x_sample);

        if (transfer_pending > ppbuf_size(&multi->buff)) {
                transfer_pending = ppbuf_size(&multi->buff);
                ADS125X_WRN("requested samples more than size of buffers\n");
        }
        transfer = transfer_pending;

        if (transfer > ppbuf_consumer_occupied(&multi->buff)) {
                transfer = ppbuf_consumer_occupied(&multi->buff);
        }

        for (transferred = 0, transfer_count = 0; 
                        transfer_count < transfer; transfer_count++) {
                struct ads125x_sample * sample;

                sample = ppbuf_get_item(&multi->buff);

                if (copy_to_user(sample_buff++, sample, sizeof(*sample))) {
                        ADS125X_ERR("copy_to_user() failed\n");

                        return (-EINVAL);
                }
                transferred++;
        }
        transfer_pending -= transferred;

        if (transfer_pending) {
                unsigned int    ret;
                unsigned long   flags;

                spin_lock_irqsave(&multi->lock, flags);
                ppbuf_set_completion_i(&multi->buff, transfer_pending);
                spin_unlock_irqrestore(&multi->lock, flags);
                ret = ppbuf_wait_complete_timed(&multi->buff, timeout);

                if (ret) {
                        ADS125X_WRN("timeout for buffer complete\n");

                        return (-ETIMEDOUT);
                }

                for (transfer_count = 0; transfer_count < transfer_pending; 
                                transfer_count++) {
                        struct ads125x_sample * sample;

                        sample = ppbuf_get_item(&multi->buff);

                        if (copy_to_user(sample_buff++, sample, sizeof(*sample))) {
                                ADS125X_ERR("copy_to_user() failed\n");

                                return (-EINVAL);
                        }
                        transferred++;
                }
        }

        return (transferred);
}
EXPORT_SYMBOL_GPL(ads125x_multi_ring_get_items);
        


int ads125x_buffer_enable(struct ads125x_chip * chip)
{
        int                     ret;

        ADS125X_INF("enabling continuous mode\n");
        init_completion(&chip->completion);

        ret = chip_set_mode_bl(chip, ADS125X_MODE_CONTINUOUS);

        if (ret) {
                return (ret);
        }
        enable_irq(gpio_to_irq(chip->drdy_gpio));

        return (0);
}
EXPORT_SYMBOL_GPL(ads125x_buffer_enable);



int ads125x_buffer_disable(struct ads125x_chip * chip)
{
        int                     ret;

        ADS125X_INF("disabling continuous mode\n");
        INIT_COMPLETION(chip->completion);
        wait_for_completion_timeout(&chip->completion, HZ);
    
        disable_irq_nosync(gpio_to_irq(chip->drdy_gpio));
        ret = chip_set_mode_bl(chip, ADS125X_MODE_IDLE);

        return (ret);
}
EXPORT_SYMBOL_GPL(ads125x_buffer_disable);



MODULE_AUTHOR("Nenad Radulovic <nenad.b.radulovic@gmail.com>");
MODULE_DESCRIPTION("Texas Instruments ADS125x ADC core");
MODULE_LICENSE("GPL v2");
