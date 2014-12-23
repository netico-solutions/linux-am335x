/*
 * Texas Instruments AD1256 ADC driver core interface
 *
 * Author: Nenad Radulovic <nenad.b.radulovic@gmail.com>
 *
 * Licensed under GPL-v2
 */

#ifndef ADS1256_H_
#define ADS1256_H_

#include <linux/types.h>
#include <linux/completion.h>
#include <linux/ioctl.h>
#include <linux/spinlock.h>

/*--  IOCTL  ----------------------------------------------------------------*/

struct ads125x_mux {
        int                     positive;
        int                     negative;
};

#define ADS125X_MAGIC                           'x'
#define ADS125X_SET_MUX                                                 \
        _IOW(ADS125X_MAGIC, 100, struct ads125x_mux)
#define ADS125X_SET_BUF_SIZE                                            \
        _IOW(ADS125X_MAGIC, 101, int)
#define ADS125X_SELF_CALIBRATE                  _IO(ADS125X_MAGIC, 102)
#define ADS125X_START_SAMPLING                  _IO(ADS125X_MAGIC, 115)
#define ADS125X_STOP_SAMPLING                   _IO(ADS125X_MAGIC, 116)

/*--  Configuration  --------------------------------------------------------*/
#define ADS125X_NAME                            "ads1256"
#define ADS125X_CONFIG_SUPPORTED_CHIPS          4


#define ADS125X_ERR(msg, ...)                                           \
        printk(KERN_ERR "# " ADS125X_NAME ":" msg, ## __VA_ARGS__)
#define ADS125X_INF(msg, ...)                                           \
        printk(KERN_INFO "# " ADS125X_NAME ":" msg, ## __VA_ARGS__)
#define ADS125X_NOT(msg, ...)                                           \
        printk(KERN_NOTICE "# " ADS125X_NAME ":" msg, ## __VA_ARGS__)
#define ADS125X_WRN(msg, ...)                                           \
        printk(KERN_WARNING "# " ADS125X_NAME ":" msg, ## __VA_ARGS__)
#define ADS125X_DBG(msg, ...)                                           \
        printk(KERN_DEFAULT "# " ADS125X_NAME ":" msg, ## __VA_ARGS__)

struct ads125x_chip {
        struct ads125x_multi *  multi;
        struct completion       completion;
        struct spi_transfer     irq_transfer;
        struct spi_message      irq_message;
        uint8_t                 irq_data[16]
                ____cacheline_aligned;
        bool                    is_irq_enabled;
        int                     id;
        int                     cs_gpio;
        int                     drdy_gpio;
};

struct ads125x_sample {
        uint32_t                raw[ADS125X_CONFIG_SUPPORTED_CHIPS];
        union ads125x_sample_info {
                uint32_t                completed_chip;
                uint32_t                sequence;
        }                       info;
};

struct ads125x_ring {
        unsigned int            head;
        unsigned int            tail;
        unsigned int            mask;
        unsigned int            free;
        struct ads125x_sample * buff;
};

struct ads125x_ppbuf {
        spinlock_t              lock;
        struct completion       completion;
        unsigned int            completion_level;
        struct ads125x_ring *   producer;
        struct ads125x_ring *   consumer;
        struct ads125x_ring     ring[2];
};

struct ads125x_sched {
        spinlock_t              lock;
        unsigned int            ready;
        unsigned int            fired;
        bool                    is_busy;
};

struct ads125x_multi {
        spinlock_t              lock;
        struct ads125x_chip *   chip[ADS125X_CONFIG_SUPPORTED_CHIPS];
        struct ads125x_sched    sched;
        struct spi_device *     spi;
        struct ads125x_ppbuf    buff;
        unsigned int            enabled_chip;
        bool                    is_bus_locked;
};



int ads125x_probe_trigger(struct ads125x_chip * chip);
int ads125x_init_multi(struct ads125x_multi * multi, struct spi_device * spi,
                uint32_t enabled_chip_mask);
int ads125x_init_chip(struct ads125x_chip * chip, struct ads125x_multi * multi,
                int id, int cs_gpio, int drdy_gpio);
int ads125x_init_hw(struct ads125x_chip * chip);
void ads125x_term_multi(struct ads125x_multi * multi);
int ads125x_term_chip(struct ads125x_chip * chip);
void ads125x_term_hw(struct ads125x_chip * chip);
void ads125x_remove_trigger(struct ads125x_chip * chip);

static inline 
struct spi_device * multi_to_spi(const struct ads125x_multi * multi)
{
        return (multi->spi);
}

static inline
bool ads125x_multi_is_locked(struct ads125x_multi * multi)
{
        return (multi->is_bus_locked);
}
int ads125x_multi_lock(struct ads125x_multi* multi);
int ads125x_multi_unlock(struct ads125x_multi * multi);
int ads125x_multi_ring_set_size(struct ads125x_multi * multi, unsigned int size);

int ads125x_self_calibrate(struct ads125x_chip * chip);
int ads125x_set_mux(struct ads125x_chip * chip, uint8_t positive, 
                uint8_t negative);
int ads125x_buffer_enable(struct ads125x_chip * chip);
int ads125x_buffer_disable(struct ads125x_chip * chip);

#endif /* ADS1256_H_ */

