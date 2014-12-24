/*
 * Texas Instruments AD1256 ADC driver core interface
 *
 * Author: Nenad Radulovic <nenad.b.radulovic@gmail.com>
 *
 * Licensed under GPL-v2
 */

#ifndef ADS1256_H_
#define ADS1256_H_

#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/completion.h>
#include <linux/spinlock.h>
#include <misc/ads1256.h>

#define ADS125X_ERR(msg, ...)                                           \
        do {                                                            \
                if (g_log_level > 0) {                                  \
                        printk(KERN_ERR ADS125X_NAME " error: " msg,    \
                                ## __VA_ARGS__);                        \
                }                                                       \
        } while (0)

#define ADS125X_INF(msg, ...)                                           \
        do {                                                            \
                if (g_log_level > 3) {                                  \
                        printk(KERN_INFO ADS125X_NAME " info: " msg,    \
                                ## __VA_ARGS__);                        \
                }                                                       \
        } while (0)

#define ADS125X_NOT(msg, ...)                                           \
        do {                                                            \
                if (g_log_level > 2) {                                  \
                        printk(KERN_NOTICE  ADS125X_NAME ": " msg,      \
                                ## __VA_ARGS__);                        \
                }                                                       \
        } while (0)

#define ADS125X_WRN(msg, ...)                                           \
        do {                                                            \
                if (g_log_level > 1) {                                  \
                        printk(KERN_WARNING ADS125X_NAME " warning: " msg,\
                                ## __VA_ARGS__);                        \
                }                                                       \
        } while (0)

#define ADS125X_DBG(msg, ...)                                           \
        do {                                                            \
                if (g_log_level > 4) {                                  \
                        printk(KERN_DEFAULT ADS125X_NAME " debug: " msg,\
                                ## __VA_ARGS__);                        \
                }                                                       \
        } while (0)

struct ads125x_chip {
        struct ads125x_multi *  multi;
        /* used to wait for one sample completion in _buffer_disable() */
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

struct ads125x_ring {
        unsigned int            head;
        unsigned int            tail;
        unsigned int            mask;
        unsigned int            free;
        struct ads125x_sample * buff;
};

struct ads125x_ppbuf {
        struct completion       completion; 
        unsigned int            completion_level;
        struct ads125x_ring *   producer;
        struct ads125x_ring *   consumer;
        struct ads125x_ring     ring[2];
};

struct ads125x_sched {
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
        unsigned int            enabled_chips;
        bool                    is_bus_locked;
};

extern int g_log_level;

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

int ads125x_set_log_level(int level);

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

static inline
unsigned int ads125x_enabled_chips(struct ads125x_multi * multi)
{
        return (multi->enabled_chips);
}

static inline
bool ads125x_is_chip_enabled(const struct ads125x_multi * multi, 
                unsigned int chip_id)
{
        if (multi->enabled_chips & (0x1u << chip_id)) {
                return (true);
        } else {
                return (false);
        }
}

int ads125x_multi_ring_set_size(struct ads125x_multi * multi, unsigned int size);
size_t ads125x_multi_ring_get_size(struct ads125x_multi * multi);

int ads125x_self_calibrate(struct ads125x_chip * chip);
int ads125x_set_mux(struct ads125x_chip * chip, uint8_t positive, 
                uint8_t negative);

ssize_t ads125x_multi_ring_get_items(struct ads125x_multi * multi,
                char * buf, size_t count, unsigned long timeout);
int ads125x_buffer_enable(struct ads125x_chip * chip);
int ads125x_buffer_disable(struct ads125x_chip * chip);


#endif /* ADS1256_H_ */

