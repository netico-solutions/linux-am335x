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
#include <linux/gpio.h>
#include <linux/kfifo.h>

#define ADS125X_NAME                            "ads1256"
#define ADS125X_CONFIG_TRANSFER_SIZE            16
#define ADS125X_CONFIG_SUPPORTED_CHIPS          4

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
        uint8_t                 irq_data[ADS125X_CONFIG_TRANSFER_SIZE]
                ____cacheline_aligned;
        bool                    is_irq_enabled;
        int                     id;
        int                     cs_gpio;
        int                     drdy_gpio;
};

struct ads125x_multi {
        struct ads125x_chip *   chip[ADS125X_CONFIG_SUPPORTED_CHIPS];
        struct spi_device *     spi;
        struct kfifo            fifo;
        bool                    is_bus_locked;
        uint32_t                enabled;
};

struct ads1256_sample {
        uint32_t                raw[ADS125X_CONFIG_SUPPORTED_CHIPS];
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

int ads125x_write_reg(struct ads125x_chip * chip, uint32_t reg, uint32_t val);
int ads125x_read_reg(struct ads125x_chip * chip, uint32_t reg, uint32_t * val);
int ads125x_self_calibrate(struct ads125x_chip * chip);
int ads125x_set_channel(struct ads125x_chip * chip, uint8_t positive, 
                uint8_t negative);

#endif /* ADS1256_H_ */

