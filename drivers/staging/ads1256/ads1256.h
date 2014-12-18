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

#define ADS125X_NAME                            "ads1256"
#define ADS125X_CONFIG_TRANSFER_SIZE            16

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


struct ads1256_chip {
    struct completion           completion;
    struct spi_device *         spi;
    struct spi_transfer         transfer[2];
    struct spi_message          message;
    /*
     * DMA (thus cache coherence maintenance) requires the transfer buffers to
     * live in their own cache lines.
     */
    uint8_t                     transfer_data[ADS125X_CONFIG_TRANSFER_SIZE]
        ____cacheline_aligned;
    bool                        is_bus_locked;
    bool                        is_irq_enabled;
    int                         cs_gpio;
    int                         drdy_gpio; 
    int                         id;
};



/**
 * ti_sd_probe_of() - setup chip data from DTS
 * spi: SPI device
 *
 * Returns 0 on success, an error code otherwise
 */
int ti_sd_probe_of(struct spi_device * spi);



/**
 * ti_sd_probe_trigger()
 * @chip: chip device
 *
 * Returns 0 on success, an error code otherwise
 */
int ti_sd_probe_trigger(struct ads1256_chip * chip);



/**
 * ti_sd_init_chip()
 * @chip: The chip device
 * @spi: SPI device
 */
int ti_sd_init_chip(struct ads1256_chip * chip, struct spi_device * spi);



/**
 * ti_sd_init_hw()
 * @chip: the chip device
 *
 * Returns 0 on success, an error code otherwise
 */
int ti_sd_init_hw(struct ads1256_chip * chip);



/**
 * ti_sd_term()
 * @chip: chip device
 *
 * Returns 0 on success, an error code otherwise
 */
int ti_sd_term(struct ads1256_chip * chip);



/**
 * ti_sd_cleanup_buffer_and_trigger()
 * @indio_dev - IIO device
 */
void ti_sd_remove_trigger(struct ads1256_chip * chip);



/**
 * ti_sd_write_reg() - Write a register
 *
 * @chip: The chip device
 * @reg: Address of the registers
 * @size: Size of the register (1 - 4)
 * @val: Value to write to the register
 *
 * Returns 0 on success, an error code otherwise
 */
int ti_sd_write_reg(struct ads1256_chip * chip, uint32_t reg, uint32_t val);



/**
 * ti_sd_read_reg()
 * @chip: The chip device
 * @reg: Address of the register
 * @val: Pointer to a buffer
 *
 * Returns 0 on success, an error code otherwise.
 */
int ti_sd_read_reg(struct ads1256_chip * chip, uint32_t reg, uint32_t * val);



/**
 * ti_sd_self_calibrate()
 * @chip: The chip device
 *
 * Returns 0 on success, an error code otherwise.
 */
int ti_sd_self_calibrate(struct ads1256_chip * chip);



/**
 * ti_sd_set_mode()
 * @chip: The chip device
 * @mode: Set mode to ADS125X_MODE_CONTINUOUS or ADS125X_MODE_IDLE
 *
 * Returns 0 on success, an error code otherwise.
 */
int ti_sd_set_mode(struct ads1256_chip * chip, uint32_t mode);



/**
 * ti_sd_set_channel()
 * @chip: The sigma delta device
 * @positive: positive input channel
 * @negative: negative input channel
 *
 * Returns 0 on success, an error code otherwise.
 */
int ti_sd_set_channel(struct ads1256_chip * chip, uint8_t positive, 
                uint8_t negative);

#endif /* ADS1256_H_ */

