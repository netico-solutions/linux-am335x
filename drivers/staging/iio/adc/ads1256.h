
#ifndef ADS1256_H_
#define ADS1256_H_

#include <linux/types.h>
#include <linux/completion.h>

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

struct spi_device;
struct iio_trigger;
struct iio_dev;

struct ti_sigma_delta {
    struct completion           completion;
    struct spi_device *         spi;
    struct iio_trigger *        trigger;
    /*
     * DMA (thus cache coherence maintenance) requires the transfer buffers to
     * live in their own cache lines.
     */
    uint8_t                     transfer_data[ADS125X_CONFIG_TRANSFER_SIZE]
        ____cacheline_aligned;
    bool                        is_bus_locked;
    bool                        is_irq_dis;
};

/**
 * ti_sd_init_sigma_delta()
 * @sigma_delta: The sigma delta device
 * @indio_dev: IIO device
 * @spi: SPI device
 */
void ti_sd_init_sigma_delta(struct ti_sigma_delta * sigma_delta, 
        struct iio_dev * indio_dev, struct spi_device * spi);
/**
 * ti_sd_setup_buffer_and_trigger()
 * @indio_dev - IIO device
 */
int ti_sd_setup_buffer_and_trigger(struct iio_dev * indio_dev);

/**
 * ti_sd_cleanup_buffer_and_trigger()
 * @indio_dev - IIO device
 */
void ti_sd_cleanup_buffer_and_trigger(struct iio_dev * indio_dev);

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
    uint32_t val);

/**
 * ti_sd_read_reg()
 * @sigma_delta: The sigma delta device
 * @reg: Address of the register
 * @val: Pointer to a buffer
 *
 * Returns 0 on success, an error code otherwise.
 */
int ti_sd_read_reg(struct ti_sigma_delta * sigma_delta, uint32_t reg,
    uint32_t * val);

/**
 * ti_sd_self_calibrate()
 * @sigma_delta: The sigma delta device
 *
 * Returns 0 on success, an error code otherwise.
 */
int ti_sd_self_calibrate(struct ti_sigma_delta * sigma_delta);

/**
 * ti_sd_set_mode()
 * @sigma_delta: The sigma delta device
 * @mode: Set mode to ADS125X_MODE_CONTINUOUS or ADS125X_MODE_IDLE
 *
 * Returns 0 on success, an error code otherwise.
 */
int ti_sd_set_mode(struct ti_sigma_delta * sigma_delta, uint32_t mode);

/**
 * ti_sd_set_channel()
 * @sigma_delta: The sigma delta device
 * @channel: 0 - 7 channel id
 *
 * Returns 0 on success, an error code otherwise.
 */
int ti_sd_set_channel(struct ti_sigma_delta * sigma_delta, uint32_t channel);

#endif /* ADS1256_H_ */
