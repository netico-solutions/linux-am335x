/*
 * Texas Instruments AD1256 ADC driver core IOCTL interface
 *
 * Author: Nenad Radulovic <nenad.b.radulovic@gmail.com>
 *
 * Licensed under GPL-v2
 */

#ifndef ADS125X_IOCTL_H_
#define ADS125X_IOCTL_H_


#define ADS125X_MAGIC                           'x'
#define ADS125X_SET_MUX                                                 \
        _IOW(ADS125X_MAGIC, 100, struct ads125x_mux)
#define ADS125X_SET_BUF_SIZE                                            \
        _IOW(ADS125X_MAGIC, 101, int)
#define ADS125X_SET_LOG_LEVEL                                           \
        _IOW(ADS125X_MAGIC, 102, int)
#define ADS125X_SELF_CALIBRATE                  _IO(ADS125X_MAGIC, 102)
#define ADS125X_START_SAMPLING                  _IO(ADS125X_MAGIC, 115)
#define ADS125X_STOP_SAMPLING                   _IO(ADS125X_MAGIC, 116)


/*--  Configuration  --------------------------------------------------------*/
#define ADS125X_NAME                            "ads1256"
#define ADS125X_CONFIG_SUPPORTED_CHIPS          4
#define ADS125X_CONFIG_BUFFER_SIZE              1024

struct ads125x_mux {
        int                     positive;
        int                     negative;
};

struct ads125x_sample {
        unsigned int            raw[ADS125X_CONFIG_SUPPORTED_CHIPS];
        union ads125x_sample_info {
                unsigned int            completed_chip;
                unsigned int            sequence;
        }                       info;
};


#endif /* ADS125X_IOCTL_H_ */

