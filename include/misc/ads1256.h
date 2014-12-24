/*
 * Texas Instruments AD1256 ADC driver core IOCTL interface
 *
 * Author: Nenad Radulovic <nenad.b.radulovic@gmail.com>
 *
 * Licensed under GPL-v2
 */

#ifndef ADS125X_IOCTL_H_
#define ADS125X_IOCTL_H_

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

#endif /* ADS125X_IOCTL_H_ */

