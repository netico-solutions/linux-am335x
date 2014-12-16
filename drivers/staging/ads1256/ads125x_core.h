/*
 * Texas Instruments ADS125x ADC driver support
 *
 * Author: Nenad Radulovic <nenad.b.radulovic@gmail.com>
 *
 * Licensed under GPL-v2
 */

#ifndef ADS125X_CORE_H_
#define ADS125X_CORE_H_

#define ADS125X_LOG_WARN_LEVEL          
#define ADS125X_CONFIG_LOG_LEVEL        1

#if (CONFIG_DEBUG_ENABLE == 1)
#define ADS125X_ASSERT(expression)                                              \
    do {                                                                        \
        if (!(expression)) {                                                    \
            printk(KERN_ERR " ASSERT FAILED: %s\n", # expression);              \
        }                                                                       \
    } while (0)
#else
#define ADS125X_ASSERT(expression)      

#endif /* ADS125X_CORE_H_ */
