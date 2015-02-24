/*
** Copyright (c) 2014, David A.W. Barton
** (david.barton@bristol.ac.uk) All rights reserved.
**
** Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are
** met:
**
** 1. Redistributions of source code must retain the above copyright
** notice, this list of conditions and the following disclaimer.
**
** 2. Redistributions in binary form must reproduce the above copyright
** notice, this list of conditions and the following disclaimer in the
** documentation and/or other materials provided with the distribution.
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#ifndef __AD760X_H__
#define __AD760X_H__

#include <stdint.h>

/* Choose the particular device we have */
#define AD7609

/* Options associated with the different device types */
#ifdef AD7606_4
#define AD760X_CHANNEL_COUNT 4   /* Number of channels on the device - either 4, 6 or 8 */
#define AD760X_BIT_COUNT 16      /* Number of bits per word - either 16 or 18 */
#define AD760X_BIT_MASK 0xFFFF   /* Bitmask to use when sampling from a 32 bit int - either 0xFFFF or 0x3FFFF */
#define AD760X_BYTE_COUNT 2      /* Number of bytes per word - either 2 or 4 */
#define AD760X_FSR_LOW 10        /* Full scale range (low voltage setting) */
#define AD760X_FSR_HIGH 20       /* Full scale range (high voltage setting) */
#else
  #ifdef AD7606_6
    #define AD760X_CHANNEL_COUNT 6   /* Number of channels on the device - either 4, 6 or 8 */
    #define AD760X_BIT_COUNT 16      /* Number of bits per word - either 16 or 18 */
    #define AD760X_BIT_MASK 0xFFFF   /* Bitmask to use when sampling from a 32 bit int - either 0xFFFF or 0x3FFFF */
    #define AD760X_BYTE_COUNT 2      /* Number of bytes per word - either 2 or 4 */
    #define AD760X_FSR_LOW 10        /* Full scale range (low voltage setting) */
    #define AD760X_FSR_HIGH 20       /* Full scale range (high voltage setting) */
  #else
    #ifdef AD7606
      #define AD760X_CHANNEL_COUNT 8   /* Number of channels on the device - either 4, 6 or 8 */
      #define AD760X_BIT_COUNT 16      /* Number of bits per word - either 16 or 18 */
      #define AD760X_BIT_MASK 0xFFFF   /* Bitmask to use when sampling from a 32 bit int - either 0xFFFF or 0x3FFFF */
      #define AD760X_BYTE_COUNT 2      /* Number of bytes per word - either 2 or 4 */
      #define AD760X_FSR_LOW 10        /* Full scale range (low voltage setting) */
      #define AD760X_FSR_HIGH 20       /* Full scale range (high voltage setting) */
    #else
      #ifdef AD7609
        #define AD760X_CHANNEL_COUNT 8   /* Number of channels on the device - either 4, 6 or 8 */
        #define AD760X_BIT_COUNT 18      /* Number of bits per word - either 16 or 18 */
        #define AD760X_BIT_MASK 0x3FFFF  /* Bitmask to use when sampling from a 32 bit int - either 0xFFFF or 0x3FFFF */
        #define AD760X_BYTE_COUNT 4      /* Number of bytes per word - either 2 or 4 */
        #define AD760X_FSR_LOW 20        /* Full scale range (low voltage setting) */
        #define AD760X_FSR_HIGH 40       /* Full scale range (high voltage setting) */
      #endif
    #endif
  #endif
#endif


/* Used for setting the input voltage range of the AD760x to either +/- 5v or +/- 10v */
typedef enum {
	AD760X_RANGE_HIGH,
	AD760X_RANGE_LOW
} ad760x_range;

/* 
Used for setting the oversampling rate 
	no oversampling (max rate 200kHz, bandwidth 22kHz)
	2x oversampling (max rate 100kHz, bandwidth 22kHz)
	4x oversampling (max rate 50kHz, bandwidth 18.5kHz)
	8x oversampling (max rate 25kHz, bandwidth 11.9kHz)
	16x oversampling (max rate 12.5kHz, bandwidth 6kHz)
	32x oversampling (max rate 6.25kHz, bandwidth 3kHz)
	64x oversampling (max rate 3.125kHz, bandwidth 1.5kHz)
*/
typedef enum {
	AD760X_OVERSAMPLE_OFF,
	AD760X_OVERSAMPLE_2,
	AD760X_OVERSAMPLE_4,
	AD760X_OVERSAMPLE_8,
	AD760X_OVERSAMPLE_16,
	AD760X_OVERSAMPLE_32,
	AD760X_OVERSAMPLE_64,
} ad760x_oversample;

/* Samples from the AD760x */
extern volatile int ad760xBuffer[AD760X_CHANNEL_COUNT];

/* Flag to indicate data ready */
extern volatile int ad760xBufferReady;

/* Data handling routine */
extern void (*ad760xDataHandler)(void);

/* Set up GPIOs, McSPI and delay timer needed for AD760X operation */
void ad760xSetup(void);

/* Set up PWM to drive conversions */
void ad760xSetupPWM(unsigned int period);

/* Enable PWM to drive conversions */
void ad760xEnablePWM();

/* Disable PWM to drive conversions */
void ad760xDisablePWM();

/* Reset the AD760X */
void ad760xReset(void);

/* Start data capture by raising the CONVST signal */
void ad760xCaptureStart(void);

/* Set the voltage range */
void ad760xSetRange(ad760x_range range);

/* Set the over-sample rate */
void ad760xSetOverSample(ad760x_oversample oversample);

#endif
