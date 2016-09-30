/*
** Copyright (c) 2016, David A.W. Barton
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
#ifndef __AKSIM_H__
#define __AKSIM_H__

#include <stdint.h>

/* For the 20 bit AksIM encoder */
#define AKSIM_BIT_MASK 0xFFFFF

/* Samples from the AksIM */
extern volatile unsigned int aksimBuffer;

/* Flag to indicate data ready */
extern volatile unsigned int aksimBufferReady;

/* Status from the AksIM */
extern volatile unsigned int aksimStatus;

/* True is there has been a CRC error in the data */
extern volatile unsigned int aksimCRCError;

/* Set up GPIOs, McSPI and delay timer needed for AD760X operation */
void aksimSetup(void);

/* Get data from the AksIM */
void aksimCaptureGet(void);

/* Data handling routine */
extern void (*aksimDataHandler)(void);

#endif
