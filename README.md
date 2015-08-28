# rtc

The RTC is a low-cost high-performance real-time control device for processing
analogue signals.

The RTC consists of a readily available Beagle Bone Black (BBB) processor base
board with an ARM7 Cortex A8 processor running at 1 GHz with 500MB of RAM and a
custom made I/O board for analogue signal processing (schematics and hardware
listings are freely available from <https://github.com/db9052/rtc>).

The I/O board uses an 18 bit, 8 channel, 200 ksps, simultaneous sampling ADC
(Analogue Devices AD7609) with differential inputs and a 16 bit, 4 channel, low
noise DAC (Analogue Devices AD5064).

The BBB runs without an operating system (on "bare metal") and uses software
built on Texas Instruments' Starterware package. The use of bare metal allows
for real-time data acquisition, processing and output with a sample rate of 20
kHz and a maximum of 4 usec latency. Communication with a host computer for data
transfer is via USB. Matlab and Python based interfaces are available for Linux,
Windows and Mac OS.
