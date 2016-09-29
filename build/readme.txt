To build, simply set the PATH to include the ARM compiler
<https://launchpad.net/gcc-arm-embedded> and then type make in one of the rig
directories.

The makedefs in the starterware directory with automagically find the correct
library files to go with the hard float ABI. Specifically, the following lines
do the magic.

LIB_GCC=$(subst libgcc.a,,$(shell $(CC) -mfloat-abi=hard -print-libgcc-file-name))
LIB_C=$(subst libc.a,,$(shell $(CC) -mfloat-abi=hard -print-file-name=libc.a))

