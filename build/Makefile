# Locate Starterware
ROOT=$(APP_ROOT)starterware/

# The device and platform being used
DEVICE=am335x
EVM=beaglebone

# Get the defaults from Starterware
#   Note: this has been modified to find the correct GCC version (the LIB_GCC
#     variable) and the CFLAGS have been modified to use the hard float
#     calling convention (-mfpu=neon -mfloat-abi=hard).
include $(ROOT)/build/armv7a/gcc/makedefs

# Starterware libraries that need to be built
STARTERWARE_BLD=$(DRIVERS_BLD) $(PLATFORM_BLD) $(SYSCONFIG_BLD) $(UTILITY_BLD) $(USBLIB_BLD)

# Starterware libraries that need to be linked to
STARTERWARE_LIB=-ldrivers  \
		-lutils    \
		-lplatform \
		-lsystem_config \
		-lusblib

# Compiler flags for this code
CFLAGS+=-ffast-math -fsingle-precision-constant -I$(APP_ROOT)include -std=gnu99 -pedantic

# Target mode
TARGET_MODE?=Release

# Source files
SRC=*.c $(APP_ROOT)/src/*.c

# Build this package
$(APP_NAME).bin: $(SRC)
	$(CC) $(CFLAGS) $(SRC)
	@mkdir -p bin
	@mv *.o* bin/
	$(LD) $(LDFLAGS) $(LPATH) -o bin/$(APP_NAME).out -Map bin/$(APP_NAME).map bin/*.o* \
		$(STARTERWARE_LIB) -lc -lgcc $(STARTERWARE_LIB) $(RUNTIMELIB) -T $(APP_ROOT)/build/loader.lds
	$(BIN) $(BINFLAGS) bin/$(APP_NAME).out $(APP_NAME).bin 

# Build Starterware libraries
starterware:
	@for i in $(STARTERWARE_BLD); \
	do \
		echo $${i} ; \
		if [ -f $${i}/makefile ] ; \
		then \
			make `echo $(TARGET_MODE) | tr A-Z a-z` -C $${i} || exit $$?; \
		fi; \
	done;

# Clean
clean:
	@rm -rf bin/ $(APP_NAME).bin

# Clean++
clean+: clean
	@make TARGET_MODE=clean starterware
