function compile_mex()

mex libusb.c -lusb-1.0
mex crc32.c

end
