function compile_mex()
% COMPILE_MEX Compile the .c files into Matlab useable MEX files.
%
% To compile, the development libraries for libusb are needed. For Linux
% these should be available via your system package manager (e.g., sudo dnf
% install libusb-devel for Fedora). For Windows  see <http://libusb.info>;
% this compiles against a static libusb library. The binary used should be
% chosen depending on whether you are running 32 bit or 64 bit Windows. 

if ispc
    mex -largeArrayDims libusb.c libusb-1.0.a
else
    mex -largeArrayDims libusb.c -lusb-1.0 -L.
end

mex -largeArrayDims crc32.c

end
