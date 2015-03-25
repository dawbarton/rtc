function compile_mex()
% COMPILE_MEX Compile the .c files into Matlab useable MEX files.
%
% To compile, the development libraries for libusb are needed. For Linux
% these should be available via your system package manager (e.g., sudo yum
% install libusb-devel for Fedora). For Windows  see <http://libusb.info>;
% the necessary .lib file is in the MS32 or MS64 folder of the Windows
% binary download depending on whether you are running 32 bit or 64 bit
% Windows. 

if ispc
    % For some reason the Windows version of Matlab doesn't seem to be able
    % to handle library file names with periods in (e.g., libusb-1.0.lib)
    % and instead truncates them (to libusb-1.lib in this case)
    if ~exist('libusb-1.lib', 'file') && exist('libusb-1.0.lib', 'file')
        copyfile('libusb-1.0.lib', 'libusb-1.lib');
    end
    mex -largeArrayDims libusb.c -lusb-1
else
    mex -largeArrayDims libusb.c -lusb-1.0
end

mex -largeArrayDims crc32.c

end
