function compile_mex()

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
