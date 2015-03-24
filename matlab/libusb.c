#include "mex.h"
#include <string.h>
#include "libusb.h"

/* Pointer to the USB device */
libusb_device_handle *device = NULL;

/* USB time out in milliseconds */
const unsigned int timeout = 5000;

/* Clear-up function */
static void clearup()
{
    if (device != NULL) {
        libusb_release_interface(device, 0);
        libusb_close(device);
        libusb_exit(NULL);
        device = NULL;
    }
}

/* The main mex function */
void mexFunction( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[])
{
    char *action;
    
    /* Check for an input */
    if (nrhs == 0)
        mexErrMsgIdAndTxt("libusb:libusb", "No input! What shall I do?!");
    
    /* Check for a string */
    if (!mxIsChar(prhs[0]))
        mexErrMsgIdAndTxt("libusb:libusb", "Expected a string as the first argument");
    
    /* Find out what the action required is */
    action = mxArrayToString(prhs[0]);
    /* Initialise the USB interface */
    if (strcmp(action, "init") == 0) {
        /* Check inputs/outputs */
        if (nrhs != 1)
            mexErrMsgIdAndTxt("libusb:init", "Expected no inputs to init");
        if (nlhs != 0)
            mexErrMsgIdAndTxt("libusb:init", "Expected no outputs for init");
        /* Clear-up as necessary and (re)initialise the library */
        clearup();
        libusb_init(NULL);
        /* Open the USB device */
        device = libusb_open_device_with_vid_pid(NULL, 0x0123, 0x4567);
        if (device == NULL) {
            libusb_exit(NULL);
            mexErrMsgIdAndTxt("libusb:init", "Unable to open USB device - is it plugged in?");
        }
        else {
            /* Claim the USB interface */
            if (libusb_claim_interface(device, 0) < 0) {
                libusb_close(device);
                libusb_exit(NULL);
                device = NULL;
                mexErrMsgIdAndTxt("libusb:init", "Failed to claim the USB device - is something else using it?");
            }
            else
                mexAtExit(clearup);
        }
    }
    /* Write to the USB interface */
    else if (strcmp(action, "write") == 0) {
        int sent, ret;
        /* Check inputs/outputs */
        if (nrhs != 2) 
            mexErrMsgIdAndTxt("libusb:write", "Expected one input to write (the data)");
        if (nlhs != 1)
            mexErrMsgIdAndTxt("libusb:write", "Expected one output for write (the size of the data written)");
        if (!mxIsUint8(prhs[1]))
            mexErrMsgIdAndTxt("libusb:write", "Expected the input data to be a uint8 array");
        /* Do the data transfer */
        ret = libusb_bulk_transfer(device, 1, (unsigned char *)mxGetData(prhs[1]), mxGetNumberOfElements(prhs[1]), &sent, timeout);
        if (ret != 0)
            mexErrMsgIdAndTxt("libusb:write", "Error in bulk transfer - %d", ret);
        /* Return the number of bytes sent */
        plhs[0] = mxCreateDoubleScalar(sent);
    }
    else if (strcmp(action, "read") == 0) {
        int received, ret;
        /* Check inputs/outputs */
        if (nrhs != 2) 
            mexErrMsgIdAndTxt("libusb:read", "Expected one input to read (the data size)");
        if (nlhs != 1)
            mexErrMsgIdAndTxt("libusb:read", "Expected one output for read (the data read)");
        if (!mxIsScalar(prhs[1]))
            mexErrMsgIdAndTxt("libusb:read", "Expected the input data to be a scalar");
        /* Create the return data structure */
        plhs[0] = mxCreateNumericMatrix(1, mxGetScalar(prhs[1]), mxUINT8_CLASS, mxREAL);
        /* Do the data transfer */
        ret = libusb_bulk_transfer(device, 129, (unsigned char *)mxGetData(plhs[0]), mxGetNumberOfElements(plhs[0]), &received, timeout);
        if (ret != 0)
            mexErrMsgIdAndTxt("libusb:read", "Error in bulk transfer - %d", ret);
        /* Update the size of the return data structure */
        mxSetN(plhs[0], received);
    }
    else if (strcmp(action, "exit") == 0)
        clearup();
    else {
        mexErrMsgIdAndTxt("libusb:libusb", "Unknown command; available commands are: init, write, read and exit");
    }
}
