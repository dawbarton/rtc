#include "mex.h"

/* CRC table */
static unsigned int crc_table[256];
static int initialised = 0;

/* Initial value for the CRC */
#define CRC32_INITIAL 0xFFFFFFFF

/* ************************************************************************ */
void crc32_setup(void)
{
	/* This CRC implementation is the same as is used for PNG images */
	unsigned int c;
	unsigned int n, k;

	for (n = 0; n < 256; n++) {
		c = n;
		for (k = 0; k < 8; k++) {
			if (c & 1)
				c = 0xEDB88320L ^ (c >> 1);
			else
				c = c >> 1;
		}
		crc_table[n] = c;
	}
}

/* ************************************************************************ */
void crc32_update(unsigned int *crc, unsigned int len, unsigned char *data)
{
	unsigned int i;
	for (i = 0; i < len; i++) {
		*crc = crc_table[(*crc ^ data[i]) & 0xFF] ^ (*crc >> 8);
	}
}

/* ************************************************************************ */
void crc32_final(unsigned int *crc)
{
	*crc ^= 0xFFFFFFFF;
}


/* The main mex function */
void mexFunction( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[])
{
    unsigned int crc = CRC32_INITIAL;
    
    /* Check inputs/outputs */
    if (nrhs != 1) 
        mexErrMsgIdAndTxt("crc32:crc32", "Expected one input to crc32 (the data)");
    if (nlhs > 1)
        mexErrMsgIdAndTxt("crc32:crc32", "Expected one output for crc32 (the checksum)");
    if (!mxIsUint8(prhs[0]))
        mexErrMsgIdAndTxt("crc32:crc32", "Expected the input data to be a uint8 array");
    
    /* Make sure the CRC table is initialised */
    if (!initialised) {
        crc32_setup();
        initialised = 1;
    }
    
    /* Calculate the checksum */
    crc32_update(&crc, (unsigned int)mxGetNumberOfElements(prhs[0]), (unsigned char *)mxGetData(prhs[0]));
    crc32_final(&crc);
    
    /* Output the checksum */
    plhs[0] = mxCreateNumericMatrix(1, 1, mxUINT32_CLASS, mxREAL);
    *(unsigned int *)mxGetData(plhs[0]) = crc;
}
