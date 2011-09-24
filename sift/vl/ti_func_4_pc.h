#ifndef TI_FUNC_4_PC_INCLUDED
#define TI_FUNC_4_PC_INCLUDED
/*
#ifdef WIN32
void *memalign(int alignment, int size);
#endif
*/
int _ext(int v, int sl, int sr);
int _extu(int v, int sl, int sr);
int _mpy(int x, int y);
int _mpyh(int x, int y);
int _mpylh(int a, int b);
int _mpyhl(int a, int b);
int _hi( long long val );
int _lo( long long val );
int _pack2( int x, int y);
int _packh2( int x, int y);
int _amem4_const( int *  ptr );
long long _amemd8_const( void *ptr);
int _mem4_const( int *  ptr );
int _mem4( int *  ptr );
int _amem4( int *  ptr );
int _sub2(int a, int b);
void _nassert( void* arg );
int _cmpgt2(int v1, int v2);
int _abs2(int a);

void IMG_conv_3x3(   	
	const unsigned char *inptr,          
	unsigned char *outptr,          
	int            x_dim,           
	const          char *mask,            
	int            shift
); 

void IMG_corr_gen
(
	const short *in_data,
	const short *h,
	short *out_data,
	int M,
	int cols
);

void DSP_fir_gen
(
	short       * x,  // Input ('nr + nh - 1' samples)
	short 	    * h,  // Filter coefficients (nh taps)
	short       * r,  // Output array ('nr' samples)
	int          nh, // Length of filter (nh >= 5)
	int          nr  // Length of output (nr >= 1)
);

void DSP_mat_trans(short *x, short rows, short columns, short *r);

void QDMA_move(void *dest, void *source, unsigned int elements);
void QDMA_wait(void);

void DSP_q15tofl(short *x, float *r, int nx);
void DSP_fltoq15(float x[], short r[], short nx);

#endif
