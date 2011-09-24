#include "ti_func_4_pc.h"

//#ifdef WIN32
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
//#endif

void DSP_fltoq15(float x[], short r[], short nx)
{
  int i, a;

  for(i = 0; i < nx; i++)
  {
    a = 32768 * x[i];
    // saturate to 16−bit //
    if (a>32767)
      a = 32767;

    if (a<-32768)
      a = -32768;

    r[i] = (short) a;
  }
}


void DSP_q15tofl(short *x, float *r, int nx)
{
  int i;
  for (i=0;i<nx;i++)
    r[i] = (float) x[i] / 0x8000;
}


void DSP_fir_gen
(
	short 		* x,  // Input ('nr + nh - 1' samples)
	short 		* h,  // Filter coefficients (nh taps)
	short       * r,  // Output array ('nr' samples)
	int          nh, // Length of filter (nh >= 5)
	int          nr  // Length of output (nr >= 1)
)
{
	int i, j, sum;
	for (j = 0; j < nr; j++)
	{
		sum = 0;
		for (i = 0; i < nh; i++)
		{
			sum += x[i + j] * h[i];
		}
		r[j] = sum >> 15;
	}
}

void QDMA_move(void *dest, void *source, unsigned int elements)
{
	memcpy(dest,source,elements*sizeof(int));
}

void QDMA_wait(void)
{
	// do nothing
}
/*
#ifdef WIN32
void * memalign(int alignment, int size)
{
	// skip alignment on windows machine
	return malloc(size);
}
#endif
*/
int _ext(int v, int sl, int sr)
{
	return ((v << sl) >> sr);
}

int _extu(int v, int sl, int sr)
{
	return ((v << sl) >> sr);
}
int _mpy(int x, int y)
{
	int s1 = (((x & 0xFFFF) << 16) >> 16);
	int s2 = (((y & 0xFFFF) << 16) >> 16);

	return(s1*s2);
}
int _mpyh(int x, int y)
{
	int s1 = (short)(x >> 16);
	int s2 = (short)(y >> 16);
	return(s1*s2);
}
long long _amemd8_const( void *ptr )
{
	long long *lptr = (long long*)ptr;
	return(*lptr);
}
int _hi( long long val )
{
	return (int)(val>>32);
}
int _lo( long long val )
{
	return (int)((val<<32)>>32);	
}
int _pack2( int x, int y)
{
	return(((x & 0xFFFF) << 16) | (y & 0xFFFF));
}
int _packh2( int x, int y)
{
	return(((x >> 16) << 16) | (y >> 16));
}
void _nassert( void* arg )
{
	// do nothing
}
int _mem4_const( int *  ptr )
{
	return *ptr;
}
int _mem4( int *  ptr )
{
	return *ptr;
}	
int _amem4( int *  ptr )
{
	return *ptr;
}
int _amem4_const( int *  ptr )
{
	return *ptr;
}
int _mpylh(int a, int b)
{
	return(((a << 16) >> 16) * (b >> 16));
}
int _mpyhl(int a, int b)
{
	return((a >> 16) * ((b << 16) >> 16));
}

int _abs2(int a)
{
	int x = ((a << 16) >> 16);
	int y = (a >> 16);
	return (abs(x) << 16) | (abs(y));
}

int _sub2(int a, int b)
{
	short al = a & 0xFFFF;
	short ah = ((a & 0xFFFF0000) >> 16);
	short bl = b & 0xFFFF;
	short bh = ((b & 0xFFFF0000) >> 16);
	short r1 = (ah - bh);
	short r2 = (al - bl);
	// printf("%d %d %d %d - %d %d\n",al,ah,bl,bh,r2,r1);
	return (((r1 & 0xFFFF) << 16) | (r2 & 0xFFFF));
}
int _cmpgt2(int a, int b)
{
	short al = a & 0xFFFF;
	short ah = ((a & 0xFFFF0000) >> 16);
	short bl = b & 0xFFFF;
	short bh = ((b & 0xFFFF0000) >> 16);
	
	return ( (ah > bh) << 1 | (al > bl));
}



void IMG_conv_3x3(   
	const unsigned char *inptr,          
	unsigned char *outptr,          
	int            x_dim,           
	const          char *mask,            
	int            shift)           
{                                                                  
     const   unsigned char   *IN1,*IN2,*IN3;                       
     unsigned char           *OUT;                                 
                                                                   
     short    pix10,  pix20,  pix30;                               
     short    mask10, mask20, mask30;                              
                                                                   
     int      sum,      sum00,  sum11;                             
     int      i;                                                   
     int      sum22,    j;                                         
                                                                   
     IN1      =   inptr;                                           
     IN2      =   IN1 + x_dim;                                     
     IN3      =   IN2 + x_dim;                                     
     OUT      =   outptr;                                          
                                                                   
     for (j = 0; j < x_dim ; j++)                                  
     {                                                             
         sum = 0;                                                  
                                                                   
         for (i = 0; i < 3; i++)                                   
         {                                                         
             pix10  =   IN1[i];                                    
             pix20  =   IN2[i];                                    
             pix30  =   IN3[i];                                    
                                                                   
             mask10 =   mask[i];                                   
             mask20 =   mask[i + 3];                               
             mask30 =   mask[i + 6];                               
                                                                   
             sum00  =   pix10 * mask10;                            
             sum11  =   pix20 * mask20;                            
             sum22  =   pix30 * mask30;                            
                                                                   
             sum   +=   sum00 + sum11 + sum22;                      
         }                                                         
                                                                   
         IN1++;                                                    
         IN2++;                                                    
         IN3++;                                                    
                                                                   
         sum = (sum >> shift);                                     
         if ( sum <  0  )       sum = 0;                           
         if ( sum > 255 )       sum = 255;                         
         *OUT++   =       sum;    
     }                                                             
}                                                                  

void IMG_corr_gen
(
	const short *in_data,
	const short *h,
	short *out_data,
	int M,
	int cols
)
{
	register int i,j;
	int iters, sum;

    iters = cols - M;                                           
    for (j = 0; j < iters; j++)                                  
    {                                                            
        sum =  0;                                             
        for (i = 0; i < M; i++)                                  
        {                                                        
            sum += in_data[i + j] * h[i];                        
        }                                                        
        out_data[j] = sum;                                              
    }                                                            
}                                                                


// img Transpose
void DSP_mat_trans(short *x, short rows, short columns, short *r)
{
	short i,j;
		for(i=0; i<columns; i++)
			for(j=0; j<rows; j++)
				*(r+i*rows+j)=*(x+i+columns*j);
}

