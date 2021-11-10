/* This header file contain function declaration of various function used for extracting feature from given input audio frame */

#ifndef __FEATURE_EX__
#define __FEATURE_EX__

#include <math.h>
#include "network.h"

#define PI (4.0 * atan(1.0)) 
#define numFFT 256
#define frameshift  480
#define samplerate  16000
#define numFFTbins  numFFT/2 
#define nFilters  128
#define hammWindowConst 0.54
#define hannWindowConst 0.5
#define melConst1 2595.0
#define melConst2 700.0
#define timeIncrment 0.03


typedef struct
{
   float real;
   float imag;
}complex;

float hanFrames[numFFT];

void windowInit();
float map(float x, float in_min, float in_max, float out_min, float out_max);
void Hanning(int frameLength, float * frames);
float computeComplexPower(complex *X);

complex complexMul(complex *X,complex *Y);
void fft(complex *X, int N);
void applyWindow(short *sample,int N, complex * winSample );
void Spectrum(complex *fft, float spec[nFilters][kernelsize], int nc);
void extractFeature(short *signal, float input[numInputs][kernelsize], float *outtime );

#endif

