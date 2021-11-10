/* header file for network.c */

#ifndef __NETWORK__
#define __NETWORK__


#define numInputs 128
#define numConvOutputs 32
#define numGruUnits 16
#define numDense1Units 32
#define numOutputs 4
#define kernelsize 10
#define stride1D 4 
#define timeIncrment 0.03

float sigmoid(float x);
void initWeights();
void convLayer(float input[numInputs][kernelsize], float * out );
void gruLayer(float *yconv, float* ygru);
void dense1Layer(float *ygru, float * ydense1);
void dense2Layer(float *ydense1, float * youtput);
void frameFeatureShifting(float Xinput[numInputs][kernelsize]);
void network(float input[numInputs][kernelsize], float * youtput);

#endif
