/* header file for process.c */

#ifndef __PROCESS__
#define __PROCESS__

#define threshold 0.75

void initprocess();
void process(short *signal);
void printResult(float *youtput);

float outtime;

#endif
