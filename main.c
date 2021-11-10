/* This file is used to Extract Audio samples from wav file */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "constant.h"
#include "process.h"
#include "wavHeader.h"
#include "portaudio.h"
#include <pthread.h>
#include <unistd.h>

#define SAMPLE_RATE (16000)
#define FRAMES_PER_BUFFER (64)
#define NUM_SECONDS (0.03)
#define NUM_CHANNELS (2)
#define DITHER_FLAG (0)
#define PA_SAMPLE_TYPE paInt16
typedef short SAMPLE;
#define SAMPLE_SILENCE (0)

typedef struct
{
	int frameIndex; /* Index into sample array. */
	int maxFrameIndex;
	SAMPLE* recordedSamples;
} paTestData;


FILE *wavFp = NULL;
wavHeader hdr;
static short FrameShiftSamples = 480*4;

short* FileOpen(char * name,unsigned short BufferLength) ;
static void FileClose(short* buffer) ;
static unsigned short  FileRead(short* buffer,unsigned short BufferLength) ;

static short* FileOpen1(unsigned short BufferLength,char *name) 
{

	int BytesRead = 0 ;
	int headerSize = sizeof(hdr);

	if(wavFp == NULL) {
		short* pBuffer;
		char filename[50];  
		sprintf(filename,"test.wav",name);
		wavFp = fopen("test.wav","rb");
		BytesRead = fread(&hdr,1,headerSize,wavFp);
		pBuffer = (short *) malloc(sizeof(short) * BufferLength);
		return pBuffer;
	} else {
		return NULL;
	}
}	


int main(int argc, char* argv[])
{

	int VAL;

	if (memcmp(argv[1],"mic",3)==0)
	{ 
	   VAL = 1;
	}else{
           VAL = 0;
        }


	if(VAL){

		initprocess();				
		unsigned short bufferLength = (FrameShiftSamples * 1); 

		PaStreamParameters inputParameters;
		PaStream* stream;
		PaError err = paNoError;
		paTestData data;
		int i,totalFrames,numSamples, numBytes;
		SAMPLE* recordedSamples_mono;
		data.maxFrameIndex  = NUM_SECONDS * SAMPLE_RATE*4; /* Record for a few seconds. */
		totalFrames = data.maxFrameIndex;
		data.frameIndex = 0;
		numSamples = totalFrames * 2;
		numBytes = numSamples * sizeof(SAMPLE);
		data.recordedSamples = (SAMPLE*)malloc(numBytes);
		recordedSamples_mono = (SAMPLE*)malloc(4* NUM_SECONDS * SAMPLE_RATE * 1 * sizeof(SAMPLE));

		if (recordedSamples_mono == NULL) {

			printf("Could not allocate record array.\n");
		}
		if (data.recordedSamples == NULL) {
			printf("Could not allocate record array.\n");
		}
		for (i = 0; i < numSamples; i++)
			data.recordedSamples[i] = 0;

		err = Pa_Initialize();
		if (err != paNoError){
                                         printf("error0");
		}

		printf("\n\n*******************************************************************"
				"\n\tAvailbale I/O Sound Devices in system\n"
				"*******************************************************************\n\n");
		for(int i = 0; i<Pa_GetDeviceCount(); i++)
			printf("Device ID: %d\nDevice Name: %s\n\n", i, Pa_GetDeviceInfo(i)->name);
		printf("Please Enter Device ID: ");
		scanf("%d", &inputParameters.device);
		printf("Please type Channel Type, 1 for MONO, 2 for STEREO : ");
		scanf("%d", &inputParameters.channelCount);
		
		//inputParameters.device = 0;//Pa_GetDefaultInputDevice(); /* default input device */
		if (inputParameters.device == paNoDevice) {
			fprintf(stderr, "Error: No default input device.\n");
		}
		//inputParameters.channelCount = 1; /* stereo input */
		inputParameters.sampleFormat = PA_SAMPLE_TYPE;
		inputParameters.suggestedLatency = Pa_GetDeviceInfo(inputParameters.device)->defaultLowInputLatency;
		inputParameters.hostApiSpecificStreamInfo = NULL;

                         
		/* Record some audio. -------------------------------------------- */
		err = Pa_OpenStream(
				&stream,
				&inputParameters,
				NULL, /* &outputParameters, */
				SAMPLE_RATE,
				FRAMES_PER_BUFFER,
				paClipOff, /* we won't output out of range samples so don't bother clipping them */
				NULL,
				NULL); 
		if (err != paNoError){
             printf("error");
			// return 0;
		}

		err = Pa_StartStream(stream);
		if (err != paNoError){
                 printf("error1");
		}

		int channel = inputParameters.channelCount;
		while(1) {


    		int err = Pa_ReadStream(stream, data.recordedSamples, 4*160*3);

			for (i = 0; i < totalFrames; i++) {
				int indx = 0;
				if (channel == 1)
					indx = i;
				else if(channel == 2)
					indx = i*2+1;
				else {
					printf("Wrong channel value");
					return -1;
				}
				recordedSamples_mono[i] = data.recordedSamples[indx]; 
			}

			process(recordedSamples_mono);
                       
		}
                
        err = Pa_StopStream( stream );
		if( err != paNoError ) {
			;
                }
		err = Pa_CloseStream(stream);
		if (err != paNoError){
		}
		Pa_Terminate();
		if (data.recordedSamples) /* Sure it is NULL or valid. */
			free(data.recordedSamples);

	}

else{	
		initprocess();				
		unsigned short bufferLength = 1920;//(FrameShiftSamples * numTwo + FrameShiftSamples / numTwo);    //(4 -> Kernelsize(10) - striplength(6)) 
		
		
		short* pBuffer = FileOpen(argv[numOne],bufferLength);
		//short* pBuffer = FileOpen(bufferLength,argv[1]);
		#if 0
		if(pBuffer ==  NULL) {
			break;
		}
		#endif
		
		while(numOne) {
			
			int ReadBytes = FileRead(pBuffer,(unsigned short)bufferLength);

			if( ReadBytes <= numZero) { 
				break;
			}
			process(pBuffer);    	            

		}
		
		FileClose(pBuffer);	
}
	
	return numZero;
}



short* FileOpen(char * name, unsigned short BufferLength) 
{
        // Function used for opening the wav file and read wav header file 
	int BytesRead = numZero;
	int headerSize = sizeof(hdr);

	if(wavFp == NULL) {
		short* pBuffer;
		wavFp = fopen(name,"rb");
		BytesRead = fread(&hdr,numOne,headerSize,wavFp);
		pBuffer = (short *) malloc(sizeof(short) * BufferLength);
		return pBuffer;
	} else {
		return NULL;
	}
}	

void FileClose(short* buffer) 
{
        // Function used for close the wav file pointer 
	free(buffer);
	fclose(wavFp);
	wavFp = NULL;
}

unsigned short  FileRead(short* pBuffer,unsigned short BufferLength) 
{
        // Function used for reading the sample from the wav file  
	int BytesRead = numZero;
	if(feof(wavFp)) {
		return numZero;
	}
	BytesRead = fread(pBuffer,numOne, BufferLength*sizeof(short),wavFp);	
	if(BytesRead < (BufferLength*numTwo)) {		
		return numZero;
	} else {
		return BufferLength;
	}	
}




