/*********************************************************************
*
* ANSI C Example program:
*    ChnlExpSync.c
*
* Example Category:
*    Sync
*
* Description:
*    This example demonstrates how to acquire a continuous amount of
*    data using a reference clock with a DSA device and a 
*    multifunction IO (MIO) device.
*
* Instructions for Running:
*    1. Select the physical channel to correspond to where your
*       signal is input on the DAQ device.
*    2. Enter the minimum and maximum voltage range.
*    Note: For better accuracy try to match the input range to the
*          expected voltage level of the measured signal.
*    3. Set the rate of the acquisition.
*    Note: The rate should be at least twice as fast as the maximum
*          frequency component of the signal being acquired.
*    4. Set the number of samples to acquire per channel.
*
* Steps:
*	 1. Change constants to preferred values (i.e. sample rate, 
*		channel names, etc.).
*    2. Create task.
*    3. Create analog input voltage channel for the DSA
*       and MIO devices.
*    4. Set timing parameters. Note that sample mode is set to
*       Continuous Samples. In this example, the Rate and the Samples
*       per Channel is set the same for both devices, you can however
*       use different values for each device.
*	 5. Remove AI filter delay for the DSA device, set reference clock
*		sources, and export start trigger from the DSA device to the
*		MIO device.
*    6. Call the Start function to start the acquisition.
*    7. Read all of the data continuously. The 'sampsPerChan'
*       control will specify how many samples per channel are read
*       each time. If either device reports an error or the user
*       presses the 'Stop' button, the acquisition will stop.
*	 8. Call the DFT function to generate frequency information such as
*		the phase skew of the two signals. Note: the DFT function uses
*		the FFTW3 library. For more information, visit www.fftw.org
*    9. Call the Clear Task function to clear the task.
*    10. Display an error, if any.
*
* I/O Connections Overview:
*    Make sure your signal input terminal matches the Physical
*    Channel I/O control.

*
*    If you have a PXI chassis, ensure it has been properly
*    identified in MAX. If you have devices with a RTSI bus, ensure
*    they are connected with a RTSI cable and that the RTSI cable is
*    registered in MAX.
*
*********************************************************************/

#include <string.h>
#include <stdio.h>
#include <NIDAQmx.h>
#include <math.h>
#include <fftw3.h>

static TaskHandle taskHandle=0;

#define DAQmxErrChk(functionCall) if( DAQmxFailed(error=(functionCall)) ) goto Error; else
#define REAL 0
#define IMAG 1
#define PI 3.14159265

int32 EveryNCallback(TaskHandle taskHandle, int32 everyNsamplesEventType, uInt32 nSamples, void *callbackData);
int32 DoneCallback(TaskHandle taskHandle, int32 status, void *callbackData);
void DFT(double *dsaData, double *mioData, double sampleRate, long long int sampsPerChan, int precision, FILE *file, double *measuredPhaseSkewDeg, double *measuredPhaseSkewSec, double *measuredFreq);
void StoreData(int arraySize, double *dataToAllocate, double *output);
double NormalizePhaseAngleDifference(double phase);

/*********************************************/
// DAQmx Configuration Options
/*********************************************/
// Sampling Options
const float64 sampleRate = 10000.0; // The sampling rate in samples per second per channel.
const uInt64 sampsPerChan = 1000; // The number of samples to acquire or generate for each channel in the task.

// DAQmxCreateAIVoltageChan Options
const char *physicalChannels = "Dev3/ai0, Dev4/ai0"; // The names of the physical channels to use to create virtual channels. You can specify a list or range of physical channels.
const int32 terminalConfig = DAQmx_Val_Cfg_Default; // The input terminal configuration for the channel. Options: DAQmx_Val_Cfg_Default, DAQmx_Val_RSE, DAQmx_Val_NRSE, DAQmx_Val_Diff, DAQmx_Val_PseudoDiff
const float64 minVal = -5.0; // The minimum value, in units, that you expect to measure.
const float64 maxVal = 5.0; // The maximum value, in units, that you expect to measure.
const int32 units = DAQmx_Val_Volts; // The units to use to return the voltage measurements. Options: DAQmx_Val_Volts, DAQmx_Val_FromCustomScale

// DAQmxCfgSampClkTiming Options
const int32 activeEdge = DAQmx_Val_Rising; // Specifies on which edge of the clock to acquire or generate samples. Options: DAQmx_Val_Rising, DAQmx_Val_Falling
const int32 sampleMode = DAQmx_Val_ContSamps; // Specifies whether the task acquires or generates samples continuously or if it acquires or generates a finite number of samples. Options: DAQmx_Val_FiniteSamps, DAQmx_Val_ContSamps, DAQmx_Val_HWTimedSinglePoint

// DAQmxSetAIRemoveFilterDelay Options
const char *dsaDeviceName = "Dev3/ai0"; // Specifies the device terminal to enable filter delay removal on.

// Reference Clock Options
const char *refClkSrc = "PXI_Clk100"; // Specifies the terminal of the signal to use as the Reference Clock.

// DAQmxRegisterEveryNSamplesEvent Options
const int32 everyNsamplesEventType = DAQmx_Val_Acquired_Into_Buffer; // The type of event you want to receive. Options: DAQmx_Val_Acquired_Into_Buffer, DAQmx_Val_Transferred_From_Buffer
const uInt32 options = 0; // Use this parameter to set certain options. Pass a value of zero if no options need to be set. Options: 0, DAQmx_Val_SynchronousEventCallbacks

// DAQmxReadAnalogF64 Options
const float64 timeout = 10.0; // The amount of time, in seconds, to wait for the function to read the sample(s).
const bool32 fillMode = DAQmx_Val_GroupByScanNumber; // Specifies whether or not the samples are interleaved. Options: DAQmx_Val_GroupByChannel, DAQmx_Val_GroupByScanNumber

// Create file variables and their names
FILE *dataFile, *dftFile;
const char *voltageDataFileName = "../../VoltageData.csv"; // Measurement data is stored in this CSV file
const char *dftDataFileName = "../../DFTData.csv"; // DFT data is stored in this CSV file
const int voltageDataFileLogPrecision = 2; // Precision at which to store the voltage data
const int dftDataFileLogPrecision = 3; // Precision at which to store the dft data
typedef struct {
	FILE *file;
	int precision;
} LogFile;
typedef struct {
	LogFile voltageData;
	LogFile dftData;
} Logs;
typedef Logs *LogsPtr;

int main(void)
{
	int32       error=0;
	char        errBuff[2048]={'\0'};
	char	    channels[256],trigName[256];
	LogFile 	voltageDataFile, dftDataFile;
	Logs		logData;

 	/*********************************************/
	// DAQmx Configure Code
	/*********************************************/
	DAQmxErrChk (DAQmxCreateTask("",&taskHandle));
	DAQmxErrChk (DAQmxCreateAIVoltageChan(taskHandle,physicalChannels,"",terminalConfig,minVal,maxVal,units,NULL));
	DAQmxErrChk (DAQmxCfgSampClkTiming(taskHandle,"",sampleRate,activeEdge,sampleMode,sampsPerChan));
	DAQmxErrChk (DAQmxSetAIRemoveFilterDelay(taskHandle,dsaDeviceName,1)); 

	DAQmxErrChk (DAQmxSetRefClkSrc(taskHandle, refClkSrc));

	// Create two CSV files and set precision
	voltageDataFile.file = fopen(voltageDataFileName, "w");
	dftDataFile.file = fopen(dftDataFileName, "w");
	voltageDataFile.precision = voltageDataFileLogPrecision;
	dftDataFile.precision = dftDataFileLogPrecision;

	// Store information in a single struct
	logData.voltageData = voltageDataFile;
	logData.dftData = dftDataFile;
	
	DAQmxErrChk (DAQmxRegisterEveryNSamplesEvent(taskHandle,everyNsamplesEventType,sampsPerChan,options,EveryNCallback,&logData));
	DAQmxErrChk (DAQmxRegisterDoneEvent(taskHandle,0,DoneCallback,&logData));

	/*********************************************/
	// DAQmx Start Code
	/*********************************************/
	DAQmxErrChk (DAQmxStartTask(taskHandle));
	
	// Print information to the terminal
	printf("\n*********************************************************\n");
	printf("Acquiring samples continuously. Press Enter to interrupt.\n");
	printf("*********************************************************\n\n");
	printf("Sample rate (Hz): %6.2f\n", sampleRate);
	printf("Samples per channel: %llu\n\n", sampsPerChan);
	printf("DSA Samples Acquired\tMIO Samples Acquired\tDetected Signal Frequency (Hz)\t\tPhase Skew (deg)\tPhase Skew (sec)\n");
	getchar();

Error:
	if( DAQmxFailed(error) )
		DAQmxGetExtendedErrorInfo(errBuff,2048);

	/*********************************************/
	// DAQmx Stop Code
	/*********************************************/
	if( taskHandle ) {
		DAQmxStopTask(taskHandle);
		DAQmxClearTask(taskHandle);
		taskHandle = 0;
	}

	if( DAQmxFailed(error) )
		printf("DAQmx Error: %s\n",errBuff);
	printf("End of program, press Enter key to quit");
	getchar();
	return 0;
}

int32 EveryNCallback(TaskHandle taskHandle, int32 everyNsamplesEventType, uInt32 nSamples, void *callbackData)
{
	int32           error=0;
	double 			measuredPhaseSkewDeg=0,measuredPhaseSkewSec=0,measuredFreq=0;
	char            errBuff[2048]={'\0'};
	static int32    dsaTotalRead=0,mioTotalRead=0;
	int32           samplesReadPerChan,dsaRead,mioRead;
	float64         totalData[2*sampsPerChan],dsaData[sampsPerChan],mioData[sampsPerChan],timeData[sampsPerChan];

	// Implicitly cast LogsPtr type def to pointer "data"
	LogsPtr data = (LogsPtr)callbackData;

	/*********************************************/
	// DAQmx Read Code
	/*********************************************/
	DAQmxErrChk (DAQmxReadAnalogF64(taskHandle,sampsPerChan,timeout,fillMode,totalData,2*sampsPerChan,&samplesReadPerChan,NULL));
	
	// Assign the amount of samples read to the respective variables
	if (samplesReadPerChan = sampsPerChan)
	{
		dsaRead = samplesReadPerChan;
		mioRead = samplesReadPerChan;
	}

	// Split data into 2 separate arrays
	int j = 0;
	for (int i = 0; i < sampsPerChan; i++)
	{
		dsaData[i] = totalData[j];
		mioData[i] = totalData[j+1];
		j = j + 2;
	}

	// Perform DFT
	DFT(dsaData,mioData,sampleRate,sampsPerChan,data->dftData.precision,data->dftData.file,&measuredPhaseSkewDeg,&measuredPhaseSkewSec,&measuredFreq);

	// Write to voltage data to CSV 
	fprintf(data->voltageData.file,"Time (s),DSA Data (V),MIO Data (V)\n");
	for (int i = 0; i < sampsPerChan; i++)
	{
		// Build time array
		timeData[i] = i * (1/sampleRate);

		// Print voltage data to CSV file
		fprintf(data->voltageData.file,"%0.*f,%2.*f,%2.*f\n",data->voltageData.precision,timeData[i],data->voltageData.precision,dsaData[i],data->voltageData.precision,mioData[i]);
	}
	// Calculate and print sample acquisition totals and DFT information
	if( dsaRead>0 )
		dsaTotalRead += dsaRead;
	if( mioRead>0 )
		mioTotalRead += mioRead;
	printf("%d\t\t\t%d\t\t\t%5.0f\t\t\t\t\t%2.2f\t\t\t%1.2e\r", (int)dsaTotalRead,(int)mioTotalRead, measuredFreq, measuredPhaseSkewDeg, measuredPhaseSkewSec);
	fflush(stdout);

Error:
	if( DAQmxFailed(error) ) {
		DAQmxGetExtendedErrorInfo(errBuff,2048);
		/*********************************************/
		// DAQmx Stop Code
		/*********************************************/
		if( taskHandle ) {
			DAQmxStopTask(taskHandle);
			DAQmxClearTask(taskHandle);
		}
		printf("DAQmx Error: %s\n",errBuff);
	}
	return 0;
}

int32 DoneCallback(TaskHandle taskHandle, int32 status, void *callbackData)
{
	int32   error=0;
	char    errBuff[2048]={'\0'};

	LogsPtr data = (LogsPtr)callbackData;

	// Check to see if an error stopped the task.
	DAQmxErrChk (status);

	// Close CSV files
	fclose(data->voltageData.file);
	fclose(data->dftData.file);

Error:
	if( DAQmxFailed(error) ) {
		DAQmxGetExtendedErrorInfo(errBuff,2048);
		DAQmxClearTask(taskHandle);
		if( taskHandle ) {
			DAQmxStopTask(taskHandle);
			DAQmxClearTask(taskHandle);
			taskHandle = 0;
		}
		printf("DAQmx Error: %s\n",errBuff);
	}
	return 0;
}

// Calculates a discrete Fourier transform using the FFTW libary
void DFT(double *dsaData, double *mioData, double sampleRate, long long int sampsPerChan, int precision, FILE *file, double *measuredPhaseSkewDeg, double *measuredPhaseSkewSec, double *measuredFreq)
{
	
	// Calculate in and out array sizes
	int n = sampsPerChan;
	int nc = (n/2)+1;

	// Calculate maximum precision of frequency bins
	double binPrecision = sampleRate/sampsPerChan;

	// Instantiate variables for FFTW
	double *dsaInput, *mioInput;
	fftw_complex *dsaOutput, *mioOutput;
	fftw_plan dsaDFT, mioDFT;

	// Allocated input and output arrays
	dsaInput = (double*)fftw_malloc(sizeof(double) * n);
	mioInput = (double*)fftw_malloc(sizeof(double) * n);
	dsaOutput = (fftw_complex*)fftw_malloc(sizeof(fftw_complex) * nc);
	mioOutput = (fftw_complex*)fftw_malloc(sizeof(fftw_complex) * nc);

	// Store real data in the input arrays
	StoreData(n,dsaData,dsaInput);
	StoreData(n,mioData,mioInput);

	// Instantiate plan for a 1D DFT of the real data
	dsaDFT = fftw_plan_dft_r2c_1d(n,dsaInput,dsaOutput,FFTW_ESTIMATE);
	mioDFT = fftw_plan_dft_r2c_1d(n,mioInput,mioOutput,FFTW_ESTIMATE);

	// Execute the DFTs
	fftw_execute(dsaDFT);
	fftw_execute(mioDFT);
	
	// Open CSV file
	fprintf(file,"Frequency (Hz),DSA Magnitude,DSA Amplitude (V),MIO Magnitude,MIO Amplitude (V)\n");

	// Insantiate variables to track the frequency in a measured signal
	int maxMagnitudeIndex = 0;
	int maxMagnitude = 0;

	// Calculate and store the frequency components
	for (int i = 0; i < nc; i++)
	{	
		// Allocate the real and imaginary DFT components of each frequency to variables
		double dsaRealComp = dsaOutput[i][REAL];
		double dsaImagComp = dsaOutput[i][IMAG];
		double mioRealComp = mioOutput[i][REAL];
		double mioImagComp = mioOutput[i][IMAG];
		
		// Calculate the frequency, magnitude, and amplitude from the DFT data
		double freq = i * binPrecision;
		double dsaMagnitude = sqrt((dsaRealComp*dsaRealComp) + (dsaImagComp*dsaImagComp));
		double mioMagnitude = sqrt((mioRealComp*mioRealComp) + (mioImagComp*mioImagComp));
		double dsaAmplitude = dsaMagnitude * 2 / n;
		double mioAmplitude = mioMagnitude * 2 / n;

		// Write DFT data to CSV file
		fprintf(file,"%5.2f,%5.*f,%5.*f,%5.*f,%5.*f\n",freq,precision,dsaMagnitude,precision,dsaAmplitude,precision,mioMagnitude,precision,mioAmplitude);

		// Find at which frequency bin the highest magnitude exists
		if ( dsaMagnitude >= maxMagnitude && mioMagnitude >= maxMagnitude)
		{
			maxMagnitude = dsaMagnitude;
			maxMagnitudeIndex = i;
		}
	}

	// Assign variables for real and imaginary components at the maximum frequency
	double dsaMaxRealComp = dsaOutput[maxMagnitudeIndex][REAL];
	double dsaMaxImagComp = dsaOutput[maxMagnitudeIndex][IMAG];
	double mioMaxRealComp = mioOutput[maxMagnitudeIndex][REAL];
	double mioMaxImagComp = mioOutput[maxMagnitudeIndex][IMAG];

	// Calculate frequency of measured signal
	double maxFreq = maxMagnitudeIndex * binPrecision;

	// Calculate phase
	double dsaPhase = atan2(dsaMaxImagComp,dsaMaxRealComp) * (180/PI);
	double mioPhase = atan2(mioMaxImagComp,mioMaxRealComp) * (180/PI);

	// Calculate phase skew between DSA and MIO device
	double phaseSkewDeg = NormalizePhaseAngleDifference(dsaPhase-mioPhase); // phase skew in degrees
	double phaseSkewSec = ((dsaPhase-mioPhase)/360) * (1/maxFreq); // phase skew in seconds

	// Assign values to be displayed on the console
	*measuredPhaseSkewDeg = phaseSkewDeg;
	*measuredPhaseSkewSec = phaseSkewSec;
	*measuredFreq = maxFreq;

	// Free the resources
	fftw_destroy_plan(dsaDFT);
	fftw_destroy_plan(mioDFT);
	fftw_free(dsaInput); 
	fftw_free(dsaOutput);
	fftw_free(mioInput); 
	fftw_free(mioOutput);
}

// Stores voltage data in arrays of real data to be used in the DFT
void StoreData(int arraySize, double *dataToAllocate, double *output)
{
	// Store data in new array
	for (int i = 0; i < arraySize; i++)
	{
		output[i] = dataToAllocate[i];
	}
}

// Ensures the phase angle is between -180 deg and 180 deg
double NormalizePhaseAngleDifference(double phase)
{
	if (phase > 270)
		phase = phase - 360;
	
	if (phase < -90)
		phase = phase + 360;

	return phase;
}