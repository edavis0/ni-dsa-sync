/*********************************************************************
*
* ANSI C Example program:
*    ContinuousAI.c
*
* Example Category:
*    Sync
*
* Description:
*    This example demonstrates how to acquire a continuous amount of
*    data using a reference clock with a DSA device and a 
*    multifunction IO device.
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
*    1. Create a task.
*    2. Create an analog input voltage channel for both the Master
*       and Slave devices.
*    3. Set timing parameters. Note that sample mode is set to
*       Continuous Samples. In this example, the Rate and the Samples
*       per Channel is set the same for both devices, you can however
*       use different values for each device.
*    4. The synchronization method chosen depends on what type of
*       device you are using.
*    5. Call the Get Terminal Name with Device Prefix utility
*       function. This will take a Task and a terminal and create a
*       properly formatted device + terminal name to use as the
*       source of the Slaves Trigger. For the Slave, set the Source
*       for the trigger to the ai/StartTrigger of the Master Device.
*       This will ensure both devices start sampling at the same
*       time. (Note: The trigger is automatically routed through the
*       RTSI cable.)
*    6. Call the Start function to start the acquisition.
*    7. Read all of the data continuously. The 'Samples per Channel'
*       control will specify how many samples per channel are read
*       each time. If either device reports an error or the user
*       presses the 'Stop' button, the acquisition will stop.
*    8. Call the Clear Task function to clear the task.
*    9. Display an error if any.
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
#include <unistd.h>
#include <NIDAQmx.h>

static TaskHandle DSATaskHandle=0,MIOTaskHandle=0;

#define DAQmxErrChk(functionCall) if( DAQmxFailed(error=(functionCall)) ) goto Error; else

static int32 GetTerminalNameWithDevPrefix(TaskHandle taskHandle, const char terminalName[], char triggerName[]);

int32 CVICALLBACK EveryNCallback(TaskHandle taskHandle, int32 everyNsamplesEventType, uInt32 nSamples, void *callbackData);
int32 CVICALLBACK DoneCallback(TaskHandle taskHandle, int32 status, void *callbackData);

/*********************************************/
// DAQmx Configuration Options
/*********************************************/
// Sampling Options
const float64 sampleRate = 1000.0; // The sampling rate in samples per second per channel.
const uInt64 sampsPerChan = 100; // The number of samples to acquire or generate for each channel in the task.

/* DSA Device Options */
// DAQmxCreateAIVoltageChan Options
const char *physicalChannelDSA = "Dev1/ai0"; // The names of the physical channels to use to create virtual channels. You can specify a list or range of physical channels.
const int32 terminalConfigDSA = DAQmx_Val_Cfg_Default; // The input terminal configuration for the channel. Options: DAQmx_Val_Cfg_Default, DAQmx_Val_RSE, DAQmx_Val_NRSE, DAQmx_Val_Diff, DAQmx_Val_PseudoDiff
const float64 minValDSA = -5.0; // The minimum value, in units, that you expect to measure.
const float64 maxValDSA = 5.0; // The maximum value, in units, that you expect to measure.
const int32 unitsDSA = DAQmx_Val_Volts; // The units to use to return the voltage measurements. Options: DAQmx_Val_Volts, DAQmx_Val_FromCustomScale
// DAQmxCfgSampClkTiming Options
const int32 activeEdgeDSA = DAQmx_Val_Rising; // Specifies on which edge of the clock to acquire or generate samples. Options: DAQmx_Val_Rising, DAQmx_Val_Falling
const int32 sampleModeDSA = DAQmx_Val_ContSamps; // Specifies whether the task acquires or generates samples continuously or if it acquires or generates a finite number of samples. Options: DAQmx_Val_FiniteSamps, DAQmx_Val_ContSamps, DAQmx_Val_HWTimedSinglePoint

/* MIO Device Options */
// DAQmxCreateAIVoltageChan Options
const char *physicalChannelMIO = "Dev3/ai0"; // The names of the physical channels to use to create virtual channels. You can specify a list or range of physical channels.
const int32 terminalConfigMIO = DAQmx_Val_Cfg_Default; // The input terminal configuration for the channel. Options: DAQmx_Val_Cfg_Default, DAQmx_Val_RSE, DAQmx_Val_NRSE, DAQmx_Val_Diff, DAQmx_Val_PseudoDiff
const float64 minValMIO = -5.0; // The minimum value, in units, that you expect to measure.
const float64 maxValMIO = 5.0; // The maximum value, in units, that you expect to measure.
const int32 unitsMIO = DAQmx_Val_Volts; // The units to use to return the voltage measurements. Options: DAQmx_Val_Volts, DAQmx_Val_FromCustomScale
// DAQmxCfgSampClkTiming Options
const int32 activeEdgeMIO = DAQmx_Val_Rising; // Specifies on which edge of the clock to acquire or generate samples. Options: DAQmx_Val_Rising, DAQmx_Val_Falling
const int32 sampleModeMIO = DAQmx_Val_ContSamps; // Specifies whether the task acquires or generates samples continuously or if it acquires or generates a finite number of samples. Options: DAQmx_Val_FiniteSamps, DAQmx_Val_ContSamps, DAQmx_Val_HWTimedSinglePoint

// Reference Clock Options
const char *refClkSrc = "PXI_Clk10"; // Specifies the terminal of the signal to use as the Reference Clock.
// DAQmxRegisterEveryNSamplesEvent Options
const int32 everyNsamplesEventType = DAQmx_Val_Acquired_Into_Buffer; // The type of event you want to receive. Options: DAQmx_Val_Acquired_Into_Buffer, DAQmx_Val_Transferred_From_Buffer
const uInt32 options = 0; // Use this parameter to set certain options. Pass a value of zero if no options need to be set. Options: 0, DAQmx_Val_SynchronousEventCallbacks
// DAQmxReadAnalogF64 Options
const float64 timeout = 10.0; // The amount of time, in seconds, to wait for the function to read the sample(s).
const bool32 fillMode = DAQmx_Val_GroupByChannel; // Specifies whether or not the samples are interleaved. Options: DAQmx_Val_GroupByChannel, DAQmx_Val_GroupByScanNumber

// Create file pointer
FILE *dataFile;

int main(void)
{
	int32       error=0;
	char        errBuff[2048]={'\0'};
	char	    str1[256],str2[256],trigName[256];

 	/*********************************************/
	// DAQmx Configure Code
	/*********************************************/
	DAQmxErrChk (DAQmxCreateTask("",&DSATaskHandle));
	DAQmxErrChk (DAQmxCreateAIVoltageChan(DSATaskHandle,physicalChannelDSA,"",terminalConfigDSA,minValDSA,maxValDSA,unitsDSA,NULL));
	DAQmxErrChk (DAQmxCfgSampClkTiming(DSATaskHandle,"",sampleRate,activeEdgeDSA,sampleModeDSA,sampsPerChan));
	DAQmxErrChk (DAQmxSetAIRemoveFilterDelay(DSATaskHandle,physicalChannelDSA, 1));

	DAQmxErrChk (DAQmxCreateTask("",&MIOTaskHandle));
	DAQmxErrChk (DAQmxCreateAIVoltageChan(MIOTaskHandle,physicalChannelMIO,"",terminalConfigMIO,minValMIO,maxValMIO,unitsMIO,NULL));
	DAQmxErrChk (DAQmxCfgSampClkTiming(MIOTaskHandle,"",sampleRate,activeEdgeMIO,sampleModeMIO,sampsPerChan));

	DAQmxErrChk (DAQmxSetRefClkSrc(DSATaskHandle, refClkSrc));
	DAQmxErrChk (DAQmxSetRefClkSrc(MIOTaskHandle, refClkSrc));
	DAQmxErrChk (DAQmxGetStartTrigTerm(DSATaskHandle, trigName, sizeof(trigName)));
	DAQmxErrChk (DAQmxCfgDigEdgeStartTrig(MIOTaskHandle,trigName,DAQmx_Val_Rising));
	
	DAQmxErrChk (DAQmxRegisterEveryNSamplesEvent(DSATaskHandle,everyNsamplesEventType,sampsPerChan,options,EveryNCallback,NULL));
	DAQmxErrChk (DAQmxRegisterDoneEvent(DSATaskHandle,0,DoneCallback,NULL));

	// Create/open CSV file
	dataFile = fopen("../../SyncData.csv", "w+");
	fprintf(dataFile,"Time (s),DSA Data (V),MIO Data (V)\n");
	fclose(dataFile);

	/*********************************************/
	// DAQmx Start Code
	/*********************************************/
	// The slave device is armed before the master so that the slave device does not miss the trigger.
	DAQmxErrChk (DAQmxStartTask(MIOTaskHandle));
	DAQmxErrChk (DAQmxStartTask(DSATaskHandle));
	
	printf("Acquiring samples continuously. Press Enter to interrupt\n");
	printf("\nRead:\tMaster\tSlave\tTotal:\tMaster\tSlave\n");
	getchar();

Error:
	if( DAQmxFailed(error) )
		DAQmxGetExtendedErrorInfo(errBuff,2048);

	/*********************************************/
	// DAQmx Stop Code
	/*********************************************/
	if( DSATaskHandle ) {
		DAQmxStopTask(DSATaskHandle);
		DAQmxClearTask(DSATaskHandle);
		DSATaskHandle = 0;
	}
	if( MIOTaskHandle ) {
		DAQmxStopTask(MIOTaskHandle);
		DAQmxClearTask(MIOTaskHandle);
		MIOTaskHandle = 0;
	}

	if( DAQmxFailed(error) )
		printf("DAQmx Error: %s\n",errBuff);
	printf("End of program, press Enter key to quit");
	getchar();
	return 0;
}

int32 CVICALLBACK EveryNCallback(TaskHandle taskHandle, int32 everyNsamplesEventType, uInt32 nSamples, void *callbackData)
{
	int32           error=0;
	char            errBuff[2048]={'\0'};
	static int32    masterTotal=0,slaveTotal=0,filterdelay=0;
	int32           masterRead,slaveRead;
	float64         masterData[sampsPerChan],slaveData[sampsPerChan],timeData[sampsPerChan];

	/*********************************************/
	// DAQmx Read Code
	/*********************************************/
	// Remove intitial samples from DSA device
/* 	if (filterdelay = 0)
	{
		DAQmxErrChk (DAQmxReadAnalogF64(DSATaskHandle,64,timeout,fillMode,masterData,64,&masterRead,NULL));
		filterdelay = 1;
	} */

	DAQmxErrChk (DAQmxReadAnalogF64(DSATaskHandle,sampsPerChan,timeout,fillMode,masterData,sampsPerChan,&masterRead,NULL));
	DAQmxErrChk (DAQmxReadAnalogF64(MIOTaskHandle,sampsPerChan,timeout,fillMode,slaveData,sampsPerChan,&slaveRead,NULL));

	// Create time array
	for(int i = 1; i < sampsPerChan; i++)
	{
		timeData[i] = i * (1/sampleRate);
	}

	// Open CSV file
	dataFile = fopen("../../SyncData.csv", "a");

	// Write to CSV file
	for (int i = 0; i < sampsPerChan; i++)
	{
		fprintf(dataFile,"%0.3f,%0.2f,%0.2f\n",timeData[i],masterData[i],slaveData[i]);
	}
	fclose(dataFile);

	// Calculate and print sample acquisition totals
	if( masterRead>0 )
		masterTotal += masterRead;
	if( slaveRead>0 )
		slaveTotal += slaveRead;
	printf("\t%d\t%d\t\t%d\t%d\r",(int)masterRead,(int)slaveRead,(int)masterTotal,(int)slaveTotal);
	fflush(stdout);

Error:
	if( DAQmxFailed(error) ) {
		DAQmxGetExtendedErrorInfo(errBuff,2048);
		/*********************************************/
		// DAQmx Stop Code
		/*********************************************/
		if( DSATaskHandle ) {
			DAQmxStopTask(DSATaskHandle);
			DAQmxClearTask(DSATaskHandle);
		}
		if( MIOTaskHandle ) {
			DAQmxStopTask(MIOTaskHandle);
			DAQmxClearTask(MIOTaskHandle);
		}
		printf("DAQmx Error: %s\n",errBuff);
	}
	return 0;
}

int32 CVICALLBACK DoneCallback(TaskHandle taskHandle, int32 status, void *callbackData)
{
	int32   error=0;
	char    errBuff[2048]={'\0'};

	// Check to see if an error stopped the task.
	DAQmxErrChk (status);

Error:
	if( DAQmxFailed(error) ) {
		DAQmxGetExtendedErrorInfo(errBuff,2048);
		DAQmxClearTask(taskHandle);
		if( MIOTaskHandle ) {
			DAQmxStopTask(MIOTaskHandle);
			DAQmxClearTask(MIOTaskHandle);
			MIOTaskHandle = 0;
		}
		printf("DAQmx Error: %s\n",errBuff);
	}
	return 0;
}
