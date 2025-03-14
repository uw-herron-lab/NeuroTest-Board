## NeuroTest: *In Vitro* test bench for adaptive DBS

### *In Vitro* test bench specification

#### *In Vitro* test bench:
- A Gel Electrophoresis tank filled with 0.9% saline.

#### NeuroTest DAC:
- Cortex M7 600MHz processor 
- Two independent constant current output channels with 16bit resolution 
- Up to 10kHz output sampling rate
- Ability to replay patient data directly from SD card.
- Ability to play data with an amplitude of 1µV RMS for usual DBS electrode dipole.
- Ability to auto calibrate signals for the required electrode type and dipole and location within the in Vitro test bench.

TODO: Insert figure 1 here

#### NeuroTest Sense:
- Two 24bit sensing channels with Sampling frequency up to 8kHz.  With Default ADC sampling frequency set to 2kHz.
- Noise floor of 1µV RMS at 1kHz bandwidth
- Configurable gain and high pass filter, which are set by default to gain of 8 and high pass filter cut-off corner of 311mHz.
- Sensing channels are used to calibrate signal generation in saline, 
- Sensing channels are used to monitoring the performance of the aDBS device under test.
- Configurable built in filters to detect the different stimulation programs for the SUT.
- Ability to record data over 24 hours with PC software over USB to assess the SUT performance.

#### NeuroTest Software:
- SerialPlot version 0.12.0 is used to plot and control NeuroTest board.
- NeuroTest board uses USB serial to output data and to received control commands.
- Upload the setting file attached “NeuroTest board serial plot settings (Cortec).ini”

TODO: Insert figure 2 here

- NeuroTest board has 10 serial outputs which are:
    -   CH1 raw
    -	CH1 with 50Hz notch filter
    -	Ch2 raw
    -	Ch2 with 50Hz notch filter
    -	CH1 stim (filtered between 100-900Hz)
    -	CH1 stim envelop.
    -	CH1 beta: (filtered between 18-22Hz) used for calibrating the system.
    -	CH1 BP: ( this has the band passed data from CH! Notch set to the centre frequency selected in Freq#xx; command ±2Hz).
    -	State: sends a value of 1 when an output signal is enabled, and 0 when an output signal is disabled.
    -	Play state: this sends a value of 1 when an output signal is in sinewave burst or continuous node, 2 when the system output is in patient data output mode.
- NeuroTest starts streaming the 10 output signals at 2kHz when the serial port is opened, without the need for any commands.

#### NeuroTest Serial commands
When the serial port for NeuroTest board is open and the data is streaming, there are 5 commands that can be sent to NeuroTest board to control its functionality.

1.	**Calibration**:
The CAL# without a newline or carriage return must be used if an autocalibration function is required. This requires that the reference electrode connected to CH1 and is submerged in saline and in parallel with SUT electrode as shown in figure 1. 

2.	**Play sinewave data**:
The device allows for playing 3 modes of sinewave data: 
    1. Mode 1: Continuous sinewave with set frequency and amplitude. 
    2. Mode 2: Sinewave burst of fixed duration in 1 second intervals. Minimum 1 second; Maximum 300 seconds. 
    3. Mode 3: Pseudo random sinewave bursts. (ON set between 1,3 seconds, OFF set between 6,60 seconds) this can be changed in code if required.
    
    To play the sinewave the following command is required to be sent without a newline return:
    > PlayB#*Amplitude*, *burst_mode*, *Mode2_burst_ON_t*, *Mode2_burst_OFF_t*;
    
    Example: Set amplitude to 10uV RMS, Mode 2 of fixed bursts, burst ON 2 seconds, burst off 30 seconds.
    > PlayB#10;2;2;30;
    
    By default, the NeuroTest board will play a 20Hz sinewave if this was not changed using the frequency command.
    
***Note***: It is recommended to run this command when the system is first connected before calibration with an amplitude set between 200-500 to make sure that everything is connected correctly and working as it should. If the system seems to be connected correctly then send a stop command and start the calibration process, which after that you can play the required test with the required specification.

3.	**Stop command**:
To stop whichever current mode the NeruoTest board is currently in a Stop# command must be sent without a newline return.

4.	**Change Sinewave Frequency**:
The NeuroTest board has a 20Hz frequency set by default to change this the Frequency command must be used. The command is as follows:
    > Freq#XX;

    Example: set Frequency to 10Hz. 
    > Freq#10;
    
    This command can be used after the PlayB command to change frequency halfway through the test.

5.	**Play patient data**:
The NeuroTest board can play prerecorded patient data into the saline tank using the microSD card on board. The patient data file needs to be loaded into the microSD card, and it has the have the following format:
    1.	The file needs to be in a comma separated format (CSV)
    2.	Cell 0 of the data must be the sampling frequency of the data. The system will allow sampling frequency between 500Hz-2kHz.
    3.	 Cell 1 of the data must be the total length of the patient data.
    4.	Cell 2 onwards is the actual patient data in microvolts.
    5.  Load the data to the microSD card and inserting the microSD card into the NeuroTest board.

    TODO: Insert figure 3 here

    6. Open SerialPlot software and start the data streaming by opening the NeuroTest board serial port.
    7. Calibrate the system by sending the calibration command.
    8. To play the patient data file the following command must be sent with the name of the file and the extension followed by the “;” and no line return. The command: PlayF#xxxxx.csv;
    9. If the file does not exist, the NeuroTest board will reject the command and print an error message that can be viewed in the log tap of the SerialPlot software.
    10.The NeuroTest board will stop replaying the patient data once it reaches the end of the file, or if the stop command is sent.
    11. If the user requires to stop the reply of the patient data, the stop command can be used.
    12. The Stop# command must be used if the patient data is playing, and the user requires to unplug the device from the PC or remove the microSD card. As failing to do so might result in corrupting the microSD card.


6. **Evoked potential**:
To emulate evoked potential activity, the following steps must be followed:
    1.	If the system is not calibrated, run the CAL# function as described in section 1.
    2.	Set the required main frequency for the evoked potential as shown in section 4.
    3.	Send the Evoked potential command which structure as follows:
        > PlayEv# *Signal Amplitude in µV*, *stim amplitude in µV*, *evoked potential mode*, *time delay in ms*;

        Example: Set evoked potential to 2Hz, 100uV RMS, if stimulation detected is above 400uV, with evoked potential mode 1, with a 10 ms delay.
        > Serial commands:
        >
        > 1. Stop any previous mode: Stop#
        > 2. Set frequency to 2Hz: Freq#2; 
        > 3. Set evoked potential: PlayEv#100,400,1,10;
        
        There are 3 modes of evoked potential that can be selected:

        1. Evoked potential mode 1: Consists of single frequency sine wave as shown in figure 4

        TODO: Insert figure 4 here

        2. Evoked potential mode 2: Consists of single frequency cosine wave as illustrated in figure 5.

        TODO: Insert figure 5 here

        3. Evoked potential mode 3: Consists of high frequency negative half sine wave, followed by low frequency full sine wave. If the selected base frequency is between 1-5Hz the high frequency part is equal to Low selected low frequency X 20. If the selected base frequency is above 5Hz the high frequency part is limited to 100Hz as illustrated in figure 6

        TODO: Insert figure 6 here
7. **Gain command**: A gain can be set to scale the signal accordingly if the calibration function cannot be used or its not optimal. 
    The gain can be set as a percentage starting at 1% up to 1000% ( a factor of 10 max). The gain factor is set to 100% by default. 

    The gain command is as follows: 
    > GAIN#*xx*; (where xx is the required percentage)

    Example to set a gain factor of 50%, the following command is used:
    > GAIN#50;





