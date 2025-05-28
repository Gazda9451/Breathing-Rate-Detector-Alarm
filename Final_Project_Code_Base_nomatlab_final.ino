
// OPEN A NEW SKETCH WINDOW IN ARDUINO
// CLICK IN THIS BOX, CTL-A, CTL-C (Copy code from text box.)
// CLICK IN SKETCH, CTL-A, CTL-V (Paste code into sketch.)

// Breathing Rate Detection System -- Final Integration
//
// Pieced together from code created by: Clark Hochgraf and David Orlicki Oct 18, 2018
// Modified by: Mark Thompson April 2020 to integrate MATLAB read and Write 
//              and integrate the system

#include <MsTimer2.h>
#include <SPI.h>
#include <Tone2.h>

const int TSAMP_MSEC = 100;
const int NUM_SAMPLES = 2399;  // 3600;
const int NUM_SUBSAMPLES = 160;
const int DAC0 = 3, DAC1 = 4, DAC2 = 5, LM61 = A0, VDITH = A1;
const int V_REF = 5.0;
const int SPKR = 12; // d12  PB4

volatile boolean sampleFlag = false;

const long DATA_FXPT = 1000; // Scale value to convert from float to fixed
const float INV_FXPT = 1.0 / DATA_FXPT; // division slow: precalculate


int nSmpl = 1, sample;
float xv, yv, yv_LP, yv_BP, yv_HP, stdLF, stdMF, stdHF;
float printArray[12];
int numValues = 0;



int loopTick = 0;
bool statsReset;
bool isToneEn = false;

unsigned long startUsec, endUsec, execUsec;


//  Define a structure to hold statistics values for each filter band
struct stats_t
{
  int tick = 1;
  float mean, var, stdev;
} statsLF, statsMF, statsHF;

// tone constants 
  Tone tone2; 
  Tone tone1;


//**********************************************************************
void setup()
{

  configureArduino();
  Serial.begin(115200);delay(5);

   //Handshake with MATLAB 
 // Serial.println(F("%Arduino Ready"));
 // while (Serial.read() != 'g'); // spin

  MsTimer2::set(TSAMP_MSEC, ISR_Sample); // Set sample msec, ISR name
  MsTimer2::start(); // start running the Timer  

  //tone setup
  tone2.begin(13);
  tone1.begin(7);
  // tone1.play(NOTE_F5, 2000);
  //tone timer interupt 

   


}


////**********************************************************************
void loop()
{

  // syncSample();  // Wait for the interupt when actually reading ADC data

  
  // Breathing Rate Detection

  // Declare variables

  float readValue, floatOutput;  //  Input data from ADC after dither averaging or from MATLAB
  long fxdInputValue, lpfInput, lpfOutput;  
  long eqOutput;  //  Equalizer output
  int alarmCode;  //  Alarm code


  // ******************************************************************
  //  When finding the impulse responses of the filters use this as an input
  //  Create a Delta function in time with the first sample a 1 and all others 0
 //xv = (loopTick == 0) ? 1.0 : 0.0; // impulse test input

  // ******************************************************************
  //  Use this when the test vector generator is used as an input
 // xv = testVector();


  // ******************************************************************
  //  Read input value in ADC counts  -- Get simulated data from MATLAB
//  readValue = ReadFromMATLAB();


  // ******************************************************************
  //  Read input value from ADC using Dithering, and averaging
  readValue = analogReadDitherAve();

  startUsec = micros();
  //  Convert the floating point number to a fixed point value.  First
  //  scale the floating point value by a number to increase its resolution
  //  (use DATA_FXPT).  Then round the value and truncate to a fixed point
  //  INT datatype

  fxdInputValue = long(DATA_FXPT * readValue + 0.5);

 
  

  //  Execute the equalizer
  //if (fxdInputValue > 0) maybe add a delay after code so it waits for input 
  //{
  eqOutput = EqualizerFIR( fxdInputValue, loopTick );
  //}
  //  Execute the noise filter.  
  // eqOutput = NoiseFilter( eqOutput, loopTick );
   lpfOutput = IIR_Inital(eqOutput);

  //  Convert the output of the equalizer by scaling floating point
  xv = float(lpfOutput* INV_FXPT);


  //*******************************************************************
  // Uncomment this when measuring execution times
  

  // ******************************************************************
  //  Compute the output of the filter using the cascaded SOS sections.
   yv_LP = IIR_LP(xv); // second order systems cascade  

   yv_HP = IIR_HP(xv);

   yv_BP = IIR_BP(xv);



  //  Compute the latest output of the running stats for the output of the filters.
  //  Pass the entire set of output values, the latest stats structure and the reset flag

  // stats Low Pass 
    statsReset = (statsLF.tick%100 == 0);
    getStats( yv_LP, statsLF, statsReset);
    stdLF = statsLF.stdev;

  // stats High Pass
    statsReset = (statsHF.tick%100 == 0);
    getStats( yv_HP, statsHF, statsReset);
    stdHF = statsHF.stdev;

  // stats Band Pass 
    statsReset = (statsMF.tick%100 == 0);
    getStats( yv_BP, statsMF, statsReset);
    stdMF = statsMF.stdev;
  //*******************************************************************
  // Uncomment this when measuring execution times
   

  //  Call the alarm check function to determine what breathing range 
    alarmCode = AlarmCheck( stdLF, stdMF, stdHF );

  //  Call the alarm function to turn on or off the tone
    setAlarm(alarmCode, isToneEn );

  endUsec = micros();
  execUsec = execUsec + (endUsec-startUsec);
 // To print data to the serial port, use the WriteToSerial function.  
 //
 //  This is a generic way to print out variable number of values
 //
 //  There are two input arguments to the function:
 //  printArray -- An array of values that are to be printed starting with the first column
 //  numValues -- An integer indicating the number of values in the array.  
 
   printArray[0] = loopTick;  //  The sample number -- always print this
   printArray[1] = readValue;        //  Column 2
   printArray[2] = xv;       //  Column 3
   printArray[3] = yv_LP;       //  Column 4, etc...
   printArray[4] = yv_BP;
   printArray[5] = yv_HP;
   printArray[6] = stdLF;
   printArray[7] = stdMF;
   printArray[8] = stdHF;
   printArray[9] = alarmCode;
   printArray[10] = execUsec;
 //  printArray[11] = execUsec;
   

   numValues = 11;  // The number of columns to be sent to the serial monitor (or MATLAB)

 WriteToSerial( numValues, printArray );  //  Write to the serial monitor (or MATLAB)

  if ((++loopTick <= 0) || (++loopTick >= 0)){
    //Serial.print("Average execution time (uSec) = ");Serial.println( float(execUsec)/NUM_SAMPLES );
    while(true); // spin forever
  }

} // loop()

//******************************************************************
int AlarmCheck( float stdLF, float stdMF, float stdHF)
{
static int Fail_high, alarmCode, MF_high, LF_high, HF_high = 0; 
// for sweep data use 0.25
// for breathingdata.mat, breathingdata2.mat, and breathingdata3.mat use 0.09
// for reading data from the sensor use 0.25
const float LF_noise_floor = 0.25;
const float MF_noise_floor = 0.25;
const float HF_noise_floor = 0.25;
  
if ((stdMF < MF_noise_floor) && (stdLF < LF_noise_floor) && (stdHF < HF_noise_floor))
{
  if (Fail_high > 10)
  {
    alarmCode = 4;
    Fail_high = 0;
    LF_high = 0;
    MF_high = 0;
    HF_high = 0;
  }
  else
  {
    Fail_high++;
  }
}
else if ((stdMF > stdLF) && (stdMF > stdHF) && (stdMF > MF_noise_floor))
{
  if (MF_high > 10)
  {
    alarmCode = 1;
    Fail_high = 0;
    LF_high = 0;
    MF_high = 0;
    HF_high = 0;
  }
  else
  {
    MF_high++;
  }
}
else if ((stdLF > stdMF) && (stdLF > stdHF) && (stdLF > LF_noise_floor))
{
  if (LF_high > 10)
 {
    alarmCode = 2;
    Fail_high = 0;
    LF_high = 0;
    MF_high = 0;
    HF_high = 0;
 }
  else
  {
   LF_high++;
  }
}
else if ((stdHF > stdLF) && (stdHF > stdMF) && (stdHF > HF_noise_floor))
{
  if (HF_high > 10)
  {
    alarmCode = 3;
    Fail_high = 0;
    LF_high = 0;
    MF_high = 0;
    HF_high = 0;
  }
  else
  {
    HF_high++;
  }
}
else 
{
  alarmCode = 0;
}
//  Your alarm check logic code will 
	//system is reading MF freq 
	

return alarmCode;
}  // end AlarmCheck
 


//*******************************************************************


int FIR_Generic(long inputX, int sampleNumber)
{   
  // Starting with a generic FIR filter impelementation customize only by
  // changing the length of the filter using MFILT and the values of the
  // impulse response in h

  // Filter type: FIR
  
  //
  //  Set the constant HFXPT to the sum of the values of the impulse response
  //  This is to keep the gain of the impulse response at 1.
  //
  const int HFXPT = 1, MFILT = 4;
  
  int h[] = {};


 
  int i;
  const float INV_HFXPT = 1.0/HFXPT;
  static long xN[MFILT] = {inputX}; 
  long yOutput = 0;

  //
  // Right shift old xN values. Assign new inputX to xN[0];
  //
  for ( i = (MFILT-1); i > 0; i-- )
  {
    xN[i] = xN[i-1];
  }
   xN[0] = inputX;
  
  //
  // Convolve the input sequence with the impulse response
  //
  
  for ( i = 0; i < MFILT; i++)
  {
    
    // Explicitly cast the impulse value and the input value to LONGs then multiply
    // by the input value.  Sum up the output values
    
    yOutput = yOutput + long(h[i]) * long( xN[i] );
  }

  //  Return the output, but scale by 1/HFXPT to keep the gain to 1
  //  Then cast back to an integer
  //

  // Skip the first MFILT  samples to avoid the transient at the beginning due to end effects
  if (sampleNumber < MFILT ){
    return long(0);
  }else{
    return long(float(yOutput) * INV_HFXPT);
  }
}
//*******************************************************************************
float IIR_Inital(float xv)
{  


  //  ***  Copy variable declarations from MATLAB generator to here  ****

//Filter specific variable declarations
const int numStages = 4;
static float G[numStages];
static float b[numStages][3];
static float a[numStages][3];


//  *** Stop copying MATLAB variable declarations here
  
  int stage;
  int i;
  static float xM0[numStages] = {0.0}, xM1[numStages] = {0.0}, xM2[numStages] = {0.0};
  static float yM0[numStages] = {0.0}, yM1[numStages] = {0.0}, yM2[numStages] = {0.0};
  
  float yv = 0.0;
  unsigned long startTime;



//  ***  Copy variable initialization code from MATLAB generator to here  ****


//BWRTH low, order 7, 70 BPM

G[0] = 0.1224934;
b[0][0] = 1.0000000; b[0][1] = 0.9927070; b[0][2]= 0.0000000;
a[0][0] = 1.0000000; a[0][1] =  -0.4452287; a[0][2] =  0.0000000;
G[1] = 0.1224934;
b[1][0] = 1.0000000; b[1][1] = 2.0134915; b[1][2]= 1.0135479;
a[1][0] = 1.0000000; a[1][1] =  -0.9272701; a[1][2] =  0.2477651;
G[2] = 0.1224934;
b[2][0] = 1.0000000; b[2][1] = 2.0030945; b[2][2]= 1.0031498;
a[2][0] = 1.0000000; a[2][1] =  -1.0487537; a[2][2] =  0.4112373;
G[3] = 0.1224934;
b[3][0] = 1.0000000; b[3][1] = 1.9907069; b[3][2]= 0.9907608;
a[3][0] = 1.0000000; a[3][1] =  -1.2936682; a[3][2] =  0.7408023;

//  **** Stop copying MATLAB code here  ****



  //  Iterate over each second order stage.  For each stage shift the input data
  //  buffer ( x[kk] ) by one and the output data buffer by one ( y[k] ).  Then bring in 
  //  a new sample xv into the buffer;
  //
  //  Then execute the recusive filter on the buffer
  //
  //  y[k] = -a[2]*y[k-2] + -a[1]*y[k-1] + g*b[0]*x[k] + b[1]*x[k-1] + b[2]*x[k-2] 
  //
  //  Pass the output from this stage to the next stage by setting the input
  //  variable to the next stage x to the output of the current stage y
  //  
  //  Repeat this for each second order stage of the filter

  
  for (i =0; i<numStages; i++)
    {
      yM2[i] = yM1[i]; yM1[i] = yM0[i];  xM2[i] = xM1[i]; xM1[i] = xM0[i], xM0[i] = G[i]*xv;
      yv = -a[i][2]*yM2[i] - a[i][1]*yM1[i] + b[i][2]*xM2[i] + b[i][1]*xM1[i] + b[i][0]*xM0[i];
      yM0[i] = yv;
      xv = yv;
    }
//
//  execUsec += micros()-startTime;
  
  return yv;
}
//*******************************************************************************
float IIR_LP(float xv)
{  


  //  ***  Copy variable declarations from MATLAB generator to here  ****

//Filter specific variable declarations
const int numStages = 5;
static float G[numStages];
static float b[numStages][3];
static float a[numStages][3];

//  *** Stop copying MATLAB variable declarations here
  
  int stage;
  int i;
  static float xM0[numStages] = {0.0}, xM1[numStages] = {0.0}, xM2[numStages] = {0.0};
  static float yM0[numStages] = {0.0}, yM1[numStages] = {0.0}, yM2[numStages] = {0.0};
  
  float yv = 0.0;
  unsigned long startTime;



//  ***  Copy variable initialization code from MATLAB generator to here  ****

//BWRTH low, order 9, 12 BPM

G[0] = 0.0064017;
b[0][0] = 1.0000000; b[0][1] = 0.9636095; b[0][2]= 0.0000000;
a[0][0] = 1.0000000; a[0][1] =  -0.8816431; a[0][2] =  0.0000000;
G[1] = 0.0064017;
b[1][0] = 1.0000000; b[1][1] = 2.0700639; b[1][2]= 1.0714584;
a[1][0] = 1.0000000; a[1][1] =  -1.7751479; a[1][2] =  0.7892567;
G[2] = 0.0064017;
b[2][0] = 1.0000000; b[2][1] = 2.0363846; b[2][2]= 1.0377629;
a[2][0] = 1.0000000; a[2][1] =  -1.8103904; a[2][2] =  0.8247824;
G[3] = 0.0064017;
b[3][0] = 1.0000000; b[3][1] = 1.9862718; b[3][2]= 0.9876257;
a[3][0] = 1.0000000; a[3][1] =  -1.8672247; a[3][2] =  0.8820653;
G[4] = 0.0064017;
b[4][0] = 1.0000000; b[4][1] = 1.9436702; b[4][2]= 0.9450028;
a[4][0] = 1.0000000; a[4][1] =  -1.9419652; a[4][2] =  0.9573997;


//  **** Stop copying MATLAB code here  ****



  //  Iterate over each second order stage.  For each stage shift the input data
  //  buffer ( x[kk] ) by one and the output data buffer by one ( y[k] ).  Then bring in 
  //  a new sample xv into the buffer;
  //
  //  Then execute the recusive filter on the buffer
  //
  //  y[k] = -a[2]*y[k-2] + -a[1]*y[k-1] + g*b[0]*x[k] + b[1]*x[k-1] + b[2]*x[k-2] 
  //
  //  Pass the output from this stage to the next stage by setting the input
  //  variable to the next stage x to the output of the current stage y
  //  
  //  Repeat this for each second order stage of the filter

  
  for (i =0; i<numStages; i++)
    {
      yM2[i] = yM1[i]; yM1[i] = yM0[i];  xM2[i] = xM1[i]; xM1[i] = xM0[i], xM0[i] = G[i]*xv;
      yv = -a[i][2]*yM2[i] - a[i][1]*yM1[i] + b[i][2]*xM2[i] + b[i][1]*xM1[i] + b[i][0]*xM0[i];
      yM0[i] = yv;
      xv = yv;
    }
//
//  execUsec += micros()-startTime;
  
  return yv;
}

//*******************************************************************************
float IIR_HP(float xv)
{  


  //  ***  Copy variable declarations from MATLAB generator to here  ****

//Filter specific variable declarations
const int numStages = 11;
static float G[numStages];
static float b[numStages][3];
static float a[numStages][3];


//  *** Stop copying MATLAB variable declarations here
  
  int stage;
  int i;
  static float xM0[numStages] = {0.0}, xM1[numStages] = {0.0}, xM2[numStages] = {0.0};
  static float yM0[numStages] = {0.0}, yM1[numStages] = {0.0}, yM2[numStages] = {0.0};
  
  float yv = 0.0;
  unsigned long startTime;



//  ***  Copy variable initialization code from MATLAB generator to here  ****


//BWRTH high, order 21, 40 BPM

G[0] = 0.7731454;
b[0][0] = 1.0000000; b[0][1] = -0.6981012; b[0][2]= 0.0000000;
a[0][0] = 1.0000000; a[0][1] =  -0.9142523; a[0][2] =  0.0000000;
G[1] = 0.7731454;
b[1][0] = 1.0000000; b[1][1] = -2.8183211; b[1][2]= 1.9907878;
a[1][0] = 1.0000000; a[1][1] =  -1.1127368; a[1][2] =  0.3108313;
G[2] = 0.7731454;
b[2][0] = 1.0000000; b[2][1] = -2.7141587; b[2][2]= 1.8827651;
a[2][0] = 1.0000000; a[2][1] =  -1.1344318; a[2][2] =  0.3332237;
G[3] = 0.7731454;
b[3][0] = 1.0000000; b[3][1] = -2.5238594; b[3][2]= 1.6840180;
a[3][0] = 1.0000000; a[3][1] =  -1.1788995; a[3][2] =  0.3789221;
G[4] = 0.7731454;
b[4][0] = 1.0000000; b[4][1] = -2.2876542; b[4][2]= 1.4359297;
a[4][0] = 1.0000000; a[4][1] =  -1.2491235; a[4][2] =  0.4501706;
G[5] = 0.7731454;
b[5][0] = 1.0000000; b[5][1] = -1.4108462; b[5][2]= 0.5033420;
a[5][0] = 1.0000000; a[5][1] =  -1.3491612; a[5][2] =  0.5492754;
G[6] = 0.7731454;
b[6][0] = 1.0000000; b[6][1] = -1.4568935; b[6][2]= 0.5534098;
a[6][0] = 1.0000000; a[6][1] =  -1.4793439; a[6][2] =  0.6740481;
G[7] = 0.7731454;
b[7][0] = 1.0000000; b[7][1] = -2.0486163; b[7][2]= 1.1838350;
a[7][0] = 1.0000000; a[7][1] =  -1.6296325; a[7][2] =  0.8125635;
G[8] = 0.7731454;
b[8][0] = 1.0000000; b[8][1] = -1.5397927; b[8][2]= 0.6428544;
a[8][0] = 1.0000000; a[8][1] =  -1.8084677; a[8][2] =  0.8505826;
G[9] = 0.7731454;
b[9][0] = 1.0000000; b[9][1] = -1.6654145; b[9][2]= 0.7773164;
a[9][0] = 1.0000000; a[9][1] =  -1.7682675; a[9][2] =  0.8930963;
G[10] = 0.7731454;
b[10][0] = 1.0000000; b[10][1] = -1.8363422; b[10][2]= 0.9591112;
a[10][0] = 1.0000000; a[10][1] =  -1.7710512; a[10][2] =  0.9430586;

//  **** Stop copying MATLAB code here  ****



  //  Iterate over each second order stage.  For each stage shift the input data
  //  buffer ( x[kk] ) by one and the output data buffer by one ( y[k] ).  Then bring in 
  //  a new sample xv into the buffer;
  //
  //  Then execute the recusive filter on the buffer
  //
  //  y[k] = -a[2]*y[k-2] + -a[1]*y[k-1] + g*b[0]*x[k] + b[1]*x[k-1] + b[2]*x[k-2] 
  //
  //  Pass the output from this stage to the next stage by setting the input
  //  variable to the next stage x to the output of the current stage y
  //  
  //  Repeat this for each second order stage of the filter

  
  for (i =0; i<numStages; i++)
    {
      yM2[i] = yM1[i]; yM1[i] = yM0[i];  xM2[i] = xM1[i]; xM1[i] = xM0[i], xM0[i] = G[i]*xv;
      yv = -a[i][2]*yM2[i] - a[i][1]*yM1[i] + b[i][2]*xM2[i] + b[i][1]*xM1[i] + b[i][0]*xM0[i];
      yM0[i] = yv;
      xv = yv;
    }
//
//  execUsec += micros()-startTime;
  
  return yv;
}

//*******************************************************************
float IIR_BP(float xv)
{  


  //  ***  Copy variable declarations from MATLAB generator to here  ****

//Filter specific variable declarations
const int numStages = 5;
static float G[numStages];
static float b[numStages][3];
static float a[numStages][3];

//  *** Stop copying MATLAB variable declarations here
  
  int stage;
  int i;
  static float xM0[numStages] = {0.0}, xM1[numStages] = {0.0}, xM2[numStages] = {0.0};
  static float yM0[numStages] = {0.0}, yM1[numStages] = {0.0}, yM2[numStages] = {0.0};
  
  float yv = 0.0;
  unsigned long startTime;



//  ***  Copy variable initialization code from MATLAB generator to here  ****


//BWRTH bandpass, order 5, [12 40] BPM

G[0] = 0.1342429;
b[0][0] = 1.0000000; b[0][1] = -0.0007646; b[0][2]= -0.9996017;
a[0][0] = 1.0000000; a[0][1] =  -1.6145822; a[0][2] =  0.7180267;
G[1] = 0.1342429;
b[1][0] = 1.0000000; b[1][1] = 2.0009417; b[1][2]= 1.0009420;
a[1][0] = 1.0000000; a[1][1] =  -1.6966715; a[1][2] =  0.7426655;
G[2] = 0.1342429;
b[2][0] = 1.0000000; b[2][1] = 1.9996397; b[2][2]= 0.9996400;
a[2][0] = 1.0000000; a[2][1] =  -1.8434591; a[2][2] =  0.8648240;
G[3] = 0.1342429;
b[3][0] = 1.0000000; b[3][1] = -1.9997055; b[3][2]= 0.9997056;
a[3][0] = 1.0000000; a[3][1] =  -1.7169175; a[3][2] =  0.8730364;
G[4] = 0.1342429;
b[4][0] = 1.0000000; b[4][1] = -2.0001113; b[4][2]= 1.0001113;
a[4][0] = 1.0000000; a[4][1] =  -1.9415600; a[4][2] =  0.9575946;

//  **** Stop copying MATLAB code here  ****



  //  Iterate over each second order stage.  For each stage shift the input data
  //  buffer ( x[kk] ) by one and the output data buffer by one ( y[k] ).  Then bring in 
  //  a new sample xv into the buffer;
  //
  //  Then execute the recusive filter on the buffer
  //
  //  y[k] = -a[2]*y[k-2] + -a[1]*y[k-1] + g*b[0]*x[k] + b[1]*x[k-1] + b[2]*x[k-2] 
  //
  //  Pass the output from this stage to the next stage by setting the input
  //  variable to the next stage x to the output of the current stage y
  //  
  //  Repeat this for each second order stage of the filter

  
  for (i =0; i<numStages; i++)
    {
      yM2[i] = yM1[i]; yM1[i] = yM0[i];  xM2[i] = xM1[i]; xM1[i] = xM0[i], xM0[i] = G[i]*xv;
      yv = -a[i][2]*yM2[i] - a[i][1]*yM1[i] + b[i][2]*xM2[i] + b[i][1]*xM1[i] + b[i][0]*xM0[i];
      yM0[i] = yv;
      xv = yv;
    }
//
//  execUsec += micros()-startTime;
  
  return yv;
}

//*******************************************************************************
long EqualizerFIR(long xInput,int loopTick )
{

  int i;
  long yN=0; //  Current output
  const int equalizerLength = 4;
  static long xN[equalizerLength] = {0};
  long h[] = {1, 1, -1, -1};  // Impulse response of the equalizer

  //  Update the xN array

  for ( i = equalizerLength-1 ; i >= 1; i-- )
  {
    xN[i] = xN[i - 1];
  }

  xN[0] = xInput;

  //  Convolve the input with the impulse response

  for ( i = 0; i <= equalizerLength-1 ; i++)
  {
    yN += h[i] * xN[i];
  }

  if (loopTick < equalizerLength)
  {
    return 0;
  }
  else
  {
   return yN;
  }

}
//*******************************************************************************
void getStats(float xv, stats_t &s, bool reset)
{
  float oldMean, oldVar;
  
  if (reset == true)
  {
    s.stdev = sqrt(s.var/s.tick);
    s.tick = 1;
    s.mean = xv;
    s.var = 0.0;  
  }
  else
  {
    oldMean = s.mean;
    s.mean = oldMean + (xv - oldMean)/(s.tick+1);
    oldVar = s.var; 
    s.var = oldVar + (xv - oldMean)*(xv - s.mean);      
  }
  s.tick++;  
}

//*******************************************************************
float analogReadDitherAve(void)
{ 
 
float sum = 0.0;
int index;
  for (int i = 0; i < NUM_SUBSAMPLES; i++)
  {
    index = i;
    digitalWrite(DAC0, (index & B00000001)); // LSB bit mask
    digitalWrite(DAC1, (index & B00000010));
    digitalWrite(DAC2, (index & B00000100)); // MSB bit mask
    sum += analogRead(LM61);
  }
  return sum/NUM_SUBSAMPLES; // averaged subsamples 

}

//*********************************************************************
void setAlarm(int aCode, boolean isToneEn)
{
  const long second = 50;
  long ii, jj = 0;
  static int HF_tone_count = 0;
  static bool HF_toggle = 0;

  
// Your alarm code goes here
if (aCode == 3)
  {
    HF_tone_count++;
    if (HF_tone_count > 10)
    {
      HF_toggle = !(HF_toggle);
      HF_tone_count = 0;
    }
  
    if (HF_toggle == 0)
    {
      tone1.stop();
    }
    else if (HF_toggle == 1)
    {
      tone1.play(1000);
    }
    //for (ii = 0; ii < 100; ii++)
    //{
    //  tone1.play(1000);
    //  MsTimer2::start(); // start running the Timer
    //  for (jj = 0; jj < 100; jj++)
    //  {
    //    tone1.stop();
     //   MsTimer2::start(); // start running the Timer
     // }
    //}
  }
  // Low Breating Tone
  else if (aCode == 2)
  {
    tone1.play(400);


  }
  // Normal Breathing
  else if (aCode == 1)
  {
    tone1.stop();



  } 
  //Fail 
  else if (aCode == 4)
  {
    tone1.play(200);

  }
  else 
  {
    tone1.stop();

  }

}

    
 // setBreathRateAlarm()

//*************************************************************
float testVector(void)
{
  // Variable rate sinusoidal input
  // Specify segment frequencies in bpm.
  // Test each frequency for nominally 60 seconds.
  // Adjust segment intervals for nearest integer cycle count.
    
  const int NUM_BAND = 10;
  const float CAL_FBPM = 10.0, CAL_AMP = 2.0; 
  
  const float FBPM[NUM_BAND] = {5.0, 10.0, 15.0, 20.0, 30.0, 35.0, 45.0, 50.0, 60.0, 70.0}; // HPF test
  static float bandAmp[NUM_BAND] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

  //  Determine the number of samples (around 600 ) that will give you an even number
  //  of full cycles of the sinewave.  This is done to avoid a large discontinuity 
  //  between bands.  This forces the sinewave in each band to end near a value of zer
  
  static int bandTick = int(int(FBPM[0]+0.5)*(600/FBPM[0]));
  static int simTick = 0, band = 0;
  static float Fc = FBPM[0]/600, cycleAmp = bandAmp[0];

  //for (int i = 0; i < NUM_BAND; i++) bandAmp[i] = CAL_AMP*(CAL_FBPM/FBPM[i]);  

  //  Check to see if the simulation tick has exceeded the number of tick in each band.
  //  If it has then switch to the next frequency (band) again computing how many
  //  ticks to go through to end up at the end of a cycle.
  
  if ((simTick >= bandTick) && (FBPM[band] > 0.0))
  {

    //  The simTick got to the end of the band cycle.  Go to the next frequency
    simTick = 0;
    band++;
    Fc = FBPM[band]/600.0;
    cycleAmp = bandAmp[band];
    bandTick = int(int(FBPM[band]+0.5)*(600/FBPM[band]));
  }
 
  float degC = 0.0; // DC offset
  degC += cycleAmp*sin(TWO_PI*Fc*simTick++);  
  //degC += 1.0*(tick/100.0); // drift: degC / 10sec
  //degC += 0.1*((random(0,101)-50.0)/29.0); // stdev scaled from 1.0
  return degC;
}

//*******************************************************************
void configureArduino(void)
{
  pinMode(DAC0,OUTPUT); digitalWrite(DAC0,LOW);
  pinMode(DAC1,OUTPUT); digitalWrite(DAC1,LOW);
  pinMode(DAC2,OUTPUT); digitalWrite(DAC2,LOW);

  pinMode(SPKR, OUTPUT); digitalWrite(SPKR,LOW);


  analogReference(INTERNAL); // DEFAULT, INTERNAL
  analogRead(LM61); // read and discard to prime ADC registers
  Serial.begin(115200); // 11 char/msec 
}


//**********************************************************************
void WriteToSerial( int numValues, float dataArray[] )
{

  int index=0; 
  for (index = 0; index < numValues; index++)
  {
    if (index >0)
    {
      Serial.print('\t');
    }
      Serial.print(dataArray[index], DEC);
  }

  Serial.print('\n');
  delay(20);

}  // end WriteToMATLAB

////**********************************************************************
float ReadFromMATLAB()
{
  int charCount;
  bool readComplete = false;
  char inputString[80], inChar;


  // Wait for the serial port

  readComplete = false;
  charCount = 0;
  while ( !readComplete )
  {
    while ( Serial.available() <= 0);
    inChar = Serial.read();

    if ( inChar == '\n' )
    {
      readComplete = true;
    }
    else
    {
      inputString[charCount++] = inChar;
    }
  }
  inputString[charCount] = 0;
  return atof(inputString);

} // end ReadFromMATLAB

//*******************************************************************
void syncSample(void)
{
  while (sampleFlag == false); // spin until ISR trigger
  sampleFlag = false;          // disarm flag: enforce dwell  
}

//**********************************************************************
void ISR_Sample()
{
  sampleFlag = true;
}

//**********************************************************************
