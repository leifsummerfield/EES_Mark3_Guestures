

/******************************************************************************
SparkFun 9DoF Razor M0 Example Firmware
Jim Lindblom @ SparkFun Electronics
Original creation date: November 22, 2016
https://github.com/sparkfun/9DOF_Razor_IMU/Firmware

This example firmware for the SparkFun 9DoF Razor IMU M0 
demonstrates how to grab accelerometer, gyroscope, magnetometer,
and quaternion values from the MPU-9250's digital motion processor
(DMP). It prints those values to a serial port and, if a card is
present, an SD card.

Values printed can be configured using the serial port. Settings
can be modified using the included "config.h" file.

Resources:
SparkFun MPU9250-DMP Arduino Library:
  https://github.com/sparkfun/SparkFun_MPU-9250-DMP_Arduino_Library
FlashStorage Arduino Library
  https://github.com/cmaglie/FlashStorage

Development environment specifics:
  Firmware developed using Arduino IDE 1.6.12

Hardware:
  SparkFun 9DoF Razor IMU M0 (SEN-14001)
  https://www.sparkfun.com/products/14001

REV NOTES
Rev1: Simple - pull out the SD card support and port to work on RFDuino - DONE
Rev2: Simple - but now with home-brew tap detection, toggling and multi-hit detection wiht timouts and debounce 
Jan 17 2017 - this is what's getting loaded onto the RFDUIno that's acting as a guesture based foot switch prototype
              Added LED to pin3 "DataOut" that blinks on each "hitme" event...nice fast signal MELIKESIT
Feb 10      - Uploaded to Github              
  
******************************************************************************/
// MPU-9250 Digital Motion Processing (DMP) Library
#include <SparkFunMPU9250-DMP.h>

// config.h manages default logging parameters and can be used
// to adjust specific parameters of the IMU
#include "config.h"
// Flash storage (for nv storage on ATSAMD21)
#ifdef ENABLE_NVRAM_STORAGE
#include <FlashStorage.h>
#endif

MPU9250_DMP imu; // Create an instance of the MPU9250_DMP class

/////////////////////////////
// Logging Control Globals //
/////////////////////////////
// Defaults all set in config.h
bool enableSerialLogging = ENABLE_UART_LOGGING;
bool enableTimeLog = ENABLE_TIME_LOG;
bool enableCalculatedValues = ENABLE_CALCULATED_LOG;
bool enableAccel = ENABLE_ACCEL_LOG;
bool enableGyro = ENABLE_GYRO_LOG;
bool enableCompass = ENABLE_MAG_LOG;
bool enableQuat = ENABLE_QUAT_LOG;
bool enableEuler = ENABLE_EULER_LOG;
bool enableHeading = ENABLE_HEADING_LOG;
unsigned short accelFSR = IMU_ACCEL_FSR;
unsigned short gyroFSR = IMU_GYRO_FSR;
unsigned short fifoRate = DMP_SAMPLE_RATE;



///////////////////////
// IO bits //
///////////////////////
int DataOut = 3;    //put data on pin3 = LED


///////////////////////
// LED Blink Control //
///////////////////////
//bool ledState = false;
uint32_t lastBlink = 0;
void blinkLED()
{
  static bool ledState = false;
  digitalWrite(HW_LED_PIN, ledState);
  ledState = !ledState;
}

///////////////////////
// RFDuino BLE chip init //
///////////////////////
#include <RFduinoGZLL.h>
device_t role = DEVICE0;



#include <EES_Click.h>
EES_Click guestureButton(HIGH);  //Active high click function definition

// And add a running average filter for the magnetometer
#include "RunningAverage.h"
RunningAverage myRunAvg(5);

bool HitMe = 0;
bool toggler = 0; 
int samples = 0;
int mag_x;                // main variable for tiltangle
int mag_y;                // main variable for tiltangle
int mag_z;                // main variable for tiltangle
int tiltOffset;           
int TiltedMode;           // tilt and tap mode state
int Tilt;                 //running average fills this up and becomes our prime working variable for tap and tilt sensing



//     SETUP SECTION  ---------------------------------------------------------------SETUP SECTION ------------------------------------
void setup()
{
  // Initialize LED, interrupt input, and serial port.
  // LED defaults to off:
  initHardware(); 
Serial.println("Setup initializing"); 
  // Initialize the MPU-9250. Should return true on success:
  if ( !initIMU() ) 
  {
    Serial.println("Error connecting to MPU-9250");
    while (1) ; // Loop forever if we fail to connect
    // LED will remain off in this state.
  }

  imu.setAccelFSR(2); // Set accel to +/-2g

  // start the GZLL stack
  RFduinoGZLL.begin(role);


    // Setup guesture button timers (all in milliseconds / ms)
  // (These are default if not set, but changeable for convenience)
  guestureButton.debounceTime   = 1;   // Debounce timer in ms
  guestureButton.multiclickTime = 10;  // Time limit for multi clicks
  guestureButton.longClickTime  = 1000; // time until "held-down clicks" register


  // Initialize the titl sensor to a zero offset position
  for (int i=0; i<20; i++)
  {
    imu.update(); 
    mag_y=imu.calcMag(imu.my);        // let's try grabbing tilt in Y axis for the foot switch

    myRunAvg.addValue(mag_y);
    Serial.print(mag_y); Serial.printf(", %d\n",i); 
    delay(100); 
  }
    tiltOffset = myRunAvg.getAverage(); 
    Serial.print("Tilt Offset Initialized to: "); Serial.println(tiltOffset); 
    delay(1000); 

Serial.println("Setup complete"); 
}




//     MAIN LOOP SECTION  ---------------------------------------------------------------MAIN LOOP SECTION ------------------------------------
void loop()
{

  imu.update();
  
  int Accel_x = abs(imu.ax);
  int Accel_y = abs(imu.ay);
  int Accel_z = abs(imu.az);

  int Total_Accel = Accel_x+Accel_y+Accel_z;
  Total_Accel = map(Total_Accel,16000,100000,0,100) ;     //map these suckers down to something more plottable on the same chart
  
  mag_y=imu.calcMag(imu.my);
  mag_y = mag_y - tiltOffset; 
  //mag_y = map(mag_y,-60,60,0,360);
  myRunAvg.addValue(mag_y);
  Tilt = myRunAvg.getAverage(); 

  String telem = ""; // Create a fresh line to log
  //Plot this out
  telem += String(mag_y) + ",";            // Plot 1, Tilt magnatude
  telem += String(guestureThreshold) + ","; // plot 1b: 
  telem += String(Total_Accel) + ",";                 // Plot 2, titl angle
  telem +=  String(TapThreshold) + ",";
  telem += String(TiltedMode);                  // Plot 3: Tilt mode-shift
  telem += "\r\n"; // Add a new line and we are DONE
  Serial.println(telem); 

  
  if (Tilt < guestureThreshold)   //Note sign switch since we're in negative number land
  {
    TiltedMode = 50;   
  
    if (Total_Accel > TapThreshold)
      {
      TiltedMode = 100;   
      HitMe = 1; 
      digitalWrite(DataOut, HIGH);
      }
  }   
  else 
  {
    TiltedMode = 0;   
    HitMe = 0; 
      digitalWrite(DataOut, LOW);
  }

    
    guestureButton.Update(HitMe); 
    int function = guestureButton.clicks;
   // Save click codes in LEDfunction, as click codes are reset at next Update()
  if (function != 0) 
  {
  //for now just anyrecognized hit == toggle
  toggler = ! toggler; 
    // send state to Host
  RFduinoGZLL.sendToHost(toggler);
  Serial.print("Toggle = ");  Serial.println(toggler); 
  
  }
  function = 0;
  
}

//   END  MAIN LOOP SECTION  ---------------------------------------------------------------END MAIN LOOP SECTION ------------------------------------



void initHardware(void)
{
  // Set up LED pin (active-high, default to off)
  pinMode(HW_LED_PIN, OUTPUT);
  digitalWrite(HW_LED_PIN, LOW);

 pinMode(DataOut, OUTPUT);
 digitalWrite(DataOut, HIGH); 

  // Set up MPU-9250 interrupt input (active-low)
  pinMode(MPU9250_INT_PIN, INPUT_PULLUP);

  // Set up serial log port
  Serial.begin(SERIAL_BAUD_RATE);

  Serial.println("We are alive and running"); 
  
}

bool initIMU(void)
{
  // imu.begin() should return 0 on success. Will initialize
  // I2C bus, and reset MPU-9250 to defaults.
  if (imu.begin() != INV_SUCCESS)
    return false;

  // Set up MPU-9250 interrupt:
  imu.enableInterrupt(); // Enable interrupt output
  imu.setIntLevel(1);    // Set interrupt to active-low
  imu.setIntLatched(1);  // Latch interrupt output

  // Configure sensors:
  // Set gyro full-scale range: options are 250, 500, 1000, or 2000:
  imu.setGyroFSR(gyroFSR);
  // Set accel full-scale range: options are 2, 4, 8, or 16 g 
  imu.setAccelFSR(accelFSR);
  // Set gyro/accel LPF: options are5, 10, 20, 42, 98, 188 Hz
  imu.setLPF(IMU_AG_LPF); 
  // Set gyro/accel sample rate: must be between 4-1000Hz
  // (note: this value will be overridden by the DMP sample rate)
  imu.setSampleRate(IMU_AG_SAMPLE_RATE); 
  // Set compass sample rate: between 4-100Hz
  imu.setCompassSampleRate(IMU_COMPASS_SAMPLE_RATE); 

  // Configure digital motion processor. Use the FIFO to get
  // data from the DMP.
  unsigned short dmpFeatureMask = 0;
  if (ENABLE_GYRO_CALIBRATION)
  {
    // Gyro calibration re-calibrates the gyro after a set amount
    // of no motion detected
    dmpFeatureMask |= DMP_FEATURE_SEND_CAL_GYRO;
  }
  else
  {
    // Otherwise add raw gyro readings to the DMP
    dmpFeatureMask |= DMP_FEATURE_SEND_RAW_GYRO;
  }
  // Add accel and quaternion's to the DMP
  dmpFeatureMask |= DMP_FEATURE_SEND_RAW_ACCEL;
  dmpFeatureMask |= DMP_FEATURE_6X_LP_QUAT;

  // Initialize the DMP, and set the FIFO's update rate:
  imu.dmpBegin(dmpFeatureMask, fifoRate);

  return true; // Return success
}




