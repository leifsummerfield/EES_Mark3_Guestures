//#include <EES_Click_Lib.h>
//Build Notes: 
//Arduino 1.6.13
//For Teensy 3.2 with Teensiduino Drivers 1.34

/*
Haptics tool base 
Rev 2: Actually start using all the bits attached...starting with the encoder input to scroll through a menue system
       and now added touch sensor to TimerOne to be done in the background instead of polling from main loop
       ...this sets me up for doing all realtime stuff on interrupts, and just building a simple state machine to manage GUI and actions
       Sub rev A - Add an event during the hold-down period that ramps up the PWM at a rate, let's use TIMER1 for this and have it simply increment 
                   into a table of PWM values 

Rev 3: Full & final implementation
       Added custom haptics PWM look up table for during touch effect. 

Rev 4: add 5x waveform tables and make them indexable based on ...GUI_defs.h customizer table per each custom set

Rev 5: waveform table expanded to 400 entries per, at around 15ms per entry each effect can last up to 6 seconds. 

Rev 6: Big change to the tool design...touch sensing moved to QT110-DG chip, now only reading digital input on pin  17  

REV 3.7: MOVE TO EES Mark3 - Guestures prototype 
here we reuse most the hardware, (TEENSY base controller with LCD, encoder control, LEDs, Haptics ect ect ect..)
AND add an RFDuino for remote activity enabling (foot switch)
This version simplifies the GUI so remote switch == ARMED 
and touch == ACTIVE with simple haptic buzzing feedback
Working as of Jan 15, 2017

Jan 15+   --> Add check that RFDuino footswitch is sending bits before entering main loop
Jan 17        INvert touched logic (actual handset is actively inverted from what's coded by the prototype handpiece")
               Aaaand add some sound back into the loop taking advantage of the hardware on-hand
Feb 10    Uploaded to Github and built repository 
Feb 13 - Add 9DOF sensor support 
            and Cheryl Anne Mooney, Rest In Peace our Fairy God Mother
Feb 17      Build Notes Added above
            Add tilt sensor calibration step to init zone: collect a  zero angle offset
Feb 19    Token

Feb 21  -  Config serial outputting into telemetry formatting 
        -  Add running average filter to magnetometer to remove blips...

Mar 4   -  Cleanup!
          - Add dependent file GuesturesStateMachine.h to offload some functions for readability 
          - Added 2nd touchsensor reader and renamed functions appropriatly. Both get read now and set differet toggle bits. 
   
Mar 5   - At long last move the sensor reading into TIMER1 function.  We'll start by putting the ini of this at the end of the 

Mar 5pm  - And now...pull it all together and add back in Haptics support, high power LED support and TiltMode...
            that maps to the effects library from the last project. 
            NOTE that this version maps just touch #1 (green handtool board) to the action sequence
Mar 6am   - damn it's so late, it's early :()
          just a quick version to remap touch2 to the actions for the Busy Tool prototype
 */

//HERE ARE JUST "A FEW" KEY VARIABLES FOR ADJUSTMENT
 #include "HapticsGUI_defs.h"

// include the library code:
#include <LiquidCrystal.h>

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(12, 11, 4, 5, 6, 7);

#include <Adafruit_NeoPixel.h>
#define PIN 13
// Parameter 1 = number of pixels in strip
// Parameter 2 = pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(5, PIN, NEO_GRB + NEO_KHZ800);


//     -       Sound section
#include <wavTrigger.h>
wavTrigger wTrig;             // Our WAV Trigger object
  int  nTrack = 1;            // And an integer to index too
                              // Note this uses Serial1 output, only neet TX1 (pin1)


//      -      Capacitive touch sensor definitions here
//      -   Note after cleanup that these are not longer using the capacitive sense library...
//           They are just digital Inputs, two of them
int TouchIn1 = 16;      //Touch1 connected to pin3 of RJ45 
int TouchIn2 = 17;      //Touch2 only one modified board, connected to 2nd handtool breakout board 

boolean bTouched1 = false; 
boolean bTouched2 = false; 

//      -      To read the sensor in the background...let's employ TIMEROne
#include <TimerOne.h>     // We'll put all the sensor reading into this ISR, init at the end of the 


//      -       Add Encoder inputs please!
#include <Encoder.h>
Encoder myEnc(15, 14);
int  encoderButton = 2; 
long oldPosition  = 5;
int16_t last, raw, value, valueOut, b, last2, wiggle; 
boolean  lastB = 0; 
boolean  toggleBit = 0; 
int  UserSelection = 1;     //to get started


 int buttonCounter= 0;
 int buttonHoldThreshold = 200; 
 
// RFDuino input for the footswitch remote is on Pin10 == TX2
int  RFDuino_Toggel_In = 10; 

// External Interrupt variables
unsigned long interrupt1_time ; // Debounce timer variable for interrupt 1
unsigned long interrupt2_time ; //      " "    "  "            interrupt 2

static unsigned long last_interrupt1_time = 0;
static unsigned long last_interrupt2_time = 0;


//   avoid using pins with LEDs attached (not pin 13)                                                         aaaaaaaaaaaaaaa
//Encoder color selector knob
int  Shaft_RED = 21; 
int  Shaft_GREEN =22;
int  Shaft_BLUE = 23; 



//      -      Last but NOT least...add the haptics effects engine into the mix
 //    LIBRARY 
#include <Wire.h>
//Haptics Driver definitions

#include "DRV2605_defs.h"
#include "DRV2605.h"
#include "waveforms.h"
DRV2605 drv2605 = DRV2605();

//WOrking variables

char state = 0;// starting state is off
int ref0 = 450;
int CH0;      //reference values to remove offset
int thresh0 = 150; 
int touchedCounter = 100; 
int interval = 85;      //starting timer devisor
int motorID = 2;
int library = 2;
int effect = 4;
int ON_effect = 4;
int OFF_effect =5;
int REMOTE_effect = 25;
int PWM_OUT       = 3;   
int wave0 = 0; 
int wave1 = 1; 
int i = 0;
int j = 0; 
int watchdog = 0; 
int watchdog_Awake_n = 450; 

//// Motors
#define NUM_MOTORS    4

// EEPROM saved settings
#define MOTOR_VALS_ADDR 0x00  // Address of the calibration bitmask
#define MOTOR_AC_ADDR 0x01  // Address to start saving autocal vals
#define MOTOR_AC_SIZE 0x03  // 3 bytes per autocal
#define MOTOR_AC_LEN  0x0C  // MOTOR_AC_SIZE * NUM_MOTORS

#define   counterInit  100
#define   intervalInit  85



typedef struct {
  uint8_t id ;
  boolean LRA;      // TRUE if LRA, else ERM.
  uint8_t min_duty;   // Minimum PWM duty cycle to start ERM 0 = 0%, 511 = 100%
  uint8_t rated_duty;         // Rated PWM duty cycle to run ERM 0 = 0%, 511 = 100% = 3.3V
  uint8_t max_duty;   // Maximum PWM duty cycle to run ERM 0 = 0%, 511 = 100%
  char part_num[8];   // Part number string.
} SimpleMotorInfo;


uint8_t calibrations_Smpl[ NUM_MOTORS ][ 3 ];
bool calibrated_Smpl[ NUM_MOTORS ] = { false, false, false, false };

//Ohh and one more thing...9DOF sensor support
//NEW for this version is the 9DOF sensor...map it to the screen
// MPU-9250 Digital Motion Processing (DMP) Library
#include <SparkFunMPU9250-DMP.h>

// config.h manages default logging parameters and can be used
#include "config_DOF.h"

MPU9250_DMP imu; // Create an instance of the MPU9250_DMP class

unsigned short accelFSR = IMU_ACCEL_FSR;
unsigned short gyroFSR = IMU_GYRO_FSR;
unsigned short fifoRate = DMP_SAMPLE_RATE;
#define AHRS false         // Set to false for basic data read
uint8_t tiltOffset;       // Zero angle offset collected during init process
int mag_x;                // main variable for tiltangle
int tiltThreshold;        // where we trip the function change
int  Tilt;                // used for reading in the loop and compareing to tiltThreshold
int TiltedMode = 1;       // Tilt activated variable indexed on each tilt up

// And add a running average filter for the magnetometer
#include "RunningAverage.h"
RunningAverage myRunAvg(5);
int samples = 0;

int fiberLED = 20; 

//                                                                                    New in cleanup...move main loop functions into here
 #include "GuesturesStateMachine.h"

//     SETUP SECTION  ---------------------------------------------------------------SETUP SECTION ------------------------------------
void setup() {
    // initialize the pushbutton pin as an input:
  pinMode(TouchIn1, INPUT); 
  pinMode(TouchIn2, INPUT); 
  pinMode(encoderButton, INPUT);     
  pinMode(PWM_OUT, OUTPUT);
  pinMode(RFDuino_Toggel_In, INPUT);
  pinMode(fiberLED, OUTPUT); 

 // pinMode(Shaft_RED, OUTPUT);
  pinMode(Shaft_GREEN, OUTPUT);
  pinMode(Shaft_BLUE, OUTPUT);
 //digitalWrite(Shaft_RED, HIGH);
  digitalWrite(Shaft_GREEN, HIGH);
  digitalWrite(Shaft_BLUE, HIGH);
  digitalWrite(fiberLED,LOW); 

  attachInterrupt(encoderButton,      isrService1, RISING);           // interrrupt 1 for Encoder button toggle input
  attachInterrupt(RFDuino_Toggel_In,  isrService2, RISING);      // ISR2 is for RFDuino intput of the foot-switch toggle

// Turn on the trusty debug serial output port
   Serial.begin(115200);

  // Initialize the MPU-9250. Should return true on success:
  if ( !initIMU() ) 
  {
    Serial.println("Error connecting to MPU-9250");
    while (1) ; // Loop forever if we fail to connect
  }

  imu.setAccelFSR(2); // Set accel to +/-2g

  // Init the running average filter with some initial values
    myRunAvg.fillValue(100,5);
  
  // set up the LCD's number of columns and rows: 
  lcd.begin(16, 2);
  // Print a message to the LCD.
  lcd.print("EES Guestures");

  strip.begin();
  strip.show(); // Initialize all pixels to 'off'

  colorWipe(strip.Color(0, 0, 25), 150); // Blue

  // WAV Trigger startup at 57600
  delay(500);  //sound board warmup
    //  to finish reset before trying to send commands.
    wTrig.start();
  // If we're not powering the WAV Trigger, send a stop-all command in case it
  //  was already playing tracks. If we are powering the WAV Trigger, it doesn't
  //  hurt to do this.
  wTrig.stopAllTracks();
 
 //Easter Egg for Bill...track 98 is the Millenium Falcon taking off
 //wTrig.trackPlayPoly(98);               // Play first note


 
  Serial.println("Here before haptics init");
  //calibrate();
  calibrateQuite();
  Serial.println("Here AFTER haptics init");
  playFullHaptic(library, 121);
  Serial.println("Here after haptics fired UP");
     

  myEnc.write(1); // get the encoder value set to a known value befor checking it in loop   
  toggleBit = 1;      //launch into the main loop by selecting first 


      

// FIRST SETUP CHECK --> Is the touchsensor active on the tool head?

    // set up the LCD's number of columns and rows: 
  lcd.clear();
  lcd.setCursor(0, 0);
  // Print a message to the LCD.
  lcd.print("State #1: Test");
  lcd.setCursor(0,1);
  lcd.print("Touch Tip 3x:"); 

//Let's set the GUI and read N touches before continuing
  int touchCounter = 0;

  while (touchCounter < 3)
    {
    Tmr1_ReadSensors(); 
    printTelemetry("Setup#1: Touch test"); 
        if(bTouched1 == 1)
        {
        touchCounter =touchCounter +1;
      int Red=250-50*touchCounter; 
      strip.setPixelColor((touchCounter-1), strip.Color(Red, 50, 0));
      strip.show();
        while(bTouched1 ==1)
        Tmr1_ReadSensors(); 
        printTelemetry("Setup#1: TOUCHED"); 
        }
    lcd.setCursor(14,1);
    lcd.print(touchCounter); 
    }
    
// SECOND SETUP CHECK --> Is the footswtich active sending data over RFDUINO connection
//Let's set the GUI and read N taps before continuing
  //wipe the neopixels clean
  strip.show(); // Initialize all pixels to 'off'
  colorWipe(strip.Color(0, 50, 0), 150); // Blue
  delay(250); 
  
  int tapFootCounter = 0;
    // set up the LCD's number of columns and rows: 
  lcd.setCursor(0, 0);
  // Print a message to the LCD.
  lcd.print("State #1: Test");
  lcd.setCursor(0,1);
  lcd.print("Foot Tap Once:"); 
  toggleBit = 0;
  
  while (tapFootCounter < 1)
    {
     printTelemetry("Setup#2: Tap Test"); 
        if(toggleBit == 1)
        {
        tapFootCounter =tapFootCounter +1;
        toggleBit = 0; 
      //increment the neopixels to show progress
      int Red=250-50*tapFootCounter; 
      strip.setPixelColor((tapFootCounter-1), strip.Color(Red, 50, 0));
      strip.show();
        }
          // and show the count on the LCD
       lcd.setCursor(14,1);
       lcd.print(tapFootCounter);     
    }



// THIRD SETUP CHECK --> Set the tilt sensor to 'zero' angle and collect offset
//Let's set the GUI and read N taps before continuing
  //wipe the neopixels clean
  strip.show(); // Initialize all pixels to 'off'
  colorWipe(strip.Color(50, 50, 50), 20); // White
 
    // set up the LCD's number of columns and rows: 
  lcd.setCursor(0, 0);
  // Print a message to the LCD.
  lcd.print("State #1: Test");
  lcd.setCursor(0,1);
  lcd.print("Hold tool level"); 
  toggleBit = 0;
  
  while (toggleBit != 1)
    {
    imu.update(); 
    tiltOffset=imu.calcMag(imu.mx);
    Tilt = tiltOffset; 
    printTelemetry("Setup#3: Zero the tilt-o-meter"); 
    }
  //When we while out of this loop...then mag_x is our offset value to always subtract...save it to       


// FOURTH SETUP CHECK --> Set the tilt sensor to 'zero' angle and collect offset
//Let's set the GUI and read N taps before continuing
  //wipe the neopixels clean
  strip.show(); // Initialize all pixels to 'off'
  colorWipe(strip.Color(50, 50, 50), 150); // White
 
    // set up the LCD's number of columns and rows: 
  lcd.setCursor(0, 0);
  // Print a message to the LCD.
  lcd.print("State #1: Test");
  lcd.setCursor(0,1);
  lcd.print("Tilt to Threshld"); 
  toggleBit = 0;
  
  while (toggleBit != 1)
    {
      imu.update(); 
      mag_x=imu.calcMag(imu.mx);
      mag_x = mag_x - tiltOffset; 
      mag_x = map(mag_x,450,540,0,180);
      Tilt = mag_x;   //For telemetry outputing
      printTelemetry("Setup#4: Find Tilt Threshold"); 
    }     //When we while out of this loop...then we have the angle we want to trip at
    tiltThreshold = mag_x; 

    
   toggleBit = 1; //undo toggle after testing so we don't jump right into the armed stated in the loop

  lcd.clear();
  lcd.setCursor(0, 0);
  // Print a message to the LCD.
  lcd.print("Initialization");
  lcd.setCursor(0,1);
  lcd.print("    complete!"); 
  colorWipe(strip.Color(0, 0, 0), 150); // Blue
    strip.setPixelColor(0, strip.Color(0, 0, 50));
    strip.setPixelColor(1, strip.Color(0, 50, 0));
    strip.setPixelColor(2, strip.Color(0, 0, 0));
    strip.setPixelColor(3, strip.Color(0, 0, 0)); 
    strip.show();
    printTelemetry("Setup Done!"); 
  delay(500); 


//LAST and new in March 5 built, put TIMER 1 inits here
  Timer1.initialize(20000); //20ms 
  Timer1.attachInterrupt(Tmr1_ReadSensors); // blinkLED to run every 0.15 seconds


}


//----------------------THE MAIN LOOP-----------------------THE MAIN LOOP--------------------STARTS HERE -----------------------------
//----------------------THE MAIN LOOP-----------------------THE MAIN LOOP--------------------STARTS HERE -----------------------------


void loop(){

if(toggleBit == 1)    // If toggledBit is high (via Ext Interrupt routine)...We've entered standby mode then...
  {

    enterStandBy_Mode(); 
    
    //While we wait for the footswitch bit to flip...
    while(toggleBit == 1)
      {
      printTelemetry("I'm waiting..."); 
      UserSelection = getEncoder(); 

      }
  }

//  TEST for footswitch activation via the toggle bit-flip, if so then enter the ARMED mode branch
if(toggleBit == 0)
  {
  enterArmed_Mode(); 

  while(toggleBit == 0)
    {
      armedAndWaiting(); 

      //ARMED and touched, actions go in here
      if (bTouched1 == true)
        { 
          justTouched_on1(); 
          doSomethingAboutIt_JustTouched(); 

        while(bTouched1 ==true)   //Note this needs an escape clause if the footswith is activated again
           {
           printTelemetry("Touched!"); 
           }
           doSomethingAboutIt_Released(); 
         }


      if (Tilt > tiltThreshold)   //Note Tilt is updated in armedAndWaiting(), but I want to move this to a timer
      {
        
        justTilted();     //This handles the display and MODE indexing after tilt threshold has been reached

        while(Tilt > tiltThreshold)
           {
            printTelemetry("Tilted"); 
           }//WHILE tilted
        
      }//IF tilted
         
    }//WHILE in armed mode and waiting
  }//IF toggle == enter ARMED mode
  
}//MAIN loop end

//        THE MAIN LOOP            ------      THE MAIN LOOP             ENDS HERE------      ENDMAIN   ENDMAN    ENDMAIN


int  getEncoder(void)
{
  
  //   value += (encoder->getValue()*scaler);
  raw = myEnc.read();
 // value = raw; 
 value = map(raw,0,33,1,5);
  
  if (value != last) 
  {
    last = value;
//    Serial.print(raw); 
//    Serial.print("  to --> "); 
//    Serial.print("Encoder Value: ");
//    Serial.print(value);
//    Serial.print("  to -->  ");
   //  valueOut = map(value,0,26,1,5);
   valueOut = value; 
      //     lcd.clear();
     //      showSomething(valueOut);   //For Haptics project this ran the dispalay...moved out for EES Mrk3 dev.
           
if(valueOut > 5)
{
valueOut = 5;
myEnc.write(33);
}

if(valueOut < 1)
{
  valueOut = 1;
  myEnc.write(1);
}


  for(uint16_t i=0; i<strip.numPixels(); i++) {
  //    strip.setPixelColor(i, strip.Color(0, 0, 0));
  //    strip.show();
  }
  
 // strip.setPixelColor((valueOut-1), strip.Color(0, 50, 0));
 // strip.show();

    
  }


return valueOut; 

}

// ------------------------------------------------------------------

  
// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, c);
      strip.show();
      delay(wait);
  }
}





void calibrate()
{
            

        
//       id,    LRA,  min_duty rated    max     part number
//  { 0,  false,  56, 127,  255,  "305-000" },  // M0 - ERM 305-000, min 1.1V, rated 1.3V, max 1.7V
//  { 1,  false,  51, 127,  255,  "306-109" },  // M1 - ERM 306-109, min 1.0V Rated 3.0V max 3.6V
//  { 2,  false,  77, 127,  255,  "308-102" },  // M2 - ERM 308-102, min 1.5V, rated 3.0V, max 3.0V
//  { 3,  true, 39, 127,  255,  "C10-100" },  // M3 - Leaded, LRA, C13-000 min 0.50 Vrms rated 2.30 Vrms max 3.30 Vrms        

  // Auto calibrate the motor
  uint8_t compensation, backEMF, feedback;


  if( drv2605.autoCal(127, 255, false, &compensation, &backEMF, &feedback ) )
  {
    Serial.println( F("AutoCal success" ));
          calibrations_Smpl[ motorID ][ 0 ] = compensation;
          calibrations_Smpl[ motorID ][ 1 ] = backEMF;
          calibrations_Smpl[ motorID ][ 2 ] = feedback;
          calibrated_Smpl[ motorID ] = true;

        Serial.print(" Comp = ");   Serial.println(compensation);
        Serial.print(" BackEMF = ");   Serial.println(backEMF);
        Serial.print(" Feedback = ");   Serial.println(feedback);

  }
  else
  {
    Serial.println( F("AutoCal failed" ));
        Serial.print(" Comp = ");   Serial.println(compensation);
        Serial.print(" BackEMF = ");   Serial.println(backEMF);
        Serial.print(" Feedback = ");   Serial.println(feedback);
  }


}

void calibrateQuite()
{
    // Auto calibrate the motor
  uint8_t compensation, backEMF, feedback;

    calibrations_Smpl[ motorID ][ 0 ] = 16;
    calibrations_Smpl[ motorID ][ 1 ] = 136;
    calibrations_Smpl[ motorID ][ 2 ] = 56;
    calibrated_Smpl[ motorID ] = true;
  
}

void playFullHaptic( uint8_t library, uint8_t effect )
{




  uint8_t compensation, backEMF, feedback;


  compensation = calibrations_Smpl[ motorID ][ 0 ];
  backEMF = calibrations_Smpl[ motorID ][ 1 ];
  feedback = calibrations_Smpl[ motorID ][ 2 ];
        

  drv2605.playFullHaptic( library, effect, 127, 255, compensation, backEMF, feedback );

}

/*  ----------------------------------------------------------------------------------------------------------------
showSomething function
This will be our screen display for different settings



VARIABLES 
--------------------------------------------------------------------------------------------------------------------
*/
void showSomething(int whatYouWant)
{
        lcd.setCursor(0,0);
        lcd.print(LCD_GUI[whatYouWant]);
        lcd.setCursor(0,1);
        lcd.print(LCD_GUI_Line2[whatYouWant]);
}

/*  ----------------------------------------------------------------------------------------------------------------


VARIABLES 
--------------------------------------------------------------------------------------------------------------------
*/



/*  ----------------------------------------------------------------------------------------------------------------
EXTERNAL interrupt ISR
reads the encoder button knob and toggles an output from 1 to 0 if it's been pressed
//this is working well in teh cleaned up version :)
/
VARIABLES   
--------------------------------------------------------------------------------------------------------------------
*/
void isrService1()
{
cli();
//Serial.println("ISR #1");

   interrupt1_time = millis();
  // If interrupts come faster than 200ms, assume it's a bounce and ignore
  if (interrupt1_time - last_interrupt1_time > 200) 
  {
    toggleBit = !toggleBit;
    Serial.print("Toggle = ");  Serial.println(toggleBit); 

  }
  last_interrupt1_time = interrupt1_time;

 sei();    //Re-enable ISR after we're do 
}

/*  ----------------------------------------------------------------------------------------------------------------
doSomethingAboutIt
launches GUI selected effects for touch
/
VARIABLES   
--------------------------------------------------------------------------------------------------------------------
*/

void isrService2()
{
cli();
Serial.println("At ISR_#2");


interrupt2_time = millis();

 // If interrupts come faster than 200ms, assume it's a bounce and ignore
 if (interrupt2_time - last_interrupt2_time > 200) 
 {
    // ... do your thing
    toggleBit = !toggleBit;
    Serial.print("Toggle = ");  Serial.println(toggleBit); 
     
      wTrig.stopAllTracks();
      wTrig.trackPlayPoly(1);   
      //for refernce this is the dound effect on selection used in the last design int SoundEffectON_i[] = {0,1,2,3,8,9};

 }

 last_interrupt2_time = interrupt2_time;
 sei();
 
 }
 
//End of the Second ISR for remote switch activation through RFDuino




/*  ----------------------------------------------------------------------------------------------------------------
doSomethingAboutIt
launches GUI selected effects for touch
/
VARIABLES   
--------------------------------------------------------------------------------------------------------------------
*/
void doSomethingAboutIt_JustTouched(void)
{
  
    wTrig.stopAllTracks();
    wTrig.trackPlayPoly(SoundEffectON_i[UserSelection]);   

  Timer1.detachInterrupt(); // 

  playFullHaptic(HapLib_i[TiltedMode],HapEffectON_i[TiltedMode]);

  Timer1.attachInterrupt(Tmr1_ReadSensors); // blinkLED to run every 0.15 seconds

 
}


/*  ----------------------------------------------------------------------------------------------------------------
doingSomethingWhileTouched
This handles a while-touched incrementer (is that a word?) and do things live after a touch but before release

Haptics project uses this to index into the waveform table and run a PWM waveform
EES Mark3 starts with just a fixed PWM amplitidue to run to the PWM haptics module

/
VARIABLES   
--------------------------------------------------------------------------------------------------------------------
*/
void doingSomethingWhileTouched(void)
{

  drv2605.PWM_Setup(); 
  analogWrite(Shaft_BLUE, map(j,0,255,255,0));   //inverse the input becuase the PWM is active low
  analogWrite(PWM_OUT, 0x80); 
  delay(10); 

                                                                                    
}


/*  ----------------------------------------------------------------------------------------------------------------
    doSomethingAboutIt_Released
This handles the GUI selected release action
/
VARIABLES   
--------------------------------------------------------------------------------------------------------------------
*/
void doSomethingAboutIt_Released(void)
{
    wTrig.stopAllTracks();
    wTrig.trackPlayPoly(SoundEffectOFF_i[UserSelection]);   

    Timer1.detachInterrupt(); // 
    playFullHaptic(HapLib_i[TiltedMode],HapEffectOFF_i[TiltedMode]);
    Timer1.attachInterrupt(Tmr1_ReadSensors); // blinkLED to run every 0.15 seconds
      
    i = 0;      // PWM waveform table index - reset for next round.
    watchdog = 0; 
    

    delay(50); 
}





void goBLUE(void)
{
  analogWrite(Shaft_RED,0xFF);
  analogWrite(Shaft_GREEN,0xFF);
  analogWrite(Shaft_BLUE,0x00);
}

void goGREEN(void)
{
  analogWrite(Shaft_RED,0xFF);
  analogWrite(Shaft_GREEN,0x00);
  analogWrite(Shaft_BLUE,0xFF);
}

void goRED(void)
{
  analogWrite(Shaft_RED,0);
  analogWrite(Shaft_GREEN,0xFF);
  analogWrite(Shaft_BLUE,0xFF);
}

void goOFF(void)
{
  digitalWrite(Shaft_RED, HIGH); 
  digitalWrite(Shaft_GREEN, HIGH); 
  digitalWrite(Shaft_BLUE, HIGH); 

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


void  Tmr1_ReadSensors(void)
{
  int T1_in = digitalRead(TouchIn2);                                                                                                   // NEW QT section here

  if (T1_in ==1)
  {
    bTouched1 = true; 
  
  }
    else bTouched1 = false; 



// int T2_in = digitalRead(TouchIn2);      
//  if (T2_in ==1)
//  {
//    bTouched2 = true; 
//  }
//    else bTouched2 = false; 



  imu.update(); 
  mag_x=imu.calcMag(imu.mx);
  mag_x = mag_x - tiltOffset; 
  mag_x = map(mag_x,450,540,0,180);
  myRunAvg.addValue(mag_x);
  Tilt = myRunAvg.getAverage(); 

  String telem = ""; // Create a fresh line to log
  //Plot this out
  telem += String(toggleBit) + ",";            // Plot 1: Toggle state (push button)
  telem += String(bTouched1) + ",";             // Plot 2: Touch State
  telem += String(bTouched2) + ","; 
  telem += String(Tilt) + ",";            // Plot 3: Tilt value
  telem += String(TiltedMode);                  // Plot 4: Tilt mode-shift
  
  
  telem += "\r\n"; // Add a new line and we are DONE
 // Serial.print(telem);   
}


