//#include <EES_Click_Lib.h>

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
               
 */

//HERE ARE A FEW KEY VARIABLES FOR ADJUSTMENT
 
#include "HapticsGUI_defs.h"


// include the library code:
#include <LiquidCrystal.h>

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(12, 11, 4, 5, 6, 7);
//quidCrystal lcd(8, 7, 3, 4, 5, 6);


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
#include <CapacitiveSensor.h>
CapacitiveSensor   C1 = CapacitiveSensor(20,16);        // 10M resistor between pins 21 & 22, pin 22 is sensor pin, add a wire and or foil if desired
CapacitiveSensor   C2 = CapacitiveSensor(20,17);        // 10M resistor between pins 21 & 23, pin 23 is sensor pin, add a wire and or foil
volatile long total2, total1; 
int QTin = 16; 

boolean bTouched = false; 

//      -      To read the sensor in the background...let's employ TIMEROne
#include <TimerOne.h>


//      -       Add Encoder inputs please!
#include <Encoder.h>
Encoder myEnc(15, 14);
int  encoderButton = 2; 
long oldPosition  = 5;
int16_t last, raw, value, valueOut, b, last2, wiggle; 
boolean  lastB = 0; 
boolean  toggleBit = 0; 
int  UserSelection = 1;     //to get started
 static unsigned long last_interrupt_time = 0;
 int buttonCounter= 0;
 int buttonHoldThreshold = 200; 
 
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

//     SETUP SECTION  ---------------------------------------------------------------SETUP SECTION ------------------------------------
void setup() {
    // initialize the pushbutton pin as an input:
  pinMode(QTin, INPUT); 
  pinMode(encoderButton, INPUT);     
  pinMode(PWM_OUT, OUTPUT);
 // pinMode(Shaft_RED, OUTPUT);
  pinMode(Shaft_GREEN, OUTPUT);
  pinMode(Shaft_BLUE, OUTPUT);
      
  //digitalWrite(Shaft_RED, HIGH);
  digitalWrite(Shaft_GREEN, HIGH);
  digitalWrite(Shaft_BLUE, HIGH);
  
  attachInterrupt(encoderButton, isrService, RISING); // interrrupt 1 is data ready
  attachInterrupt(10, isrService2, RISING);      //Test of 2nd ISR
  
  // set up the LCD's number of columns and rows: 
  lcd.begin(16, 2);
  // Print a message to the LCD.
  lcd.print("EES Guestures");

  strip.begin();
  strip.show(); // Initialize all pixels to 'off'

  colorWipe(strip.Color(0, 0, 25), 150); // Blue

  // WAV Trigger startup at 57600
  delay(1000);  //sound board warmup
    //  to finish reset before trying to send commands.
    wTrig.start();
  // If we're not powering the WAV Trigger, send a stop-all command in case it
  //  was already playing tracks. If we are powering the WAV Trigger, it doesn't
  //  hurt to do this.
  wTrig.stopAllTracks();
 
 //Easter Egg for Bill...track 98 is the Millenium Falcon taking off
 //wTrig.trackPlayPoly(98);               // Play first note

   Serial.begin(9600);
 
  Serial.println("Here before haptics init");
  //calibrate();
  //calibrateQuite();
  Serial.println("Here AFTER haptics init");
  //playFullHaptic(library, 121);

     Serial.println("Here after haptics fired UP");
      lcd.clear();

      myEnc.write(1); // get the encoder value set to a known value befor checking it in loop

   
      toggleBit = 1;      //launch into the main loop by selecting first 

// FIRST SETUP CHECK --> Is the touchsensor active on the tool head?
//Let's set the GUI and read N touches before continuing
  int touchCounter = 0;
    // set up the LCD's number of columns and rows: 
  lcd.setCursor(0, 0);
  // Print a message to the LCD.
  lcd.print("State #1: Test");
  lcd.setCursor(0,1);
  lcd.print("Touch Tip 3x:"); 

  while (touchCounter < 3)
    {
    readTouch_QT110(); 

        if(bTouched == 1)
        {
        touchCounter =touchCounter +1;
      int Red=250-50*touchCounter; 
      strip.setPixelColor((touchCounter-1), strip.Color(Red, 50, 0));
      strip.show();
        while(bTouched ==1)
        readTouch_QT110(); 
      
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
  lcd.print("Foot Tap 3x:  0"); 
  toggleBit = 0;
  
  while (tapFootCounter < 3)
    {
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
  delay(500); 


}


//        THE MAIN LOOP            ------      THE MAIN LOOP             STARTS HERE ------     ---------------     ------------------      ------------------      ------------



void loop(){

if(toggleBit == 1)    // If toggledBit is high (via Ext Interrupt routine)...We've entered standby mode then...
  {
    Serial.println("entering toggle = 1 standby mode");   
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("State #2");
    lcd.setCursor(0,1);
    lcd.print("STANDBY");

    strip.setPixelColor(0, strip.Color(0, 0, 50));
    strip.setPixelColor(1, strip.Color(0, 50, 0));
    strip.setPixelColor(2, strip.Color(0, 0, 0));
    strip.setPixelColor(3, strip.Color(0, 0, 0)); 
    strip.show();

    
    while(toggleBit == 1)
      {
        Serial.println("i'm WAITING for arming"); 
      UserSelection = getEncoder(); 
      
      }
  }

if(toggleBit == 0)
  {
    Serial.println("entering toggle = 0 ARMED mode");
    lcd.clear();
    
  while(toggleBit == 0)
    {
    lcd.setCursor(0,0);
    lcd.print("State #3");
    lcd.setCursor(0,1);
    lcd.print("ARMED"); 
    
    strip.setPixelColor(0, strip.Color(0, 0, 50));
    strip.setPixelColor(1, strip.Color(0, 50, 0));
    strip.setPixelColor(2, strip.Color(50, 25, 0));
    strip.setPixelColor(3, strip.Color(0, 0, 0));
    strip.show();
    
    readTouch_QT110();
    delay(10);        
      if (bTouched == true)
        { 
        lcd.setCursor(0,0);
        lcd.print("State #3");
        lcd.setCursor(0,1);
        lcd.print("ACTIVE"); 
            
        strip.setPixelColor(3, strip.Color(50, 0, 0));
        strip.show();
        //play some sound the first time through
        wTrig.stopAllTracks();
        wTrig.trackPlayPoly(2); 
        
        while(bTouched ==true)
           {
           readTouch_QT110();
           Serial.println("TOUCHED IN LOOP");
           }
         }
    }
  }
  
}

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
    Serial.print(raw); 
    Serial.print("  to --> "); 
    Serial.print("Encoder Value: ");
    Serial.print(value);
    Serial.print("  to -->  ");
   //  valueOut = map(value,0,26,1,5);
   valueOut = value; 
     Serial.print(valueOut); 
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
    Serial.print(" limited -->  ");
    Serial.println(valueOut); 

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
readTouches
This will be our screen display for different settings



VARIABLES 
--------------------------------------------------------------------------------------------------------------------
*/


/*  ----------------------------------------------------------------------------------------------------------------
readTouches_QT110
This reads the input on pin17 which is now tied to the stand-alone touchsensor with a single digital output

VARIABLES 
QT_in
--------------------------------------------------------------------------------------------------------------------
*/
void readTouch_QT110(void)
{
  int QT_in = digitalRead(QTin);                                                                                                   // NEW QT section here

  if (QT_in ==1)
  {
    bTouched = true; 
  //  Serial.println("Touched = True"); 
    
  }
  else bTouched = false; 


  Serial.print(bTouched);  Serial.print(" -->"); Serial.println(QT_in);

  
  }

// -----------------------Timer Interrupt...for experimentation---------------------------------



/*  ----------------------------------------------------------------------------------------------------------------
EXTERNAL interrupt ISR
reads the encoder button knob and toggles an output from 1 to 0 if it's been pressed
/
VARIABLES   
--------------------------------------------------------------------------------------------------------------------
*/
void isrService()
{
cli();
Serial.println("At ISR0  ");

 unsigned long interrupt_time = millis();
 // If interrupts come faster than 200ms, assume it's a bounce and ignore
 if (interrupt_time - last_interrupt_time > 200) 
 {
  // ... do your thing
toggleBit = !toggleBit;
Serial.print("And Toggle= ");
Serial.println(toggleBit); 
sei();
 buttonCounter = 1;
 
if(toggleBit == 1)
{

 //  wTrig.stopAllTracks();
 // wTrig.trackPlayPoly(1); 

}

if(toggleBit == 0)
{
   
 // wTrig.stopAllTracks();
 // wTrig.trackPlayPoly(1); 
}


 }
 last_interrupt_time = interrupt_time;


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

 unsigned long interrupt_time = millis();
 // If interrupts come faster than 200ms, assume it's a bounce and ignore
 if (interrupt_time - last_interrupt_time > 200) 
 {
  // ... do your thing
toggleBit = !toggleBit;
Serial.print("And Toggle= ");
Serial.println(toggleBit); 
 }
 
  wTrig.stopAllTracks();
  wTrig.trackPlayPoly(1);   

  
//for refernce this is the dound effect on selection used in teh last design int SoundEffectON_i[] = {0,1,2,3,8,9};
  
sei();

 last_interrupt_time = interrupt_time;

 }
 
//SECond ISR for remote switch activation through RFDuino




/*  ----------------------------------------------------------------------------------------------------------------
doSomethingAboutIt
launches GUI selected effects for touch
/
VARIABLES   
--------------------------------------------------------------------------------------------------------------------
*/
void doSomethingAboutIt_JustTouched(void)
{

  //playFullHaptic(HapLib_i[UserSelection],HapEffectON_i[UserSelection]);
  // Serial.print("Touched here..."); Serial.println();

 
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
    playFullHaptic(HapLib_i[UserSelection],HapEffectOFF_i[UserSelection]);
    
    Serial.print(", and release"); Serial.println();

    i = 0;      // PWM waveform table index - reset for next round.
    watchdog = 0; 
    
    analogWrite(Shaft_BLUE,0xFF); 

    delay(100); 
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



