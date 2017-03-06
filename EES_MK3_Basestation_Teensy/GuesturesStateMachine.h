
/*  ----------------------------------------------------------------------------------------------------------------
This chunk of codes simply move the routines out of the main loop for readabilty and code maintantance 

--------------------------------------------------------------------------------------------------------------------
*/


#ifndef include_GuesturesStateMachine_h
#define include_GuesturesStateMachine_h


#include <Arduino.h> //needed for Serial.println
#include <string.h> //needed for memcpy
//#include <HapticsGUI_defs.h> //for effects. 

//SERIAL OUTPUT FUNCTION
//For plotting in CSV type format using a Telemetry viewer software JarRscrLoader
// Our Structure is going to be just an array of numbers that represent differnet global variables digging in from 
void  printTelemetry(const char* message)
{
  String telem = ""; // Create a fresh line to log
  
  telem += String(toggleBit) + ",";            // Plot 1: Toggle state (push button)
  telem += String(bTouched1) + ",";             // Plot 2: Touch State
  telem += String(Tilt) + ",";                 // Plot 3: Tilt value
  telem += String(TiltedMode);           // Plot 4: Tilt mode-shift
  
 // telem += ","; telem += message;                             // Last chunk == debug strings NOTE: Remove to use telemetry viewer
  
  telem += "\r\n"; // Add a new line and we are DONE
  Serial.print(telem);      
//This is a blocking function...so let's not spin to fast for COMport reading purposes, put a delay in here (20hz is plenty fast)
  delay(50); 
}


//		Standby mode is where we start, the tool is in-active       ---------------------------------MAIN LOOP FUNCTIONS ----------------------
//		but we want to spin through the telemetry outputs and poll everyting
void	enterStandBy_Mode(void)
{
    printTelemetry("Entering Toggle=1 waiting state");   
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("State #2");
    lcd.setCursor(0,1);
    lcd.print("STANDBY");

    strip.setPixelColor(0, strip.Color(0, 0, 50));
    strip.setPixelColor(1, strip.Color(0, 50, 0));
    strip.setPixelColor(2, strip.Color(0, 0, 0));
    strip.setPixelColor(3, strip.Color(0, 0, 0)); 
    strip.setPixelColor(4, strip.Color(0, 0, 0));
    strip.show();

    //fiber LED state set
    digitalWrite(fiberLED,LOW); 

    TiltedMode = 1; //Re-init Tilt Mode 
}

//		Armed Mode is where we're in stand-by for a touch event that activates the tool
void	enterArmed_Mode(void)
{
	//LCD Display GUI actions
    lcd.clear();
    lcd.setCursor(0,1);
    lcd.print("ARMED    Mode(1)"); 

        //fiber LED state set
    digitalWrite(fiberLED,LOW); 
}


//		Waiting in Armed mode for the touch event
void	armedAndWaiting(void)
{
	//LCD Support
    lcd.setCursor(0,0);
    lcd.print("State #3");
    lcd.setCursor(0,1);
    lcd.print("ARMED"); 
    
    //LED strip support
    strip.setPixelColor(0, strip.Color(0, 0, 50));
    strip.setPixelColor(1, strip.Color(0, 50, 0));
    strip.setPixelColor(2, strip.Color(50, 25, 0));
    strip.setPixelColor(3, strip.Color(0, 0, 0));
    strip.setPixelColor(4, strip.Color(0, 0, 0));
    strip.show();

    //fiber LED state set
    digitalWrite(fiberLED,LOW); 
    
    printTelemetry("Armed"); 
    delay(50);    //so we don't get too busy. otherwise we crash here ! 
}

//	If were just touched on sensor #1...do some stuff before waiting to be un-touched
void	justTouched_on1(void)
{
	//LCD Support
	lcd.setCursor(0,0);
	lcd.print("State #3");
	lcd.setCursor(0,1);
	lcd.print("ACTIVE"); 

	//LED strip support            
	strip.setPixelColor(3, strip.Color(50, 0, 0));
	strip.show();

    //fiber LED state set
    digitalWrite(fiberLED,HIGH); 

	//play some sound the first time through
	wTrig.stopAllTracks();
  wTrig.trackPlayPoly(SoundEffectON_i[TiltedMode]); 
  
}

// 	When the tilt-sensor goes past threshold, then the tilted action takes place that indexes the MODE variable
//   I'd like this whole thing to happen in the background next. 
void	justTilted(void)
{
int SoundEffectON_i[] = {0,1,2,3,8,9};
        TiltedMode = TiltedMode + 1; 
        if(TiltedMode > 5)
          TiltedMode = 1; 

      
        lcd.setCursor(0,0);
        lcd.print("State #3");
        lcd.setCursor(9,1);
        lcd.print("Mode(");
        lcd.setCursor(14,1);
        lcd.print(TiltedMode);
        lcd.setCursor(15,1);
        lcd.print(")"); 
            
        strip.setPixelColor(4, strip.Color(30, 0, 50));
        strip.show();
        //play some sound the first time through
        wTrig.stopAllTracks();
        wTrig.trackPlayPoly(SoundEffectON_i[TiltedMode]); 
        
        
        delay(500); //a little T-delay now and then,.. relished by the wisest men.
        


}







#endif // include_GuesturesStateMachine_h
