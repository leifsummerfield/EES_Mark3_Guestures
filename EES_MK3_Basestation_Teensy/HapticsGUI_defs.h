
/*  ----------------------------------------------------------------------------------------------------------------
This chunk of code defines all the GUI display string arrays
all the ON/OFF effects arrays ect ect
--------------------------------------------------------------------------------------------------------------------
*/


#ifndef include_HapticsGUI_defs_h
#define include_HapticsGUI_defs_h


// Up First...the LCD Display Strings
char* LCD_GUI[]={"This is string 0", "Custom Set #1", "Custom Set #2",
"Custom Set #3", "Custom Set #4","Custom Set #5"};

char* LCD_GUI_Line2[]={" zero ",
//Note you have 16 characters to fit..or this many
/*
 abcdefghijklmnop  
 */
"ON= 24,OFF=64",
"ON= 2, OFF=11",
"ON= 3, OFF=12",
"ON= 4, OFF=13",
"ON= 5, OFF=14"};

//Next ...haptics motor definitions
// The index into this array is the selection #...so 
//  Note index 0 will be unused for clarity..so 0 is a placeholder
int HapLib_i[] = {0, 2, 2, 2, 2, 2};

int HapEffectON_i[] = {0, 24, 2, 3, 4, 5}; 

int HapEffectOFF_i[] = {0, 64, 11, 12, 13, 14}; 

int HapPWM_During_i[] ={0,1,2,3,4,5}; 

int SoundEffectON_i[] = {0,1,2,3,8,9};

int SoundEffectDURING_i[] = {0,4,5,9,11,12};

int SoundEffectOFF_i[] = {0,20,21,22,23,24};

#endif // include_HapticsGUI_defs_h

