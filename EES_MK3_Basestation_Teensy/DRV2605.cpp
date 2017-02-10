#include <Wire.h>

#include "DRV2605.h"


#define __PROG_TYPES_COMPAT__
#include <avr/pgmspace.h>

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "defs.h"
#include "DRV2605_defs.h"

DRV2605::DRV2605()
{
   Wire.begin(); // join i2c bus (address optional for master)
}




bool DRV2605::autoCal( uint8_t ratedVoltage, uint8_t overdriveClamp, boolean LRA, uint8_t* compensation, uint8_t* backEMF, uint8_t* feedback )
{
  // Set defaults
  setDefaults();

  // Work out control registers
  uint8_t fb = FB_BRAKE_4x | FB_LOOP_FAST;// | (LRA ? FB_MODE_LRA : FB_MODE_ERM);
  uint8_t control1 = DEFAULT_CTRL1;

  if( LRA )
  {
    fb |= FB_MODE_LRA;
  }
  else
  {
    fb |= FB_MODE_ERM;
  }


  // Write required registers
  I2C_Write( ADDR_RATED_VOLT, ratedVoltage );
  I2C_Write( ADDR_OD_CLAMP, overdriveClamp );
  I2C_Write( ADDR_FEEDBACK, fb );
  I2C_Write( ADDR_CTRL1,    control1 );
  I2C_Write( ADDR_MODE,   MODE_AUTOCAL );


  fb = GO;
  control1 = 0;
  I2C_Write( ADDR_GO, GO );

  do
  {
    I2C_Read( ADDR_GO, &fb );
    delay( 100 );                            //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    ++control1;
  } while( fb & GO && control1 < 100 );

  // Read status bit
  I2C_Read( ADDR_STATUS, &fb );

  if( fb & STAT_DIAG_BAD )
  {
    //Results did not converge
    Serial.print(F( "Fail: " ));
    Serial.print( control1 );
    Serial.print(F( " tries. Status " ));
    Serial.println( fb, BIN );
    return false;
  }

  Serial.print(F( "Status " ));
  Serial.println( fb, BIN );

  I2C_Read( ADDR_AC_COMP,     compensation );
  I2C_Read( ADDR_AC_BACK_EMF,   backEMF );
  I2C_Read( ADDR_FEEDBACK,    feedback );
  return true;
}

void DRV2605::playFullHaptic( uint8_t library, uint8_t effect, uint8_t ratedVoltage, uint8_t overdriveClamp, uint8_t compensation, uint8_t backEMF, uint8_t feedback )
{
  // Set defaults
  setDefaults();

  // Work out control registers
  uint8_t fb = FB_BRAKE_4x | FB_LOOP_FAST | (6 == library ? FB_MODE_LRA : FB_MODE_ERM); // Library 6 means LRA
  uint8_t control1 = DEFAULT_CTRL1;
  uint8_t control2 = DEFAULT_CTRL2;
  uint8_t control3 = DEFAULT_CTRL3;

  // Set saved BEMF Gain
  fb |= ( feedback & FB_BEMF_BITMASK );

  // Set open or closed loop based on library
  if( 1 == library )
    control3 |= CTRL3_ERM_OPEN; // Set bit 5 for open loop operation
  else
    control3 &= ~CTRL3_ERM_OPEN;  // Clear bit 5 for closed loop operation


  Wire.beginTransmission(DRV2605_ADDR);    // write 0xB4
        Wire.write(0x16);          // write first register address
        Wire.write(ratedVoltage);
        Wire.write(overdriveClamp);
        Wire.write(compensation);
        Wire.write(backEMF);
        Wire.write(fb);
  Wire.write(control1);
  Wire.write(control2);
  Wire.write(control3);
        Wire.endTransmission();
        
  Wire.beginTransmission(DRV2605_ADDR);    // write 0xB4
        Wire.write(0x03);          // write first register address
  Wire.write(library);
  Wire.write(effect);
  Wire.write(MODE_ACTIVE);
        Wire.endTransmission();    




  I2C_Write( ADDR_MODE,     MODE_ACTIVE );     //0x01

  fb = GO;
  control1 = 0;
  I2C_Write( ADDR_GO,     GO );
  do
  {
    // For effect 118, we have to stop by clearing the GO bit.
    if ( 118 == effect ) 
    {
      // Let it run for 1 seconds.
      if ( 10 == control1 ) 
      {
        I2C_Write( ADDR_GO, STOP );     // Clear GO bit to stop effect.
      }

      delay( 100 );            
      ++control1;
    }

    // Read GO bit to see if effect is still playing.
    I2C_Read( 0x0C, &fb );   
  } while ( fb & GO );
}

void DRV2605::setDefaults()
{   
  //                   // Protect I2C transaction from MPR121 interrupt.
        Wire.beginTransmission(DRV2605_ADDR);
        Wire.write(0x01);
        Wire.write(   DEFAULT_MODE );
  Wire.write(   DEFAULT_RTP_INPUT );
  Wire.write(   DEFAULT_LIBRARY );
  Wire.write(   DEFAULT_WAV_SEQ );
  Wire.write(   DEFAULT_WAV_SEQ );
  Wire.write(   DEFAULT_WAV_SEQ );
  Wire.write(   DEFAULT_WAV_SEQ );
  Wire.write(   DEFAULT_WAV_SEQ );
  Wire.write(   DEFAULT_WAV_SEQ );
  Wire.write(   DEFAULT_WAV_SEQ );
  Wire.write(   DEFAULT_WAV_SEQ );
  Wire.write(   DEFAULT_GO );
  Wire.write(   DEFAULT_OD_OFFSET );
  Wire.write(   DEFAULT_ST_OFFSETP );
  Wire.write(   DEFAULT_ST_OFFSETN );
  Wire.write(   DEFAULT_BT_OFFSET );
  Wire.write(   DEFAULT_A2H_CTRL );
  Wire.write(   DEFAULT_A2H_MIN_IN );
  Wire.write(   DEFAULT_A2H_MAX_IN );
  Wire.write(         DEFAULT_A2H_MIN_OUT );
  Wire.write(         DEFAULT_A2H_MAX_OUT );
  Wire.write(   DEFAULT_RATED_VOLT );
  Wire.write(   DEFAULT_OD_CLAMP );
  Wire.write(   DEFAULT_AC_COMP );
  Wire.write(         DEFAULT_AC_BACK_EMF );
  Wire.write(   DEFAULT_FEEDBACK );
  Wire.write(   DEFAULT_CTRL1 );
  Wire.write(   DEFAULT_CTRL2 );
  Wire.write(   DEFAULT_CTRL3 );
  Wire.write(   DEFAULT_AC_MEM );
  Wire.write(   DEFAULT_VBAT_VOLT );
  Wire.write(   DEFAULT_LRA_RES );
        Wire.endTransmission();
        //
        
}

void DRV2605::I2C_Read( uint8_t address, uint8_t *data )
{
  // // Protect I2C transaction from MPR121 interrupt, timer0, and usart
        
        Wire.beginTransmission(DRV2605_ADDR);
  Wire.write(address);           // write register address
        Wire.endTransmission();

  Wire.requestFrom(DRV2605_ADDR,1);
        data[0] = Wire.read();

       // Serial.println(data[0]); 
/*
        uint8_t numBytes = 1;
        for (uint8_t i=0; i<numBytes; ++i) {
                data[i] = i2cReceiveByte(i<numBytes-1); //ACK all but last byte.
  }
*/


}

void DRV2605::I2C_Write( uint8_t address, uint8_t data )
{
      
    Wire.beginTransmission(DRV2605_ADDR);
    Wire.write(address);
    Wire.write(data);
    Wire.endTransmission();

}

void DRV2605::PWM_Setup(void)
{

  I2C_Write( ADDR_MODE, MODE_PWM_ANALOG );     //0x03
  I2C_Write( ADDR_CTRL3,  CTRL3_IN_MODE_PWM);     //0x00


}


void DRV2605::SOUND_Setup(void)
{

  I2C_Write( ADDR_MODE ,  MODE_A2H );     //0x04
  I2C_Write( ADDR_CTRL1,  CTRL1_AC_COUPLE);
  I2C_Write( ADDR_CTRL3,  CTRL3_IN_MODE_AN );     //0x00
        

}
