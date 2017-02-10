/*
This sketch demonstrates how to send data from a Device
to a Host in a Gazell network.

The host and upto 3 devices should have the RGB shield
attached.  When Button A on a Device is pressed, the
associated led on the Host will toggle.  Device1 is
associated with the Red led, Device2 with the Green led
and Device3 with the Blue led.

The Green led on the Device will blink to indicate
that an acknowledgement from the Host was received.

Jan 15 2017
This is what's loaded on the RFDuino that's soldered onto the Teensy base station board
Use this program to relay a blip everytime something comes from the "foot switch" RFDuino

Ideally there should be some sort of serial communication goodness between these two (Teensy to RFDuino)...
but for now it's just a bit flip off-->on-->off and wait to repeat
*/

#include <RFduinoGZLL.h>

device_t role = HOST;

// pin for the Green Led
int green_led = 3;

void setup()
{
  pinMode(green_led, OUTPUT);

  // start the GZLL stack  
  RFduinoGZLL.begin(role);

  // initialize serial:
  Serial.begin(115200);
  Serial.println("Rfduino Online");
  
}

void loop()
{
}

void RFduinoGZLL_onReceive(device_t device, int rssi, char *data, int len)
{
  char state = data[0];

  // this test is not needed for a single device
  if (device == DEVICE0)
  
    digitalWrite(green_led, HIGH);
    delay(10); 
    digitalWrite(green_led, LOW); 
     
     Serial.println(String(state,BIN));
  
  // no data to piggyback on the acknowledgement sent back to the Device
  // RFduinoGZLL.sendToDevice(device, "OK");
}
