/*********************************************************************
 * File Name: btSwitch.info
 * 
 * Author: Jordan Baxter
 * 
 * Group Members: Jordan Baxter, Suyang Liu, Trenton Kilgore
 * 
 * Description: This is the main firmware file for the bluetooth power outlet
 *              designed by group Blue(1) (AKA "Reverse Biased") for Junior design
 *              ECE 342 Spring 2019.
 * 
 * Copyright Information: This code was heavily derived from the Adafruit ble library "controller"
 *                        example sketch. The author started with an exact copy of the example sketch, 
 *                        and modified it to fit the needs of the project. The main portions that are 
 *                        copied from Adafruit Industries are set up and ble connection settings. 
 *                        Custom portions are timer, outlet structs, and current sense settings.
*********************************************************************/

#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "HW.h"
#include "outlet.h"

#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif

/*=========================================================================
    APPLICATION SETTINGS

    FACTORYRESET_ENABLE       Perform a factory reset when running this sketch
   
                              Enabling this will put your Bluefruit LE module
                              in a 'known good' state and clear any config
                              data set in previous sketches or projects, so
                              running this at least once is a good idea.
   
                              When deploying your project, however, you will
                              want to disable factory reset by setting this
                              value to 0.  If you are making changes to your
                              Bluefruit LE device via AT commands, and those
                              changes aren't persisting across resets, this
                              is the reason why.  Factory reset will erase
                              the non-volatile memory where config data is
                              stored, setting it back to factory default
                              values.
       
                              Some sketches that require you to bond to a
                              central device (HID mouse, keyboard, etc.)
                              won't work at all with this feature enabled
                              since the factory reset will clear all of the
                              bonding data stored on the chip, meaning the
                              central device won't be able to reconnect.
    MINIMUM_FIRMWARE_VERSION  Minimum firmware version to have some new features
    MODE_LED_BEHAVIOUR        LED activity, valid options are
                              "DISABLE" or "MODE" or "BLEUART" or
                              "HWUART"  or "SPI"  or "MANUAL"
    -----------------------------------------------------------------------*/
    #define FACTORYRESET_ENABLE         1
    #define MINIMUM_FIRMWARE_VERSION    "0.6.6"
    #define MODE_LED_BEHAVIOUR          "MODE"
/*=========================================================================*/


/* Insantiate BLE object */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

// Instantiate outlet objects
Outlet leftOutlet("Left Outlet", RELAY_1, CURRENT_SENSE_1);
Outlet rightOutlet("Right Outlet", RELAY_2, CURRENT_SENSE_2);
 


// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

// function prototypes over in packetparser.cpp
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);

// the packet buffer
extern uint8_t packetbuffer[];


/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void)
{

 // while (!Serial);  // required for Flora & Micro
  delay(500);

  Serial.begin(115200);
  Serial.println(F("Bluetooth LE Switch Controller"));
  Serial.println(F("-----------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }
  //Change the device name so it is easily identifiable in app.

  if (! ble.sendCommandCheckOK(F("AT+GAPDEVNAME= RB Bluetooth Switch")) ) {
    error(F("Could not set device name?"));
  }
  
  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in Controller mode"));
  Serial.println(F("Then activate/use the sensors, color picker, game controller, etc!"));
  Serial.println();

  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
      delay(500);
  }

  Serial.println(F("******************************"));

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }

  // Set Bluefruit to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  Serial.println(F("******************************"));

  pinMode(RELAY_1, OUTPUT);
  pinMode(RELAY_2, OUTPUT);
  pinMode(CURRENT_SENSE_1, INPUT);
  pinMode(CURRENT_SENSE_2, INPUT);

}

/**************************************************************************/
/*!
    Description: Compiles all outlet data and forms into a cohesive data packet with a
                 checksum. Then checks for received data and processes if received.
*/
/**************************************************************************/
void loop(void)
{
  //Create datapacket
  uint8_t switchData[19];
  switchData[0] = '!';
  //Place S in first slot. Leave second slot empty for device ID.
  switchData[1] = 'S';
  /************************
   * Retrieve left outlet data.
   ***************************/
  switchData[3] = leftOutlet.getOnOff();
  uint16_t leftCurrent = leftOutlet.getCurrent();
  switchData[4] = highByte(leftCurrent);
  switchData[5] = lowByte(leftCurrent);
  //Get time remaining before filling timer on/off slot, 
  //getTimeRemaining() will update timer on/off if time remaining == 0.
  uint16_t leftTimer = leftOutlet.getTimeRemaining();
  switchData[6] = leftOutlet.getTimerOnOff();
  switchData[7] = highByte(leftTimer);
  switchData[8] = lowByte(leftTimer);

  /************************
   * Retrieve right outlet data.
   ***************************/
  switchData[11] = rightOutlet.getOnOff();
  uint16_t rightCurrent = rightOutlet.getCurrent();
  switchData[12] = highByte(rightCurrent);
  switchData[13] = lowByte(rightCurrent);
  //Get time remaining before filling timer on/off slot, 
  //getTimeRemaining() will update timer on/off if time remaining == 0.
  uint16_t rightTimer = rightOutlet.getTimeRemaining();
  switchData[14] = rightOutlet.getTimerOnOff();
  switchData[15] = highByte(rightTimer);
  switchData[16] = lowByte(rightTimer);

  uint8_t checksum = 0; //Create a checksum variable.

  for (int i = 0; i < 19; ++i){
    checksum += switchData[i]; //Add data to checksum. Discarding overflow bits.
    ble.write(switchData[i]);
  }
  checksum = ~checksum; // Take one's compliment of checksum
  ble.write(checksum); // Send checksum as 20th Byte in packet.
  ble.write('\r'); //This sends the data, whether the buffer is full or not.

  /************************
   * Check for overcurrent. This function will be commented out during block checkoff. 
   ***************************/
  if((leftCurrent + rightCurrent) / 1000 >= 4.95){ //If combined currents are close to 5A, turn off both relays.
    if(leftOutlet.getOnOff()){
      leftOutlet.switchOnOff();
    }
    if(rightOutlet.getOnOff()){
      rightOutlet.switchOnOff();
    }
    ble.write('!');
    ble.write('O');
    checksum = ~('!' + 'O');
    ble.write(checksum);
    ble.write('/r');
  } 

  /* Check if new data has arrived */
  uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
  if (len == 0) return;

  /* Got a packet! */
  printHex(packetbuffer, len);

  //Outlet Controller
  if(packetbuffer[1] == 'P'){
     if(packetbuffer[3] == 'L'){
       leftOutlet.switchOnOff();
     } else if(packetbuffer[3] == 'R'){
       rightOutlet.switchOnOff();
     }
  }

  //Timer Controller
  if(packetbuffer[1] == 'T'){
    if(packetbuffer[4] == 'S'){
      int timeset = packetbuffer[5];
      timeset << 4;
      timeset += packetbuffer[6];
      if(packetbuffer[3] == 'L'){
        leftOutlet.setTimer(timeset);
      } else if(packetbuffer[3] == 'R'){
        rightOutlet.setTimer(timeset);
      }
    } else if (packetbuffer[4] == 'C'){
      if(packetbuffer[3] == 'L'){
        leftOutlet.timerCancel();
      } else if (packetbuffer[3] == 'R'){
        rightOutlet.timerCancel();
      }
    }
  }

  // Color
  if (packetbuffer[1] == 'C') {
    uint8_t red = packetbuffer[2];
    uint8_t green = packetbuffer[3];
    uint8_t blue = packetbuffer[4];
    Serial.print ("RGB #");
    if (red < 0x10) Serial.print("0");
    Serial.print(red, HEX);
    if (green < 0x10) Serial.print("0");
    Serial.print(green, HEX);
    if (blue < 0x10) Serial.print("0");
    Serial.println(blue, HEX);
  }

  // Buttons
  if (packetbuffer[1] == 'B') {
    uint8_t buttnum = packetbuffer[2] - '0';
    boolean pressed = packetbuffer[3] - '0';
    Serial.print ("Button "); Serial.print(buttnum);
    if (pressed) {
      Serial.println(" pressed");
    } else {
      Serial.println(" released");
    }

    // if (buttnum == 1 && pressed){
    //   leftOutlet.switchOnOff();
    // }
    // if(buttnum == 2 && pressed){
    //   rightOutlet.switchOnOff();
    // }
    // if(buttnum == 3 && pressed){
    //   leftOutlet.setTimer(10);
    // }
    // if(buttnum == 4 && pressed){
    //   rightOutlet.setTimer(10);
    // }
    // if(buttnum == 7 && pressed){
    //   leftOutlet.timerCancel();
    // }
    // if(buttnum == 8 && pressed){
    //   rightOutlet.timerCancel();
    // }
  }

  // GPS Location
  if (packetbuffer[1] == 'L') {
    float lat, lon, alt;
    lat = parsefloat(packetbuffer+2);
    lon = parsefloat(packetbuffer+6);
    alt = parsefloat(packetbuffer+10);
    // Serial.print("GPS Location\t");
    // Serial.print("Lat: "); Serial.print(lat, 4); // 4 digits of precision!
    // Serial.print('\t');
    // Serial.print("Lon: "); Serial.print(lon, 4); // 4 digits of precision!
    // Serial.print('\t');
    // Serial.print(alt, 4); Serial.println(" meters");
  }

  // Accelerometer
  if (packetbuffer[1] == 'A') {
    float x, y, z;
    x = parsefloat(packetbuffer+2);
    y = parsefloat(packetbuffer+6);
    z = parsefloat(packetbuffer+10);
    // Serial.print("Accel\t");
    // Serial.print(x); Serial.print('\t');
    // Serial.print(y); Serial.print('\t');
    // Serial.print(z); Serial.println();
  }

  // Magnetometer
  if (packetbuffer[1] == 'M') {
    float x, y, z;
    x = parsefloat(packetbuffer+2);
    y = parsefloat(packetbuffer+6);
    z = parsefloat(packetbuffer+10);
    // Serial.print("Mag\t");
    // Serial.print(x); Serial.print('\t');
    // Serial.print(y); Serial.print('\t');
    // Serial.print(z); Serial.println();
  }

  // Gyroscope
  if (packetbuffer[1] == 'G') {
    float x, y, z;
    x = parsefloat(packetbuffer+2);
    y = parsefloat(packetbuffer+6);
    z = parsefloat(packetbuffer+10);
    // Serial.print("Gyro\t");
    // Serial.print(x); Serial.print('\t');
    // Serial.print(y); Serial.print('\t');
    // Serial.print(z); Serial.println();
  }

  // Quaternions
  if (packetbuffer[1] == 'Q') {
    float x, y, z, w;
    x = parsefloat(packetbuffer+2);
    y = parsefloat(packetbuffer+6);
    z = parsefloat(packetbuffer+10);
    w = parsefloat(packetbuffer+14);
    // Serial.print("Quat\t");
    // Serial.print(x); Serial.print('\t');
    // Serial.print(y); Serial.print('\t');
    // Serial.print(z); Serial.print('\t');
    // Serial.print(w); Serial.println();
  }
}
