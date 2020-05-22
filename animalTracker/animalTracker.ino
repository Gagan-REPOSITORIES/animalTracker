/*** 
 * Title: animalTracker and Health Monitor
 * Author : Gagan<gagan.databee@gmail.com>
 * Status: Active
 * Created : 21/05/2020
 * Description: This code is run on Microcontroller atmega328pAu to monitor G500 GSM and GPS module 
 * and get data from MPU6050 sensor with batter management IC
 * The hardware is design and developed by Gagan Deepak
 * Pins Usage:-
 * Digital pin 0: Receiver pin of atmega[careful with GSM Tx connection due to changes in pin mapping]
 * Digital pin 1: Transmission pin of atmega[careful with GSM Rx connection due to changes in pin mapping]
 * Digital pin 3: OUTPUT : GNSS enable pin , LOW:OFF, HIGH:ON
 * digital pin 4: INPUT : battery charger indicator, LOW:Fault, HIGH: Charging
 * digital pin 5: INPUT : Battery Fault Indicator, LOW: Fault, HIGH: Charging
 * digital pin 6: OUTPUT : MPU6050 enable pin, LOW: OFF, HIGH:ON
 * digital pin 7: OUTPUT : GSM_ON Control pin, LOW: Turn ON, HIGH:Turn OFF [Solder jumper is used]
 * digital pin 8: Software Receiver pin of atmega to interface with GPS Transmission pin[wire jumper is used]
 * digital pin 9: Software Transmission pin of atmega to insterface with GPS Receiver pin[wire jumper is used]
 * digital pin 10: OUTPUT : Status Indicator Green LED
 * analog pin 0: OUTPUT : GSM interrupt pin, LOW: Interrupts, HIGH:Check this due to current sink of gsm
 * analog pin 1: INPUT : LPG Signal of GSM Toggle Signals [Solder jumper is used]
 * analog pin 2: OUTPUT : Status Indicator Green LED
 * analog pin 3: OUTPUT : Status Indicator Green LED
 * analog pin 4: SDA : Connected to MPU6050[solder jumper is used]
 * analog pin 5: SCl : Connected to MPU6050[solder jumper is used]
***/

#define gpsen 3
#define  batcharge 4
#define batfault 5
#define mpuen 6
#define gsmpwrctrl 7
#define srx 8
#define stx 9
#define gsmint A0
#define gsmlpg A1
#define statusled1 10
#define statusled2 A2
#define statusled3 A3

#include "Wire.h" // This library allows you to communicate with I2C devices.

const int MPU_ADDR = 0x68; // I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69.
int16_t accelerometer_x, accelerometer_y, accelerometer_z; // variables for accelerometer raw data
int16_t gyro_x, gyro_y, gyro_z; // variables for gyro raw data
int16_t temperature; // variables for temperature data
char tmp_str[7]; // temporary variable used in convert function

char* convert_int16_to_str(int16_t i) 
{ // converts int16 to string. Moreover, resulting strings will have the same length in the debug monitor.
  sprintf(tmp_str, "%6d", i);
  return tmp_str;
}

int mpudata()
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
  Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
  Wire.requestFrom(MPU_ADDR, 7*2, true); // request a total of 7*2=14 registers
  
  // "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
  accelerometer_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
  accelerometer_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
  accelerometer_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
  temperature = Wire.read()<<8 | Wire.read(); // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
  gyro_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
  gyro_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
  gyro_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)
  
  // print out data
  //Serial.print("aX = "); 
  Serial.print(convert_int16_to_str(accelerometer_x));
  //Serial.print(" | aY = "); 
  Serial.print(convert_int16_to_str(accelerometer_y));
  Serial.print(" ");
  //Serial.print(" | aZ = "); 
  Serial.print(convert_int16_to_str(accelerometer_z));
  // the following equation was taken from the documentation [MPU-6000/MPU-6050 Register Map and Description, p.30]
  Serial.print(" "); 
  Serial.print(temperature/340.00+36.53);
  //Serial.print(" | gX = "); 
  Serial.print(convert_int16_to_str(gyro_x));
  //Serial.print(" | gY = "); 
  Serial.print(convert_int16_to_str(gyro_y));
  //Serial.print(" | gZ = "); 
  Serial.print(convert_int16_to_str(gyro_z));
  Serial.println();
  return 0
}
void setup()
{
  pinMode(gpsen, OUTPUT);
  pinMode(mpuen, OUTPUT);
  pinMode(batcharge, INPUT);
  pinMode(batfault, INPUT);
  pinMode(statusled1, OUTPUT);
  pinMode(statusled2, OUTPUT);
  pinMode(statusled3, OUTPUT);
  digitalWrite(gpsen, HIGH);//turn on gps module
  digitalWrite(mpuen, HIGH);//turn on mpu6050
  delay(2000);//time delay of mpu6050 between turn ON and data transmission
  Serial.begin(9600); //baud rate of Serial Monitor
  Wire.begin();//I2C communication begin
  Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
}

void loop() 
{

}
