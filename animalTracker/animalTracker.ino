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
#define batcharge 4
#define batfault 5
#define mpuen 6
#define gsmpwrctrl 7
#define stx 8
#define srx 9
#define gsmint A0
#define gsmlpg A1
#define statusled1 10
#define statusled2 A2
#define statusled3 A3

#include "Wire.h" // This library allows you to communicate with I2C devices.
#include "SoftwareSerial.h" 
#include <avr/wdt.h> //for watchdog timer

const int MPU_ADDR = 0x68; // I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69.
int16_t accelerometer_x, accelerometer_y, accelerometer_z; // variables for accelerometer raw data
int16_t gyro_x, gyro_y, gyro_z; // variables for gyro raw data
int16_t temperature; // variables for temperature data
char tmp_str[7]; // temporary variable used in convert function

unsigned long previousMillis = 0;
const long interval = 60;//enter numbers in seconds like 60 for 60seconds
String a;
String gpsdata;
int volt = 0;
bool TCPconn = false;
bool dataline = false;

SoftwareSerial mySerial(srx,stx);

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
  // Serial.print("aX = "); 
  // Serial.print(convert_int16_to_str(accelerometer_x));
  // Serial.print(" | aY = "); 
  // Serial.print(convert_int16_to_str(accelerometer_y));
  // Serial.print(" ");
  // Serial.print(" | aZ = "); 
  // Serial.print(convert_int16_to_str(accelerometer_z));
  // the following equation was taken from the documentation [MPU-6000/MPU-6050 Register Map and Description, p.30]
  // Serial.print(" "); 
  // Serial.print(" | temp = "); 
  // Serial.print(temperature/340.00+36.53);
  // Serial.print(" | gX = "); 
  // Serial.print(convert_int16_to_str(gyro_x));
  // Serial.print(" | gY = "); 
  // Serial.print(convert_int16_to_str(gyro_y));
  // Serial.print(" | gZ = "); 
  // Serial.print(convert_int16_to_str(gyro_z));
  // Serial.println();
  //if data is absence or mpu is not responding then send false
  if(accelerometer_x == -1 && accelerometer_y == -1 && accelerometer_z == -1)
    return false;
  else
    return true;
  
}
/**
blink1 is battery status indicator D9 led
blink2 is GSM status indicator D7 led
blink3 is GPS and MPU6050 data indicator D8 led
blink1 = 0 no issues with the battery
blink1 = 1 battery is below 
blink1 = 2 battery charging indicator
blink1 = 3 battery full indicator
blink1 = 4 battery fault indicator
blink2 = 0 GSM and Microcontroller communication is properly established
blink2 = 1 GSM at command is not responding
blink2 = 2 GSM sim card not inserted
blink2 = 3 sim card registration failed
blink2 = 4 problem with connectivity to TCP
blink3 = 0 receiving proper data from both the GPS and MPU
blink3 = 1 not receiving data from mpu6050
blink3 = 2 not receiving data from GPS
**/
int statusind(int blink1,int blink2,int blink3)
{
  for(int i = 0;i<blink1;i++)
  {
    digitalWrite(statusled1, HIGH);
    delay(250);
    digitalWrite(statusled1, LOW);
    delay(250);
  }
  for(int i = 0;i<blink2;i++)
  {
    digitalWrite(statusled2, HIGH);
    delay(250);
    digitalWrite(statusled2, LOW);
    delay(250);
  }
  for(int i = 0;i<blink3;i++)
  {
    digitalWrite(statusled3, HIGH);
    delay(250);
    digitalWrite(statusled3, LOW);
    delay(250);
  }
  return 0;
}

/**
 * batcharge = HIGH(1) battery is drawing less current than 1/10 of programmed
 * batcharge = LOW(0) battery is drawing more current than 1/10 of programmed(battery charging)
 * batfault = HIGH(1) when no fault is found connected to powersource
 * batfault = LOW(0) when fault is found not connected to powersource
**/
int batstatus()
{//checks for battery charging conditions
  if(!digitalRead(batcharge))
    statusind(2,0,0);
  if(digitalRead(batcharge))
    statusind(3,0,0);
  if(!digitalRead(batfault))
    statusind(4,0,0);
  return 0;
}

/**
 * Microcontroller sends AT command to GSM module
 * GSM will reply only if the baudrate and other things matches properly 
 **/
int at()
{
  for (int i = 0; i < 5; i++)
  {
    mySerial.println("at");
    mySerial.flush();
    delay(100);
    while (mySerial.available())
    {
      if (mySerial.readString().indexOf("OK") > 0)
        return true;
    }
  }
  return false;
}

/**
 * Microcontroller gets the battery voltage from the GSM module
 * The function returns the integer value without the decimal point like 3300
 **/
int cbc()
{
  for (int i = 0; i < 5; i++)
  {
    mySerial.println("at+cbc");//+CBC: 0,3663 
    mySerial.flush();
    delay(100);
    while (mySerial.available())
    {
     a = mySerial.readString();
     a.remove(0,a.indexOf("+CBC: 0,")+8);
     a.remove(4);
     return a.toInt();
    }
  }
  return false;
}

/**
 * Checks the simcard presence in the device, if sim card is present then GSM will reply with READY
 * if simcard is not present the module will reply with error
 **/
int cpin()
{
  for (int i = 0; i < 5; i++)
  {
    mySerial.println("at+cpin?");//ERROR
    mySerial.flush();
    delay(100);
    while (mySerial.available())
    {
      if (mySerial.readString().indexOf("+CPIN: READY") > 0)
        return true;
    }
  }
  return false;
}

/**
 * Checks the simcard registration with network
 * if registered returns true else false
 **/
int creg()
{
  for (int i = 0; i < 5; i++)
  {
    mySerial.println("at+creg?");//+CREG: 0,0 without sim card
    mySerial.flush();
    delay(100);
    while (mySerial.available())
    {
      if (mySerial.readString().indexOf("+CREG: 0,1" || "+CREG: 0,5") > 0)
        return true;
    }
  }
  return false;
}

//used for hot plugging of sim card
int msmpd()
{
  for (int i = 0; i < 5; i++)
  {
    mySerial.println("at+msmpd=1");
    mySerial.flush();
    delay(100);
    while (mySerial.available())
    {
      if (mySerial.readString().indexOf("OK") > 0)
        return true;
    }
  }
}

/**
 * Turn ON the GPS with GSM
 * its not required bcoz the microcontroller controls the power input to gps
 * written this function to avoid error
 **/
int gpspowerON()
{
  for (int i = 0; i < 5; i++)
  {
    mySerial.println("at+gtgpspower=1");
    mySerial.flush();
    delay(100);
    while (mySerial.available())
    {
      if (mySerial.readString().indexOf("OK") > 0)
        return true;
    }
  }
  return false;
}

/**
 * Fetches GPS RMC data
 * if not available returns false
 * if available returns true and stores in the gps data
 **/
int getgpsdata()
{
  mySerial.println("at+gtgps=\"RMC\"");
  mySerial.flush();
  delay(100);
  while (mySerial.available())
  {
    gpsdata = mySerial.readString();
    if (gpsdata.indexOf("A,") > 0)
    {
      gpsdata = gpsdata.substring(gpsdata.indexOf("A,") + 2, gpsdata.indexOf("A,") + 28);
      return true;
    }
  } //Serial.println(mySerial.readString().length());
  return false;
}

/**
 * Checks the battery Voltage and charging levels
 * other routine checks 
 **/
int regularCheck()
{
  batstatus();//checks the charging conditions of battery
  volt = cbc();//gets battery voltage in integer number
  if(volt<3400)
  {
  statusind(1,0,0);
  digitalWrite(mpuen, LOW);//turn off mpu6050 when battery level is below certain limit
  digitalWrite(gpsen, LOW);//turn off gps when battery level is below certain limit
  }
  if(at()==false)//checks gsm and microcontroller communication
  statusind(0,1,0);
  if(cpin()==false)//checks simcard presence
  statusind(0,2,0);
  if(creg()==false)//checks network registration
  statusind(0,3,0);
  if(getgpsdata()==false)
  statusind(0,0,2);
  if(mpudata() == false)
  statusind(0,0,1);
}
/**
 * sets up a PPP (Point to Point Protocol) connection with the GGSN (Gate GPRS SupportNode)
 * returns a valid dynamic IP for the Module
 **/
int mipcall()
{
  for (int i = 0; i < 5; i++)
  {
    mySerial.println("AT+MIPCALL=1,\"internet\"");
    mySerial.flush();
    delay(1000);
    while (mySerial.available())
    {
      if (mySerial.readString().indexOf("+MIPCALL:") > 0)
        return true;
    }
  }
  return false;
}

/**
 * Terminates the PPP connection with server 
 * If established connection is not closed properly there will be problem in reconnecting to server again
 **/
int mipcall_close()
{
  for (int i = 0; i < 5; i++)
  {
    mySerial.println("AT+MIPCALL=0");
    mySerial.flush();
    delay(1300);
    while (mySerial.available())
    {
      if (mySerial.readString().indexOf("+MIPCALL: 0") > 0)
        return true;
    }
  }
  return false;
}

/**
 * Connects to TCP server with specified IP address and port number
 **/
int mipodm()
{
  for (int i = 0; i < 5; i++)
  {
    mySerial.println("AT+MIPODM=1,41960,\"GuhanGagan-41960.portmap.host\",41960,0");
    mySerial.flush();
    delay(3000);
    while (mySerial.available())
    {
      if (mySerial.readString().indexOf("+MIPODM: 1,1") > 0)
        return true;
    }
  }
  return false;
}

/**
 * Establishes TCP connection to communicate without any error
 **/
int establishTCP()
{
  if(creg()==true)
  {
    if(mipcall()==true)
    {
      if(mipodm()==true)
      {
        wdt_reset();
        TCPconn = true;
        return true;
      }
    }
  }
  return false;
}

/**
 * Closes TCP Connection properly so that next time there wont be any problem at the Server side
 **/
void closeTCP()
{
 for (int i = 0; i < 5; i++)
  {
    mySerial.print("DISCONNECT");
    mySerial.flush();
    delay(2000);
    while (mySerial.available())
    {
      if (mySerial.readString().indexOf("+DISCONNECT") > 0)
        dataline = false;
    }
  }
  if(mipcall_close()==true)
  TCPconn = false;
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
  pinMode(stx, OUTPUT);
  pinMode(srx, INPUT);
  statusind(2,0,0);//indicates the working condition of ATmega328;blinks the blinks twice
  digitalWrite(mpuen, HIGH);//turn on mpu6050 just to check working
  digitalWrite(gpsen, HIGH);//turn on gps module
  delay(2000);//time delay of mpu6050 between turn ON and data transmission
  Serial.begin(9600); //baud rate of Serial Monitor
  while(!Serial);
  mySerial.begin(9600);
  while(!mySerial);
  Wire.begin();//I2C communication begin
  Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);//ends the communication with the mpu6050
  if(at()==false) // Check the GSM connection established with atmega328; if connection is not established the led will blink once
  statusind(0,1,0);
  if(cpin()==false)// Check the sim card presence in the module; if the sim card is not present led will blink twice
  statusind(0,2,0);
  if(mpudata() == false)//Check the mpu6050 data; if we are not getting data led will blink once
  {
    statusind(0,0,1);//blinks once
    digitalWrite(mpuen,LOW);//turns off mpu6050 when not getting data
  }
  msmpd();
  wdt_enable(WDTO_8S);//enables watchdog timer for 8S
  Serial.println("Program is running and Setup is done");
}

void loop() 
{
  wdt_reset();//added in begining to avoid failure due to other lines duration
  unsigned long currentMillis = millis()/1000;
  if (currentMillis - previousMillis >= interval*2) 
  {
    previousMillis = currentMillis;
    regularCheck();
  }
  if (currentMillis - previousMillis >= interval) 
  {
    previousMillis = currentMillis;
    if(establishTCP()==false)
      statusind(0,4,0); //blinks 4 times when trouble in tcp connection
  }
  if(mySerial.available()>0 && TCPconn)
  {
    while (mySerial.available())
    {
      a = mySerial.readString();
      if (a.indexOf("+CONNECT") > 0)
        dataline = true;
    }
  }
  delay(1000);
  Serial.println("Program Author : GAGAN DEEPAK R");
  wdt_reset();//resets watch dog timer and if stuck anywhere it will restart the controller
  if(TCPconn && dataline)
  { //Send TCP data in this block
    mySerial.print("SerialData1");
    mySerial.print("SerialData2");
    mySerial.print("SerialData3");
    mySerial.print("SerialData4");
    closeTCP();
  }
}