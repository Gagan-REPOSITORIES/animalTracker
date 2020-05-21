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

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
