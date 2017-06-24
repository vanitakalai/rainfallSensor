/*
Project: Climate Edge
Group: 9
Author: Vanita K 
Function: Rain Sensor 

Overall Function:


Functions:
ISR (ANALOG_COMP_vect)
ISR(WDT_vect)
configure_wdt
configure_analog_comp
sleepModeRain
sleepModeNoRain
calcEnergy
signalMapper
dataReadWrite


*/


#include <avr/wdt.h>            // library for default watchdog functions
#include <avr/interrupt.h>      // library for interrupts handling
#include <avr/sleep.h>          // library for sleep
#include <avr/power.h>          // library for power control
#include <EEPROM.h>             // library for storing data in EEPROM
#include <SoftwareSerial.h>     // library for bluetooth data transfer
#include <Wire.h>

#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit)) // define functions for ADC sampling cycle set
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

/* Global Variables */

/* Sleep Timers */
int sleepCyclesLeft;  // tracks sleep cycles left
int sleepCyclesNo=1; // sets number of sleep cycles (1 min to 5 min---> 5 to 32)

/* Awake Cycle Timers */
unsigned long timeStart = 0; // tracks start time of each awake cycle 
unsigned long timeNow;  // tracks current time
unsigned long timeInterval=3000; // sets time for awake cycle -> time scaled w.r.t clock (1 min to 5 min---> 7500 to 37500)
volatile boolean triggered = false; // checks for interrupt from analog comparator

/* Energy Computation Variables */
unsigned int piezoE = 0; // energy of signal
int peakCount = 0;      // number of signals
const int peakThresh = 3; // threshold for signal
int piezoMaxArray[] = {0,0,0,0}; // tracks maximum peak value
float dropSizeInst = 0.00; // tracks drop size
float rainDepth = 0.0000; // tracks overall rain volumev 
float rainConst = 0.3; // rain constant dependent on sensor area /

//analog comparator interrupt vector
ISR (ANALOG_COMP_vect)
  {
  triggered = true;
  }
  
// watchdog timer interrupt vector
ISR(WDT_vect)
{
 wdt_reset();
}

//configures analog comparator
void configure_analog_comp(){

  ADCSRB = 0;           // disable analog comparator multiplexer enable
  ACSR =  bit (ACI)     // clear analog comparator interrupt flag
        | bit (ACIE)    
        | bit (ACIS1);  // select analog comparator interrupt mode- trigger on falling edge
}

// configures watchdog timer
void configure_wdt(void)
{
 
  cli();                           // disable interrupts for changing the registers

  MCUSR = 0;                       // reset status register flag                                                                 
  WDTCSR |= 0b00011000;            
  WDTCSR =  0b01000000 | 0b100001; // set watchdog timer to 8 seconds per cycle

  sei();                           // re-enable interrupts

}

// put the Arduino to sleep while raining
void sleepModeRain(int sleepCycles)
{ 
 // configure the watchdog
  configure_wdt();
  
  sleepCyclesLeft = sleepCycles; // defines how many cycles should sleep

  // Set sleep to full power down
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  
  while (sleepCyclesLeft > 0){ // while some cycles left, sleep

  // Enable sleep and enter sleep mode
  sleep_enable();
  sleep_mode();
 
  // When awake, disable sleep mode
  sleep_disable();
  
  // Reduce number of sleep cycles left
  sleepCyclesLeft = sleepCyclesLeft - 1;
 
  }
 
  // set timings to determine time program samples
  //Serial.println("Waking Up");
  timeStart = millis();
}


// Put Arduino to sleep when not raining
void sleepModeNoRain(){
  
  // Set sleep to idle power down - enables analog comparator interrupt
  set_sleep_mode (SLEEP_MODE_IDLE);  

  // Turn off everything while asleep
  power_all_disable();        

  // Enable sleep and enter sleep mode.
  sleep_enable();  
  sleep_mode();  
  
  if( triggered ){
    
    // When awake, disable sleep mode
    triggered = false;
    sleep_disable();
    power_all_enable();
    
    //Serial.println("Triggered");
    //delay(100);
    }  
  
  }

// Compute approximate energy of signal
void calcEnergy(int piezoV, int sensorNo){
  if(piezoV != 0) {//checks for signal
    if( piezoV > piezoMaxArray[sensorNo]) { 
      piezoMaxArray[sensorNo] = piezoV; // finds maximum value of signal
    }
  } else {
    if (piezoMaxArray[sensorNo] > peakThresh) {// ensures maximum peak value above noise threshold
      peakCount++;  // increments number of peaks
      piezoE= (sq(piezoMaxArray[sensorNo]) + piezoE)/2; // computes moving average signal energy(simplified form)
    }
    piezoMaxArray[sensorNo]=0; // sets peak value tracker back to 0 once peak detected 
    
  }
}

// Map signal energy to drop size and rain depth
void signalMapper(){
  
   if(piezoE != 0){
    dropSizeInst = (0.4562*pow(piezoE,0.2853)); // compute instantaneous drop size
    rainDepth = rainConst*pow(dropSizeInst, 3)*peakCount; // compute total rainfall depth in um(micro metre)
   }
  }

 
// Read data from awake cycle and store value  
void dataReadWrite(int piezoValue0, int piezoValue1, int piezoValue2){


  int EEPROMval0 = (int) EEPROM.read(0); // read previous drop size
  int EEPROMval1 = (int) EEPROM.read(1); // read previous peak count
  int EEPROMval2 = (int) EEPROM.read(2); // read previous rain depth
  int diff = (EEPROMval2 - piezoValue2) / EEPROMval2; // check if there is a difference in rain depth

  
  if (piezoValue0 == 0) { // no peaks --> no rain --> sleep 

    // configure analog comparator again for sleep  
    configure_analog_comp();
    delay(200);

    // sleep until rain interrupts
    digitalWrite(13, LOW);
    sleepModeNoRain();
       
  }

  else if(EEPROMval1 == 0){ // input first value
    EEPROM.write(0, (byte) piezoValue0);    
    EEPROM.write(1, (byte) piezoValue1); 
    EEPROM.write(2, (byte) piezoValue2);  
    }
    
  else if(diff > 0.2 or diff < -0.2){ // large difference in measurements
    EEPROM.update(0, (byte) (piezoValue0 + EEPROMval0)); // update measurements
    EEPROM.update(1, (byte) (piezoValue1 + EEPROMval1));
    EEPROM.update(2, (byte) (piezoValue2 + EEPROMval2));
    timeInterval = max(7500, (1 - diff)*timeInterval); // compute new cycle timings
    sleepCyclesNo = (1 - diff)*sleepCyclesNo; 
    }
  else if(diff < 0.2 or diff > -0.2){ // no difference in measurements
    EEPROM.update(0, (byte) (piezoValue0 + EEPROMval0));
    EEPROM.update(1, (byte) (piezoValue1 + EEPROMval1));
    EEPROM.update(2, (byte) (piezoValue2 + EEPROMval2));
    timeInterval = 37500; // new cycle timings set back to maximum
    sleepCyclesNo = 32;
    }   
  }

// Clear data at startup
void EEPROM_clear(){
  for(int i=0; i < 3; i++){
      EEPROM.write(i,0);  
    }
  }
  
// Setup at switch on 
void setup(){
  
  //set baud rate for serial port
  Serial.begin(76800);  
  
  // configure analog comparator
  configure_analog_comp();

  // clock prescaler setup 
  CLKPR = _BV(CLKPCE);  // enable change of the clock prescaler
  CLKPR = _BV(CLKPS0) | _BV(CLKPS1) ;  // divide frequency by 4, 4MHz clock

  // clear EEPROM
  EEPROM_clear();

  // turn off brown-out enable (low voltage detection)
  MCUCR = bit (BODS) | bit (BODSE);  
  MCUCR = bit (BODS);

  //set inputs and outputs
  pinMode(6, INPUT); // set ADC comparator pins as inputs
  pinMode(7, INPUT);
  pinMode(13, OUTPUT); // set pin 13 as output 
  pinMode(A0, INPUT); // set ADC pins as inputs
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);

  // flash to signal end of configuration
  digitalWrite(13, HIGH); 
  delay(100);
  digitalWrite(13, LOW);

  // sleep until interrupt detected/starts raining
  sleepModeNoRain();

  // set ADC clock in acceptable range (50-200kHz)
  sbi(ADCSRA, ADPS2); 
  cbi(ADCSRA, ADPS1);
  sbi(ADCSRA, ADPS0);

  // check time at start of program
  timeStart = millis();

  // light ON to signal awake cycle 
  digitalWrite(13, HIGH);
}



 
// loop time dependent on clock time and program time
void loop(){
  
    timeNow=millis(); // set current time during program

    //perform ADC
    digitalWrite(12, HIGH);
    calcEnergy(analogRead(A0), 0);
    digitalWrite(12, LOW);     
    calcEnergy(analogRead(A1), 1);
    digitalWrite(12, HIGH);
    calcEnergy(analogRead(A2), 2); 
    digitalWrite(12, LOW); 
    calcEnergy(analogRead(A3), 3); 

    if( (timeNow - timeStart) >= timeInterval ){ // if awake cycle time over
      
    signalMapper(); // map the signal energy and peak count to rain variables

    dataReadWrite(dropSizeInst, peakCount, rainDepth); // write variables to EEPROM   

    digitalWrite(13, LOW); // light OFF to signal sleep cycle
   
    sleepModeRain(sleepCyclesNo); // go to sleep during rain 

    digitalWrite(13, HIGH); // light ON to signal awake cycle

    peakCount = 0; // set variables to 0 for next awake cycle
    dropSizeInst = 0;
    rainDepth = 0;
    piezoE = 0;
  }
    
}
