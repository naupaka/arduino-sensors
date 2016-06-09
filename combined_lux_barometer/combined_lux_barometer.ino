// Arduino sketch to log data from a range of environmental sensors
// Send output over the serial connection, where a processing script on
// a Raspberry Pi Linux computer logs the data, writes it to a file, and
// uploads it to a server

// Compiled and revised by Naupaka Zimmerman naupaka@gmail.com
// December 30, 2014
// Updated to add RTC and SD card, remove chirp soil moisture
// June 9, 2016

// Output format is comma-delimited, fields are:
// YYYYMMDD, TIME24HR, MPL3115A2_barometric_pressure_in_pascals, 
// MPL3115A2_temp_in_deg_C, TSL2561_light_in_lux, DHT_humidity_pcnt, DHT_temp_in_deg_C,
// liquid_flow_frequency, liquid_flow_pulses_cumulative_count, 
// liquid_flow_cumulative_liters, FSR_Analog_reading, FSR_voltage_in_mV, 
// FSR_resistance_in_ohms, FSR_conductance_in_microMhos, FSR_force_in_Newtons


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// Environmental sensors (and associated libraries) used include:

// Adafruit TSL2561 Digital Luminosity/Lux/Light Sensor Breakout
// http://www.adafruit.com/product/439
// Library and examples: https://github.com/adafruit/TSL2561-Arduino-Library
// Public domain

// Adafruit MPL3115A2 - I2C Barometric Pressure/Altitude/Temperature Sensor
// http://www.adafruit.com/product/1893
// Library and examples: https://github.com/adafruit/Adafruit_MPL3115A2_Library
// Driver for the Adafruit MPL3115A2 barometric/altitude pressure sensor breakout
// Designed specifically to work with the MPL3115A2 Breakout in the Adafruit shop 
// ----> https://www.adafruit.com/products/1893
// This sensor uses I2C to communicate, 2 pins are required to interface
// Adafruit invests time and resources providing this open source code, 
// please support Adafruit and open-source hardware by purchasing products from Adafruit!
// Written by Limor Fried/Kevin Townsend for Adafruit Industries.
// BSD license, all text above must be included in any redistribution

// Adafruit AM2302 (wired DHT22) temperature-humidity sensor
// http://www.adafruit.com/products/393
// Library and example code: https://github.com/adafruit/DHT-sensor-library
// Public domain

// Adafruit Square Force-Sensitive Resistor (FSR) - Interlink 406
// https://www.adafruit.com/products/1075
// Code and examples (no library needed): 
// https://learn.adafruit.com/force-sensitive-resistor-fsr/using-an-fsr

// Adafruit Liquid Flow Meter - Plastic 1/2" NPS Threaded
// http://www.adafruit.com/products/828
// Code and example sketch: https://github.com/adafruit/Adafruit-Flow-Meter
// This is example code for using the Adafruit liquid flow meters. 
// Tested and works great with the Adafruit plastic and brass meters
//     ------> http://www.adafruit.com/products/828
//     ------> http://www.adafruit.com/products/833
// Connect the red wire to +5V, the black wire to common ground and the 
// yellow sensor wire to pin #2
// Adafruit invests time and resources providing this open source code, 
// please support Adafruit and open-source hardware by purchasing 
// products from Adafruit!
// Written by Limor Fried/Ladyada  for Adafruit Industries.  
// BSD license, check license.txt for more information
// All text above must be included in any redistribution

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// Arduino libraries used for serial communication and time logging include:

// Time: http://playground.arduino.cc/Code/Time
// Wire: http://playground.arduino.cc/Main/WireLibraryDetailedReference
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

// To add moisture sensor code back in, use code from:
// https://gist.github.com/naupaka/654c37ae11eb887e267b34679cbc161e
// After connecting reset pin on sensor to interrupt pin (number 2) on arduino

// Load libraries
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>
#include <Adafruit_MPL3115A2.h>
#include <Time.h>  
#include <DHT.h>
#include "RTClib.h"
#include <SD.h>

/* This driver uses the Adafruit unified sensor library (Adafruit_Sensor),
   which provides a common 'type' for sensor data and some helper functions.
   
   To use this driver you will also need to download the Adafruit_Sensor
   library and include it in your libraries folder.

   You should also assign a unique ID to this sensor for use with
   the Adafruit Sensor API so that you can identify this particular
   sensor in any data logs, etc.  To assign a unique ID, simply
   provide an appropriate value in the constructor below (12345
   is used by default in this example).
   
   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3V DC
   Connect GROUND to common ground

   I2C Address
   ===========
   The address will be different depending on whether you leave
   the ADDR pin floating (addr 0x39), or tie it to ground or vcc. 
   The default addess is 0x39, which assumes the ADDR pin is floating
   (not connected to anything).  If you set the ADDR pin high
   or low, use TSL2561_ADDR_HIGH (0x49) or TSL2561_ADDR_LOW
   (0x29) respectively.
    
   History
   =======
   2013/JAN/31  - First version (KTOWN)
*/

// Add code for RTC
///////////////////////////////
RTC_DS1307 RTC;
// char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};



// Add code for SD Data logging
///////////////////////////////
// how many milliseconds between grabbing data and logging it. 1000 ms is once a second
#define LOG_INTERVAL  1000 // mills between entries (reduce to take more/faster data)

// how many milliseconds before writing the logged data permanently to disk
// set it to the LOG_INTERVAL to write each time (safest)
// set it to 10*LOG_INTERVAL to write all data every 10 datareads, you could lose up to 
// the last 10 reads if power is lost but it uses less power and is much faster!
#define SYNC_INTERVAL 1000 // mills between calls to flush() - to write data to the card
uint32_t syncTime = 0; // time of last sync()

#define ECHO_TO_SERIAL   1 // echo data to serial port
#define WAIT_TO_START    0 // Wait for serial input in setup()

// the digital pins that connect to the LEDs
#define redLEDpin 2
#define greenLEDpin 3

// for the data logging shield, we use digital pin 10 for the SD cs line
const int chipSelect = 10;

// the logging file
File logfile;

void error(char *str)
{
  Serial.print("error: ");
  Serial.println(str);
  
  // red LED indicates error
  digitalWrite(redLEDpin, HIGH);

  while(1);
}





// Set parameters for DHT (temperature and humidity sensor) 
#define DHTPIN 2     // what pin we're connected to for DHT sensor

// Uncomment whatever type you're using for DHT sensor!
//#define DHTTYPE DHT11   // DHT 11 
#define DHTTYPE DHT22   // DHT 22  (AM2302)
//#define DHTTYPE DHT21   // DHT 21 (AM2301)

// Connect pin 1 (on the left) of the sensor to +5V
// NOTE: If using a board with 3.3V logic like an Arduino Due connect pin 1
// to 3.3V instead of 5V!
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor

// Initialize DHT sensor for normal 16mhz Arduino
DHT dht(DHTPIN, DHTTYPE);
// NOTE: For working with a faster chip, like an Arduino Due or Teensy, you
// might need to increase the threshold for cycle counts considered a 1 or 0.
// You can do this by passing a 3rd parameter for this threshold.  It's a bit
// of fiddling to find the right value, but in general the faster the CPU the
// higher the value.  The default for a 16mhz AVR is a value of 6.  For an
// Arduino Due that runs at 84mhz a value of 30 works.
// Example to initialize DHT sensor for Arduino Due:
//DHT dht(DHTPIN, DHTTYPE, 30);




// Set pin to use for reading the flow rate sensor. Can use any pin!
#define FLOWSENSORPIN 3

volatile uint16_t pulses = 0; // count how many pulses!
volatile uint8_t lastflowpinstate; // track the state of the pulse pin
volatile uint32_t lastflowratetimer = 0; // try to keep track of how long between pulses
volatile float flowrate; // and use that to calculate a flow rate

// Interrupt is called once a millisecond, looks for any pulses from the sensor!
SIGNAL(TIMER0_COMPA_vect) {
  uint8_t x = digitalRead(FLOWSENSORPIN);
  
  if (x == lastflowpinstate) {
    lastflowratetimer++;
    return; // nothing changed!
  }
  
  if (x == HIGH) {
    //low to high transition!
    pulses++;
  }
  lastflowpinstate = x;
  flowrate = 1000.0;
  flowrate /= lastflowratetimer;  // in hertz
  lastflowratetimer = 0;
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
  }
}




// Initialize TSL2561 light sensor
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);




// Initialize MPL3115A2 barometric pressure sensor
Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();




// Force Sensitive Resistor setup
// Connect one end of FSR to power, the other end to Analog 0.
// Then connect one end of a 10K resistor from Analog 0 to ground 
// For more information see www.ladyada.net/learn/sensors/fsr.html
 
int fsrPin = 0;     // the FSR and 10K pulldown are connected to a0
int fsrReading;     // the analog reading from the FSR resistor divider
int fsrVoltage;     // the analog reading converted to voltage
unsigned long fsrResistance;  // The voltage converted to resistance, can be very big so make "long"
unsigned long fsrConductance; 
long fsrForce;       // Finally, the resistance converted to force




void setup() {
  Serial.begin(9600);
  
    // use debugging LEDs
  pinMode(redLEDpin, OUTPUT);
  pinMode(greenLEDpin, OUTPUT);

  // initialize the SD card
  Serial.print("Initializing SD card...");
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(10, OUTPUT);
  
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    error("Card failed, or not present");
  }
  Serial.println("card initialized.");
  
  // create a new file
  char filename[] = "LOGGER00.CSV";
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = i/10 + '0';
    filename[7] = i%10 + '0';
    if (! SD.exists(filename)) {
      // only open a new file if it doesn't exist
      logfile = SD.open(filename, FILE_WRITE); 
      break;  // leave the loop!
    }
  }
  
  if (! logfile) {
    error("couldnt create file");
  }
  
  Serial.print("Logging to: ");
  Serial.println(filename);


  
  // Initialise the TSL2561 light sensor
  if(!tsl.begin()) {
    Serial.print("Ooops, no TSL2561 detected ... Check your wiring or I2C ADDR!");
    logfile.print("Ooops, no TSL2561 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  displaySensorDetails(); // Display some basic information on this sensor
  configureSensor(); // Setup the sensor gain and integration time
  // Serial.println("");
  // Serial.println("TSL2561 light sensor test successful");
  
  // Initialise the MPL3115A2 barometric pressure sensor
  // Serial.println("Loading MPL3115A2 library");
  
  // Initialise the water flow sensor
  // Serial.println("DHTxx test!");
  dht.begin();
  
  // Initialise the water flow sensor
  pinMode(FLOWSENSORPIN, INPUT);
  digitalWrite(FLOWSENSORPIN, HIGH);
  lastflowpinstate = digitalRead(FLOWSENSORPIN);
  useInterrupt(true);
  // Serial.println("Flow sensor test complete!");

  // Start RTC clock
  RTC.begin();
  
  // You only ever have to run this once, to set the RTC
  // Should only ever uncomment if RTC battery changes etc
  // It sets the RTC date and time to the date and time of
  // the computer on which it was compiled
  // RTC.adjust(DateTime(__DATE__, __TIME__));
  
  Serial.println("YYYY-MM-DD,HH:MM:SS,MPL3115A2_barometric_pressure_in_pascals,MPL3115A2_temp_in_deg_C,TSL2561_light_in_lux,DHT_humidity_pcnt,DHT_temp_in_deg_C,liquid_flow_frequency,liquid_flow_pulses_cumulative_count,liquid_flow_cumulative_liters,FSR_Analog_reading,FSR_voltage_in_mV,FSR_resistance_in_ohms,FSR_conductance_in_microMhos,FSR_force_in_Newtons");
  logfile.println("YYYY-MM-DD,HH:MM:SS,MPL3115A2_barometric_pressure_in_pascals,MPL3115A2_temp_in_deg_C,TSL2561_light_in_lux,DHT_humidity_pcnt,DHT_temp_in_deg_C,liquid_flow_frequency,liquid_flow_pulses_cumulative_count,liquid_flow_cumulative_liters,FSR_Analog_reading,FSR_voltage_in_mV,FSR_resistance_in_ohms,FSR_conductance_in_microMhos,FSR_force_in_Newtons");
}



void loop() { 

  // Print time from RTC
  DateTime now = RTC.now();
    
  // delay for the amount of time we want between readings
  // delay((LOG_INTERVAL -1) - (millis() % LOG_INTERVAL));
  
  digitalWrite(greenLEDpin, HIGH);  
    
  Serial.print(now.year(), DEC);
  logfile.print(now.year(), DEC);
  Serial.print('-');
  logfile.print('-');
  printDigits(now.month());
  Serial.print('-');
  logfile.print('-');
  printDigits(now.day());
  //Serial.print(" (");
  //Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
  //Serial.print(") ");
  Serial.print(',');
  logfile.print(',');
  printDigits(now.hour());
  Serial.print(':');
  logfile.print(':');
  printDigits(now.minute());
  Serial.print(':');
  logfile.print(':');
  printDigits(now.second());
  Serial.print(',');
  logfile.print(',');
  
  
  //////////////////// Barometric pressue ////////////////////
  // Make sure MPL3115A2 is initialized
  if (! baro.begin()) {
    Serial.println("Couldnt find MPL3115A2 barometric pressure sensor");
    logfile.println("Couldnt find MPL3115A2 barometric pressure sensor");
    return;
  }
  
  float pascals = baro.getPressure();
  // Serial.print(pascals/3377); Serial.println(" Inches (Hg)");
  Serial.print(pascals); Serial.print(",");
  logfile.print(pascals); logfile.print(",");

  // float altm = baro.getAltitude();
  // Serial.print(altm); Serial.println(" meters");

  float tempC = baro.getTemperature();
  Serial.print(tempC); Serial.print(",");// Serial.println("*C");
  logfile.print(tempC); logfile.print(",");// logfile.println("*C");


  //////////////////// Light sensor ////////////////////
  // Get a new TSL2561 light sensor event
  sensors_event_t event;
  tsl.getEvent(&event);
 
  // Display the results (light is measured in lux)
  if (event.light) {
    Serial.print(event.light); Serial.print(","); // Serial.println(" lux");
    logfile.print(event.light); logfile.print(","); // logfile.println(" lux");
  }
  else {
    // If event.light = 0 lux the sensor is probably saturated
    // and no reliable data could be generated!
    // Serial.print("Sensor overload"); Serial.print(",");
    
    // Added this back in since low light is more likely than saturation in this case
    Serial.print(event.light); Serial.print(","); // Serial.println(" lux");
    logfile.print(event.light); logfile.print(","); // logfile.println(" lux");
  }
  
  
  //////////////////// Temperature and Humidity ////////////////////
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  float t = dht.readTemperature();  // Read temperature as Celsius
  float f = dht.readTemperature(true); // Read temperature as Fahrenheit
  
  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println("Failed to read from DHT sensor!");
    logfile.println("Failed to read from DHT sensor!");
    return;
  }

  // Compute heat index
  // Must send in temp in Fahrenheit!
  // float hi = dht.computeHeatIndex(f, h);

  // Serial.print("Humidity: "); 
  Serial.print(h); Serial.print(",");
  logfile.print(h); logfile.print(",");
  // Serial.print(" %\t");
  // Serial.print("Temperature: "); 
  Serial.print(t); Serial.print(",");
  logfile.print(t); logfile.print(",");
  // Serial.print(" *C ");
  // Serial.print(f);
  // Serial.print(" *F\t");
  // Serial.print("Heat index: ");
  // Serial.print(hi);
  // Serial.println(" *F");
  
  
   //////////////////// Water flow rate  ////////////////////
  // Serial.print("Freq: "); 
  Serial.print(flowrate); Serial.print(",");
  logfile.print(flowrate); logfile.print(",");
  // Serial.print("Pulses: "); 
  Serial.print(pulses, DEC); Serial.print(",");
  logfile.print(pulses, DEC); logfile.print(",");
  
  // if a plastic sensor use the following calculation
  // Sensor Frequency (Hz) = 7.5 * Q (Liters/min)
  // Liters = Q * time elapsed (seconds) / 60 (seconds/minute)
  // Liters = (Frequency (Pulses/second) / 7.5) * time elapsed (seconds) / 60
  // Liters = Pulses / (7.5 * 60)
  float liters = pulses;
  liters /= 7.5;
  liters /= 60.0;

/*
  // if a brass sensor use the following calculation
  float liters = pulses;
  liters /= 8.1;
  liters -= 6;
  liters /= 60.0;
*/
  Serial.print(liters); Serial.print(",");
  logfile.print(liters); logfile.print(",");
  // Serial.println(" Liters");

  
   //////////////////// Force sensitive resistor  ////////////////////
  fsrReading = analogRead(fsrPin);  
  // Serial.print("Analog reading = ");
  Serial.print(fsrReading); Serial.print(",");
  logfile.print(fsrReading); logfile.print(",");
 
  // analog voltage reading ranges from about 0 to 1023 which maps to 0V to 5V (= 5000mV)
  fsrVoltage = map(fsrReading, 0, 1023, 0, 5000);
  // Serial.print("Voltage reading in mV = ");
  Serial.print(fsrVoltage); Serial.print(",");
  logfile.print(fsrVoltage); logfile.print(",");
 
  if (fsrVoltage == 0) {
    Serial.print("No_pressure,NA,NA,");  
    logfile.print("No_pressure,NA,NA,");  
  } else {
    // The voltage = Vcc * R / (R + FSR) where R = 10K and Vcc = 5V
    // so FSR = ((Vcc - V) * R) / V        yay math!
    fsrResistance = 5000 - fsrVoltage;     // fsrVoltage is in millivolts so 5V = 5000mV
    fsrResistance *= 10000;                // 10K resistor
    fsrResistance /= fsrVoltage;
    // Serial.print("FSR resistance in ohms = ");
    Serial.print(fsrResistance); Serial.print(",");
    logfile.print(fsrResistance); logfile.print(",");
 
    fsrConductance = 1000000;           // we measure in micromhos so 
    fsrConductance /= fsrResistance;
    // Serial.print("Conductance in microMhos: ");
    Serial.print(fsrConductance); Serial.print(",");
    logfile.print(fsrConductance); logfile.print(",");
 
    // Use the two FSR guide graphs to approximate the force
    if (fsrConductance <= 1000) {
      fsrForce = fsrConductance / 80;
      // Serial.print("Force in Newtons: ");
      Serial.println(fsrForce);     
      logfile.println(fsrForce);     
    } else {
      fsrForce = fsrConductance - 1000;
      fsrForce /= 30;
      // Serial.print("Force in Newtons: ");
      Serial.println(fsrForce);
      logfile.println(fsrForce);
    }
  }
  // Serial.println("--------------------");

  digitalWrite(greenLEDpin, LOW);

  // Now we write data to disk! Don't sync too often - requires 2048 bytes of I/O to SD card
  // which uses a bunch of power and takes time
  // if ((millis() - syncTime) < SYNC_INTERVAL) return;
  // syncTime = millis();
  
  // blink LED to show we are syncing data to the card & updating FAT!
  digitalWrite(redLEDpin, HIGH);
  
  // logfile.println("This is a test");

  logfile.flush();
  digitalWrite(redLEDpin, LOW);
  
   //////////////////// Pause between sets of measurements  ////////////////////
  // measure once per minute
  delay(60000);

}


/*********************************************************************************/
// HELPER FUNCTIONS
/*********************************************************************************/

//////////////////// TSL2561 functions  ////////////////////
// Displays some basic information on the TSL2561 sensor from the unified
// sensor API sensor_t type (see Adafruit_Sensor for more information)
void displaySensorDetails(void)
{
  sensor_t sensor;
  tsl.getSensor(&sensor);
  /*
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" lux");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" lux");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" lux");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
  */
}

// Configures the gain and integration time for the TSL2561
void configureSensor(void)
{
  /* You can also manually set the gain or enable auto-gain support */
  tsl.setGain(TSL2561_GAIN_1X);      /* No gain ... use in bright light to avoid sensor saturation */
  // tsl.setGain(TSL2561_GAIN_16X);     /* 16x gain ... use in low light to boost sensitivity */
  // tsl.enableAutoRange(true);            /* Auto-gain ... switches automatically between 1x and 16x */
  
  /* Changing the integration time gives you better sensor resolution (402ms = 16-bit data) */
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);      /* fast but low resolution */
  // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);  /* medium resolution and speed   */
  // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);  /* 16-bit data but slowest conversions */

  /* Update these values depending on what you've set above! */  
  // Serial.println("------------------------------------");
  // Serial.print  ("Gain:         "); Serial.println("Auto");
  // Serial.print  ("Timing:       "); Serial.println("13 ms");
  // Serial.println("------------------------------------");
}


//////////////////// time functions  ////////////////////
void digitalClockDisplay(){
  // digital clock display of the time
  Serial.print(year());
  Serial.print("-");
  printDigits(month());
  Serial.print("-");
  printDigits(day());
  Serial.print(",");
  printDigits(hour());
  Serial.print(":");
  printDigits(minute());
  Serial.print(":");
  printDigits(second());
  // Serial.println(); 
}

void printDigits(int digits){
  // utility function for digital clock display: prints preceding colon and leading 0
  // Serial.print(":");
  if(digits < 10)
    Serial.print('0');
    logfile.print('0');
  Serial.print(digits);
  logfile.print(digits);
}



