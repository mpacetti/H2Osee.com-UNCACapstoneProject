/**
  ArduMegaSensorDriver.ino 
    
  Arduino Sensor Readings Driver for H20see.com Project

  Developer:  Mark Pacetti
  Date:       09/18/19

  Collects current readings from attached analog/digital sensors,
    converts integer and float values into a bytearray payload buffer,
    transmits payload buffer over LoRa packet radio to RasPi Edge Gateway and 
    then sleeps for defined amount of time
  
  Sensors Attached:
  
    1.  NEO-6M GPS        UTC Date, UTC Time, Longitude, Latitude, Altitude, Satelites Locked
    2.  BME280            Relative Humidity & Barometric Pressure
    3.  UV                UV Index
    4.  WaterTemp0ft      Water Temp @ 0 ft.
    5.  WaterTemp2ft      Water Temp @ 2 ft.
    6.  surfaceTemp       Air Temp @ 2 ft.
    7.  groundTemp        Ground Temp @ 1 ft.
    8.  Turbidity Sensor  Water Turbidity at 0ft.
    9.  TDS Meter 1.0     Water TDS @ 2ft.
    10. PH Meter          Water PH
    11. Rain              Inches of Rain per Hour/Day

    UPGRADED:  10/19/19 to Arduino Mega 2560 board due to memory constraints on UNO board
    BN: Arduino/Genuino Mega or Mega 2560
    VID: 0x2341
    PID: 0x0042
    SN: 95736323632351C032C0

*/

// ###############    INCLUDED LIBRARIES    ###############

#include <SPI.h>                          // SPI library
#include <OneWire.h>                      // OneWire library
#include <TinyGPS++.h>                    // TinyGPSPlus library to control and query GPS NMEA
#include <Time.h>                         // Time Library
#include <math.h>                         // Math library
#include <Adafruit_Sensor.h>              // Adafruit generic sensor library
#include <RTClib.h>                       // Adafruit Real Time Clock library
#include <Adafruit_BME280.h>              // BME280 sensor library
#include <DallasTemperature.h>            // Dallas temperature control library
#include <RH_RF95.h>                      // RadioHead RFM95 LoRa library
#include <GravityTDS.h>                   // DFRobot Gravity TDS library


// ###############    HARDWARE PIN DEFINITIONS    ###############

// =================   A N A L O G   P I N S  =================

// analog pin for interfacing UV sensor
static const int UV_SENSOR_PIN      = A0;
// analog pin for interfacing Turbidity Sensor       
static const int TURBID_SENSOR_PIN  = A1;
// analog pin for interfacing PH Sensor
static const int PH_SENSOR_PIN      = A2;
// analog pin for interfacing TDS Meter
static const int TDS_METER_PIN      = A3;


// =================   D I G I T A L   P I N S  =================

const int RFM_RST_PIN = 2;                // LoRa transciever reset pin
const int RFM_INT_PIN = 3;                // LoRa transciever interupt pin
const int RFM_CS_PIN = 4;                 // LoRa transciever synchronous SPI bus - Chip Select pin

static const int ONE_WIRE_BUS_PIN = 7;    // OneWire Bus pin

static const int TX_PIN = 16;             // digital transmit (Tx) pin for Serial2 UART connected to GPS module
static const int RX_PIN = 17;             // digital receive (Rx) pin for Serial2 UART connected to GPS module

static const int RAIN_SENSOR_PIN = 22;    // digital pin interfacing Rain Gauge tipping bucket

static const int SPI_MISO = 50;           // synchronous SPI bus - Master In / Slave Out pin
static const int SPI_MOSI = 51;           // synchronous SPI bus - Master Out / Slave In pin
static const int SPI_SCK = 52;            // synchronous SPI bus - Serial Clock pin
static const int SPI_CS = 53;             // synchronous SPI bus - Chip Select pin


// ###############    GLOBAL VARIABLES AND DEFINES    ###############

// set DEBUG mode for testing (1=verbose, 0=off)
#define DEBUG 1

// reference voltage(V) of the Arduino Mega2560 ADC
#define VREF 5.0

// 10-bit analog to digital converter resolution
#define ADCRES 1024.0

// number of minutes to sleep() between samples     
static const int SLEEP_MINUTES = 2;            

// GPS delay time in milliseconds
static const int GPS_DELAY_MILIS = 2000;

// baud rate setting for console
static const uint32_t TERM_BAUD = 9600;  

// Serial baud rate setting for GPS module
static const uint32_t GPS_BAUD = 9600;    

// this node's client address
static const uint8_t LORA_CLIENT_ADDR = 0x01;

// LoRa receiving server address
static const uint8_t LORA_SERVER_ADDR = 0x07;    

// frequency to use for RFM95W radio           
static const float RFM_FREQ = 915.0;  
     
// radio transmit power to use (5..23 dBm), 13 dBm is factory default      
static const int RFM_TX_POWER = 13;              

// which position tipping bucket is in
static bool BUCKET_POS1 = false;

// amount of rain bucket holds
const double BUCKET_VOL = 0.01610595;

// accumulate rain for each day
static float DAILY_RAIN = 0.0;

// accumulate rain for each hour
static float HOUR_RAIN = 0.0;

// accumulated amount of total rain prior to past hour
static float PRIOR_HOURS_RAIN = 0.0;

// to get reading at first hour
static bool FIRST = false;


// -------- global variables to build LoRa transmit buffer --------

// LoRa transmit buffer size
static const int TX_BUF_SIZE = 36; 

// LoRa transmit buffer
static uint8_t tx_buffer[TX_BUF_SIZE];

// LoRa transmit buffer global variables to stuff
static float humidity;
static float pressure;
static float pressure_inhg;
static float waterTemp0ftC, waterTemp2ftC, airTempC, groundTempC;
static float waterTemp0ftF, waterTemp2ftF, airTempF, groundTempF;
static float altitude_ft;
static float uv_index;
static float turbidity_ntu;
static float tds;
static float ph;
static float rain_in;
static float latitude;
static float longitude;
static int sats_tracked;
static int date_yyyy;
static int date_mm;
static int date_dd;
static int time_hh;
static int time_mm;
static int time_ss;


// -------- OneWire Hardware Addresses of 2 x DS18B20 Temperature Sensors --------

// temperature sensor at water surface (0 feet)
uint8_t waterTemp0ft[8] = { 0x28, 0xAA, 0x4D, 0x52, 0x4C, 0x14, 0x01, 0x54 };

// temperature sensor below water surface (2 feet)
uint8_t waterTemp2ft[8] = { 0x28, 0xAA, 0xDE, 0x50, 0x4C, 0x14, 0x01, 0xAA };

// temperature sensor above water surface (2 feet)
uint8_t airTemp[8] = { 0x28, 0xAA, 0x4A, 0x70, 0x51, 0x14, 0x01, 0x28 };

// temperature sensor below ground surface (1 foot)
uint8_t groundTemp[8] = { 0x28, 0xAA, 0xE7, 0x49, 0x51, 0x14, 0x01, 0x2C };


// var to track hardware sensors discovered on OneWire bus
int deviceCount = 0;


// ###############    INSTANTIATE GLOBAL OBJECTS     ###############

// instantiate BME280 object for humidity and barometer pressure
Adafruit_BME280 bme(SPI_CS);

// instantiate OneWire object to communicate with all attached OneWire sensors
OneWire oneWire(ONE_WIRE_BUS_PIN);

// pass OneWire reference and instantiate Dallas temperature object
DallasTemperature sensors(&oneWire);

// instantiate TinyGPS+ object to parse NMEA sentences
TinyGPSPlus gps;

// instantiate radio object over SPI bus
RH_RF95 rfm(RFM_CS_PIN, RFM_INT_PIN);

// instantiate software RTC time
RTC_Millis rtc;


// ###############  <<<   SETUP() CODE RUNS ONCE ON Arduino POWER UP  >>>   ###############

void setup() {

  // ========  POWER MANAGEMENT OPTIMIZATION TO TRY AND CONSERVE BATTERY POWER  =========
  // turn off onboard LED (pin 13) to save ~3mA
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // convert unused analog pins to digital to save ~5mA 
  pinMode(A4, OUTPUT);
  pinMode(A5, OUTPUT);
  pinMode(A6, OUTPUT);
  pinMode(A7, OUTPUT);
  pinMode(A8, OUTPUT);
  pinMode(A9, OUTPUT);
  pinMode(A10, OUTPUT);
  pinMode(A11, OUTPUT);
  pinMode(A12, OUTPUT);
  pinMode(A13, OUTPUT);
  pinMode(A14, OUTPUT);
  pinMode(A15, OUTPUT);
  
  // write unused analog pins LOW to save ~5mA 
  digitalWrite(A4, LOW);
  digitalWrite(A5, LOW);
  digitalWrite(A6, LOW);
  digitalWrite(A7, LOW);
  digitalWrite(A8, LOW);
  digitalWrite(A9, LOW);
  digitalWrite(A10, LOW);
  digitalWrite(A11, LOW);
  digitalWrite(A12, LOW);
  digitalWrite(A13, LOW);
  digitalWrite(A14, LOW);
  digitalWrite(A15, LOW);
  
  // set the remaining unused digital pins as output and write them low to save ~9mA 
  for (int i = 8; i <= 13; i++) {
    pinMode(i, OUTPUT);
    digitalWrite(i, LOW);
  }

  for (int i = 23; i <= 49; i++) {
    pinMode(i, OUTPUT);
    digitalWrite(i, LOW);
  }

  // *** jumpered reset and ground pins of USB ICSP header together 
  // which puts the USB chip into reset mode to save ~15mA
  
  // initialize serial console
  Serial.begin(TERM_BAUD);
  // wait until serial console is available
  while(!Serial);    

  // start the software real-time clock
  rtc.begin(DateTime(__DATE__, __TIME__));

  // initialize NEO-6M GPS module
  initGPS();

  // initialize BME280 sensor module
  initBME280();

  // initialze UV sensor module
  initUV();

  // initialize DS18B20 temperature sensors on OneWire bus
  initDS18B20();

  // initialize Turbidity sensor 
  initTurbidity();

  // initialize TDS sensor
  initTDS();

  // initialize pH meter
  initPH();

  // initialize rain gauge
  initRain();
  
  // initialize LoRa Packet Radio 
  initLoRaRadio();
}


// ###############   <<<   MAIN LOOPS FOREVER   >>>   ###############
void loop() {

  #if DEBUG
    Serial.println();
    Serial.println("-----------------------------------------");
  #endif
  
  delayForGPS(GPS_DELAY_MILIS);

  getGPSvalues();

  getBME280values();

  getUVIndex();

  getDS18B20values();

  getTurbidityValues();

  getTDSvalues();

  getPHvalues();

  getRainInches();

  txLoRaBuffer();

  goToSleepMins(SLEEP_MINUTES);
  #if DEBUG
    Serial.println("-----------------------------------------");
  #endif
}


static void initGPS() {

  // initialize GPS module on hardware UART Serial2
  Serial2.begin(GPS_BAUD);    
  #if DEBUG 
    Serial.println("GPS module successfully initialized...");
  #endif
}


static void initBME280() {

  // initialize BME280 module over SPI bus
  if (!bme.begin()) {
    #if DEBUG
      Serial.println("could not connect to BME280 sensors over SPI bus...");
      Serial.print("sensor ID was: 0x"); 
      Serial.println(bme.sensorID(), 16);
      while (1); 
    #endif
  }
  #if DEBUG
    Serial.println("BME280 sensors successfully initialized...");
  #endif
}


static void initUV() {

  pinMode(UV_SENSOR_PIN, INPUT);
  #if DEBUG
    Serial.println("UV sensor successfully initialized...");
  #endif
}


static void initDS18B20() {
  
  // initialize OneWire bus and detect connected temperature sensors
  sensors.begin();
  // locate temperature sensors on OneWire bus
  #if DEBUG
    Serial.println("locating OneWire sensors...");
    Serial.print("found ");
    deviceCount = sensors.getDeviceCount();
    Serial.print(deviceCount, DEC);
    Serial.println(" OneWire temperature sensors...");
    Serial.println("DS18B20 temperature sensors successfully initialized...");
  #endif
}


static void initTurbidity() {

  pinMode(TURBID_SENSOR_PIN, INPUT);
  #if DEBUG
    Serial.println("analog turbidity sensor successfully initialized...");
  #endif
}


static void initTDS() {

  pinMode(TDS_METER_PIN, INPUT);
  #if DEBUG
    Serial.println("analog TDS sensor successfully initialized...");
  #endif
}


static void initPH() {

  pinMode(PH_SENSOR_PIN, INPUT);
  #if DEBUG
    Serial.println("analog pH sensor successfully initialized...");
  #endif
}


static void initRain() {

  pinMode(RAIN_SENSOR_PIN, INPUT);
  #if DEBUG
    Serial.println("rain tipping bucket gauge successfully initialized...");
  #endif
}


static void initLoRaRadio() {

  // manually reset RFM95W LoRa transceiver
  pinMode(RFM_RST_PIN, OUTPUT);
  digitalWrite(RFM_RST_PIN, HIGH);
  delay(100);
  digitalWrite(RFM_RST_PIN, LOW);
  delay(10);
  digitalWrite(RFM_RST_PIN, HIGH);
  delay(10);

  // Initialize RFM95W radio
  if ( !rfm.init() ) {
    #if DEBUG
        Serial.println("could not initialize RFM95 radio...");
    #endif
    while (1);
  }

  #if DEBUG
    Serial.println("RFM95 radio successfully initialized...");
    delay(100);
  #endif

  // Set RFM95W frequency
  if ( !rfm.setFrequency(RFM_FREQ) ) {
    #if DEBUG
      Serial.println("could not configure frequency on RFM95 transceiver...");
    #endif
    while (1);
  }

  #if DEBUG
    Serial.print("RFM95 radio frequency set to: ");
    Serial.print(RFM_FREQ);
    Serial.println(" MHz");
    delay(100);
  #endif

  // Set RFM95W transmit power from PA_BOOST pin
  rfm.setTxPower(RFM_TX_POWER, false);
  #if DEBUG
    Serial.print("RFM95 radio transmit power set to: ");
    Serial.println(RFM_TX_POWER);
  #endif
  
}


static void delayForGPS(unsigned long ms) {

  // delay() to ensure GPS object values have time to propagate
  unsigned long start = millis();
  
  do {
    while (Serial2.available())
      gps.encode(Serial2.read());
  } while (millis() - start < ms);
  
}


static void getGPSvalues() {

  // get current number of satellites GPS is tracking
  sats_tracked = gps.satellites.value();
  
  // get current latitude from GPS
  latitude = gps.location.lat();
  
  // get current longitude from GPS
  longitude = gps.location.lng();

  // get month (1-12) (u8) from GPS
  date_mm = gps.date.month();
  
  // get day (1-31) (u8) from GPS
  date_dd = gps.date.day();
  
  // get year (2000+) (u16) from GPS
  date_yyyy = gps.date.year();

  // get hour (0-23) (u8) from GPS
  time_hh = gps.time.hour();

  // get minute (0-59) (u8) from GPS
  time_mm = gps.time.minute();

  // get second (0-59) (u8) from GPS
  time_ss = gps.time.second();

  // get altitude in feet (double) from GPS
  altitude_ft = gps.altitude.feet();   

  #if DEBUG
    // print LAT/LON to console
    Serial.print("Number of Satelites GPS is Tracking:  "); 
    Serial.println(sats_tracked);
    Serial.print("  Lat, Lng : "); 
    Serial.print(latitude, 4);
    Serial.print(", "); 
    Serial.println(longitude, 4);
  
    // print DATE to console
    Serial.print("  UTC Date : ");
    Serial.print(date_mm);
    Serial.print("/");
    Serial.print(date_dd);
    Serial.print("/");
    Serial.println(date_yyyy);
  
    // print TIME to console
    Serial.print("  UTC Time : ");
    Serial.print(time_hh);
    Serial.print(":");
    Serial.print(time_mm);
    Serial.print(":");
    Serial.println(time_ss);  
  
    // print ALTITUDE to console
    Serial.print("  Altitude : ");
    Serial.print(altitude_ft, 1);
    Serial.print(" ft."); 
    Serial.println();
  #endif
}


static void getBME280values() {

  // set BME280 sensor to forced mode to get higher accuracy readings
  bme.setSampling(Adafruit_BME280::MODE_FORCED,
                  Adafruit_BME280::SAMPLING_X1,     // ambient temperature reading
                  Adafruit_BME280::SAMPLING_X1,     // barometric pressure reading
                  Adafruit_BME280::SAMPLING_X1,     // relative humidity reading
                  Adafruit_BME280::FILTER_OFF   );
                    
  // read BME280 Sensor Values
  float tempC = bme.readTemperature();
  // convert to farenheight
  float tempF = 1.8 * tempC + 32;

  // humidity returned from BME280 in %
  humidity = bme.readHumidity();
  
  // pressure returned from BME280 in hPa
  pressure = bme.readPressure() / 100.0F;

  // convert presssure hPa into inHg (1 hPa to inHg = 0.02953 inhg)
  pressure_inhg = (pressure * 0.02953) + 3.0;

  // display values to console
  #if DEBUG
    Serial.println();
    Serial.print("ambient Temperature:   ");
    Serial.print(tempF, 1);
    Serial.println(" F");
    Serial.print("relative Humidity:     ");
    Serial.print(humidity, 1);
    Serial.println(" %");
    Serial.print("barometric Pressure:   ");
    Serial.print(pressure_inhg, 1);
    Serial.println(" inHg");
  #endif
}


static void getUVIndex() {

  int   UvSamples = 100;
  int   UvIndex = 0;
  float UvSamplesSum = 0;
  float UvSensorVoltage = 0;
  float UvSensorValue = 0;
  float UvSensorValueAvg = 0;
  float UVmV = 0;

  // collect the UV sensor samples and average
  for (int i = 0; i < UvSamples; i++) {
    
    UvSensorValue = analogRead(UV_SENSOR_PIN);
    UvSamplesSum = UvSamplesSum + UvSensorValue;
    delay(5);
  }

  UvSensorValueAvg = UvSamplesSum / UvSamples;
  UVmV = UvSensorValueAvg * VREF;
  UvSensorVoltage = UVmV / ADCRES;
  // calcule the UV index value
  uv_index = UvSensorVoltage / 0.1;

  #if DEBUG
    Serial.print("UV Index:               ");
    Serial.println(UvIndex, 1);
  #endif
}


static void getDS18B20values() {

  float tempC;
  float tempF;

  // query ALL DS18B20 sensors for current temperature readings
  sensors.requestTemperatures();
  
  // sample WATER temperature in C from 0ft sensor
  waterTemp0ftC = sensors.getTempC(waterTemp0ft);
  // convert to Farenheit
  waterTemp0ftF = (1.8 * waterTemp0ftC + 32);

  #if DEBUG
    Serial.print("H2O Temp @ 0ft :       ");
    Serial.print(waterTemp0ftF, 1);
    Serial.println(" F");
  #endif

  // sample WATER temperature in C from 2ft sensor
  waterTemp2ftC = sensors.getTempC(waterTemp2ft);
  // convert to Farenheit
  waterTemp2ftF = (1.8 * waterTemp2ftC + 32);

  #if DEBUG
    Serial.print("H2O Temp @ 2ft :       ");
    Serial.print(waterTemp2ftF, 1);
    Serial.println(" F");
  #endif

  // sample AIR temperature in C from 2ft sensor
  airTempC = sensors.getTempC(airTemp);
  // convert to Farenheit
  airTempF = (1.8 * airTempC + 32);

  #if DEBUG
    Serial.print("AIR Temp @ 2ft :       ");
    Serial.print(airTempF, 1);
    Serial.println(" F");
  #endif
  
  // sample GROUND temperature in C from 1ft sensor
  groundTempC = sensors.getTempC(groundTemp);
  // convert to Farenheit
  groundTempF = (1.8 * groundTempC + 32);

  #if DEBUG
    Serial.print("GROUND Temp @ 1ft :    ");
    Serial.print(groundTempF, 1);
    Serial.println(" F");
  #endif
}


static void getTurbidityValues() {

  int   turbiditySamples = 500;
  float turbiditySamplesSum = 0;
  float NTUvalue = 0;
  float turbidityVoltage = 0;
  
  turbidityVoltage = analogRead(TURBID_SENSOR_PIN) / ADCRES * VREF;

  #if DEBUG
    Serial.print("Turbidity Voltage :     ");
    Serial.print(turbidityVoltage, 2);
    Serial.println("V");
  #endif
  
  // get Nephelometric Turbidity Unit (NTU) [Total Suspended Solids]
  if (turbidityVoltage < 2.5) {
    // if voltage read is less than 2.5V set to maximum NTU of 3000
    NTUvalue = 3000;  
  } else {
    // convert turbidity voltage to NTU value
    // NTU formula from manufactuerer:  [y = -1120.4x^2 + 5742.3x -4352.9]
    NTUvalue = ((-1120.4 * square(turbidityVoltage) + (5742.3 * turbidityVoltage) - 4000.0)); 
  }

  // if the NTU turns out to be under 0 set to ZERO otherwise set to actual NTU
  if (NTUvalue < 0 ) {
      turbidity_ntu = 0;    
  } else {
      turbidity_ntu = NTUvalue;
  }

  #if DEBUG
    Serial.print("Turbidity NTU :      ");
    Serial.println(NTUvalue, 2);
  #endif
}


static void getTDSvalues() {

  int   COUNT = 50;
  int   analogBuffer[COUNT];    
  int   analogBufferTemp[COUNT];
  int   analogBufferIndex = 0;
  int   copyIndex = 0;
  float averageVoltage = 0;
  float tdsValue = 0;

  for (int i = 0; i < 10; i++) {
       
    static unsigned long analogSampleTimepoint = millis();
    // read analog value from the ADC every 40 milliseconds
    if (millis() - analogSampleTimepoint > 40U) {
       
      analogSampleTimepoint = millis();
      //read the analog value and store into the buffer
      analogBuffer[analogBufferIndex] = analogRead(TDS_METER_PIN); 
      // increment buffer index
      analogBufferIndex = analogBufferIndex + 1;
      
      if (analogBufferIndex == COUNT) {
         analogBufferIndex = 0;
      }
    }   
   
   static unsigned long printTimepoint = millis();
   
   if (millis() - printTimepoint > 800U) {
       
      printTimepoint = millis();
      
      for (copyIndex = 0; copyIndex < COUNT; copyIndex++) {
          analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
      }

      // read stable analog value by the median filter and convert into a voltage value  
      averageVoltage = getMedianNum(analogBufferTemp, COUNT) * (float)VREF / ADCRES; 

      // temperature compensation formula for TDS sensor
      float compensationCoefficient = 1.0 + 0.02 * (waterTemp0ftC - 25.0);    
      // temperature compensation using water temperature to compensate
      float compensationVolatge = averageVoltage / compensationCoefficient;
      //convert voltage value to TDS value
      tds = (133.42 * compensationVolatge * compensationVolatge * compensationVolatge - 255.86 *compensationVolatge * compensationVolatge + 857.39 * compensationVolatge) * 0.5; 
    }  
    
    delay(500);
  }
  #if DEBUG
    Serial.print("TDS Voltage :           ");
    Serial.print(averageVoltage, 2);
    Serial.println("V   ");
    
    Serial.print("TDS Value :          ");
    Serial.print(tdsValue, 0);
    Serial.println("ppm");
  #endif
}


static void getPHvalues() {

  // store average value of the sensor reading for most accurate result
  int   SAMPLES = 50;
  int sample_array[SAMPLES];

  unsigned long int sumVoltage;
  unsigned long int avgVoltage;  
  
  float phVoltage;
  float offset = 0.00;

  // get COUNT samples from pH sensor and store into sample_array
  for (int i = 0; i < SAMPLES; i++) {   
    sample_array[i] = analogRead(PH_SENSOR_PIN);
    delay(10);
  }
 
  // get average voltage read from pH sensor
  sumVoltage = 0;
  avgVoltage = 0;
  for (int i = 0; i < SAMPLES; i++) {                 
    sumVoltage = sumVoltage + sample_array[i];
  }
  avgVoltage = sumVoltage / SAMPLES;

  // convert analog readings into milliVolts
  phVoltage = (float)avgVoltage * VREF / ADCRES; 
  
  //convert milliVolts into pH value 
  ph = 3.5 * phVoltage + offset;                 
  
  #if DEBUG
    Serial.print("pH voltage:             ");  
    Serial.print(phVoltage, 2);
    Serial.println();
    Serial.print("pH value:              ");  
    Serial.print(ph, 2);
    Serial.println();
  #endif
}


static void getRainInches() {
  
  // get rain inches from tipping bucket
  
}


static void txLoRaBuffer() {

  // multiply sampled readings by 10 or 100 and cast to in so floats can be transmitted over LoRa packet radio
  int16_t airTempF_int = (int16_t)round(airTempF * 10.0);
  int16_t humidity_int = (int16_t)round(humidity * 10.0);
  int16_t pressure_inhg_int = (int16_t)round(pressure_inhg * 10.0); 
  int16_t groundTempF_int = (int16_t)round(groundTempF * 10.0);
  int16_t waterTemp0ftF_int = (int16_t)round(waterTemp0ftF * 10.0);
  int16_t waterTemp2ftF_int = (int16_t)round(waterTemp2ftF * 10.0);
  int16_t altitude_ft_int = (int16_t)round(altitude_ft * 10.0);
  int16_t uv_index_int = (int16_t)uv_index;
  int16_t turbidity_ntu_int = (int16_t)round(turbidity_ntu * 10.0);
  int16_t tds_int = (int16_t)round(tds * 10.0);
  int16_t ph_int = (int16_t)round(ph * 10.0); 
  uint8_t sats_tracked_int = sats_tracked;
  int16_t latitude_int = (int16_t)round(latitude * 100.0);
  int16_t longitude_int = (int16_t)round(longitude * 100.0);
  int16_t date_year_int = date_yyyy;
  uint8_t date_month_int = date_mm;
  uint8_t date_day_int = date_dd;
  uint8_t time_hour_int = time_hh;
  uint8_t time_minutes_int = time_mm;
  uint8_t time_seconds_int = time_ss;
   
  // LoRa transmit buffer setup for transmission
  tx_buffer[0] =  LORA_CLIENT_ADDR;                        // FROM client address       [1 byte]
  tx_buffer[1] =  LORA_SERVER_ADDR;                        // TO server address         [1 byte] 
  tx_buffer[2] =  (0xff & airTempF_int);                   // air temperature           [2 bytes] little-endian
  tx_buffer[3] =  (0xff & (airTempF_int >> 8));
  tx_buffer[4] =  (0xff & humidity_int);                   // humidity                  [2 bytes] little-endian
  tx_buffer[5] =  (0xff & (humidity_int >> 8));
  tx_buffer[6] =  (0xff & pressure_inhg_int);              // pressure                  [2 bytes] little-endian
  tx_buffer[7] =  (0xff & (pressure_inhg_int >> 8));
  tx_buffer[8] =  (0xff & groundTempF_int);                // ground temperature        [2 bytes] little-endian
  tx_buffer[9] =  (0xff & (groundTempF_int >> 8));
  tx_buffer[10] = (0xff & waterTemp0ftF_int);             // water temperature 0ft      [2 bytes] little-endian
  tx_buffer[11] = (0xff & (waterTemp0ftF_int >> 8));
  tx_buffer[12] = (0xff & waterTemp2ftF_int);             // water temperature 2ft      [2 bytes] little-endian
  tx_buffer[13] = (0xff & (waterTemp2ftF_int >> 8));
  tx_buffer[14] = (0xff & altitude_ft_int);               // altitude in feet           [2 bytes] little-endian
  tx_buffer[15] = (0xff & (altitude_ft_int >> 8));
  tx_buffer[16] = (0xff & uv_index_int);                  // uv index                   [2 bytes] little-endian
  tx_buffer[17] = (0xff & (uv_index_int >> 8));
  tx_buffer[18] = (0xff & turbidity_ntu_int);             // turbidity                  [2 bytes] little-endian
  tx_buffer[19] = (0xff & (turbidity_ntu_int >> 8));
  tx_buffer[20] = (0xff & tds_int);                       // tds                        [2 bytes] little-endian
  tx_buffer[21] = (0xff & (tds_int >> 8));
  tx_buffer[22] = (0xff & ph_int);                        // ph                         [2 bytes] little-endian
  tx_buffer[23] = (0xff & (ph_int >> 8));
  tx_buffer[24] = sats_tracked;                           // satellites tracking        [1 byte]
  tx_buffer[25] = (0xff & latitude_int);                  // latitude                   [2 bytes] little-endian
  tx_buffer[26] = (0xff & (latitude_int >> 8)); 
  tx_buffer[27] = (0xff & longitude_int);                 // longitude                  [2 bytes] little-endian
  tx_buffer[28] = (0xff & (longitude_int >> 8));
  tx_buffer[29] = (0xff & date_year_int);                 // year                       [2 bytes] little-endian
  tx_buffer[30] = (0xff & (date_year_int >> 8));
  tx_buffer[31] = date_month_int;                         // month                      [1 byte]                
  tx_buffer[32] = date_day_int;                           // day                        [1 byte]
  tx_buffer[33] = time_hour_int;                          // hour                       [1 byte]
  tx_buffer[34] = time_minutes_int;                       // minutes                    [1 byte]
  tx_buffer[35] = time_seconds_int;                       // seconds                    [1 byte]
  
  #if DEBUG
    Serial.print("");
    Serial.print("sending LoRa buffer:");
    for (int i = 0; i < TX_BUF_SIZE; i++) {
      Serial.print(" 0x");
      Serial.print(tx_buffer[i], HEX);
    }
    Serial.println();
    Serial.println();
  #endif

  // Send data to Raspberry Pi LoRa Gateway
  rfm.send(tx_buffer, TX_BUF_SIZE);
  #if DEBUG
    Serial.println("sending buffer to LoRa receiver...");
  #endif
  // wait for successful radio transmission
  rfm.waitPacketSent();

  // wait for reply from Raspberry Pi LoRa Gateway
  uint8_t reply_buffer[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t buffer_len = sizeof(reply_buffer);

  #if DEBUG
    Serial.println("waiting for reply from LoRa receiver..."); delay(10);
    
    if (rfm.waitAvailableTimeout(1000)) {
      // should have received a reply message from LoRa receiver
      if (rfm.recv(reply_buffer, &buffer_len)) {
        Serial.print("got reply from LoRa receiver: ");
        Serial.println((char*)reply_buffer);
        Serial.print("RSSI: ");
        Serial.println(rfm.lastRssi(), DEC);
      } else {
        Serial.println("reply receipt failed...");
      }
    } else {
      Serial.println("no reply from LoRa receiver, is LoRa receiver application listening???");
    }
  #endif
}


static void goToSleepMins(int sleepMins) {

  #if DEBUG
    Serial.print("going to sleep for ");
    Serial.print(sleepMins);
    Serial.println(" minutes...");
  #endif
  
  delay(sleepMins * 60 * 1000UL); 
}


int getMedianNum(int bArray[], int iFilterLen) {
  
      int bTab[iFilterLen];
      for (byte i = 0; i < iFilterLen; i++)
        bTab[i] = bArray[i];
      int i, j, bTemp;
      
      for (j = 0; j < iFilterLen - 1; j++) {
        for (i = 0; i < iFilterLen - j - 1; i++) {
          if (bTab[i] > bTab[i + 1]) {
              
            bTemp = bTab[i];
            bTab[i] = bTab[i + 1];
            bTab[i + 1] = bTemp;
          }
        }
      }
      
      if ((iFilterLen & 1) > 0) {    
        bTemp = bTab[(iFilterLen - 1) / 2];    
      } else {
        bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
      }
       
      return bTemp;
}
