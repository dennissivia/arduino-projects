
// ----- DHT 22 -------

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#define DHTPIN     13
#define DHTTYPE    DHT22 // DHT 22 (AM2302)

DHT_Unified dht(DHTPIN, DHTTYPE);
uint32_t delayMS;
// --------

#define DEBUG 1

// ----- LDC I2C -------------
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x3F,16,2);
// ---------------------------

// -------- Thermistor ------

#include <NTC_Thermistor.h>

#define SENSOR_PIN             A0
#define REFERENCE_RESISTANCE   28000
#define NOMINAL_RESISTANCE     115000
#define NOMINAL_TEMPERATURE    22
#define B_VALUE                3950

/**
   How many readings are taken to determine a mean temperature.
   The more values, the longer a calibration is performed,
   but the readings will be more accurate.
*/
#define READINGS_NUMBER  5

/**
   Delay time between a temperature readings
   from the temperature sensor (ms).
*/
#define DELAY_TIME  10

NTC_Thermistor* thermistor = NULL;

void initThermistor() {
  thermistor = new NTC_Thermistor(
    SENSOR_PIN,
    REFERENCE_RESISTANCE,
    NOMINAL_RESISTANCE,
    NOMINAL_TEMPERATURE,
    B_VALUE,
    READINGS_NUMBER,
    DELAY_TIME
  );
}


// --------------------

void setupLCD(){
  Serial.println("Setting up LCD");
  lcd.init();
  lcd.clear();
  // Print a message to the LCD.
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("Starting LCD");
}

// ----- MCP 9808 ------------------


#include "Adafruit_MCP9808.h"

// Create the MCP9808 temperature sensor object
Adafruit_MCP9808 mcp = Adafruit_MCP9808();

void initMCP9808(){
  if (!mcp.begin()) {
    Serial.println("Couldn't find MCP9808!");
    while (1);
  }
}

// ------------ BMP 180 -----------
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>

Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);


void displayBMP180Details(void)
{
  sensor_t sensor;
  bmp.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" hPa");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" hPa");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" hPa");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void initBMP180() {
  bmp.begin();
  /* Initialise the sensor */
  if (!bmp.begin()) {
    /* There was a problem detecting the BMP085 ... check your connections */
    Serial.print("Ooops, no BMP180 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  Serial.println("BMP180 Started");
#ifdef DEBUG
  displayBMP180Details();
#endif
}


void initDHT22() {
  dht.begin();
  Serial.println("DHTxx Unified Sensor Example");
  // Print temperature sensor details.
  sensor_t sensor;

  dht.temperature().getSensor(&sensor);

#ifdef DEBUG
  Serial.println("------------------------------------");
  Serial.println("Temperature");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" *C");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" *C");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" *C");
  Serial.println("------------------------------------");

  dht.humidity().getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.println("Humidity");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println("%");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println("%");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println("%");
  Serial.println("------------------------------------");
#endif

  // Set delay between sensor readings based on sensor details.
  delayMS = sensor.min_delay / 1000;
}


double readThermistor() {
  const double celsius = thermistor->readCelsius();
  Serial.print("T (thermistor): " + String(celsius) + " °C");
  return celsius;
}

const float readBMP180() {
  /* Get a new sensor event */
  sensors_event_t event;
  bmp.getEvent(&event);

  /* Display the results (barometric pressure is measure in hPa) */
  if (event.pressure){
    /* Display atmospheric pressue in hPa */
    float pressure = event.pressure;

#ifdef DEBUG
    Serial.print("Pressure:    ");
    Serial.print(pressure);
    Serial.println(" hPa");

    float temperature;
    bmp.getTemperature(&temperature);
    Serial.print("T (BMP180): ");
    Serial.print(temperature);
    Serial.println(" C");

    //    float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
    float seaLevelPressure = 1027.5;
    Serial.print("Altitude (BMP180):    ");
    Serial.print(bmp.pressureToAltitude(seaLevelPressure, pressure));
    Serial.println(" m");
#endif

    return pressure;
  } else {
    Serial.println("Sensor error");
    return NULL;
  }
}
// ---------------------------------


float readMCP9808(){
  mcp.wake();   // wake up, ready to read!

  float c = mcp.readTempC();
#ifdef DEBUG
  Serial.print("Temp (MCP): "); Serial.print(c); Serial.println("*C");
#endif
  mcp.shutdown(); // shutdown MSP9808 - power consumption ~0.1 mikro Ampere

  return c;
}
// ---------------------------------

double readDHT22() {
  // Delay between measurements.
  delay(delayMS);

  sensors_event_t event;
  dht.temperature().getEvent(&event);
  const double t = event.temperature;
  dht.humidity().getEvent(&event);
  const double h = event.relative_humidity;

  if (isnan(t)) {
    Serial.println("Error reading temperature!");
  }
  if (isnan(h)) {
    Serial.println("Error reading humidity!");
  }

  if(isnan(h) || isnan(t)){
    return;
  }

#ifdef DEBUG
  Serial.print("T (DHT22): ");
  Serial.print(t);
  Serial.println(" °C");
  Serial.print("Humidity (DHT22): ");
  Serial.print(h);
  Serial.println("%");
#endif

  return h;
}

float readUV() {
  int analogValue = analogRead(A1);
  float voltage = analogValue * (5 / 1023);
  float uvIndex = voltage / 0.1;

#ifdef DEBUG
  Serial.println(String("Analog UV val: ") + analogValue);
  Serial.println(String("UV Index: ") + uvIndex);
#endif

  return uvIndex;
}

int calculateHeadIndex(float temp, float humidity){
  DHT dht_raw = DHT(DHTPIN, DHTTYPE, 6);
  dht_raw.begin();
  boolean isFarenheit = false;
  float heatIndex = dht_raw.computeHeatIndex(temp, humidity, isFarenheit);

#ifdef DEBUG
  Serial.print("Heat index: ");
  Serial.print(heatIndex);
  Serial.println(" °C");
#endif

  return heatIndex;
}

void blinkLEDs() {
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);
}

void writeLCD(char* msg, int col, int row){
  lcd.setCursor(col,row);
  Serial.println(msg);
  lcd.print(msg);
}


void emptyLCD(){
  writeLCD("                ",0,0);
  writeLCD("                ",0,1);
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(57600);

  // Initialize devices
  initDHT22();
  initThermistor();
  initBMP180();
  initMCP9808();
  setupLCD();
}


void loop() {
  //emptyLCD();
  double humidity, temp1, uvIndex, pressure, temp2, heatIndex;
  String lcdMsg;

//  blinkLEDs();
  humidity = readDHT22();      // Humidity
  temp1    = readThermistor(); // Temperature
  uvIndex  = readUV();         // UV Index
  pressure = readBMP180();     // Pressure
  temp2    = readMCP9808();    // Temperature (accurate)
  heatIndex = calculateHeadIndex(temp2, humidity);
  String degreeC = "*C";

  Serial.println("--- Performing readings ---");
  lcdMsg = String(temp2) + degreeC + ", " + humidity + "%";
  writeLCD(lcdMsg.c_str(), 0, 0);

  lcdMsg = String(heatIndex) + degreeC + " (feel)";
  writeLCD(lcdMsg.c_str(), 0, 1);

  delay(5000);
  emptyLCD();

  lcdMsg = String(int(pressure)) + " hPa, " + String(int(uvIndex)) + "UV";
  writeLCD(lcdMsg.c_str(), 0, 0);
  if(uvIndex > 3){
    writeLCD("Sun lotion!", 0, 1);
  }
  Serial.println("------- Done --------");
  delay(5000);

}
