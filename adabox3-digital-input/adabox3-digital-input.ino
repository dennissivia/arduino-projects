// Adafruit IO Digital Input Example

#include "config.h"

#define BUTTON_PIN 5
#define LED_PIN   16
#define RED_PIN   15
#define GREEN_PIN 13
#define BLUE_PIN  12
#define SERVO_PIN  2

// analog pin 0
#define PHOTOCELL_PIN A0
// pin connected to DH22 data line
#define DHT_PIN 14

#define FEED_NAME_IN  "adabox3.digital-in"
#define FEED_NAME_OUT "adabox3.digital-out"
#define FEED_NAME_RGB "adabox3.rgb-colors"
#define FEED_NAME_ANALOG_IN "adabox3.analog-in"
#define FEED_NAME_ANALOG_OUT "adabox3.analog-out"
#define FEED_NAME_SERVO "adabox3.servo"
#define FEED_NAME_TEMP "adabox3.dht11-temp"
#define FEED_NAME_HUMIDITY "adabox3.dht11-humidity"

// photocell state
int photo_current = 0;
int photo_last = -1;

// button state
bool current = false;
bool last = false;

// ----

/************************ Example Starts Here *******************************/
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
// create DHT22 instance
DHT_Unified dht(DHT_PIN, DHT22);


// stuff we push to the adadruit
AdafruitIO_Feed *temperature = io1.feed(FEED_NAME_TEMP);
AdafruitIO_Feed *humidity    = io1.feed(FEED_NAME_HUMIDITY);
AdafruitIO_Feed *digital_in  = io1.feed(FEED_NAME_IN);
AdafruitIO_Feed *analog_in   = io1.feed(FEED_NAME_ANALOG_IN);

// subscriptions
AdafruitIO_Feed *digital_out= io1.feed(FEED_NAME_OUT);
AdafruitIO_Feed *rgb_color  = io1.feed(FEED_NAME_RGB); 
AdafruitIO_Feed *analog_out = io1.feed(FEED_NAME_ANALOG_OUT);
AdafruitIO_Feed *servo_feed = io1.feed(FEED_NAME_SERVO);


#include <Adafruit_SSD1306.h>
Adafruit_SSD1306 oled = Adafruit_SSD1306(128, 32, &Wire);


// ----------- servo  -------
#if defined(ARDUINO_ARCH_ESP32)
  // ESP32Servo Library (https://github.com/madhephaestus/ESP32Servo)
  // installation: library manager -> search -> "ESP32Servo"
  #include <ESP32Servo.h>
#else
  #include <Servo.h>
#endif
Servo servo;



void setup() {
//  oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)
//  oled.display();
  
  // set button pin as an input
  pinMode(BUTTON_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);

  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);

  // start the serial connection
  Serial.begin(115200);

  // wait for serial monitor to open
  while(! Serial);

  servo.attach(SERVO_PIN);
  dht.begin();

  
  // connect to io.adafruit.com
  Serial.println("Connecting to Adafruit IO");
  io1.connect();

  analog_out->onMessage(handleAnalogOutMessage);
  digital_out->onMessage(handleLedOnOffMessage);
  rgb_color->onMessage(handleRgbMessage);
  servo_feed->onMessage(handleServoMessage);

  // wait for a connection
  while(io1.status() < AIO_CONNECTED) {
    Serial.print(".");
    delay(100);
  }

  // we are connected
  Serial.println("connect status 1");
  Serial.println(io1.statusText());
  
  digital_out->get();
  analog_out->get();
  rgb_color->get();
  servo_feed->get();

  // set analogWrite range for ESP8266
  #ifdef ESP8266
    Serial.println("we are running on an ESP8266");
    analogWriteRange(255);
  #endif
  
  // text display tests
//  oled.setTextSize(1);
//  oled.setTextColor(WHITE);
}

void loop() {
  io1.run(400);

  performButtonReading();
  performPhotocellReading();
  delay(200); // photo sensor
  performDHTReading();
  delay(300); // for DHT
  delay(500); // OLED

  delay(2000);
}

void performButtonReading(){
    // grab the current state of the button.
  // we have to flip the logic because we are
  // using a pullup resistor.
  if(digitalRead(BUTTON_PIN) == LOW)
    current = true;
  else
    current = false;

  // return if the value hasn't changed
  if(current == last)
    return;

  // save the current state to the 'digital' feed on adafruit io
  Serial.print("sending button -> ");
  Serial.println(current);
  digital_in->save(current);

  // store last button state
  last = current;
}

void performPhotocellReading(){
    // grab the current state of the photocell
  photo_current = analogRead(PHOTOCELL_PIN);
  
  // return if the value hasn't changed a lot
  // we changed it to a delta of 30 to avoid writes on 
  // oscillating values
  if(abs(photo_current - photo_last) < 30)
    return;

  // save the current state to the analog feed
  Serial.print("sending -> ");
  Serial.println(photo_current);
  analog_in->save(photo_current);

  // store last photocell state
  photo_last = photo_current;

  // delay(300); // done in loop
}

void handleRgbMessage(AdafruitIO_Data *data){
  Serial.println("Received hex value: " + String(data->value()));

  long int red   = data->toRed();
  long int green = data->toGreen();
  long int blue  = data->toBlue();
  Serial.println(String(red));
  Serial.println(String(green));
  Serial.println(String(blue));
  analogWrite(RED_PIN, 255 - red);
  analogWrite(GREEN_PIN, 255 - green);
  analogWrite(BLUE_PIN, 255 - blue);
}

void handleLedOnOffMessage(AdafruitIO_Data *data) {
  Serial.print("received <- ");

  if(data->toPinLevel() == HIGH)
    Serial.println("HIGH");
  else
    Serial.println("LOW");
  digitalWrite(LED_PIN, data->toPinLevel());
}

void handleAnalogOutMessage(AdafruitIO_Data *data) {
  Serial.println("In Analog out");
  int reading = data->toInt();
  Serial.print("A-OUT: received <- ");
  Serial.println(reading);
  analogWrite(LED_PIN, reading);
}

void performDHTReading(){
  sensors_event_t event;
  dht.temperature().getEvent(&event);

  float celsius = event.temperature;

#if DEBUG
  Serial.print("celsius: ");
  Serial.print(celsius);
  Serial.println("C");
#endif 

  // save fahrenheit (or celsius) to Adafruit IO
  temperature->save(celsius);

  dht.humidity().getEvent(&event);
#if DEBUG
  Serial.print("humidity: ");
  Serial.print(event.relative_humidity);
  Serial.println("%");
#endif
  // save humidity to Adafruit IO
  humidity->save(event.relative_humidity);



  // -------- print it to the OLED
//  oled.clearDisplay();
//  oled.setCursor(0,0);
//  oled.print("SSID: "); oled.println(WIFI_SSID);
//  oled.print("IP: "); oled.println(WiFi.localIP());
//  oled.print("Temp: "); oled.print(celsius,0); oled.print(" *C ");
//  oled.print("Hum: "); oled.print(event.relative_humidity,0); oled.println(" %");
//  oled.print("IO Status: ");
//  aio_status_t aio_status = io.status();
//  //Serial.print("Status: "); Serial.println(aio_status);
//  switch (aio_status) {
//     case AIO_IDLE:  oled.println("IDLE"); break;
//     case AIO_DISCONNECTED:
//     case AIO_NET_DISCONNECTED:  oled.println("DISCONNECT"); break;
//     case AIO_NET_CONNECTED:
//     case AIO_CONNECTED_INSECURE:
//     case AIO_CONNECTED: oled.println("CONNECTED"); break;
//  }
//  oled.display();
  // ----------------

//  delay(500); // done in loop
}

void handleServoMessage(AdafruitIO_Data *data) {
  Serial.println("In servo handler");
  // convert the data to integer
  int angle = data->toInt();
  Serial.println("Servo: received value: " + String(angle));

  // make sure we don't exceed the limit
  // of the servo. the range is from 0
  // to 180.
  if(angle < 0)
    angle = 0;
  else if(angle > 180)
    angle = 180;

  servo.write(180 - angle);
}
