#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include "LowPower.h"

#define DE÷BUG 1

#define PIR_PIN 8
#define RED_LED_PIN 9
#define GREEN_LED_PIN 10
#define BLUE_LED_PIN 11

#define PIR_WARMUP_TIME 30000
#define TEST_PIR_WARMUP_TIME 2000
#define LIGHT_ON_DURATION 30000
#define DARKNESS_MAX 950
// go to sleep after this period of being bright
#define SLEEP_THRESHOLD 30000

//Adafruit_TCS34725 tcs = Adafruit_TCS34725();

int LED_VALUE = 0;
boolean motion_detected_now = false;
boolean motion_detected_prev = false;
unsigned long switch_off_time = 0;
unsigned long bright_since = 0;

const bool detect_motion(){
  const int value = digitalRead(PIR_PIN);
  return (value == HIGH);
}

int readPhotoResistor(){
  int value = analogRead(A0);
#ifdef DEBUG
  Serial.print("Photoresistor: ");
  Serial.println(value);
#endif
  return value;
}

const boolean isLEDOn(){
  return LED_VALUE > 0;
}

//uint16_t readBrightness(){
//  uint16_t r, g, b, c, colorTemp, lux;
//  
//  tcs.getRawData(&r, &g, &b, &c);
//  colorTemp = tcs.calculateColorTemperature(r, g, b);
//  lux = tcs.calculateLux(r, g, b);
//
//#ifdef DEBUG
//  Serial.println("------------------------");
//  Serial.print("C: "); Serial.print(c, DEC);  
//  Serial.println(" ");
//  Serial.print("Lux: "); Serial.print(lux, DEC);
//  Serial.println("");
//  Serial.println("------------------------");
//#endif
//  return lux;
//}

const boolean isTooDark(){
  int photoResistorValue = readPhotoResistor();
  return (photoResistorValue > DARKNESS_MAX);
}

void increaseBrightness(){
  // Short circuit to safe energy
  if(LED_VALUE == 255){
    return;
  }
  
    if(LED_VALUE + 10 > 255){
      LED_VALUE = 255;
    }else{
      Serial.println("Increasing led pwm values: ");
      LED_VALUE += 25;
    }
    // http://www.tayloredmktg.com/rgb/#PI
    // Pink-ish
    // TODO: gamma correction
    const int red   = LED_VALUE * 1;
    const int green = LED_VALUE * 0.02;
    const int blue  = LED_VALUE * 0.05;
    
    Serial.println(red);
    Serial.println(green);
    Serial.println(blue);
    
    analogWrite(RED_LED_PIN, red);
    analogWrite(GREEN_LED_PIN, green);
    analogWrite(BLUE_LED_PIN, blue);
  
}

void switchOffLights(){
  LED_VALUE = 0;
  analogWrite(9, LED_VALUE);
  analogWrite(10, LED_VALUE);
  analogWrite(11, LED_VALUE);
}

void blink_leds(unsigned long delay_ms = 1000){
  analogWrite(RED_LED_PIN, HIGH);
  delay(delay_ms);

  analogWrite(RED_LED_PIN, LOW);
  analogWrite(GREEN_LED_PIN, HIGH);
  delay(delay_ms);
  
  analogWrite(GREEN_LED_PIN, LOW);
  analogWrite(BLUE_LED_PIN, HIGH);
  delay(delay_ms);
  analogWrite(BLUE_LED_PIN, LOW);
}


void setup() {
  Serial.begin(115200);
  Serial.println("Starting setup");
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(BLUE_LED_PIN, OUTPUT);
  pinMode(PIR_PIN, INPUT);
  
//  if (tcs.begin()) {
//    Serial.println("Found sensor");
//  } else {
//    Serial.println("No TCS34725 found ... check your connections");
//    delay(10000);
//    while (1);
//  }
//  Serial.println("TCS initialized");
  
  Serial.println("Warming up PIR sensor");
  
#ifdef DEBUG
  delay(TEST_PIR_WARMUP_TIME);
  Serial.println("Done warming up");
#else
  delay(PIR_WARMUP_TIME/3);
  blink_leds();
  delay(PIR_WARMUP_TIME/3);
  blink_leds();
  delay(PIR_WARMUP_TIME/3);
#endif

  blink_leds();
}


//
//Too Dark |  Motion | LED ON | Action 
//---------------------------------------
//    Y   |    Y    |   Y     |  Increase brightness
//    Y   |    Y    |   N     |  Increase brightness
//    Y   |    N    |   Y     |  Pending Switch off
//    Y   |    N    |   N     |  Pending Switch off
//    N   |    Y    |   Y     |  Pending Switch off
//    N   |    Y    |   N     |  NOOP
//    N   |    N    |   Y     |  Switch off
//    N   |    N    |   N     |  NOOP
//       
void loop() {
  unsigned long current_time = millis();
#ifdef DEBUG
  Serial.print("Current time: "); Serial.print(current_time); Serial.println();
  Serial.print("Time to shutdown: "); Serial.print(switch_off_time); Serial.println();
#endif

  if(isTooDark()){
    if(detect_motion()){
      // we dont care if the LED is already on.
      // if its too dark and there is motion, we always
      // continue increasing brightness
      Serial.println("motion detected");
      switch_off_time = current_time + LIGHT_ON_DURATION;
      increaseBrightness();
    }else{
      // No motion but too dark means switch off is pending
      if(isLEDOn() && switch_off_time < current_time){
        switchOffLights();
      }else{
        // either LEDs are off or we need to wait until the 
        // light-on-time is over
      }
    }
  }else{ // Room is bright enough
    if(isLEDOn() && switch_off_time < current_time){
        switchOffLights();
    }else{
        // We dont care about motion in case of bright rooms
        // we already know the lights are not pending, so there
        // is nothing to do.
    }
    // if its bright for longer than 30 seconds we can go to sleep
    if(switch_off_time + SLEEP_THRESHOLD < current_time){
#ifdef DEBUG
      Serial.println("Its bright enough. Time to sleep");
      delay(500);
#endif
      LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); 
    }
  }

  delay(1000);
}
