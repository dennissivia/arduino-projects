// include the library code:
#include <LiquidCrystal.h>
#include <math.h>

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(6, 5, 9, 10, 11, 12);

// Degree symbol bitmap
byte degree[8] = {
  B01000,
  B10100,
  B01000,
  B00000,
  B00000,
  B00000,
  B00000,
  B00000,
};

//TMP36 Pin Variables
const int sensorPin = A0; //the analog pin the TMP36's Vout (sense) pin is connected to

//Pins for Tactile buttons used for raising or lowering alarm temperature
const int lowerAlarmTemp = 2;
const int raiseAlarmTemp = 3;



//Buzzer Pin
const int buzzer = 13;
//frequency out
int freq;

//Red and Green status LED's
const int redLed = 1;
const int greenLed = 0;

//Set to 1 to display Celsius instead of Fahrenheit
int celsius = 1;
//Alarm value, change to what starting value should be.
int alarmTemp = 25;
void setup() {
  //Create the degree symbol bitmap
  lcd.createChar(0, degree);

  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);

  //Set tactile buttons as inputs, use of pullup means we dont need external resistor
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);

  // Set buzzer pin as output
  pinMode(13, OUTPUT);

  //Set LED's as output
  pinMode(0, OUTPUT);
  pinMode(1, OUTPUT);
}

//Function that runs when button pressed to lower alarm temperature
void lowerAlarm()
{

  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  // If interrupts come faster than 400ms, assume it's a bounce and ignore
  if (interrupt_time - last_interrupt_time > 400)
  {
    alarmTemp = alarmTemp - 1; //Lower alarmTemp by one
  }
  last_interrupt_time = interrupt_time;

}

//Function that runs when button pressed to raise alarm temperature
void raiseAlarm()
{
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  // If interrupts come faster than 400ms, assume it's a bounce and ignore
  if (interrupt_time - last_interrupt_time > 400)
  {
    alarmTemp = alarmTemp + 1; //Raise alarmTemp by one
  }
  last_interrupt_time = interrupt_time;
}

void loop() {

  //Interupt that when lowerAlarmTemp button is pressed runs lowerAlarm function
  attachInterrupt(digitalPinToInterrupt(lowerAlarmTemp), lowerAlarm, FALLING);
  //Interupt that when raiseAlarmTemp button is pressed runs raiseAlarm function
  attachInterrupt(digitalPinToInterrupt(raiseAlarmTemp), raiseAlarm, FALLING);

  //Clear LCD
  lcd.clear();

  //Display Currently on the LCD
  lcd.print("Currently ") ;

  //getting the voltage reading from the temperature sensor
  int reading = analogRead(sensorPin);
  // converting that reading to voltage, for 3.3v arduino use 3.3
  float voltage = reading * 3.3;
  voltage /= 1024.0;

  float temperatureC = (voltage - 0.5) * 100 ;  //converting from 10 mv per degree with 500 mV offset
  //to degrees ((voltage - 500mV) times 100)

  if (celsius == 1) //If you set temperature as Celsius it will print Celsius values
  {
    //Round the temperature to a whole number
    float roundedTempC = round(temperatureC);

    // Display temperature in C
    lcd.print(roundedTempC, 0);
    lcd.write(byte(0)); //Degree symbol we created earlier
    lcd.print("C");

    // Display alarm Temp
    lcd.setCursor(0, 1);
    lcd.print("Alarm at ");
    lcd.print(alarmTemp);
    lcd.write(byte(0));
    lcd.print("C");

    //Check if temperature is equal or greater than alarmTemp
    if (roundedTempC >= alarmTemp)
    {
      tone(buzzer, 200); // Play alarm tone
      delay(400);
      noTone(buzzer);
      digitalWrite(redLed, HIGH); //Turn red LED on
      digitalWrite(greenLed, LOW); //Turn green LED off
    }
    else
    {
      noTone(buzzer); //Make sure alarm is off
      digitalWrite(redLed, LOW); //Turn red LED off
      digitalWrite(greenLed, HIGH); //Turn green LED on
    }
  }
  else //Display in Fahrenheit
  {
    //Convert from Celsius to Fahrenheit
    float temperatureF = (temperatureC * 9.0 / 5.0) + 32.0;

    //Round the temperature to a whole number
    float roundedTempF = round(temperatureF);

    // Display temperature in F
    lcd.print(roundedTempF, 0);
    lcd.write(byte(0)); //Degree symbol we created earlier
    lcd.print("F");

    // Display alarm Temp
    lcd.setCursor(0, 1);
    lcd.print("Alarm at ");
    lcd.print(alarmTemp);
    lcd.write(byte(0));
    lcd.print("F");

    //Check if temperature is equal or greater than alarmTemp
    if (roundedTempF >= alarmTemp)
    {
      tone(buzzer, 200); // Play alarm tone
      delay(400);
      noTone(buzzer);
      digitalWrite(redLed, HIGH); //Turn red LED on
      digitalWrite(greenLed, LOW); //Turn green LED off
    }
    else
    {
      noTone(buzzer); //Make sure alarm is off
      digitalWrite(redLed, LOW); //Turn red LED off
      digitalWrite(greenLed, HIGH); //Turn green LED on
    }
  }
  delay(1000);
}
