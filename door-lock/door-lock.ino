// #include <Keypad.h>
#include <OnewireKeypad.h>
#include <EEPROM.h>
#include <SPI.h>
#include <MFRC522.h>
#include <Wire.h>

#include "config.h"
#define DEBUG 1

#define RELAY_PIN 6 
#define BUZZER_PIN 7

#define RST_PIN 9
#define SS_PIN 10
MFRC522 mfrc522(SS_PIN, RST_PIN); // Create MFRC522 instance
byte tagID[5]; // input buffer for the tag serial

String inputBuffer;
char lastKey;
short int inputBufferPos = 0;

// const byte numRows= 4; //number of rows on the keypad
// const byte numCols= 4; //number of columns on the keypad
//keymap defines the key pressed according to the row and columns just as appears on the keypad
// char keymap[numRows][numCols]= {
//   {'1', '2', '3', 'A'}, 
//   {'4', '5', '6', 'B'}, 
//   {'7', '8', '9', 'C'},
//   {'*', '0', '#', 'D'}
// };
// byte rowPins[numRows] = {2,3,4,5}; //Rows 0 to 3
// byte colPins[numCols] = {6,7,8,9}; //Columns 0 to 3
// Keypad myKeypad= Keypad(makeKeymap(keymap), rowPins, colPins, numRows, numCols);

char keymap[]= {
  '1', '2', '3', 'A', 
  '4', '5', '6', 'B', 
  '7', '8', '9', 'C',
  '*', '0', '#', 'D'
};
OnewireKeypad <Print, 16 > KP(Serial, keymap, 4, 4, A0, 5300, 1000);
// OnewireKeypad <Print, 16 > KP(Serial, keymap, 4, 4, A0, 4700, 1000);
// OnewireKeypad <Print, 16 > KP(Serial, keymap, 4, 4, A0, 2000, 330, 1000);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); // Initialize serial communications with the PC
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  SPI.begin();
  mfrc522.PCD_Init();

  KP.SetHoldTime(500);
  KP.SetDebounceTime(300);
  KP.SetAnalogPinRange(1023.0);
  KP.SetKeypadVoltage(5.0);

  KP.ShowRange();
  Serial.println("Started");
  
}

void loop() {
  readKeypad();
  if(checkInputBuffer()){
    handleCodeValid();
    resetInputBuffer();
  }
  if(validateNFC()){
    handleTagValid();
  }
}

void readKeypad(){
  byte state = KP.Key_State();
  if(state == PRESSED || state == HELD){
    char c = KP.Getkey();
    if(c != NO_KEY){
      lastKey = c;
    }
  }else if(state == RELEASED){
    if(lastKey){
      Serial.println("Key is: " + String(lastKey));
      handleKeyInput(lastKey);
    }
  }else{
    // Serial.println("Waiting for input...")
  }
}

void handleKeyInput(const char c){
  if(c == 'C'){
    resetInputBuffer();         
  }else{
    if(inputBufferPos < ACCESS_CODE.length()){
      if(c == ACCESS_CODE[inputBufferPos]){
        indicateCorrectKey(); 
      }else{
        indicateIncorrectKey();
      }
      recordInput(c);

    }else{
      //resetBuffer();
    }
  }
  Serial.println("buf len: " + String(inputBufferPos));
}

void recordInput(const char c){
  Serial.println("recoding input");
  inputBuffer += c;
  inputBufferPos++;
}

void resetInputBuffer(){
  inputBuffer    = "";
  inputBufferPos = 0;
  lastKey = NULL;
}

void indicateCorrectKey(){}
void indicateIncorrectKey(){}

const bool checkInputBuffer(){
  return (inputBufferPos == ACCESS_CODE.length() && inputBuffer == ACCESS_CODE);
}

void unlock(){
  tone(BUZZER_PIN, 4000, 100);
  delay(200);
  tone(BUZZER_PIN, 4000, 200);
  // noTone(BUZZER_PIN);
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  digitalWrite(RELAY_PIN, HIGH);
  delay(2000);
  digitalWrite(RELAY_PIN, LOW);
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
}

void handleTagValid(){
  Serial.println("NFC/RFID tag recognized.");
  unlock();
}
void handleCodeValid(){
  Serial.println("correct code detected");
  unlock();
}

boolean readTagID() {
  // Getting ready for Reading PICCs
  if ( ! mfrc522.PICC_IsNewCardPresent()) { //If a new PICC placed to RFID reader continue
    Serial.println("no card present");
    return false;
  }
  if ( ! mfrc522.PICC_ReadCardSerial()) {   //Since a PICC placed get Serial and continue
    Serial.println("cannot read card.");
    return false;
  }
  // There are Mifare PICCs which have 4 byte or 7 byte UID care if you use 7 byte PICC
  // I think we should assume every PICC as they have 4 byte UID
  // Until we support 7 byte PICCs
  Serial.println(F("Scanned PICC's UID:"));
  for ( uint8_t i = 0; i < 4; i++) {  //
    tagID[i] = mfrc522.uid.uidByte[i];
  }
  printTagID(tagID);
  printTagID(goodTagID);
  // mfrc522.PICC_HaltA(); // Stop reading
  return true;
}

boolean validateNFC(){
  if(readTagID()){
    Serial.println("comparing goodTag with tagID");

    for ( uint8_t i = 0; i < 4; i++) {
      if(goodTagID[i] != tagID[i]){
        Serial.println("Sorry. Card not recognized.");
        return false;
      }
    }

    Serial.println("Matching Card detected.");
    return true;
  }else{
    return false;
  }
}

void printTagID(byte *tag){
  for ( uint8_t i = 0; i < 4; i++) {  //
    Serial.print(tag[i], HEX);
  }
  Serial.println("");
}