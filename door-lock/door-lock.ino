#include <Keypad.h>
#include "config.h"


const byte numRows= 4; //number of rows on the keypad
const byte numCols= 4; //number of columns on the keypad
//char *inputBuffer;
String inputBuffer;
short int inputBufferPos = 0;

//keymap defines the key pressed according to the row and columns just as appears on the keypad
char keymap[numRows][numCols]= {
  {'1', '2', '3', 'A'}, 
  {'4', '5', '6', 'B'}, 
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};

//Code that shows the the keypad connections to the arduino terminals
byte rowPins[numRows] = {2,3,4,5}; //Rows 0 to 3
byte colPins[numCols] = {6,7,8,9}; //Columns 0 to 3
//initializes an instance of the Keypad class
Keypad myKeypad= Keypad(makeKeymap(keymap), rowPins, colPins, numRows, numCols);


#define RELAY_PIN 12

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); // Initialize serial communications with the PC
  Serial.println("Started");
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  char c=myKeypad.getKey();
  if(c != NO_KEY){
    Serial.println("Key is: " + String(c));
    handleKeyInput(c);
  }
  if(checkInputBuffer()){
    handleCodeValid();
    resetInputBuffer();
  }else{
    
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
}

void indicateCorrectKey(){
 
}
void indicateIncorrectKey(){}

const bool checkInputBuffer(){
  return (inputBufferPos == ACCESS_CODE.length() && inputBuffer == ACCESS_CODE);
}
void handleCodeValid(){
  Serial.println("correct code detected");
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  digitalWrite(RELAY_PIN, HIGH);
  delay(2000);
  digitalWrite(RELAY_PIN, LOW);
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW

}


