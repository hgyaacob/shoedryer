/*
Drying Box
After welcome message, the user inputs the timer and set the temperature intended for drying
Upon start, bulb and fan turned on
The bulb will turn off when the sensed temperature is above 2 deg than setpoint temperature and turn back on again when reached below 2 deg of setpoint
Fans and bulb will turn off when the relative humidity reached the dried setpoint. (around 5% to 30%) [https://patents.google.com/patent/WO2018006190A1/en]
*/

/*---------- LIBRARY -------------------------------------------*/
#include <DHT.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ezButton.h>

/*_____________________________________________________________________*/
/*---------- DEFINES -------------------------------------------*/

/*Define pin for Arduino*/
#define START_BTN_PIN 2
#define MINUS_BTN_PIN 3
#define PLUS_BTN_PIN 4
#define DHT11_PIN 5
#define RELAY_BULB_PIN 6  //IN1 relay to D6
#define RELAY_FAN_PIN 7   //IN1 relay to D7
#define BUZZER_PIN 11
#define HEATING_TEMP_VALUE 40; //drying temperature
#define DRY_VALUE 40; //humidity value when dry


/*_____________________________________________________________________*/
/*---------- OBJECTS & VARIABLES DECLARATION --------------------------*/
ezButton startButton(2);
ezButton minusButton(3);
ezButton plusButton(4);

/*Constant variable*/
const int LONG_PRESS_TIME = 1000;  // 1000 milliseconds

/*Global variables initialization*/
int setTimerSecond;          //Set timer value in seconds
int setTimerMinute = 5;          //Set timer value in minutes
int setTemp;                 //Set temperature value
float sensorTemp, sensorRh;  //Initialize sensor temperature and relative humidity variable
bool isHeating = 0;

/*Long press functionality */
int currentState = 0;
int lastState = 0;
unsigned long pressedTime = 0;
bool isPressing = false;
bool isLongDetected = false;

//below variables is used for sensing time interval (multitasking)
const long eventTime_dht11 = 1000; //time interval for sensor
unsigned long previousTime_dht11 = 0;
unsigned long previousTime_counter = 0;

char a[8];
char b[8];


/*Create Object*/
LiquidCrystal_I2C lcd(0x27, 16, 2);  //i2c address is 0x3F or 0x27
DHT dht11(5,DHT11);

/*_____________________________________________________________________*/
/*---------- SETUP FUNCTION -------------------------------------------*/
void setup() {
  //Initialize serial monitor, DHT sensor and also lcd.
  Serial.begin(9600);
  Serial.println("Test");
  dht11.begin();

  lcd.init();
  lcd.backlight();

  //Set pin modes
  pinMode(START_BTN_PIN, INPUT_PULLUP);  // set START BUTTON as INPUT
  pinMode(MINUS_BTN_PIN, INPUT_PULLUP);  // set MINUS BUTTON as INPUT
  pinMode(PLUS_BTN_PIN, INPUT_PULLUP);   // set PLUS BUTTON as INPUT
  pinMode(RELAY_BULB_PIN, OUTPUT);       // set IN1 RELAY as OUTPUT
  pinMode(RELAY_FAN_PIN, OUTPUT);        // set IN2 RELAY as OUTPUT

  startButton.setDebounceTime(50);
  minusButton.setDebounceTime(50);
  plusButton.setDebounceTime(50);

  //Directly set initial relay state (to avoid flickering on power up)
  digitalWrite(RELAY_BULB_PIN, HIGH);  // set default BULB and FAN as OFF
  digitalWrite(RELAY_FAN_PIN, HIGH);   // set default BULB and FAN as OFF
  pinMode(BUZZER_PIN, OUTPUT);         // set BUZZER as OUTPUT

  //Print welcome message
  lcd.setCursor(0, 0);             // first row
  lcd.print("   WELCOME :)   ");   // first row words
  lcd.setCursor(0, 1);             // second row
  lcd.print("                 ");  // second row words
  
  //check sensor, if NOK, please check sensor and reset
  sensorTemp = dht11.readTemperature();
  sensorRh = dht11.readHumidity();
  checkSensor();

  delay(2000);
  lcd.clear();
}
/*________________________________________________________________*/
/*---------- MAIN LOOP -------------------------------------------*/

void loop() {

  //Loop for button debounce (don't remove this)
  startButton.loop();
  minusButton.loop();
  plusButton.loop();

  // Below is the code for the sensor, avoid using delay() and check the sensor every eventTime_dht11 value.
  unsigned long currentTime = millis();

  if (currentTime - previousTime_dht11 >= eventTime_dht11){
    readDHT11();
    lcd.setCursor(0,1);
    lcd.print("T:");
    lcd.print(int(sensorTemp));
    lcd.print("       Rh:");
    lcd.print(int(sensorRh));
    previousTime_dht11 = currentTime;
  }

  
 //Different action when push button is pressed.
  if(isHeating == 0 && minusButton.isPressed()){ //Not in heating mode and minus button is pressed
    setTimerMinute = setTimerMinute - 5; //minus 10 minutes each time the minus button is pressed
    if(setTimerMinute < 5){ //make sure that the minute timer doesn't go below 5 minutes
      setTimerMinute = 5;
    }
  }
  else if (isHeating == 0 && plusButton.isPressed()){ //Not in heating mode and plus button is pressed
    setTimerMinute = setTimerMinute + 5; //minus 10 minutes each time the minus button is pressed
    if(setTimerMinute > 120){ //make sure that the timer doesn't exceed 120 minutes
      setTimerMinute = 120;
    }
  }
  else if (isHeating == 0 && startButton.isPressed()){ //Not in heating mode and start button is pressed
    isHeating = 1; //Heating is starting
  }
  else if (isHeating == 1){
    Serial.println("Heating started");
    heating(); //Start heating process
    timerRunning();
    longClick(); //Detect long click to get out of the heating mode
  }

  updateDisplayTimer(); //Update display on each loop cycle.

}

/*________________________________________________________________*/
/*---------- CUSTOM FUNCTIONS -------------------------------------------*/

void heating(){

  int heating_temp = HEATING_TEMP_VALUE + 2;

   //Turn on both bulb and fan
  digitalWrite(RELAY_BULB_PIN, LOW); 
  digitalWrite(RELAY_FAN_PIN, LOW); 
  
  if (int(sensorTemp) > heating_temp + 2){ //turn off if temperature reach +2 degrees than setpoint
    digitalWrite(RELAY_BULB_PIN, HIGH); 
  } 
  else if (int(sensorTemp) < heating_temp - 2){ //turn bulb back on when temperature falls -2 degrees than setpoint
    digitalWrite(RELAY_BULB_PIN, LOW); 
  }

}

void timerRunning(){

  unsigned long currentTimerTime = millis();

  if (currentTimerTime - previousTime_counter >= 1000){
    setTimerSecond = setTimerSecond - 1;

    if (setTimerSecond < 0){ //reduce seconds and minutes for each elapsed 60 seconds
      setTimerSecond = 59; 
      setTimerMinute = setTimerMinute - 1;
    }

    if (setTimerMinute < 0 ){ //when reached below 0 minutes, stopped heating
      stopHeating();
    }

    previousTime_counter = currentTimerTime;
  }

}




void longClick() {
  // read the state of the switch/button:
  currentState = digitalRead(START_BTN_PIN);

  if (lastState == HIGH && currentState == LOW) {  // button is pressed
    pressedTime = millis();
    isPressing = true;
    isLongDetected = false;
  } else if (lastState == LOW && currentState == HIGH) {  // button is released
    isPressing = false;
  }

  if (isPressing == true && isLongDetected == false) {
    long pressDuration = millis() - pressedTime;

    if (pressDuration > LONG_PRESS_TIME) {
      Serial.println("A long press is detected, dryer stopped ");
      isLongDetected = true;
      stopHeating();
    }
  }

  // save the the last state
  lastState = currentState;
}

void readDHT11() {
  
  sensorTemp = dht11.readTemperature();
  sensorRh = dht11.readHumidity();
  Serial.print(sensorTemp);
  Serial.print("   ");
  Serial.print(sensorRh);
  Serial.print("  ");
  Serial.println(setTimerMinute);

}

void checkSensor(){
  if (isnan(sensorTemp) ||isnan(sensorRh)){ //check sensor, if NOK, turn off bulb and fan
    lcd.clear();
    lcd.setCursor(0, 0);             // first row
    lcd.print("  SENSOR ERROR  ");   // first row words
    lcd.setCursor(0, 1);             // second row
    lcd.print("  CHECK SENSOR  ");  // second row words
    digitalWrite(RELAY_BULB_PIN, HIGH);
    digitalWrite(RELAY_FAN_PIN, HIGH);
    return;
  }
}

void updateDisplayTimer(){
  sprintf(a, "%.2d", setTimerMinute);
  sprintf(b, "%.2d", setTimerSecond);
  lcd.setCursor(0,0);
  lcd.print("     ");
  lcd.print(a);
  lcd.print(":");
  lcd.print(b);
  lcd.print("   ");   // first row words

}

void stopHeating(){
      isHeating = 0;
      setTimerMinute = 5;
      setTimerSecond = 0;
      digitalWrite(RELAY_BULB_PIN, HIGH); 
      digitalWrite(RELAY_FAN_PIN, HIGH); 
}