#include <ESP32Servo.h>

//Declare Global Variables
int lidServoPin = 33;
int switchServoPin = 34;
int switchPin = 26;

//Declare a class for the switch
class UselessSwitch{
  //Variable for the pin
  int pin;
  //The switches are digital signals, so it will read 1 or 0
  //Variable to hold the current state of the switch
  int currentState;
  //Variable to hold the last state of the switch
  int lastState = 1;

  //Code the constructor for the class
  public:
  UselessSwitch(int input_pin){
    //Set the pin member variable to the input parameter for the pin
    pin = input_pin;
    //Set the switch pin to input pullup
    pinMode(pin, INPUT_PULLUP);
  }

  //Class functions
  bool isPressed(){
    //Read the state of switch and store it in the current state variable
    currentState = digitalRead(pin);

    //Conditional statements for the switch
    if (currentState == 0 && lastState == 1){
      //Store the current state as the last state
      lastState = currentState;
      //Return the desired state of true or false
      return false;
    }
    else if (currentState == 1 && lastState == 0){
      //Store the current state as the last state
      lastState = currentState;
      //Return the desired state of true or false
      return true;
    }
  }
};

//Declare a class for the servos
class UselessServo{
  //Member variables
  Servo servo;
  int servoPin;
  //Current position of the servo
  int pos;
  //Start position of the servo
  int startPos;
  //End position of the servo
  int endPos;
  //Time in between each servo movement
  int updateInterval;
  //How much to increment the position (start at -1 because start position is at 180)
  int increment = -1;
  //Time variable to hold last checked time
  unsigned long previousMillis;

  //Code the constructor for the class
  public:
  UselessServo(int input_pin, int interval){

    //Set the servoPin variable to the input parameter for the pin
    servoPin = input_pin;

    //Set the start position
    startPos = 180;
    pos = startPos;
    endPos = 0;

    //Set the update interval member variable to the input parameter for the update interval
    updateInterval = interval;

  }

  void Attach(){
    //Attach the servo to the servoPin
    servo.attach(servoPin);
    //Start the servo at the starting position
    servo.write(startPos);
  }

  void moveForward(){
    //Conditional statement for checking the time and updating the position of the servo to move Forward
    //If the current time minus the previous time is greater than the update interval AND
    //The position of the servo hasn't reached the end position
    if((millis() - previousMillis) > updateInterval && pos != endPos){
      //Set previous millis to millis()
      previousMillis = millis();
      //Increment the position variable
      pos += increment;
      //Move the servo
      servo.write(pos);
    }
  }

  //Conditional statement for checking the time and updating the position of the servo to move Backward
  //If the current time minus the previous time is greater than the update interval AND
  //The position of the servo hasn't reached the start position
  void moveBackward(){
    if( (millis() - previousMillis) > updateInterval && pos != startPos ){

      //Set previous millis to millis()
      previousMillis = millis();

      //Decrement the position variable
      pos -= increment;

      //Move the servo
      servo.write(pos);

    }
  }

  //Check if a servo is moving so only one servo moves at a time
  bool isMoving(){
    //If the position is equal to the start or end position, then the servo not moving so return 0 or false
    if(pos == endPos || pos == startPos){
      return 0;
    }
    //If it isn't at the start or end position, then the servo is moving so return 1 or true
    else{
      return 1;
    }
  }
};

//Declare new member of the UselessServo class for the lid Servo with update interval of 5 (or whatever you want)
UselessServo lidServo(lidServoPin, 5);

//Declare new member of the UselessServo class for the switch Servo with update interval of 1 (or whatever you want)
UselessServo switchServo(switchServoPin, 1);

//Declare new member of the UselessSwitch class
UselessSwitch mainSwitch(switchPin);

void setup() {
  //Use the Attach() class function to attach the lid servo and switch servo
  lidServo.Attach();
  switchServo.Attach();
}

void loop() {
  //Use the switch class function isPressed() to check the state of the switch
  if(mainSwitch.isPressed() == 1){
    //Use the servo class function moveForward() to move the lid servo forward
    lidServo.moveForward();

    //Check if the lid servo is done moving, once it is, move the switch servo forward using the moveForward() function
    if(lidServo.isMoving() == 0){

      //Move the switch servo forward
      switchServo.moveForward();

    }
  } else{

    //Move the switch servo backward using the moveBackward()
    switchServo.moveBackward();

    //Check if the switch servo is done moving, once it is, move the lid servo backward using the moveBackward() function
    if(switchServo.isMoving() == 0){

      //Move the lid servo backward
      lidServo.moveBackward();

    }
  }
}
