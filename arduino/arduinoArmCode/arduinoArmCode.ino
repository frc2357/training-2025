
//define the wires and CLK DT and SW reading wires to arduino ports
// make variables to store information such as position from the start and from the end
// have a way for that information to show up on the monitor and define button pressed and amount of rotation
// tell the motor what to do with the information and lets say run half a rotation clockwise if the variable changes by -1 or vise versa
// make it so the button activates a claw motor and deactivates it to ???
//maybe use a double rotary encoder instead of a button but that would be a learning curve
// fun idea the rotary encoder can get mixed up so when button reset the position
// make sure that the motor will not go to far like if the fully retracted position is 0 then you cannot go under 0
// Rotary Encoder Inputs
// instead of a double rotary encoder I could use a button to switch between control of the length and of the claw

#define CLK 1
#define DT 2
#define SW 0
//define variables such as current state and last state as well as button press
#include <Servo.h>

int buttonLastState;
int buttonCurrentState;
int direction;
Servo myservo;
Servo myservo2;
int counter = 0;
int currentStateCLK;
int lastStateCLK;
String currentDir = "";
unsigned long lastButtonPress = 0;
bool hasBeenPressed;

void setup() {
  //main settup
  // Set encoder pins as inputs
  pinMode(CLK, INPUT);
  pinMode(DT, INPUT);
  pinMode(SW, INPUT_PULLUP);

  myservo.attach(9, 500, 2500);
  myservo.write(90);
  myservo2.attach(10, 500, 2500);
  myservo2.write(90);
  direction = 90;

  // Setup Serial Monitor
  Serial.begin(9600);

  // Read the initial state of CLK
  lastStateCLK = digitalRead(CLK);
}

void loop() {

  // Read the current state of CLK
  currentStateCLK = digitalRead(CLK);

  // If last and current state of CLK are different, then pulse occurred
  // React to only 1 state change to avoid double count
  if (currentStateCLK != lastStateCLK && currentStateCLK == 1) {

    // If the DT state is different than the CLK state then
    // the encoder is rotating CCW so decrement
    if (digitalRead(DT) != currentStateCLK) {
      counter--;
      currentDir = "CCW";
    } else {
      // Encoder is rotating CW so increment
      counter++;
      currentDir = "CW";
    }

    Serial.println("Direction: ");
    Serial.println(currentDir);
    Serial.println(" | Counter: ");
    Serial.println(counter);
  }

  // Remember last CLK state
  lastStateCLK = currentStateCLK;

  // Read the button state
  int btnState = digitalRead(SW);

  buttonCurrentState = digitalRead(SW);
  //If we detect LOW signal, button is pressed
  if (btnState == LOW) {
    //if 50ms have passed since last LOW pulse, it means that the
    //button has been pressed, released and pressed again
    if (buttonCurrentState != buttonLastState) {
      if (buttonCurrentState == LOW) {
        hasBeenPressed = !hasBeenPressed;
      } else hasBeenPressed = hasBeenPressed;
    }
  }
  if (hasBeenPressed == true) {
    myservo.write(150);
  } else {
    myservo.write(90);
  }
  


  // Remember last button press event


  buttonLastState = buttonCurrentState;
  // Put in a slight delay to help debounce the reading
  delay(1);

  myservo2.write(90 + direction);

  if (counter > 0) {
    direction = 50;
  } else
    (direction = -50);
  if (counter == 0) {
    direction = 0;
  }
  if (btnState == LOW) {
    myservo2.write(91);

    if (btnState == LOW)
      ;
    myservo2.write(90);
  }
}