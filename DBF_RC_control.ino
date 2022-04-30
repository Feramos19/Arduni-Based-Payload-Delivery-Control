/*
   NOTES:
   - Use servo.write(0) to move conveyor belt forward
   - Use servo.write(180) to move conveyor belt backwards
   - Use servo.detach to stop conveyor belt - Remember to attach again before starting
     motion again
 */

 /*
  * 
  * ARMING
  * BELT SERVO - PIN 3
  * CHANNEL    - PIN 5
  * SWITCH     - PIN 7
  * DOOR SERVO - PIN 10
  * LOCK SERVO - PIN 12
  */


#include <ezButton.h>
ezButton limitSwitch(7);  // create ezButton object that attach to pin 7;

#include <Servo.h>

// Create objects for servo control:
Servo servo_belt;
Servo servo_door;

// Variables:
int ch1_previous=0;        // Here's where we'll keep our current channel value
int ch2_previous=0;        // Here's where we'll keep our current channel value
int ch1_current;           // Here's where we'll keep the previous channel value
int ch2_current;           // Here's where we'll keep the previous channel value
int Close = 45;            // Angle of servo corresponding to closed door
int Open = 135;            // Angle of servo corresponding to open door
int servoPin_belt = 3;     // Attach conveyor belt servo to pin 3
int servoPin_door = 10;    // Attach door servo to pin 10
int channelPin1 = A0;      // Attach desired channel to activate deployment motion to pin A0
int channelPin2 = A1;      // Attach desired channel to activate load motion to pin A1

void setup() {

  pinMode(channelPin1, INPUT);               // Set channelPin1 as input
  pinMode(channelPin2, INPUT);               // Set channelPin2 as input
  Serial.begin(9600);                        // Start up serial monitor
  limitSwitch.setDebounceTime(50);           // set debounce time to 50 milliseconds
  servo_door.attach(servoPin_door);          // Attach door servo 
  ch1_previous = analogRead(channelPin1);    // Read PWM signal at moment of start
  ch2_previous = analogRead(channelPin2);    // Read PWM signal at moment of start
  // Set closed door
  servo_door.write(Close);
}

void loop() {
  
  int ch1_current = analogRead(channelPin1);     // Read PWM signal
  int ch2_current = analogRead(channelPin2);
  limitSwitch.loop();                            // MUST call the loop() funct before reading limit switch
  int state = limitSwitch.getState();            //gets limit switch state
  Serial.print(ch1_current);                     //print current PWM value
  Serial.print("\t");
  Serial.println(ch2_current);
  
  // DEPLOYING
  if ((abs(ch1_current - ch1_previous) > 200)) 
  {
    Serial.println("JUMP");

    // Open Door:
    servo_door.write(Open);
    delay(1250);                                // Change as neccessary to allow door to fully open
        
    while (state == HIGH)                       // While the limit switch is released
    {
        // Run motor 
        servo_belt.attach(servoPin_belt);
        servo_belt.write(0);
        limitSwitch.loop();
        state = limitSwitch.getState();
        Serial.println("Still in loop");
    }
  
    //Stop motor
    servo_belt.write(90);
    // Move motor backwards now:
    servo_belt.write(180);
    delay(500);
    //Stop motor
    servo_belt.detach(); 
    // Close door
    delay(1000);                                  // Change as neccessary to allow door to fully close
    servo_door.write(Close);
    
  }

  // Open/Close Door
  if ((abs(ch2_current - ch2_previous) > 200)) 
  {
    Serial.println("JUMP - DOOR");

    if( servo_door.read() == Open)
    {
      // Close Door
      servo_door.write(Close);
     }
     else{
      // Open Door:
      servo_door.write(Open);
     }
  }
  
  // Loading cubes
   // Loading cubes
  if (state == LOW && ((abs(ch1_current - ch1_previous) < 150)))
  {
        
        while (state == LOW){
        // Run motor 
        servo_belt.attach(servoPin_belt);
        servo_belt.write(180);
        limitSwitch.loop();
        state = limitSwitch.getState();
        Serial.println("Still in loop 2");
        }
        servo_belt.detach(); //Stop motor
  }
  
  
  ch1_previous = ch1_current;                 // Store channel width for next loop reference
  ch2_previous = ch2_current;                 // Store channel width for next loop reference
  delay(500);                                 // Delay to give enough time for jump signal
}
