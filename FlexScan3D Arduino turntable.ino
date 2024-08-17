#include <AccelStepper.h>

int stepsPerRevolution = 200, pos = 0;
const int microsteps = 32; // Microstepping setting

#define DIR_PIN 2
#define STEP_PIN 5
#define ENABLE_PIN 8

AccelStepper stepper1(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
boolean stepper1Dir = false; //False = Clockwise, True = CCW
boolean stepper2Dir = false; //False = Clockwise, True = CCW
boolean stepper3Dir = false; //False = Clockwise, True = CCW
boolean stepper4Dir = false; //False = Clockwise, True = CCW

int scancount = 0; // Number of scans already done
int angle = 22.5;
int max_scan = 360 / angle;

const float gearRatio = 3.23;

int degreeToSteps(float degree) {
  return (int)(degree * (stepsPerRevolution * microsteps * gearRatio / 360.0));
}

////////////////////////////////////////
// Move the motors
////////////////////////////////////////

void moveStepper(int motorID, float degree) {
  int steps = degreeToSteps(degree);

  if (motorID == 1 && stepper1Dir == true) steps *= -1; // Set motor 1 to CW or CCW
  if (motorID == 2 && stepper2Dir == true) steps *= -1; // Set motor 1 to CW or CCW
  if (motorID == 3 && stepper3Dir == true) steps *= -1; // Set motor 1 to CW or CCW
  if (motorID == 4 && stepper4Dir == true) steps *= -1; // Set motor 1 to CW or CCW
  
  if (motorID == 1) {
    stepper1.move(steps);
  } 
  
  // Placeholder for motor 2
  else if (motorID == 2) {
    // Add logic if motor 2 is connected in the future
  } 
  // Placeholder for motor 3
  else if (motorID == 3) {
    // Add logic if motor 3 is connected in the future
  } 
  // Placeholder for motor 4
  else if (motorID == 4) {
    // Add logic if motor 4 is connected in the future
  }
  
  while (stepper1.distanceToGo() != 0) {
    digitalWrite(ENABLE_PIN, LOW); // Enable the motor
    stepper1.run();
  }
  digitalWrite(ENABLE_PIN, HIGH); // Disable the motor
}

void setMotorSpeed(int motor, int speed) {
  float maxSpeed = speed * 1.0; // Convert speed value to float for calculation
  if (motor == 1) {
    stepper1.setMaxSpeed(maxSpeed * microsteps);
  } else if (motor == 2) {
//    stepper2.setMaxSpeed(maxSpeed * microsteps);
  } else if (motor == 3) {
//    stepper3.setMaxSpeed(maxSpeed * microsteps);
  } else if (motor == 4) {
//    stepper4.setMaxSpeed(maxSpeed * microsteps);
  }
}


void processMoveCommand(int motor, int steps) {
    // Acknowledge the move command
    sendSerialCommand("^B" + String(motor) + String(steps));

    // Move the stepper motor based on the motor ID
    moveStepper(motor, steps);

    // Send a done signal when the movement is complete
    sendSerialCommand("^R" + String(motor));
}

////////////////////////////////////////
// Set motor M to clockwise
////////////////////////////////////////

void setDirectionCW(int motor) {
    if (motor == 1) {
        stepper1Dir = false; //False = Clockwise, True = CCW
    }
    if (motor == 2) {
        stepper2Dir = false; //False = Clockwise, True = CCW
    }
    if (motor == 4) {
        stepper4Dir = false; //False = Clockwise, True = CCW
    }
    if (motor == 4) {
        stepper4Dir = false; //False = Clockwise, True = CCW
    }

    // Placeholder for other motors
    // else if (motor == 2) { stepper2.setPinsInverted(false, false, true); }
    //sendSerialCommand("D" + String(motor) + "CWLO");
    Serial.println("^");
}

////////////////////////////////////////
// Set motor M to counterclockwise
////////////////////////////////////////

void setDirectionCCW(int motor) {
    if (motor == 1) {
        stepper1Dir = true; //False = Clockwise, True = CCW
    }
    if (motor == 2) {
        stepper2Dir = true; //False = Clockwise, True = CCW
    }
    if (motor == 4) {
        stepper4Dir = true; //False = Clockwise, True = CCW
    }
    if (motor == 4) {
        stepper4Dir = true; //False = Clockwise, True = CCW
    }

    // Placeholder for other motors
    // else if (motor == 2) { stepper2.setPinsInverted(false, false, true); }
    //sendSerialCommand("D" + String(motor) + "CWHi");
    Serial.println("^");
}

////////////////////////////////////////
// Set motor M to counterclockwise
////////////////////////////////////////

void haltMotor(int motor) {
    if (motor == 1) {
        stepper1.stop();
        digitalWrite(ENABLE_PIN, HIGH); // Disable the motor
    }
    // Placeholder for other motors
    // else if (motor == 2) { stepper2.stop(); digitalWrite(ENABLE_PIN_2, HIGH); }
    sendSerialCommand("E" + String(motor) + "HALT");
}










void sendSerialCommand(String command) {
  Serial.println(command); // Send command to FlexScan3D
}

String buffer = "";

////////////////////////////////////////
//INIT
////////////////////////////////////////

void setup() {
  Serial.begin(57600);
  pinMode(ENABLE_PIN, OUTPUT);
  
  // Enable the stepper driver
  digitalWrite(ENABLE_PIN, LOW);

  // Set max speed and acceleration
  stepper1.setMaxSpeed(1000 * microsteps);    // Adjust speed for microstepping
  stepper1.setAcceleration(500 * microsteps); // Adjust acceleration for microstepping
  
  // Initialize communication with FlexScan3D
  sendSerialCommand("V"); // Send version command to FlexScan3D
  
  // Wait for acknowledgement or response
  while (Serial.available() == 0) {}
  char response = Serial.read();
}


////////////////////////////////////////
//Respond to "V" with current motor status
////////////////////////////////////////

void sendStatus() {
  String status = "^";
  
  if (stepper1.distanceToGo() == 0) {
    status += "R1";  // Motor 1 is ready
  } else {
    status += "B1" + String(stepper1.distanceToGo(), HEX);
  }
  
  // Placeholder for motor 2
  status += "R2"; // Assuming motor 2 is ready
  // Placeholder for motor 3
  status += "R3"; // Assuming motor 3 is ready
  // Placeholder for motor 4
  status += "R4"; // Assuming motor 4 is ready
  
  Serial.println(status);
}


//Motor numbers are passed as characters but the number of steps and speed are passed as 3 bytes of
// binary for simplicity.
// send: V reply: ^R1R2R3R4
// send: S1M1791 reply: ^191
// sned: D1CWLO reply: ^
// send: I1M100 reply: ^B1100


void loop() { 
    if (Serial.available() > 0) {
        char ch = Serial.read();
        buffer += ch;

        if (buffer.endsWith("\n")) {
            buffer.trim();

      ////////////////////////////////////////
      // Command "V"
      // Request the status of the rotary table. Usual reply would be ^R1R2R3R4 indicating rotary 1 ready, rotary
      // 2 ready, etc. ^B1xxxR2R3R4 means rotary 1 is busy where xxx are 3 bytes indicates how many steps the
      // rotary still has to perform.
      ////////////////////////////////////////
      
            // Check and respond to "V" command with motor status
            if (buffer.startsWith("V")) {
              Serial.println("V recieved " + buffer);
                sendStatus();
            }

            
      ////////////////////////////////////////
      // Command: "SmMxxx"
      // Sets the speed of the motor m to xxx, where xxx is a 3 bytes of data indicating the speed. Example code:
      // port.Write("S1M" + (char)0 + (char)6 + (char)255); // set motor 1 to speed 1791. The standard speed
      // range of our rotary table is: 0x000001 to 0x0012FF (1 to 4863). Controller will respond with ^mxx
      // mirroring the motor number and 2 last bytes of speed setting.
     ////////////////////////////////////////

            if (buffer.startsWith("S"))  {
              
              Serial.println("S recieved " + buffer + "Len " + buffer.length() );
                int motor = buffer.substring(1, 2).toInt(); // Motor ID
                int speed = ((unsigned char)buffer[4] << 8) | (unsigned char)buffer[5]; // Combine the last 2 bytes to form speed

                setMotorSpeed(motor, speed); // Call the function to set speed
        
                // Respond with mirrored motor number and last 2 bytes of speed setting
                sendSerialCommand("^" + String(motor) + buffer.substring(4));
             }
      
      ////////////////////////////////////////
      // Command: "ImMxxx"
      // Turns motor m xxx number of steps. Controller will acknowledge with ^Bmxxx.
      ////////////////////////////////////////
             
             if (buffer.startsWith("I")) {
              Serial.println("I recieved " + buffer);
                int motor = buffer.substring(1, 2).toInt();
                int steps = buffer.substring(3).toInt();
                processMoveCommand(motor, steps);
             }
            
      ////////////////////////////////////////
      // Command: "DmCWLO"
      // Set motor number m to rotate clockwise. Each consecutive command to rotate the motor m will rotate it
      // clockwise.
      ////////////////////////////////////////

            if (buffer.startsWith("D") && buffer.endsWith("CWLO")) {
                          Serial.println("DxCLO recieved " + buffer);
                int motor = buffer.substring(1, 2).toInt();
                setDirectionCW(motor);
            }
            
      ////////////////////////////////////////
      // Command: "DmCWHi"
      // Sets rotary m to rotate counterclockwise.
      ////////////////////////////////////////
      
            if (buffer.startsWith("D") && buffer.endsWith("CWHi")) {
              Serial.println("DxCHi recieved " + buffer);
                int motor = buffer.substring(1, 2).toInt();
                setDirectionCCW(motor);
            }

      ////////////////////////////////////////
      // Command: "EmHALT"
      // Rotary m stop.
      ////////////////////////////////////////

            if (buffer.startsWith("E") && buffer.endsWith("HALT")) {
              Serial.println("EHALT recieved " + buffer);
                int motor = buffer.substring(1, 2).toInt();
                haltMotor(motor);
            }


            buffer = ""; // Clear buffer after processing
        }
    }
}
