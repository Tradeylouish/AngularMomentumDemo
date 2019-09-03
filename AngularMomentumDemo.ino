
#include <AccelStepper.h>
#include <Streaming.h>
#include <RunningMedian.h>

// Define stepper motor connections and motor interface type. Motor interface type must be set to 1 when using a driver:
#define dirPin 4
#define stepPin 5
#define motorInterfaceType 1

// Create a new instance of the AccelStepper class:
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);

// Input pins
#define POSITION_POT A0
#define ENCODER 2
#define LIMIT_SWITCH 6

// Adjustable parameters
#define MOTOR_SPEED 1500
#define PRINT_PERIOD 200
#define FILTER_LENGTH 5

// Physical constants
#define DISK_SLOTS 36
#define DISTANCE_MIN 0.03
#define DISTANCE_MAX 0.159
#define STEP_MAX 5180

// Variables
unsigned int encoderCount = 0;
bool calibrated = false;

int stepCount = 0;
unsigned long printTimer = millis();
RunningMedian samples = RunningMedian(FILTER_LENGTH);

void setup() {
    // Initialize serial port
    Serial.begin(115200);

    // Setup potentiometer as input
    pinMode(ENCODER,INPUT);
    pinMode(POSITION_POT,INPUT);
    pinMode(LIMIT_SWITCH, INPUT_PULLUP);
    
    // Set the maximum speed in steps per second (equal to the default speed):
    stepper.setMaxSpeed(MOTOR_SPEED);
}

void loop() {
    // Read movement command from potentiometer
    int x = analogRead(POSITION_POT);
    //Serial.println(x);

    int stepperSpeed = 0;

    // Set speed based on potentiometer
    if (x < 500 || !calibrated) {
        // Move inwards
        stepperSpeed = -MOTOR_SPEED;
    } else if (x > 550) {
        // Move outwards
        stepperSpeed = MOTOR_SPEED;
    }
    
    // Reset step count when limit switch is hit (
    if (digitalRead(LIMIT_SWITCH)) {
        calibrated = true;
        stepCount = 0;
        stepper.setCurrentPosition(0);
        stepperSpeed = max(0, stepperSpeed);
    }

    // Software limit switch
    if (stepCount >= STEP_MAX) {
        stepperSpeed = min(0, stepperSpeed);
    }

    // Count encoder rising edges
    bool encoderState = digitalRead(ENCODER);
    static bool lastEncoder = 0;
    if (encoderState && (lastEncoder == 0)) {
        encoderCount++;
    }
    lastEncoder = encoderState;

    // Run the motor
    //Serial.print(stepperSpeed);
    stepper.setSpeed(stepperSpeed);
    stepper.runSpeed();
    stepCount = stepper.currentPosition();

    // Print data at regular interval
    if (millis() - printTimer >= PRINT_PERIOD) {
        printTimer = millis();
        printData();
    }
}

void printData() {
    samples.add(encoderCount);
    
    // Perform unit conversion
    float rotationSpeed = (samples.getAverage()/DISK_SLOTS)*2*PI*(1000/PRINT_PERIOD);
    
    float distance = (DISTANCE_MAX-DISTANCE_MIN)*float(stepCount)/STEP_MAX + DISTANCE_MIN;

    // Print data
    Serial << millis() << "," << rotationSpeed << "," << _FLOAT(distance, 3) << endl;

    encoderCount = 0;
}
