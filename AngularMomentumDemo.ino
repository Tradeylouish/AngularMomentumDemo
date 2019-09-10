
#include <AccelStepper.h>
#include <Streaming.h>
#include <RunningMedian.h>
#include <MovingAverage.h>



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
#define EXP_MOVING_AVG_ON true
#define MOTOR_SPEED 1500
#define PRINT_PERIOD 200 // Too low a period and stepper control is impacted
#define FILTER_LENGTH 4 // Higher means better resolution but more phase delay

// Physical constants
#define DISK_SLOTS 36
#define DISTANCE_MIN 0.03 // in metres
#define DISTANCE_MAX 0.159 // in metres
#define STEP_MAX 5180

// Global variables
unsigned int encoderCount = 0;
RunningMedian samples = RunningMedian(FILTER_LENGTH);
MovingAverage samples2 = MovingAverage(FILTER_LENGTH);

void setup() {
    // Initialize serial port
    Serial.begin(115200); // Faster rate to reduce impact on stepper motor

    // Setup potentiometer as input
    pinMode(ENCODER,INPUT);
    pinMode(POSITION_POT,INPUT);
    pinMode(LIMIT_SWITCH, INPUT_PULLUP);
    
    // Set the maximum speed in steps per second (equal to the default speed):
    stepper.setMaxSpeed(MOTOR_SPEED);
}

void loop() {
    
    int stepperSpeed = getUserInput(); // Get user input from joystick to determine speed

    stepperSpeed = applyLimits(stepperSpeed); // Apply limit switch logic

    countEncoderEdges(); // Read the encoder

    // Run the motor
    stepper.setSpeed(stepperSpeed);
    stepper.runSpeed();
    
    // Prevent the reported position from going negative
    if (stepper.currentPosition() < 0) stepper.setCurrentPosition(0);

    // Print data at regular interval
    printData();
}

int getUserInput() {
    // Read movement command from potentiometer
    int potentiometerReading = analogRead(POSITION_POT);

    // Set speed based on potentiometer
    if (potentiometerReading < 256) {
        // Move inwards
        return -MOTOR_SPEED;
    } else if (potentiometerReading > 768) {
        // Move outwards
        return MOTOR_SPEED;
    }
    return 0;
}

int applyLimits(int stepperSpeed) {
    static bool calibrated = false;
    
    // Reset step count when limit switch is hit and complete initial calibration
    if (digitalRead(LIMIT_SWITCH)) {
        calibrated = true;
        stepper.setCurrentPosition(0);
        stepperSpeed = max(0, stepperSpeed); // Prevent inwards movement at limit by zeroing negative speeds
    } else if (!calibrated) {
        stepperSpeed = -MOTOR_SPEED; // Automatically calibrate on startup by moving to the centre
    }

    // Software limit switch
    if (stepper.currentPosition() >= STEP_MAX) {
        stepperSpeed = min(0, stepperSpeed); // Prevent outwards movement at limit by zeroing positive speeds
    }
    
    return stepperSpeed;
}

void countEncoderEdges() {
    // Count encoder rising edges
    bool encoderState = digitalRead(ENCODER);
    static bool lastEncoder = digitalRead(ENCODER); // Read the initial state of the encoder on startup
    if (encoderState && (lastEncoder == 0)) {
        encoderCount++;
    }
    lastEncoder = encoderState;
}

float movingAverage() {
    if (EXP_MOVING_AVG_ON) {
        return samples2.update(encoderCount);
    }
    
    samples.add(encoderCount);
    return samples.getAverage();
}

void printData() {
    // Prints data if enough time has passed
    static unsigned long lastPrintTime = millis();

    if (millis() - lastPrintTime >= PRINT_PERIOD) {
        lastPrintTime = millis();
        
        // Calculate speed using Simple Moving Average filter on recent encoder counts, convert to rad/s
        float rotationSpeed = (movingAverage()/DISK_SLOTS)*2*PI*(1000/PRINT_PERIOD);
        // Calculate distance using linear mapping of steps to distance boundaries
        float distance = (DISTANCE_MAX-DISTANCE_MIN)*float(stepper.currentPosition())/STEP_MAX + DISTANCE_MIN;
        
        // Print data using Streaming.h syntax, limit distance to mm precision
        Serial << millis() << "," << rotationSpeed << "," << _FLOAT(distance, 3) << endl;
        
        encoderCount = 0;
    }
}
