#include <Adafruit_MotorShield.h>
#include <CmdBuffer.hpp>
#include <CmdCallback.hpp>
#include <CmdParser.hpp>

#define LEFT_SENSOR A0
#define RIGHT_SENSOR A1

CmdCallback<3> cmdCallback;
CmdBuffer<32> cmdBuffer;
CmdParser cmdParser;

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Adafruit_DCMotor *leftMotor = AFMS.getMotor(2);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(1);

void readSensor(CmdParser *cmdParser) {  // "readSensor LEFT|RIGHT"
    if (cmdParser->getParamCount() != 2) {
        Serial.println("wrong number of parameters!");
    }

    char *sensorParam = cmdParser->getCmdParam(1);
    if(strcmp(sensorParam, "LEFT") == 0) {
        Serial.println("LEFTSENSOR="+analogRead(LEFT_SENSOR));
    } else if (strcmp(sensorParam,  "RIGHT") == 0) {
        Serial.println("RIGHTSENSOR="+analogRead(RIGHT_SENSOR));
    }
}

void setState(CmdParser *cmdParser) {    // "setState LEFT|RIGHT FORWARD|BACKWARD|RELEASE"
    if (cmdParser->getParamCount() != 3) return;

    char *motorParam = cmdParser->getCmdParam(1);
    char *modeParam = cmdParser->getCmdParam(2);
    
    Adafruit_DCMotor *motor = NULL;
    int mode;

    if(strcmp(motorParam, "LEFT") == 0) {
        motor = leftMotor;
    } else if (strcmp(motorParam, "RIGHT") == 0) {
        motor = rightMotor;
    } else return;

    if(strcmp(modeParam, "FORWARD") == 0) {
        mode = FORWARD;
    } else if (strcmp(modeParam, "BACKWARD") == 0) {
        mode = BACKWARD;
    } else if (strcmp(modeParam, "RELEASE") == 0) {
        mode = RELEASE;
    } else return;

    motor->run(mode);
}

void setSpeed(CmdParser *cmdParser) {  // "setSpeed LEFT|RIGHT 0-255"
    if (cmdParser->getParamCount() != 3) return;

    char *motorParam = cmdParser->getCmdParam(1);
    String speed = cmdParser->getCmdParam(2);

    Adafruit_DCMotor *motor = NULL;

    if(strcmp(motorParam, "LEFT") == 0) {
        motor = leftMotor;
    } else if (strcmp(motorParam, "RIGHT") == 0) {
        motor = rightMotor;
    } else return;

    uint8_t numSpeed = speed.toInt();
    if (numSpeed < 0 || numSpeed > 255) {
        return;
    }

    motor->setSpeed(numSpeed);
}

void setup() {
    Serial.begin(115200);

    if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
        // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
        Serial.println("Could not find Motor Shield. Check wiring.");
        while (1);
    }

    Serial.println("Motor Shield found.");

    leftMotor->setSpeed(0);
    leftMotor->run(RELEASE);
    rightMotor->setSpeed(0);
    rightMotor->run(RELEASE);

    cmdCallback.addCmd("readSensor", &readSensor);
    cmdCallback.addCmd("setState", &setState);
    cmdCallback.addCmd("setSpeed", &setSpeed);
}

void loop() {
    // Check for new char on serial and call function if command was entered
    cmdCallback.loopCmdProcessing(&cmdParser, &cmdBuffer, &Serial);
}