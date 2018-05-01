//MAE 211 Project
//Code by Conor McCormick

#include <QTRSensors.h>
#include <OtherMotorDriver.h>
#include <Ultrasonic.h>

#define RAISE_LIFT 10000
#define LOWER_LIFT -10000
#define CRIT_DIST 18
#define LOAD_TIME 1500
#define Kp 0.1
#define Kd 2
#define SHELF_TIME 700
#define SHELF_DIST 10
#define INITIAL_SPEED 115

//Motor 1
int pol1 = 12;
int pwm1 = 3;
bool rev1 = true;
OtherMotorDriver motor1(pol1, rev1, pwm1);

//Motor 2
int pol2 = 13;
int pwm2 = 11;
bool rev2 = true;
OtherMotorDriver motor2(pol2, rev2, pwm2);

int motorSpeed = INITIAL_SPEED;
int motor1val, motor2val;

//Stepper Motor
//Quick note here, D16-19 is actually A2-5, Analog ports on Arduino can output digital signals
int IN1 = 19; //IN1
int IN2 = 18; //IN2
int IN3 = 17; //IN3
int IN4 = 16; //IN4
int Steps = 0;
bool Direction;
unsigned long last_time;
unsigned long currentMillis;
long time;

//Ultrasonic
int Trig = 9; //Trigger Pin
int Echo = 8; //Echo Pin
Ultrasonic Ultra(Trig, Echo);

//Line Follower Array
unsigned int sensors[5], last_sensors[5];
int distance, lastError, error, line, derivative;
int last_error = 0;
QTRSensorsRC qtr((unsigned char[]) {
	2, 4, 5, 6, 7
}, 5);

unsigned long time1, timer, timer0;
int timer2;

int stage = 1; //sets stage to 1, used to track what the robot is doing

void setup()
{
	Serial.begin(9600); //Serial for debugging

	//Sets stepper motor pins to output
	pinMode(IN1, OUTPUT);
	pinMode(IN2, OUTPUT);
	pinMode(IN3, OUTPUT);
	pinMode(IN4, OUTPUT);

}

void loop()
{
	//current time for any timed stages
	time1 = millis();

	//Read Sensors
	distance = Ultra.distanceRead();
	qtr.read(sensors);

	//filters out reflectance values below 1500 for line following
	for (int i = 0; i < 5; i++) {
		if (sensors[i] < 1500) {
			sensors[i] = 0;
		}
	}

	//Logic for handling different phases using integer stage. Stage = 1 when the bot starts out, 
	//when the block is detected stage = 2, when the block is picked up stage = 3, and when the 
	//block is being placed down, stage = 4.
	if (stage == 1) {
		//Check if distance < 14
		if (distance < 14) {
			motorSpeed = 100;
			timer0 = time1;
			stage = 2;
		}
	}
	else if (stage == 2) {
		//Calculates time from block detection
		//Non-blocking timer
		timer = time1 - timer0;

		//Check if time >= to time delay set
		if (timer >= LOAD_TIME) {
			//Stop motors
			motor1.setMotor(0);
			motor2.setMotor(0);
			delay(100);
			Step(RAISE_LIFT);//Raise the lift
			delay(100);
			stage = 3;
			motorSpeed = INITIAL_SPEED; //Move forward
		}
	}
	else if (stage == 3) {
		//Check that line follower is over a line on ALL sensors. The logic here is weird because the sensor was being buggy.
		//Basically this checks if it is over the end sensors, or if it is over an end sensor and the second sensor in on the opposite side, for both sides.
		if ((sensors[0] > 1500 && sensors[4] > 1500) || (sensors[1] > 1500 && sensors[4] > 1500) || (sensors[0] > 1500 && sensors[3] > 1500)) {
			motor1.setMotor(0); //Stop motors
			motor2.setMotor(0);
			delay(100);
			Step(LOWER_LIFT); //Lower block down
			delay(500);
			motor1.setMotor(-180); //Back up off of the shelf
			motor2.setMotor(-180);
			delay(1000);
			stage = 4; //Stop
		}
	}


	//Serial.print(time1);
	//Serial.print("\t");
	//Serial.print(timer0);
	//Serial.print("\t");
	//Serial.print(timer);
	//Serial.print("\t");
	//Serial.print(distance);
	//Serial.print("\t");
	//Serial.print(motorSpeed);

	//Serial.print(stage);
	//Serial.print("\t");
	//for (int i = 0; i < 5; i++) {
	//	if (i < 4) {
	//		Serial.print(sensors[i]);
	//		Serial.print("\t");
	//	}
	//	else {
	//		Serial.println(sensors[i]);
	//	}
	//}

	error = getLine(sensors); //returns the error from the center sensor

	derivative = error - last_error; //calcualtes the derivative term for the PD controller

	line = Kp * error + Kd * derivative; //Calculates a motor value

	last_error = error; //Sets last error for derivative term next loop

	//Assigns motor values
	motor1val = motorSpeed - line; 
	motor2val = motorSpeed + line;

	//Filters motor values so that any value over 200 = 200. 200 is the max speed we're allowing.
	//Also ensures the robot never goes backwards
	if (motor1val > 200) {
		motor1val = 200;
	}
	if (motor2val > 200) {
		motor2val = 200;
	}
	if (motor1val < 0) {
		motor1val = 0;
	}
	if (motor2val < 0) {
		motor2val = 0;
	}

	//  Serial.print(error);
	//  Serial.print("\t");
	//  Serial.print(derivative);
	//  Serial.print("\t");
	//  Serial.print(line);
	//  Serial.print("\t");
	//  Serial.print(last_error);
	//  Serial.print("\t");
	//  Serial.print(motor2val);
	//  Serial.print("\t");
	//  Serial.println(motor1val);

	//Ensures that the robot won't move no matter what
	if (stage != 4) {
		motor1.setMotor(motor1val);
		motor2.setMotor(motor2val);
	}
	else {
		motor1.setMotor(0);
		motor1.setMotor(0);
	}

	delay(20);
}

//Gets the current line position using the line follower. Takes the sensors array as a parameter.
//The output is an integer, giving the error from the center sensor.
int getLine(unsigned int sensors[]) {

	//In order to take a weighted average for our line-follower, integer multiplication must be done using long
	long int num;
	long int den;

	//Takes the weighted average about the end sensor
	for (int i = 0; i < 5; i++) {
		num += (long)sensors[i] * i * 1000;
		den += (long)sensors[i];
	}
	
	//Shifts the average to the center sensor
	int err = (num / den) - (long)2000;

	//Tracks the line even when the line exceeds the bounds of the sensors. Uses global int last_error
	//in order to find where the line was the last iteration. Ensures that the robot will always come back to the line.
	if (sensors[0] == 0 && sensors[1] == 0 && sensors[2] == 0 && sensors[3] == 0 && sensors[4] == 0) {
		if (last_error > 0) {
			err = 2000;
		}
		else {
			err = -2000;
		}
	}

	return err;

}

//Step function for stepping the motor a certain number of steps. This is a blocking function
//The number of steps is taken as an argument. The sign is used to determine direction. The motor is then stepped
//using the functionality provided in the code below this function (void stepper()).
void Step(int steps) {

	int steps_left;

	if (steps > 0) {
		steps_left = steps;
		Direction = true;
	}
	else {
		steps_left = abs(steps);
		Direction = false;
	}

	while (steps_left > 0) {
		currentMillis = micros();
		if (currentMillis - last_time >= 1000) {
			stepper(1);
			time = time + micros() - last_time;
			last_time = micros();
			steps_left--;
		}
	}

}

/*
  BYJ48 Stepper motor code
  Connect :
  IN1 >> D8
  IN2 >> D9
  IN3 >> D10
  IN4 >> D11
  VCC ... 5V Prefer to use external 5V Source
  Gnd
  written By :Mohannad Rawashdeh
  https://www.instructables.com/member/Mohannad+Rawashdeh/
  28/9/2013
*/

void stepper(int xw) {
	for (int x = 0; x < xw; x++) {
		switch (Steps) {
		case 0:
			digitalWrite(IN1, LOW);
			digitalWrite(IN2, LOW);
			digitalWrite(IN3, LOW);
			digitalWrite(IN4, HIGH);
			break;
		case 1:
			digitalWrite(IN1, LOW);
			digitalWrite(IN2, LOW);
			digitalWrite(IN3, HIGH);
			digitalWrite(IN4, HIGH);
			break;
		case 2:
			digitalWrite(IN1, LOW);
			digitalWrite(IN2, LOW);
			digitalWrite(IN3, HIGH);
			digitalWrite(IN4, LOW);
			break;
		case 3:
			digitalWrite(IN1, LOW);
			digitalWrite(IN2, HIGH);
			digitalWrite(IN3, HIGH);
			digitalWrite(IN4, LOW);
			break;
		case 4:
			digitalWrite(IN1, LOW);
			digitalWrite(IN2, HIGH);
			digitalWrite(IN3, LOW);
			digitalWrite(IN4, LOW);
			break;
		case 5:
			digitalWrite(IN1, HIGH);
			digitalWrite(IN2, HIGH);
			digitalWrite(IN3, LOW);
			digitalWrite(IN4, LOW);
			break;
		case 6:
			digitalWrite(IN1, HIGH);
			digitalWrite(IN2, LOW);
			digitalWrite(IN3, LOW);
			digitalWrite(IN4, LOW);
			break;
		case 7:
			digitalWrite(IN1, HIGH);
			digitalWrite(IN2, LOW);
			digitalWrite(IN3, LOW);
			digitalWrite(IN4, HIGH);
			break;
		default:
			digitalWrite(IN1, LOW);
			digitalWrite(IN2, LOW);
			digitalWrite(IN3, LOW);
			digitalWrite(IN4, LOW);
			break;
		}
		SetDirection();
	}
}

void SetDirection() {
	if (Direction == 1) {
		Steps++;
	}
	if (Direction == 0) {
		Steps--;
	}
	if (Steps > 7) {
		Steps = 0;
	}
	if (Steps < 0) {
		Steps = 7;
	}
}
