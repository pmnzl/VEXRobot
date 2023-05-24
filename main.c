#pragma config(I2C_Usage, I2C1, i2cSensors)
#pragma config(Sensor, in1, lightLeft, sensorReflection)
#pragma config(Sensor, in2, lightMid, sensorReflection)
#pragma config(Sensor, in3, lightRight, sensorReflection)
#pragma config(Sensor, dgtl1, btnStop, sensorTouch)
#pragma config(Sensor, dgtl2, btnStart, sensorTouch)
#pragma config(Sensor, dgtl3, sonar, sensorSONAR_mm)
#pragma config(Sensor, dgtl5, encRight, sensorQuadEncoder)
#pragma config(Sensor, dgtl7, encLeft, sensorQuadEncoder)
#pragma config(Sensor, dgtl9, LED_Right, sensorDigitalOut)
#pragma config(Sensor, dgtl10, LED_Left, sensorDigitalOut)
#pragma config(Sensor, dgtl11, armLimit_low, sensorTouch)
#pragma config(Sensor, dgtl12, armLimit_high, sensorTouch)
#pragma config(Sensor, I2C_1, armEncoder, sensorQuadEncoderOnI2CPort, , AutoAssign )
#pragma config(Motor, port2, motorArm, tmotorVex269_MC29, openLoop, reversed, encoderPort, I2C_1)
#pragma config(Motor, port7, motorRight, tmotorVex393_MC29, openLoop)
#pragma config(Motor, port8, motorLeft, tmotorVex393_MC29, openLoop, reversed)

/* Timers:
T_1	Free
T_2	Free
T_3	Free
T_4	Used by checkButtons() -- DO NOT USE
*/

#include "backgroundProcesses.h"

//Define constants
#define brown 1100		//Light sensor value for brown
#define black 2500		//Light sensor value for black
#define ratio 3/5		//Gear ratio of the driven wheels
#define diameter 103.0		//Diameter of the robot wheels
#define width 114.0		//Width of the half the robot to the centre of a drive wheel
#define basePower 30		//The base value for the drive wheels andalso the arm motor
#define saturatePower 30	//The highest power a wheel can go past the base power
#define linePower 30		//Base power of lineFollow
#define lineSaturate 30		//Saturate for lineFollow
#define armSaturate 60		//Power saturation for the arm motor

//Declare functions
void armAngle(float angle, float Kp);
void drive(float target, float Kp, float Ki, float KpStraight);
void rotate(float angleTarget, float Kp, float Ki);
void lineFollow(float Kp, float Ki);
void waitForStart();
void resetDrive();
void resetArm();
float getPosition();
void sensorRead();
float encoderToAngle(int encoderValue);
void overshootProtection(float error, float previousPower);
float ramp(float power, float count);
float antiWindup(float integral, float error);
float exitCondition(float error, int count);
void blackLineDetection();

//Declare global variables
float left, middle, right;

// *************************** Main ***************************************
task main() {
	//Background Tasks
	startTask(checkArm);       // DO NOT DELETE THIS LINE
	startTask(checkButtons);   // DO NOT DELETE THIS LINE

	//controller gains
	float KpArm = 20;

	float KpDrive = 1.2;
	float KpDriveStraight = 1.5;
	float KiDrive = 0.05;

	float KpRotate = 1.2;
	float KiRotate = 0.05;

	float KpLine = 0.01;
	float KiLine = 0;

	//wait for start
	waitForStart();

	//calibrate robot
	resetDrive();			//stops drive wheels
	armUp(70);				//start with arm up
	resetArm();				//calibrates arm angle
	armAngle(-7, KpArm);	//set robot arm to -7 degrees

	//encoder based navigation
	drive(428, KpDrive, KiDrive, KpDriveStraight);		//drive to can
	armUp(70);											//pick up can

	//drive to target from encoder based navigation
	drive(-120, KpDrive, KiDrive, KpDriveStraight); 	//reversing to avoid red line
	rotate(-90, KpRotate, KiRotate);					//rotate ccw
	drive(410, KpDrive, KiDrive, KpDriveStraight);		//drive to middle non-red zone
	rotate(90, KpRotate, KiRotate);						//rotate cw
	drive(1150, KpDrive, KiDrive, KpDriveStraight); 	//drive through middle non-red zone
	rotate(90, KpRotate, KiRotate);						//rotate cw
	drive(110, KpDrive, KiDrive, KpDriveStraight);		//drive to target

	delay(1000);										//wait for can to settle
	armAngle(-9, KpArm);								//drop off can
	delay(1000);										//make sure robot arm clears can
	drive(-160, KpDrive, KiDrive, KpDriveStraight);		//drive to black line
	rotate(180, KpRotate, KiRotate);					//rotate to face brown line

	blackLineDetection();								//stopping at black line
	lineFollow(KpLine, KiLine);							//line tracking for brown line

	//return to base
	drive(691, KpDrive, KiDrive, KpDriveStraight);		//drive to base from brown line
	armUp(70);											//arm up at end of run

	stopAllTasks();										// end of program - stop everything
}
// *****************************************************************************
// *****************************************************************************

///////////////////////////////////CONTROLLER FUNCTIONS///////////////////////////////////

void armAngle(float angle, float Kp){
	/*The armAngle function is used to precisely position the robot arm angle to the nearest two degrees.
	The angle is controlled by a p controller and a motor encoder connected to the arm.
	Input:	angle target in degrees
		Kp
	Output:	none
	*/

	float currentAngle, error = 0, u = 0;
	int armPower;

	//p controller
	do {
		currentAngle = encoderToAngle(readSensor(EncoderArm)); //uses encoder to angle function to convert encoder counts into degrees
		error = angle - currentAngle;
		u = Kp * error;

		armPower = saturate(u, -armSaturate, armSaturate);
		motorPower(ArmMotor, armPower);

		delay(25);

	} while (abs(error)> 2); //controller exits when the arm is within two degrees of the target

	motorPower(ArmMotor, 0); //resets the arm power after controller exits
}

void drive(float target, float Kp, float Ki, float KpStraight){
	/*The drive function is used to precisely place the robot a set distance in a straight line.
	The angle is controlled by a pi controller and motor controllers connected to both drive wheels.
	Input:	target distance in mm
		Kp
		Ki
		Kp for driving straight
	Output:	none
	*/

	float currentPosition, error = 0, integral = 0, u = 0;
	int encoderError = 0, dU = 0, power, count, exit = 0;

	resetDrive();		//resets the drive wheels
	resetTimer(T_1);	//reset timer used for the ramp function

	//pi controller
	do {
		currentPosition = getPosition();		//averages left and right encoder values
		error = target - currentPosition;
		integral = integral + error;
		integral = antiWindup(integral, error);	//antiwindup to prevent windup of integral

		u = Kp * error + Ki * integral;
		power = saturate(u, -saturatePower, saturatePower);
		count = readTimer(T_1);
		power = ramp(power, count);				//ramps from zero to full power in a second

		encoderError = readSensor(RightEnc) - readSensor(LeftEnc);
		dU = KpStraight * encoderError;			//allows the robot to drive straight

		motorPower(LeftMotor, power + dU);
		motorPower(RightMotor, power - dU);

		exit = exitCondition(error, exit);		//checks that steady state has occured

		delay(25);

	} while(exit < 10);	//exit condition
	resetDrive();		//resets the power to the drive wheels
}

void rotate(float angleTarget, float Kp, float Ki){
	/*The rotate function is used to rotate the robot about the centre of the rear axle.
	The rotation is controlled by a pi controller and motor encoders connected both drive wheels.
	Input:	rotate target in degrees (-ve for CCW)
		Kp
		Ki
	Output:	none
	*/

	float currentPosition, arcTarget, error = 0, integral = 0, u = 0;
	int power, count, exit = 0;

	resetDrive();		//resets the drive wheels
	resetTimer(T_1);	//reset timer used for the ramp function

	arcTarget = (angleTarget * (PI/180.0) * width); //converts angle in degrees to arc distance

	//pi controller
	do {
		currentPosition = getPosition(); //average distance between left and right wheel
		error = arcTarget - currentPosition;
		integral = integral + error;
		integral = antiWindup(integral, error); //prevents integral windup

		u = Kp * error + Ki * integral;
		power = saturate(u, -saturatePower, saturatePower);
		count = readTimer(T_1);
		power = ramp(power, count); //ramp the begining of the movement to prevent differences in acceleration
		//SEND POWER TO THE MOTORS
		motorPower(LeftMotor, power);
		motorPower(RightMotor, -power);

		exit = exitCondition(error, exit); //checks exit conditon

		delay(25);

	} while(exit < 15); //exit condition
	resetDrive(); //stops the drive wheels
}

void lineFollow(float Kp, float Ki) {
	/*The lineFollow function is used to align the robot to a line.
	The drive power is controlled by a pi controller and 3 light sensors at the front of the robot.
	Input:	Kp
		Ki
	Output:	none
	*/

	float lineError = 0, middleError = 0, integral = 0, u = 0;
	int power = 0;

	//pi controller
	do {
		sensorRead();
		lineError = left - right;
		middleError = brown - middle;

		overshootProtection(middleError, 10 * power); //when robot cannot detect the line go back to the last execution when line was visible
		integral = integral + lineError;
		integral = antiWindup(integral, lineError); //prevents integral windup

		u = Kp * lineError + Ki * integral;
		power = saturate(u, -lineSaturate, lineSaturate);
		motorPower(LeftMotor, linePower - power);
		motorPower(RightMotor, linePower + power);

		delay(25);

	} while ((left < black) || (middle < black) || (right < black)); //exits controller when a black line is seen by any light sensor
	resetDrive();
}

///////////////////////////////////HELPER FUNCTIONS///////////////////////////////////

void waitForStart() {
	/*waitForStart detects a button press and delays any furhter code for 0.5s after execution.
	Input:	none
	Output:	none
	*/

	while (readSensor(StartButton) == 0){
		delay(50);
	}
	delay(500);
}

void resetDrive(){
	/*resetDrive stops power from going to the drive wheels and resets the drive wheel encoders.
	Input:	none
	Output:	none
	*/

	motorPower(LeftMotor, 0);
	motorPower(RightMotor, 0);
	setSensor(LeftEnc,0);
	setSensor(RightEnc,0);
}

void resetArm(){
	/*resetArm stops power from going to the arm and resets the arm encoder value.
	Input:	none
	Output:	none
	*/

	armDown(70);
	motorPower(ArmMotor, 0);
	setSensor(EncoderArm,0);
}

float getPosition(){
	/*getPosition converts left motor encoder count to a distance in mm.
	Input:	none
	Output:	distance in mm
	*/

	float encoderCount = (abs(readSensor(LeftEnc)) + abs(readSensor(RightEnc))) / 2.0;
	if (readSensor(LeftEnc) < 0){
		encoderCount = encoderCount * -1;
	}
	float distance = encoderCount*(PI/360.0)*ratio*diameter; //accounts for wheel diameter and gear ratio when coverting encoder values to distance
	return distance;
}

void sensorRead() {
	/*sensorRead reads the current value being read by each of the three light sensors.
	Input:	none
	Output:	none
	*/

	left = readSensor(LeftLight);
	middle = readSensor(MidLight);
	right = readSensor(RightLight);
}

float encoderToAngle(int encoderValue){
	/*encoderToAngle converts encoder counts to an arm angle value.
	Input:	encoder value
	Output:	angle value in degrees
	*/

	float angle;
	angle = (encoderValue - 306.76) / 14.359; //based on a linear trend of the encoder sensor vs angle
	return angle;
}

void overshootProtection(float error, float previousPower) {
	/*overshootProtection is used by the lineFollow function to prevent overshoot.
	Input:	middle light sensor error
		previous correction power sent to the drive motors
	Output:	none
	*/

	previousPower = saturate(previousPower, -50, 50);
	if (error > 200){
		do {
			sensorRead();
			motorPower(LeftMotor, - previousPower);
			motorPower(RightMotor, + previousPower);
		} while (middle < brown);
	}
}

float ramp(float power, float count){
	/*ramp is used by drive and rotate functions to try and reduce the difference in the drive wheel acceleration.
	Input:	drive motor power
		time passed since ramp started
	Output:	drive motor power
	*/

	if (count < 500){
		power = (float)(count/500)*(power);
	}
	return power;
}

float antiWindup(float integral, float error) {
	/*antiWindup is used in all the pi controllers to prevent the windup of the integral term in the controller.
	Input:	integral value
		current error
	Output:	intergral value
	*/

	if (abs(error) > 20) {
		integral = 0;
	}
	return integral;
}

float exitCondition(float error, int count) {
	/*exitCondition is used to exit the pi controllers of drive and rotate to make sure the controllers have reached steady state.
	Input:	error
		count of consecutive errors close to zero
	Output:	count of consecutive errors close to zero
	*/

	if (abs(error) <= 5) {
		count++;
	}
	else {
		count = 0;
	}
	return count;
}

void blackLineDetection() {
	/*blackLineDetection is used to stop on a black line and stay stationary for one second.
	Input:	error
					count of consecutive errors close to zero
	Output:	count of consecutive errors close to zero
	*/

	//drive robot straight until a black line is found
	do {
		motorPower(LeftMotor, 30);
		motorPower(RightMotor, 30);
		delay(25);
		sensorRead();
	} while ((left < black) || (middle < black) || (right < black)); //exits loop when a light sensor sees black
	resetDrive(); //stops robot
	delay(1000); //stop for one second
}
