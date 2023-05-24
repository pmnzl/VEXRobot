#pragma systemFile

#define printf writeDebugStreamLine

// Predefined arm movement code
int arm_State = 0;				// a global variable that defines the state of the robot arm (-1 at lower limit, 1 at upper limit, and 0 in between)
int checkArmRunning = 0;	// a global status variable used to indicate whether the checkArm task is running or not

void armUp(float percentPower);
void armDown(float percentPower);

// This task should always be running in parallel to the main task.
// It constantly monitors the arm limit switches and stops the arm
// motor if it's commanded to move in the direction where a limit switch
// is activated.
task checkArm(){
	checkArmRunning = 1;
	int arm_motorPwr;
	while(1){
		arm_motorPwr = motor[motorArm];	// Check what the motor is doing
		if(SensorValue[armLimit_low]){	// Check lower limit switch
			arm_State = -1;			// update arm_State (-1 = at lower limit)
			if (arm_motorPwr <0){motor[motorArm] = 0;}	// stop if commanded to lower the arm
		}
		else{
			if (SensorValue[armLimit_high]) {	// Check upper limit switch
				arm_State = 1;	// update arm_State (1 = at upper limit)
				if (arm_motorPwr >0){motor[motorArm] = 0;}	// Stop if commanded to raise the arm
			}
			else {	// arm is in between the limit switches
				arm_State = 0; // update arm_State (0 = somewhere between the limits)
			}
		}
		wait1Msec(20); // repeat this loop approx every 20 ms (50Hz)
	}
}

task armReset(){
	armUp(50);
	SensorValue [armEncoder] = 0;
	motor[motorArm]= -40;
	while(SensorValue [armEncoder]>-250 ){
		delay(20);
	}
	motor[motorArm]= 0;
	SensorValue[LED_Left] = 1;
	SensorValue[LED_Right] = 1;
	stopAllTasks();
}


/* 	This task constantly runs in the background and checks the Stop button.
If the Stop button is pressed while the robot was moving it will stop everything.
If the Stop button was pressed and held for a certain time and the robot was stationary,
then it will raise the arm up to about 30 degrees using timers.
*/
task checkButtons() {
	int counter;
	while(1) {

		if(SensorValue[btnStop]){ // if stop button was pressed
			// if the robot is not moving, then wait to see how long the user pressed the button.
			//Otherwise, stop immediately.
			if (motor[motorArm]==0 && motor[motorLeft]==0 && motor[motorRight]==0){
				counter = 0;
				clearTimer(T4);
				while(time1(T4)<1500){	// 1.5 second timer to measure how long the stop button was pressed during that interval
					if(SensorValue[btnStop]){++counter;}
					if((time1(T4)/100)%2==0){
						SensorValue[LED_Left] = 1;
						SensorValue[LED_Right] = 1;
					}
					else{
						SensorValue[LED_Left] = 0;
						SensorValue[LED_Right] = 0;
					}
					delay(1);
				}
				//printf("counter = %d, timer: %d",counter, time1[T3]);
				if(counter>1150){
					startTask(armReset);
					counter = 0;
					clearTimer(T4);// to allow for button debounce otherwise the arm will stop
					while(time1(T4)<500 && counter<20){
						if(SensorValue[btnStop]==0){++counter;} 	// count when not pressed
						delay(1);
					}// end of while
				}// end of if(counter>1150)
				else{
					SensorValue[LED_Left] = 1;
					SensorValue[LED_Right] = 1;
					stopAllTasks();
				}
			}//end of if (motor[motorArm]==0 && motor[motorLeft]==0 && motor[motorRight]==0)
			else{ // stop immediately if any of the motors is being used.
				SensorValue[LED_Left] = 1;
				SensorValue[LED_Right] = 1;
				stopAllTasks();
			}
		}// end of if(SensorValue[btnStop]==1)
		delay(50);
	}// end of while(1)
}



// Function for raising the arm all the way up until it hits the top limit switch
// Input is from 0 to 100 percent of motor power
void armUp (float percentPower){
	// Check if checkArm is running. if not, start it.
	if(!checkArmRunning	){startTask(checkArm);}
	// use absolute to prevent moving in the wrong direction
	int absPower = abs(round(percentPower/100.0*127));
	// Saturation
	if (absPower > 127) {
		absPower = 127;
	}
	motor[motorArm] = absPower;
	while(arm_State<1){}
	motor[motorArm] = 0; // should be redundant but just to be safe.
	wait1Msec(1000);
}

// Function for lowering the arm all the way down until it hits the bottom limit switch
// Input is from 0 to 100 percent of motor power
void armDown (float percentPower){
	// Check if checkArm is running. if not, start it.
	if(!checkArmRunning	){startTask(checkArm);}
	// use absolute to prevent moving in the wrong direction
	int absPower = abs(round(percentPower/100.0*127));
	// Saturation
	if (absPower > 127) {
		absPower = 127;
	}
	motor[motorArm] = -absPower;
	while(arm_State>-1){}
	motor[motorArm] = 0; // should be redundant but just to be safe.
	wait1Msec(1000);
}


//macros for SENSORS
#define MidLight	 	0
#define LeftLight 		1
#define RightLight	 	2
#define StartButton 	3	// select (dgtl 2)
#define StopButton 		4	// confirm (dgtl 1)
#define RightEnc 		5
#define LeftEnc 		6
#define LowArmLimit 	7
#define HighArmLimit 	8
#define EncoderArm		9
#define sonarSensor		10
#define rightLED		11
#define leftLED			12

//macros for MOTORS
#define LeftMotor 		0
#define RightMotor 		1
#define ArmMotor 		2

//macros Timers
#define T_1 0 //T1
#define T_2 1 //T2
#define T_3 2 //T3
#define T_4 3 //T4

//macros For Tasks to be used on RobotC similar to how they are used on PROS
#define Task_1 Task1
#define Task_2 Task2
#define Task_3 Task3

//Function prototypes
void motorPower(int motorSelect,int power);//set motor power
int getMotorPower(int motorSelect);//read motor power value
int readSensor(int sensorSelect);//read sensor value
void setSensor(int sensorSelect, int sValue);//set or reset sensor/component
void resetTimer(int timerName);//resetting timer
int readTimer(int timerName);//reading timer time
float saturate (float input, float lower, float upper);//saturates the power sent to motors

//MOTORS - Setting Power
void motorPower(int motorSelect,int power)
{
	//saturate the power being sent to the motors using saturate function
	int outPower = (int) saturate(power,-127,127);

	//switch case statement checking which motor is selected
	switch (motorSelect){
	case 0:
		motor[motorLeft] = outPower; //sets Left Motor to power level
		break;
	case 1:
		motor[motorRight] = outPower; //sets Right Motor to power level
		break;
	case 2: //Arm motor, first checking limit switches
		if (SensorValue[armLimit_low] && power < 0){ //if low limit
			motor[motorArm] = 0; //Stop Arm
		}
		else if (SensorValue[armLimit_high] && power > 0){ // if high limit
			motor[motorArm] = 0; //Stop Arm
		}
		else
			motor[motorArm] = outPower; //sets Arm Motor to power level
		break;
	default:
		writeDebugStreamLine("Unrecognised Motor Selected"); //invalid motor/input selected
		break;
	}
}
//MOTORS - Getting Power
int getMotorPower(int motorSelect)
{
	int powerValue;
	//switch case statement checking which motor is selected
	switch (motorSelect){
	case 0:
		powerValue = motor[motorLeft]; //reads the Left Motor power value
		break;
	case 1:
		powerValue = motor[motorRight]; //reads the Right Motor power value
		break;
	case 2:
		powerValue = motor[motorArm]; //reads the Arm Motor power value
		break;
	default:
		writeDebugStreamLine("Unrecognised Motor Selected"); //invalid motor/input selected
		break;
	}
	return powerValue; // return the motor power
}

//SENSORS - Reading Sensor Values
int readSensor(int sensorSelect)
{
	int result;
	//switch case statement checking which sensor is selected
	switch (sensorSelect){
	case 0:
		result = SensorValue [ lightMid ]; //read Middle Light Sensor
		break;
	case 1:
		result = SensorValue [ lightLeft ]; //read Left Light Sensor
		break;
	case 2:
		result = SensorValue [ lightRight ]; //read Right Light Sensor
		break;
	case 3:
		result = SensorValue [ btnStart ]; //read Start Button Touch Sensor
		break;
	case 4:
		result = SensorValue [ btnStop ]; //read Stop Button Touch Sensor
		break;
	case 5:
		result = SensorValue [ encRight ]; //read Right wheel Encoder
		break;
	case 6:
		result = SensorValue [ encLeft ]; //read Left wheel Encoder
		break;
	case 7:
		result = SensorValue [ armLimit_low ]; //read Lower Limit Switch on Arm
		break;
	case 8:
		result = SensorValue [ armLimit_high ]; //read Higher Limit Switch on Arm
		break;
	case 9:
		result = SensorValue [ armEncoder ]; //read Integrated Motor Encoder on Arm motor
		break;
	case 10:
		result = SensorValue [ sonar ]; //read ultrasonic sensor
		break;
	case 11:
		result =	SensorValue [ LED_Right ]; //read LED value for Right LED
		break;
	case 12:
		result =	SensorValue [ LED_Left ]; //read LED value for Left LED
		break;
	default:
		writeDebugStreamLine("Unrecognised Sensor Selected"); //invalid sensor selected
		break;
	}
	return result; //return the sensor reading
}

//SENSORS - Setting Sensor Values
void setSensor(int sensorSelect, int sValue)
{
	//switch case statement checking which sensor is selected
	switch (sensorSelect){
	case 5:
		SensorValue [ encRight ] = sValue; //Set/Reset Right Encoder
		break;
	case 6:
		SensorValue [ encLeft ] = sValue; //Set/Reset Left Encoder
		break;
	case 9:
		SensorValue [ armEncoder ] = sValue; //Set/Reset Arm Motor Encoder
		break;
	case 11:
		SensorValue [ LED_Right ] = sValue; //Set Right LED
		break;
	case 12:
		SensorValue [ LED_Left ] = sValue; //Set Left LED
		break;
	default:
		writeDebugStreamLine("Unrecognised Sensor Selected"); //invalid sensor selected (some sensors cannot be 'set')
		break;
	}
}

//RESET TIMERS
void resetTimer(int timerName)
{
	//switch case statement checking which Timer is selected
	switch (timerName){
	case 0:
		clearTimer(T1);//T1 is reset
		break;
	case 1:
		clearTimer(T2);//T2 is reset
		break;
	case 2:
		clearTimer(T3);//T3 is reset
		break;
	case 3:
		clearTimer(T4);//T4 is reset
		break;
	default:
		writeDebugStreamLine("Unrecognised Timer Selected"); //Invalid timer selected
		break;
	}
}

// GET TIMER COUNT
int readTimer(int timerName)
{
	//switch case statement checking which Timer is selected
	int timereturn;
	switch (timerName){
	case 0:
		timereturn = time1[T1];//returns value of T1
		break;
	case 1:
		timereturn = time1[T2];//returns value of T2
		break;
	case 2:
		timereturn = time1[T3];//returns value of T3
		break;
	case 3:
		timereturn = time1[T4];//returns value of T4
		break;
	default:
		writeDebugStreamLine("Unrecognised Timer Selected"); //Invalid timer selected
		break;
	}
	return timereturn; // return the count of the selected timer
}

// This function saturates an input number to be between the lower and upper limits defined by the user.
float saturate (float input, float lower, float upper){

	if (input > upper){ //if input is greater than upper limit, return upper limit
		return upper;
	}
	else if (input < lower){ //if input is less than lower limit, return lower limit
		return lower;
	}
	return input;
}
