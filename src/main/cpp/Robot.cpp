#include "Robot.h"

#include <iostream>
#include <stdlib.h>

#include <frc/Timer.h>
#include <frc/I2C.h>
#include <frc/shuffleboard/shuffleboard.h>
#include <frc/DigitalInput.h>
#include <frc/AnalogInput.h>
#include <frc/DoubleSolenoid.h>
#include <frc/Encoder.h>
#include <CameraServer/CameraServer.h>
#include <cscore.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include "ctre/Phoenix.h"

Robot::Robot() {
	
}

const double THRESHOLD = 0.1;


frc::Joystick stick0{0};
bool prevTrigger0 = false;//For camera toggle
frc::Joystick stick1{1};
bool prevTrigger1 = false;//Unused toggle
bool prev3 = false;//Puhsy toggle
bool prev5 = false;//Tilty toggle

bool manualOverride = false;//Manual arm control override

TalonSRX driveLF = {0};//Mecanum Left Front
TalonSRX driveLR = {1};//Mecanum Left Rear
TalonSRX driveRF = {2};//Mecanum Right Front
TalonSRX driveRR = {3};//Mecanum Right Rear
TalonSRX lift1 = {4};//Lift Master
TalonSRX lift2 = {5};//Lift Slave
TalonSRX arm1 = {6};//Arm Master
VictorSPX arm2 = {7};//Arm Slave
VictorSPX intake = {8};//Intake
//TalonSRX climber1 = {9};//Climber Master
//VictorSPX climber2 = {10};//Climber Slave

frc::DoubleSolenoid pushyBoi(0, 1);//Disc Scoring Cylinder
frc::DoubleSolenoid tiltyBoi(2, 3);//Disc Tilting Cylinder

//frc::I2C I2Channel(frc::I2C::Port::kOnboard, 0x08);//I2C for communicating with LED arduino
frc::I2C I2CChannel(frc::I2C::kOnboard, 0x08);


frc::DigitalInput limAMin(1);//Limit switch for arm minimum rotation
frc::DigitalInput limAMax(2);//Limit switch for arm maximum rotation
frc::DigitalInput limLTop(3);//Limit switch lift top
frc::DigitalInput limLBot(4);//Limit switch lift bottom

frc::AnalogInput potArm(0);//Potentiometer for arm rotation

cs::UsbCamera camera1;//Front Camera
//cs::UsbCamera camera2;//Rear Camera
cs::VideoSink server;


nt::NetworkTableEntry Bearing;
nt::NetworkTableEntry Displacement;

//Mecanum Output Variables
double LF_Out;
double LR_Out;
double RF_Out;
double RR_Out;

double armLast;//For position holding
int manipPosition = 0;

void WriteArduino(uint8_t data){
  I2CChannel.Write(0x08, data);
  std::cout << "Sent" << data << std::endl;
}

void Robot::RobotInit() {
	//Motor Configurations
	driveLF.SetInverted(false);
	driveLF.SetNeutralMode(Brake);
	driveLR.SetInverted(true);
	driveLR.SetNeutralMode(Brake);
	driveRF.SetInverted(true);
	driveRF.SetNeutralMode(Brake);
	driveRR.SetInverted(true);
	driveRR.SetNeutralMode(Brake);
	arm1.SetInverted(true);
	arm1.SetNeutralMode(Brake);
	arm2.SetInverted(false);
	arm2.SetNeutralMode(Brake);
	lift1.SetInverted(true);
	lift1.SetNeutralMode(Brake);
	lift2.SetInverted(false);
	lift2.SetNeutralMode(Brake);
	intake.SetInverted(false);
	intake.SetNeutralMode(Brake);
	//climber1.SetInverted(false);
	//climber1.SetNeutralMode(Brake);
	//climber2.SetInverted(true);
	//climber2.SetNeutralMode(Brake);

	//lift2.Set(ControlMode::Follower, 4);
	//arm2.Set(ControlMode::Follower, 6);//Victor can't follow Talon
	//climber2.Set(ControlMode::Follower, 9);//Victor can't follow Talon

	//Mecanum encoders, unused ATM
	driveLF.ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 0);
	driveLR.ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 0);
	driveRF.ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 0);
	driveRR.ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 0);

	armLast = potArm.GetVoltage();	

	pushyBoi.Set(frc::DoubleSolenoid::kReverse);//Disc pushing solenoid
	tiltyBoi.Set(frc::DoubleSolenoid::kForward);//Bar Moving solenoid

	//Camera stuff
	camera1 = frc::CameraServer::GetInstance()->StartAutomaticCapture(0);
	//camera2 = frc::CameraServer::GetInstance()->StartAutomaticCapture(1);
	server = frc::CameraServer::GetInstance()->GetServer();

	server.SetSource(camera1);

	WriteArduino(119);

	//Network table stuff
	auto inst = nt::NetworkTableInstance::GetDefault();
	auto table = inst.GetTable("dataTable");
	Bearing = table->GetEntry("Bearing");
	Displacement = table->GetEntry("Displacement");

	printf("Valentia \'Val\' Tereshkova online\n");
}

//Absolute value function for floating point numbers
//Arg input: number to be absolutely valued
double fabs(double input){
	if(input >= 0){
		return input;
	}
	else if(input < 0){
		return -input;
	}
	else{
		std::cout << "Math is broken" << std::endl;
	}
}

//Mapping function to scale values
double map(double inMax, double inMin, double outMax, double outMin, double input){
	double output;

	output = (input - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;

	return output;
}

//Clip function to constrain values
double clip(double max, double min, double input){
	double output;
	if(input >= max){
		output = max;
	}
	else if(input <= min){
		output = min;
	}
	else{
		output = input;
	}
	return output;
}

void setLeds(int mode){
//'r' = 114, 'g' = 103, 'b' = 98, 'u' = 117, 'c' = 99, 'h' = 104, 'o' = 111 'w' = 199
//u = rainbow all , c = flash green at top of elev., h = flash red at top of elev, r= red, g = green, b= blue
// o = all off

  switch(mode){
    case 0:
      WriteArduino(117); //'u'
      break; //Rainbow All Strips
    case 1:
      WriteArduino(99);// 'c'
      break; //Rainbow Elect. Board and Blink Green at top 
    case 2:
      WriteArduino(104);// 'h'
      break; //Rainbow Elect. Board and Blink Red at bottom
    case 3:
      WriteArduino(114);// 'r'
      break; // All Red
    case 4:
      WriteArduino(103); //'g'  
      break; //All Green
    case 5:
      WriteArduino(98);//'b'
      break; //All Blue
    case 6:
      WriteArduino(111); //'o'
      break; //All Off
    case 7:
      WriteArduino(119);
      break; // blue blinking before DS connection
  }
}

//Sets speed for all drive base motors
void setMotors(double LF, double LR, double RF, double RR){
	driveLF.Set(ControlMode::PercentOutput, LF);
	driveLR.Set(ControlMode::PercentOutput, LR);
	driveRF.Set(ControlMode::PercentOutput, RF);
	driveRR.Set(ControlMode::PercentOutput, RR);
}

//Sets speed for all lift motors
void setLift(double speed){
	lift1.Set(ControlMode::PercentOutput, speed);
	lift2.Set(ControlMode::PercentOutput, speed);
}

//Sets speed for all arm motors
void setArm(double speed){
	arm1.Set(ControlMode::PercentOutput, speed);
	arm2.Set(ControlMode::PercentOutput, speed);
}

/*
//Sets speed for all climber motors
void setClimber(double speed){
	climber1.Set(ControlMode::PercentOutput, speed);
	climber2.Set(ControlMode::PercentOutput, speed);
}
*/

void lazyMecanum(){//Working and preffered
	double Y_in = (fabs(stick0.GetY()) > THRESHOLD) ? stick0.GetY() : 0;
	double X_in = (fabs(stick0.GetX()) > THRESHOLD) ? stick0.GetX() : 0;
	double Z_in = (fabs(stick0.GetThrottle()) > THRESHOLD) ? stick0.GetThrottle() : 0;//For some reason the twist axis is getthrottle not gettwist or getz

	LF_Out = X_in - Y_in + Z_in;
	LR_Out = X_in + Y_in - Z_in;
	RF_Out = X_in + Y_in + Z_in;
	RR_Out = X_in - Y_in - Z_in;

	setMotors(LF_Out, LR_Out, RF_Out, RR_Out);
}

/*
void fancyMecanum(){//WIP
	double Magnitude = sqrt((stick0.GetY() * stick0.GetY()) + (stick0.GetX() * stick0.GetX()));
	double Angle = atan(stick0.GetY()/stick0.GetX());
	double Twist = stick0.GetThrottle();

	RR_Out = Magnitude * sin(Angle + (3.14159/4)) - Twist;
	LF_Out = Magnitude * sin(Angle + (3.14159/4)) + Twist;
	LR_Out = Magnitude * cos(Angle + (3.14159/4)) + Twist;
	RF_Out = Magnitude * cos(Angle + (3.14159/4)) - Twist;

	setMotors(LF_Out, LR_Out, RF_Out, RR_Out);
}
*/

/*
void fieldCentricMecanum(){//WIP and no gyro
	double turnTuner = 0;//Adjust to increase the speed of turns
	double X_in = (fabs(stick0.GetX()) > THRESHOLD) ? stick0.GetX() : 0;
	double Y_in = (fabs(stick0.GetY()) > THRESHOLD) ? stick0.GetY() : 0;
	double Z_in = (fabs(stick0.GetThrottle()) > THRESHOLD) ? stick0.GetThrottle() : 0;
	
	double angle = ahrs.GetYaw();//Gyro angle from initial heading
	double theta = angle * (180/3.14159);//Angle but int radians
	double straight = (Y_in * cos(theta)) + (X_in * sin(theta));//Forward/backward vector component
	double strafe =  (-Y_in * sin(theta)) + (X_in * cos(theta));//Left/right vector component
	double spin = turnTuner * Z_in;//Rotational component
	
	//Vector math for each motor
	LF_Out = strafe - straight + spin;
	LR_Out = strafe + straight - spin;
	RF_Out = strafe + straight + spin;
	RR_Out = strafe - straight - spin;

	setMotors(LF_Out, LR_Out, RF_Out, RR_Out);
}
*/

void liftManualControl(){//Manual control for Lift motors
	if(stick1.GetRawButton(4) == true && limLTop.Get() == true){
		setLift(1.0);
	}
	else if(stick1.GetRawButton(6) == true && limLBot.Get() == true){
		setLift(-1.0);
	}
	else{
		setLift(0.0);
	}
}

double prevSP = 0.0;//Previous setpoint use to keep track of when a new PID loop is used
double error;//Error
double preError;//Previous error for use in integral and derivative
double pOut;//Proportional output
double integral;//Integral math variable
double iOut;//Integral output
double derivative;//Derivative math variable
double dOut;//Derivative output
double armPIDOutput;//Total output
double armPID(double input, double P, double I, double D){
	double PV = map(3, 1, 1.0, 0.0, (5.0 - potArm.GetVoltage()));//The current pot voltage, inverted and scaled between 1 and 0
	double SP = map(3, 1, 1.0, 0.0, (5.0 - input));//The setpoint, inverted and scaled between 1 and 0
	
	printf("Setpoint: %f, PV: %f\n", SP, PV);

	error = SP - PV;//Error is the difference between where we are and where we want to be

	if(SP != prevSP){//Reset consistent variables when a new setpoint is called
		prevSP = SP;
		integral = 0;
		derivative = 0;
		preError = 0;
	}

	double dt = 0.02;//Time step

	pOut = P * error;//Proportional output
	integral += (error * dt);//Integral math
	iOut = I * integral;//Integral ouput
	derivative = (error - preError) / dt;//Derivative math
	dOut = D * derivative;//Derivativec ouput
	armPIDOutput = pOut + iOut + dOut;//Total output
	
	printf("Output pre adjust: %f, ", armPIDOutput);
	/*
	armPIDOutput = clip(999.0, -999.0, armPIDOutput);

	printf("Output post clip: %f, ", armPIDOutput);
	*/
	armPIDOutput = map(0.6, -0.6, 10.0, -10.0, armPIDOutput);
	
	printf("Output post	 scale: %f ", armPIDOutput);

	printf("Error: %f\n", error);

	preError = error;//Error propogation
	prevSP = SP;//Setpoint propogation

	return armPIDOutput;
}


void armManualControl(){//Manual control for Arm motors
	if(stick1.GetY() > THRESHOLD  && stick1.GetRawButton(2) == true){
		arm1.Set(ControlMode::PercentOutput, stick1.GetY());
		arm2.Set(ControlMode::PercentOutput, stick1.GetY());
		armLast = potArm.GetVoltage();
	}
	else if(stick1.GetY() < -THRESHOLD && stick1.GetRawButton(2) == true){
		arm1.Set(ControlMode::PercentOutput, stick1.GetY());
		arm2.Set(ControlMode::PercentOutput, stick1.GetY());
		armLast = potArm.GetVoltage();
	}
	else{
		setArm(armPID(armLast, 0.4, 0, 0));
	}
}

double countdown = 0.0;
void intakeControl(){//Control for Intake motor 
	if(stick1.GetRawButton(1) == true){
		intake.Set(ControlMode::PercentOutput, 0.7);
		countdown = 0.7;
	}
	else if(stick1.GetRawButton(7) == true){
		intake.Set(ControlMode::PercentOutput, -0.7
		);
		countdown = 0.7;
	}
	else{ 
		intake.Set(ControlMode::PercentOutput, (0.35));
		//if(countdown > 0.0){
		//	countdown -= 0.02;
		//}
	}
}

/*
void climberControl(){//Control for climber motors
	if(stick1.GetRawButton(5) == true){
		climber1.Set(ControlMode::PercentOutput, 1.0);
	}
	else if(stick1.GetRawButton(6) == true){
		climber1.Set(ControlMode::PercentOutput, -1.0);
	}
	else{
		climber1.Set(ControlMode::PercentOutput, 0.0);
	}
}
*/

void pushy(){
	if(stick1.GetRawButton(3) && !prev3){
		if(pushyBoi.Get() == frc::DoubleSolenoid::kReverse){
			pushyBoi.Set(frc::DoubleSolenoid::kForward);
		}
		else if(pushyBoi.Get() == frc::DoubleSolenoid::kForward){
			pushyBoi.Set(frc::DoubleSolenoid::kReverse);
		}
	}else if (!stick1.GetRawButton(3) && prev3){
		
	}
	prev3 = stick1.GetRawButton(3);
}


void tilty(){
	if(stick1.GetRawButton(5) && !prev5){
		if(tiltyBoi.Get() == frc::DoubleSolenoid::kReverse){
			tiltyBoi.Set(frc::DoubleSolenoid::kForward);
		}
		else if(tiltyBoi.Get() == frc::DoubleSolenoid::kForward){
			tiltyBoi.Set(frc::DoubleSolenoid::kReverse);
		}
	}else if (!stick1.GetRawButton(5) && prev5){
		
	}
	prev5 = stick1.GetRawButton(5);
}

//Min Ref: 3.831787
//0:Starting 
//1:Hatch Load 3.753662 
//2:Hatch score 3.605957 
//3:Ball load 3.743896
//4:Ball Score 3.289795 

void manipulatorControl(){//Button control for the arm and lift
	if(stick1.GetRawButton(2) && !stick1.GetRawButton(1)){//Top
		manipPosition = 3;
	}
	else if(stick1.GetRawButton(6) && !stick1.GetRawButton(1)){//Middle
		manipPosition = 3;
	}
	else if(stick1.GetRawButton(4) && !stick1.GetRawButton(1)){//Bottom
		manipPosition = 4;
	}
	else if(stick1.GetRawButton(2) && stick1.GetRawButton(1)){//Load
		//manipPosition = 1;
	}

	switch(manipPosition){
		case 1://Hatch load
		setArm(armPID(3.753662, 0.1, 0, 0));
		break;
		case 2://Hatch score
		setArm(armPID(3.605957, 0.3, 0, 0));
		break;
		case 3://Cargo load
		setArm(armPID(3.7481685, 0.1, 0, 0));
		break;
		case 4://Cargo score
		setArm(armPID(3.289795, 0.5, 0 , 0));
		break;
		case 0:
		default:
		/*
			if(limLTop.Get() == true){
				lift1.Set(ControlMode::PercentOutput, 0.5);
			}
			else{
				lift1.Set(ControlMode::PercentOutput, 0.0);
			}

			if(limAMin.Get() == true){
				arm1.Set(ControlMode::PercentOutput, -0.5);
			}
			else{
				arm1.Set(ControlMode::PercentOutput, 0.0);
			}
		break;
		*/
		break;
	}
}

/*
void cameraControl(){//Control for Camera switching
	if(stick0.GetTrigger() && !prevTrigger0){
		if(server.GetSource() == camera2){
			printf("Setting camera 2\n");
			server.SetSource(camera1);
		}
		else if(server.GetSource() == camera1){
			printf("Setting camera 1\n");
			server.SetSource(camera2);
		}
	}else if (!stick0.GetTrigger() && prevTrigger0){
		
	}
	prevTrigger0 = stick0.GetTrigger();
}
*/

/*
float encoderValues[] = {0.0, 0.0, 0.0, 0.0};//Current Rev voltage, current number of revs, last rev voltage, last number of revs
void potEncoder(){
	encoderValues[2] = encoderValues[0];
	encoderValues[3] = encoderValues[1];
	encoderValues[0] = potArm.GetVoltage();

	if((encoderValues[0] - encoderValues[2]) < -1){
		encoderValues[1]++;
	}
	else if((encoderValues[0] - encoderValues[2]) > 1){
		encoderValues[1]--;
	}
}
*/

void Robot::Autonomous() {
	while (IsAutonomous() && IsEnabled()) {
		printf("Pot Voltage %f\n", potArm.GetVoltage());

		lazyMecanum();
		liftManualControl();
		//manipulatorControl();
		armManualControl();
		intakeControl();
		//climberControl();
		pushy();
		tilty();
		//cameraControl();
		
		frc::Wait(0.005);// The motors will be updated every 5ms
	}
}

void Robot::OperatorControl() {
	WriteArduino(99);
	while (IsOperatorControl() && IsEnabled()) {
		printf("Pot Voltage %f\n", potArm.GetVoltage());
		//potEncoder();
		
		lazyMecanum();
		//liftManualControl();

		/*
		if(manualOverride == true){
			armManualControl();
		}
		else if(manualOverride == false){
			manipulatorControl();
		}
		*/

		manipulatorControl();
		//armManualControl();
		intakeControl();
		//climberControl();
		pushy();
		tilty();
		//cameraControl();

		//printf("limamax: %i\n", limAMax.Get());		
		//printf("limamin: %i\n", limAMin.Get());		
		//printf("limlbot: %i\n", limLBot.Get());		
		//printf("limltop: %i\n", limLTop.Get());				

		frc::Wait(0.02);// The motors will be updated every 5ms
	}
}


void Robot::Test() {

}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif