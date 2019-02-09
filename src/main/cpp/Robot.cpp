#include "Robot.h"

#include <iostream>
#include <stdlib.h>

#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DigitalInput.h>
#include <frc/AnalogInput.h>
#include "ctre/Phoenix.h"

Robot::Robot() {
	
}

const double THRESHOLD = 0.1;
const double WHEEL_RADIUS = 3;
const double WHEEL_SEPERATION_WIDTH = 23/2;
const double WHEEL_SEPERATION_LENGTH = 27/2;

frc::Joystick stick0{0};
frc::Joystick stick1{1};

TalonSRX driveLF = {1};//Mecanum Left Front
TalonSRX driveLR = {2};//Mecanum Left Rear
TalonSRX driveRF = {3};//Mecanum Right Front
TalonSRX driveRR = {4};//Mecanum Right Rear
TalonSRX lift1 = {5};//Lift Master
VictorSPX lift2 = {6};//Lift Slave
TalonSRX arm1 = {7};//Arm Master
VictorSPX arm2 = {8};//Arm Slave
VictorSPX intake = {9};//Intake

frc::DigitalInput LimLTop(0);//Limit switch lift top
frc::DigitalInput LimLBot(1);//Limit switch lift bottom
frc::DigitalInput LimAMax(2);//Limit switch arm max
frc::DigitalInput LimAMin(3);//Limit switch arm min

frc::AnalogInput PotArm(0);//Potentiometer for arm rotation

//Mecanum Output Variables
double LF_Out;
double LR_Out;
double RF_Out;
double RR_Out;


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

void Robot::RobotInit() {
	//Motor Configurations
	driveLF.SetInverted(true);
	driveLF.SetNeutralMode(Brake);
	driveLR.SetInverted(true);
	driveLR.SetNeutralMode(Brake);
	driveRF.SetInverted(false);
	driveRF.SetNeutralMode(Brake);
	driveRR.SetInverted(false);
	driveRR.SetNeutralMode(Brake);
	arm1.SetInverted(false);
	arm1.SetNeutralMode(Brake);
	arm2.SetInverted(true);
	arm2.SetNeutralMode(Brake);
	lift1.SetInverted(false);
	lift1.SetNeutralMode(Brake);
	lift2.SetInverted(true);
	lift2.SetNeutralMode(Brake);
	intake.SetInverted(false);
	intake.SetNeutralMode(Brake);

	lift2.Set(ControlMode::Follower, 5);
	arm2.Set(ControlMode::Follower, 7);


	m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
	m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
	frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
}

void Robot::Autonomous() {
	std::string autoSelected = m_chooser.GetSelected();	
	
	std::cout << "Auto selected: " << autoSelected << std::endl;	
	
	if (autoSelected == kAutoNameCustom) {
		std::cout << "Running custom Autonomous" << std::endl;
	}
	else {
		std::cout << "Running default Autonomous" << std::endl;
	}
}


void Robot::OperatorControl() {
	while (IsOperatorControl() && IsEnabled()) {
		//Lazy mecanum
		double Y_in = (fabs(stick0.GetY()) > THRESHOLD) ? stick0.GetY() : 0;
		double X_in = (fabs(stick0.GetX()) > THRESHOLD) ? stick0.GetX() : 0;
		double Z_in = (fabs(stick0.GetZ()) > THRESHOLD) ? stick0.GetZ() : 0;

		LF_Out = (1/WHEEL_RADIUS) * (X_in - Y_in - (WHEEL_SEPERATION_LENGTH + WHEEL_SEPERATION_WIDTH) * Z_in);
		LR_Out = (1/WHEEL_RADIUS) * (X_in + Y_in - (WHEEL_SEPERATION_LENGTH + WHEEL_SEPERATION_WIDTH) * Z_in);
		RF_Out = (1/WHEEL_RADIUS) * (X_in + Y_in + (WHEEL_SEPERATION_LENGTH + WHEEL_SEPERATION_WIDTH) * Z_in);
		RR_Out = (1/WHEEL_RADIUS) * (X_in - Y_in + (WHEEL_SEPERATION_LENGTH + WHEEL_SEPERATION_WIDTH) * Z_in);
		
		//Fancy Mecanum
		double Magnitude = sqrt((stick0.GetY() * stick0.GetY()) + (stick0.GetX() * stick0.GetX()));
		double Angle = atan(stick0.GetY()/stick0.GetX());
		double Twist = stick0.GetZ();

		RR_Out = Magnitude * sin(Angle + (3.14159/4)) - Twist;
		LF_Out = Magnitude * sin(Angle + (3.14159/4)) + Twist;
		LR_Out = Magnitude * cos(Angle + (3.14159/4)) + Twist;
		RF_Out = Magnitude * cos(Angle + (3.14159/4)) - Twist;
		
		driveLF.Set(ControlMode::PercentOutput, LF_Out);
		driveLR.Set(ControlMode::PercentOutput, LR_Out);
		driveRF.Set(ControlMode::PercentOutput, RF_Out);
		driveRR.Set(ControlMode::PercentOutput, RR_Out);

		if(stick1.GetRawButton(3) == true && LimLTop.Get() == false){
			lift1.Set(ControlMode::PercentOutput, 1.0);
		}
		else if(stick1.GetRawButton(4) == true && LimLBot.Get() == false){
			lift1.Set(ControlMode::PercentOutput, -1.0);
		}
		else{
			lift1.Set(ControlMode::PercentOutput, 0.3);
		}

		if(stick1.GetY() > THRESHOLD && LimAMax.Get() == false){
			arm1.Set(ControlMode::PercentOutput, stick1.GetY());
		}
		else if(stick1.GetY() < -THRESHOLD && LimAMin.Get() == false){
			arm1.Set(ControlMode::PercentOutput, stick1.GetY());
		}
		else{
			arm1.Set(ControlMode::PercentOutput, 0.3);
		}

		if(stick1.GetRawButton(1) == true){
			intake.Set(ControlMode::PercentOutput, 1.0);
		}
		else if(stick1.GetRawButton(2) == true){
			intake.Set(ControlMode::PercentOutput, -1.0);
		}
		else{
			intake.Set(ControlMode::PercentOutput, 0.0);
		}

		frc::Wait(0.005);// The motors will be updated every 5ms
	}
}


void Robot::Test() {

}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif