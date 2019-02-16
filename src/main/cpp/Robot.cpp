#include "Robot.h"

#include <iostream>
#include <stdlib.h>

#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DigitalInput.h>
#include <frc/AnalogInput.h>
#include <frc/DoubleSolenoid.h>
#include "ctre/Phoenix.h"

Robot::Robot() {
	
}

const double THRESHOLD = 0.1;
const double WHEEL_RADIUS = 3;
const double WHEEL_SEPERATION_WIDTH = 23/2;
const double WHEEL_SEPERATION_LENGTH = 27/2;

frc::Joystick stick0{0};
frc::Joystick stick1{1};

TalonSRX driveLF = {0};//Mecanum Left Front
TalonSRX driveLR = {1};//Mecanum Left Rear
TalonSRX driveRF = {2};//Mecanum Right Front
TalonSRX driveRR = {3};//Mecanum Right Rear
TalonSRX lift1 = {4};//Lift Master
VictorSPX lift2 = {5};//Lift Slave
TalonSRX arm1 = {6};//Arm Master
VictorSPX arm2 = {7};//Arm Slave
VictorSPX intake = {8};//Intake
TalonSRX climber1 = {9};//Climber Master
VictorSPX climber2 = {10};//Climber Slave

frc::DoubleSolenoid pushyBoi1(0, 1);//Disc Scoring Cylinder
frc::DoubleSolenoid pushyBoi2(2, 3);//Disc Scoring Cylinder

frc::DigitalInput limLTop(0);//Limit switch lift top
frc::DigitalInput limLBot(1);//Limit switch lift bottom
frc::DigitalInput limAMax(2);//Limit switch arm max
frc::DigitalInput limAMin(3);//Limit switch arm min

frc::AnalogInput potArm(0);//Potentiometer for arm rotation

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
	driveLF.SetInverted(false);
	driveLF.SetNeutralMode(Brake);
	driveLR.SetInverted(true);
	driveLR.SetNeutralMode(Brake);
	driveRF.SetInverted(false);
	driveRF.SetNeutralMode(Brake);
	driveRR.SetInverted(true);
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
		double Z_in = (fabs(stick0.GetThrottle()) > THRESHOLD) ? stick0.GetThrottle() : 0;//For some reason the twist axis is getthrottle not gettwist or getz

		LF_Out = X_in - Y_in + Z_in;
		LR_Out = X_in + Y_in - Z_in;
		RF_Out = X_in + Y_in + Z_in;
		RR_Out = X_in - Y_in - Z_in;
		/*
		//Fancy Mecanum
		double Magnitude = sqrt((stick0.GetY() * stick0.GetY()) + (stick0.GetX() * stick0.GetX()));
		double Angle = atan(stick0.GetY()/stick0.GetX());
		double Twist = stick0.GetThrottle();

		RR_Out = Magnitude * sin(Angle + (3.14159/4)) - Twist;
		LF_Out = Magnitude * sin(Angle + (3.14159/4)) + Twist;
		LR_Out = Magnitude * cos(Angle + (3.14159/4)) + Twist;
		RF_Out = Magnitude * cos(Angle + (3.14159/4)) - Twist;
		*/

		printf("X in: %f, Y in: %f, Z in : %f\n", X_in, Y_in, Z_in);

		
		driveLF.Set(ControlMode::PercentOutput, LF_Out);
		driveLR.Set(ControlMode::PercentOutput, LR_Out);
		driveRF.Set(ControlMode::PercentOutput, RF_Out);
		driveRR.Set(ControlMode::PercentOutput, RR_Out);
		
		if(stick1.GetRawButton(3) == true && limLTop.Get() == false){
			lift1.Set(ControlMode::PercentOutput, 1.0);
		}
		else if(stick1.GetRawButton(4) == true && limLBot.Get() == false){
			lift1.Set(ControlMode::PercentOutput, -1.0);
		}
		else{
			lift1.Set(ControlMode::PercentOutput, 0.3);
		}

		if(stick1.GetY() > THRESHOLD && limAMax.Get() == false){
			arm1.Set(ControlMode::PercentOutput, stick1.GetY());
		}
		else if(stick1.GetY() < -THRESHOLD && limAMin.Get() == false){
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