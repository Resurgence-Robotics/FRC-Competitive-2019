#include "Robot.h"

#include <iostream>
#include <stdlib.h>

#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "ctre/Phoenix.h"

Robot::Robot() {
	
}

const double THRESHOLD = 0.1;
const double WHEEL_RADIUS = 3;
const double WHEEL_SEPERATION_WIDTH = 23/2;
const double WHEEL_SEPERATION_LENGTH = 27/2;

frc::Joystick stick0{0};
frc::Joystick stick1{1};

TalonSRX driveLF = {12};
TalonSRX driveLR = {13};
TalonSRX driveRF = {4};
TalonSRX driveRR = {6};

double LF_Out;
double LR_Out;
double RF_Out;
double RR_Out;

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

		frc::Wait(0.005);// The motors will be updated every 5ms
	}
}


void Robot::Test() {

}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif