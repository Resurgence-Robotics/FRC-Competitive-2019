#include "Robot.h"

#include <iostream>
#include <stdlib.h>

#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "ctre/Phoenix.h"

Robot::Robot() {
	
}

const double THRESHOLD = 0.1;

frc::Joystick stick0{0};
frc::Joystick stick1{1};

TalonSRX driveLF = {12};
TalonSRX driveLR = {13};
TalonSRX driveRF = {4};
TalonSRX driveRR = {6};

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
		double leftStick = -stick0.GetY();
		double rightStick = stick1.GetY();

		if(fabs(leftStick) > THRESHOLD){
			driveLF.Set(ControlMode::PercentOutput, leftStick);
			driveLR.Set(ControlMode::PercentOutput, leftStick);
		}
		else{
			driveLR.Set(ControlMode::PercentOutput, 0);
			driveLF.Set(ControlMode::PercentOutput, 0);
		}
		
		if(fabs(rightStick) > THRESHOLD){
			driveRF.Set(ControlMode::PercentOutput, rightStick);
			driveRR.Set(ControlMode::PercentOutput, rightStick);
		}
		else{
			driveRR.Set(ControlMode::PercentOutput, 0);
			driveRF.Set(ControlMode::PercentOutput, 0);
		}
		
		frc::Wait(0.005);// The motors will be updated every 5ms
	}
}


void Robot::Test() {

}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif