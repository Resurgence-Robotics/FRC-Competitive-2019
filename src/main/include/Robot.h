#pragma once

#include <string>

#include <frc/Joystick.h>
#include <frc/PWMVictorSPX.h>
#include <frc/SampleRobot.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/smartdashboard/SendableChooser.h>

class Robot : public frc::SampleRobot {
	public:
		Robot();

		void RobotInit() override;

		void Autonomous() override;
	
		void OperatorControl() override;

		void Test() override;

	private:
		// Robot drive system

		frc::SendableChooser<std::string> m_chooser;
		const std::string kAutoNameDefault = "Default";
		const std::string kAutoNameCustom = "My Auto";
};
