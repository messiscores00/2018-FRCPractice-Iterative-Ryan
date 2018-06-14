/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>
#include <IterativeRobot.h>
#include <SmartDashboard/SendableChooser.h>
#include <ctre/Phoenix.h>
#include <Drive.h>
#include <iostream>
#include <SmartDashboard/SmartDashboard.h>
#include <DriverStation.h>

class Robot : public frc::IterativeRobot {
public:
	void RobotInit() override;
	void AutonomousInit() override;
	void AutonomousPeriodic() override;
	void TeleopInit() override;
	void TeleopPeriodic() override;
	void TestPeriodic() override;


private:
	Drive drive{};
	int P = 1;
	int I = 0;
	int D = 0;
	int F = 0;
	int slotIdx = 0;
	int pidIdx = 0;
	int sensorPos = 0;
	int amps = 30;
	int timeoutMs_ConfigPeakCurrentLimit = 0;
	int timeoutMs_ConfigSelectedFeedbackSensor = 0;
	int timeoutMs_SetSelectedSensorPosition = 0;
};
