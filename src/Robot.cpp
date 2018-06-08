/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

/*
 *
 *
 *
 * CHECK DRIVE.H FOR WHAT THE ARGUMENTS SHOULD BE INPUTED AS!!!!!!
 *
 *
 *
 *
 */

void Robot::RobotInit() {
	drive.Left_Back.Set(ctre::phoenix::motorcontrol::ControlMode::Follower, drive.Left_FrontID);
	drive.Right_Back.Set(ctre::phoenix::motorcontrol::ControlMode::Follower, drive.Right_FrontID);

	drive.Left_Front.SetSensorPhase(false);
	drive.Right_Front.SetSensorPhase(false);

	drive.Left_Front.SetInverted(false);
	drive.Right_Front.SetInverted(false);
	drive.Left_Back.SetInverted(false);
	drive.Right_Back.SetInverted(false);

	drive.Left_Front.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
	drive.Right_Front.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
	drive.Left_Back.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
	drive.Right_Back.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);

	drive.PIDenable(P, I, D, F);

	drive.gyro.frc::AnalogGyro::Calibrate();
}

void Robot::AutonomousInit() {

}

void Robot::AutonomousPeriodic() {
	drive.PIDMove(120.0, 0, 18.85, 4.72, 2500, 4.0);
	drive.PIDTurn(0.0, 18.85, 4.72, 2500, 4.0, 34, 34, 62, 62, 90);
	drive.Point(90 , .5, 10);
}

void Robot::TeleopInit() {

}

void Robot::TeleopPeriodic() {
	drive.ArcadeDrive(.02, 1);
}

void Robot::TestPeriodic() {}

START_ROBOT_CLASS(Robot)
