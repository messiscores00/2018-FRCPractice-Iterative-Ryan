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
	//set motors to follower mode
	drive.Left_Back.Set(ctre::phoenix::motorcontrol::ControlMode::Follower, drive.Left_FrontID);
	drive.Right_Back.Set(ctre::phoenix::motorcontrol::ControlMode::Follower, drive.Right_FrontID);

	//select primary closed loop
    drive.Left_Front.SelectProfileSlot(slotIdx, pidIdx);
    drive.Right_Front.SelectProfileSlot(slotIdx, pidIdx);

    //set the peak current to 30 amps because we have 30 amp breakers
    drive.Left_Front.ConfigPeakCurrentLimit(amps, timeoutMs_ConfigPeakCurrentLimit);
    drive.Left_Back.ConfigPeakCurrentLimit(amps, timeoutMs_ConfigPeakCurrentLimit);
    drive.Right_Front.ConfigPeakCurrentLimit(amps, timeoutMs_ConfigPeakCurrentLimit);
    drive.Right_Back.ConfigPeakCurrentLimit(amps, timeoutMs_ConfigPeakCurrentLimit);

    //set the sensors to QuadEncoders
	drive.Right_Back.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::QuadEncoder, pidIdx, timeoutMs_ConfigSelectedFeedbackSensor);
	drive.Left_Front.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::QuadEncoder, pidIdx, timeoutMs_ConfigSelectedFeedbackSensor);

	//set the encoder value to 0
	drive.Left_Front.SetSelectedSensorPosition(sensorPos, pidIdx, timeoutMs_SetSelectedSensorPosition);
	drive.Right_Back.SetSelectedSensorPosition(sensorPos, pidIdx, timeoutMs_SetSelectedSensorPosition);

	//invert encoders
	drive.Left_Front.SetSensorPhase(true);
	drive.Right_Back.SetSensorPhase(false);

	//invert motors
	drive.Left_Front.SetInverted(false);
	drive.Right_Front.SetInverted(false);
	drive.Left_Back.SetInverted(false);
	drive.Right_Back.SetInverted(false);

	//set the neutral Mode to brake
	drive.Left_Front.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
	drive.Right_Front.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
	drive.Left_Back.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
	drive.Right_Back.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);

	//sets the PIDS
	//drive.PIDenable(P, I, D, F);

	//Sets the gyro
	drive.gyro.Reset();

	//sets the name of the objects
	drive.Left_Front.SetName("Left_Front");
	drive.Left_Back.SetName("Left_Back");
	drive.Right_Front.SetName("Right_Front");
	drive.Right_Back.SetName("Right_Back");


	//reports
	frc::DriverStation::ReportWarning("Encoder: " + std::to_string(drive.encoder()));
	//frc::DriverStation::ReportWarning("Gyro: " +  std::to_string(drive.gyro.GetAngle()));

}

void Robot::AutonomousInit() {


}

void Robot::AutonomousPeriodic() {
	frc::DriverStation::ReportWarning("Encoder: " + std::to_string(drive.encoder()));
	drive.PIDMove(120.0, 0, 18.85, 4.72, 25000000, 4.0);
	//drive.PIDTurn(0.0, 18.85, 4.72, 2500, 4.0, 34, 34, 62, 62, 90);
	//drive.Point(90 , .3, 10); DONE

}

void Robot::TeleopInit() {
	drive.xbox1.SetYChannel(1);
	drive.xbox1.SetXChannel(4);
}

void Robot::TeleopPeriodic() {
	drive.ArcadeDrive(.02, drive.xbox1.GetY(frc::GenericHID::JoystickHand::kLeftHand), -1 * drive.xbox1.GetX(frc::GenericHID::JoystickHand::kLeftHand) , true);
}

void Robot::TestPeriodic() {}

START_ROBOT_CLASS(Robot)
