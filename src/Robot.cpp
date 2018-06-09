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
	frc::DriverStation::ReportWarning("START INIT");

	//set motors to follower mode
	drive.Left_Back.Set(ctre::phoenix::motorcontrol::ControlMode::Follower, drive.Left_FrontID);
	drive.Right_Back.Set(ctre::phoenix::motorcontrol::ControlMode::Follower, drive.Right_FrontID);

	//select primary closed loop
    drive.Left_Front.SelectProfileSlot(0, 0);
    drive.Right_Front.SelectProfileSlot(0, 0);

    //set the peak current to 30 amps because we have 30 amp breakers
    drive.Left_Front.ConfigPeakCurrentLimit(30, 0);
    drive.Left_Back.ConfigPeakCurrentLimit(30, 0);
    drive.Right_Front.ConfigPeakCurrentLimit(30, 0);
    drive.Right_Back.ConfigPeakCurrentLimit(30, 0);

    //set the sensors to QuadEncoders
	drive.Right_Front.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::QuadEncoder, 0, 0);
	drive.Left_Front.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::QuadEncoder, 0, 0);

	//set the encoder value to 0
	drive.Left_Front.SetSelectedSensorPosition(0, 0, 0);
	drive.Right_Front.SetSelectedSensorPosition(0, 0, 0);

	//invert encoders
	drive.Left_Front.SetSensorPhase(false);
	drive.Right_Front.SetSensorPhase(false);

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
	drive.PIDenable(P, I, D, F);

	//Sets the gyro
	drive.gyro.frc::AnalogGyro::Calibrate();

	//sets the name of the objects
	drive.Left_Front.SetName("Left_Front");
	drive.Left_Back.SetName("Left_Back");
	drive.Right_Front.SetName("Right_Front");
	drive.Right_Back.SetName("Right_Back");

	//reports
	drive.stringConverter << drive.encoder;
	frc::DriverStation::ReportWarning("Encoder: " + drive.stringConverter.str());


	frc::DriverStation::ReportWarning("END INIT");
}

void Robot::AutonomousInit() {

}

void Robot::AutonomousPeriodic() {
	drive.PIDMove(120.0, 0, 18.85, 4.72, 2500, 4.0);

	//tests
	drive.stringConverter << drive.encoder;
	frc::DriverStation::ReportWarning("Encoder: " + drive.stringConverter.str());
	drive.stringConverter << drive.counter.Get();
	frc::DriverStation::ReportWarning("Time: " + drive.stringConverter.str());
	while(drive.ASecond() == false){
		frc::DriverStation::ReportWarning("false");
	}
	drive.stringConverter << drive.gyro.GetAngle();
	frc::DriverStation::ReportWarning("Time: " + drive.stringConverter.str());

	drive.PIDTurn(0.0, 18.85, 4.72, 2500, 4.0, 34, 34, 62, 62, 90);
	drive.Point(90 , 53, 10);
}

void Robot::TeleopInit() {

}

void Robot::TeleopPeriodic() {
	drive.ArcadeDrive(.02, 1);
}

void Robot::TestPeriodic() {}

START_ROBOT_CLASS(Robot)
