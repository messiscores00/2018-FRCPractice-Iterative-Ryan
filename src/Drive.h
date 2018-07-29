/*
 * Drive.h
 *
 *  Created on: May 30, 2018
 *      Author: Ryan Sheehy
 */

#ifndef DRIVE_H_
#define DRIVE_H_

#include <Joystick.h>
#include <ctre/phoenix.h>
#include <Drive/DifferentialDrive.h>
#include <iostream>
#include <DriverStation.h>
#include <Counter.h>
#include <Encoder.h>
#include <time.h>
#include <math.h>
#include <AnalogGyro.h>
#include <Timer.h>
#include <AHRS.h>

class Drive {
public:
	//functions
	Drive();
	virtual ~Drive();
	void PIDenable(int P, int I, int D, int F);
	void PIDdisable();
	void ArcadeDrive(double deadzone, double xSpeed, double zRotation, bool squaredInputs, double Leftsensitivity, double Rightsensitivity);
	//deadzone between 0 and 1.00
	//sensitivity between 0 and 1.00
	void PIDMove(double Dtot, double Vf_at_end, double CoW, double acceleration, int timeout, double MaxVelocity);
	void PIDTurn(double Vf_at_end, double CoW, double accelerationLeft, double  accelerationRight, int timeout, double MaxVelocity, double a_left, double b_left, double a_right, double b_right, double angle);
	bool ASecond();
	void Point(int angle, double sensitivity, double deadzone);
	//sensitivity = how fast you want to turn. value in ticks 53 ticks = one degree
	//deadzone in degrees of deadzone
	//angle left is negative right is positive
	double encoder();
	double encoderLeft();
	double encoderRight();
	double Vp();
	double VpLeft();
	double VpRight();

	//variables
	int Left_FrontID = 3;
	int Right_FrontID = 4;
	int Left_BackID = 2;
	int Right_BackID = 5;
	int P = 1;
	int I = 0;
	int D = 8;
	int F = 0;

	//objects
	ctre::phoenix::motorcontrol::can::WPI_TalonSRX Left_Front{Left_FrontID};
	ctre::phoenix::motorcontrol::can::WPI_TalonSRX Right_Front{Right_FrontID};
	ctre::phoenix::motorcontrol::can::WPI_TalonSRX Left_Back{Left_BackID};
	ctre::phoenix::motorcontrol::can::WPI_TalonSRX Right_Back{Right_BackID};
	frc::Joystick xbox1{0};
	frc::Timer counter{};
	AHRS gyro{frc::SPI::Port::kMXP};

};

#endif /* DRIVE_H_ */
