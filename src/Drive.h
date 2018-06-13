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
	void ArcadeDrive(double deadzone, double xSpeed, double zRotation, bool squaredInputs);
	//deadzone between 0 and 1.00
	//sensitivity between 0 and 1.00
	void PIDMove(double Dtot, double Vf_at_end, double CoW, double acceleration, int timeout, double sensitivity);
		//U = initial velocity
		//Dtot = total distance in inches
		//Vf_at_end = final velocity in ft/sec
		//CoW = circumference of Wheel in inches
		//acceleration = max acceleration in ft/s^2
		//timeout in milliseconds
		//sensitivity is in feet per second
	void PIDTurn(double Vf_at_end, double CoW, double acceleration, int timeout, double sensitivity, double a_left, double b_left, double a_right, double b_right, int angle);
		//a_left, b_left, a_right, b_right in inches
		//a/b is from the equation of an elipse
		//angle in degrees
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

	//objects
	ctre::phoenix::motorcontrol::can::WPI_TalonSRX Left_Front{Left_FrontID};
	ctre::phoenix::motorcontrol::can::WPI_TalonSRX Right_Front{Right_FrontID};
	ctre::phoenix::motorcontrol::can::WPI_TalonSRX Left_Back{Left_BackID};
	ctre::phoenix::motorcontrol::can::WPI_TalonSRX Right_Back{Right_BackID};
	frc::Joystick xbox1{0};
	std::time_t now;
	std::time_t end;
	frc::Timer counter{};
	std::ostringstream stringConverter;
	AHRS gyro{frc::SPI::Port::kMXP};

};

#endif /* DRIVE_H_ */
