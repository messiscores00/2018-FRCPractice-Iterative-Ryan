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

class Drive {
public:
	//functions
	Drive();
	virtual ~Drive();
	void PIDenable(int P, int I, int D, int F);
	void PIDdisable();
	void ArcadeDrive(double deadzone, double sensitivity);
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

	//variables
	int Left_FrontID = 3;
	int Right_FrontID = 4;
	int Left_BackID = 2;
	int Right_BackID = 5;
	double seconds;
	double s; //displacement
	double Uvalue = 0;
	//ark length of the left and right wheel
	double ArkLeng_left;
	double ArkLeng_right;
	//displacement of left and right wheels
	double Sleft;
	double Sright;
	double setpoint;
	double encoder = (Left_Front.GetSelectedSensorPosition(1) * 4 + Right_Front.GetSelectedSensorPosition(1) * 4) / 2 ;

	//objects
	ctre::phoenix::motorcontrol::can::WPI_TalonSRX Left_Front{Left_FrontID};
	ctre::phoenix::motorcontrol::can::WPI_TalonSRX Right_Front{Right_FrontID};
	ctre::phoenix::motorcontrol::can::WPI_TalonSRX Left_Back{Left_BackID};
	ctre::phoenix::motorcontrol::can::WPI_TalonSRX Right_Back{Right_BackID};
	frc::Joystick xbox1{0};
	frc::DifferentialDrive _diffDrive{Left_Front, Right_Front};
	std::time_t now;
	std::time_t end;
	frc::Counter counter{};
	frc::AnalogGyro gyro{0};
	std::ostringstream stringConverter;
};

#endif /* DRIVE_H_ */
