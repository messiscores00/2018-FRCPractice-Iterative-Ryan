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
	void PIDMove(double Dtot, double Vf_at_end, double CoW, double a, int timeout, double sensitivity);
		//U = initial velocity
		//Dtot = total distance in inches
		// Vf = final velocity
		//CoW = circumference of Wheel
		//a = acceleration
		//timeout in seconds
		//sensitivity is in ticks per second
	void setU(double setU);

	//variables
	int Left_FrontID = 3;
	int Right_FrontID = 4;
	int Left_BackID = 2;
	int Right_BackID = 5;
	double Uvalues[1];

	//objects
	ctre::phoenix::motorcontrol::can::WPI_TalonSRX Left_Front{Left_FrontID};
	ctre::phoenix::motorcontrol::can::WPI_TalonSRX Right_Front{Right_FrontID};
	ctre::phoenix::motorcontrol::can::WPI_TalonSRX Left_Back{Left_BackID};
	ctre::phoenix::motorcontrol::can::WPI_TalonSRX Right_Back{Right_BackID};
	frc::Joystick xbox1{0};
	frc::DifferentialDrive _diffDrive{Left_Front, Right_Front};
	frc::Encoder encoder{0, 1, false, frc::CounterBase::EncodingType::k4X};
};

#endif /* DRIVE_H_ */
