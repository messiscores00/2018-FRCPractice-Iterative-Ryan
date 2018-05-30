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

class Drive {
public:
	Drive();
	virtual ~Drive();
	frc::Joystick xbox1{0};
	void ArcadeDrive(int deadzone, double sensitivity);
	//deadzone is between 1 and 10
		//10 being max
	//sensitivity = if full forward how far do you want it to travel

	int Left_FrontID = 0;
	int Right_FrontID = 1;
	int Left_BackID = 2;
	int Right_BackID = 3;

	ctre::phoenix::motorcontrol::can::WPI_TalonSRX Left_Front{Left_FrontID};
	ctre::phoenix::motorcontrol::can::WPI_TalonSRX Right_Front{Right_FrontID};
	ctre::phoenix::motorcontrol::can::WPI_TalonSRX Left_Back{Left_BackID};
	ctre::phoenix::motorcontrol::can::WPI_TalonSRX Right_Back{Right_BackID};
};

#endif /* DRIVE_H_ */
