/*
 * PIDGenerate.h
 *
 *  Created on: Jun 14, 2018
 *      Author: Ryan Sheehy
 */

#ifndef SRC_PIDGENERATE_H_
#define SRC_PIDGENERATE_H_
#include <ctre/phoenix.h>
#include <DriverStation.h>
#include <string>
#include <IterativeRobot.h>
#include <SmartDashboard/SmartDashboard.h>
#include <Drive.h>

class PIDGenerate {
public:
	PIDGenerate();
	virtual ~PIDGenerate();

	Drive drive{};

	//methods
	void returnP(int P);
	void returnI(int I);
	void returnD(int D);
	void Graph(double Dtot, double CoW);
	void setPIDs();
	void returnRating();
	double encoder();
};

#endif /* SRC_PIDGENERATE_H_ */
