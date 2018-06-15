/*
 * PIDGenerate.cpp
 *
 *  Created on: Jun 14, 2018
 *      Author: Ryan Sheehy
 */

#include <PIDGenerate.h>

PIDGenerate::PIDGenerate() {
}

PIDGenerate::~PIDGenerate() {
}

void PIDGenerate::returnP(int P){
	double p = (double)P;
	frc::SmartDashboard::PutNumber("P", p);
}

void PIDGenerate::returnI(int I){
	double i = (double)I;
	frc::SmartDashboard::PutNumber("I", i);
}

void PIDGenerate::returnD(int D){
	double d = (double)D;
	frc::SmartDashboard::PutNumber("D", d);
}

void PIDGenerate::returnRating(){

}

void PIDGenerate::Graph(double Dtot, double CoW){
	Dtot = Dtot *(4096/CoW);
	double error = drive.encoder() - Dtot;
	frc::SmartDashboard::PutNumber("error", error);
}

void PIDGenerate::setPIDs(){

}

