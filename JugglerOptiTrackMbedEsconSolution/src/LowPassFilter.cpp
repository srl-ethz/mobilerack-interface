#include "LowPassFilter.h"

/* Constructor */
LowPassFilter::LowPassFilter(double omega, double T):
	cutOffFrequency(omega),
	samplePeriod(T),
	lastState(0),
	lastStateDerivative(0),
	stateDerivative(0),
	wT(0)
{ 
	this->setFilterParameters(omega,T);
}

void LowPassFilter::setFilterParameters(const double& omega, const double& T)
{
	cutOffFrequency = omega;
	samplePeriod = T;
	wT = cutOffFrequency * samplePeriod;
}
double LowPassFilter::calculate(double currentState)
{
	stateDerivative = (2 - wT) / (2 + wT) * lastStateDerivative
		+ (2*cutOffFrequency) / (2 + wT) * (currentState - lastState);
	lastState = currentState;
	lastStateDerivative = stateDerivative;
	return stateDerivative;
 }