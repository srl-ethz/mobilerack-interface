#ifndef LOW_PASS_FILTER_H
#define LOW_PASS_FILTER_H

class LowPassFilter
{
public:
	LowPassFilter(){}
	LowPassFilter(double omega, double T);
	~LowPassFilter() {}
	void setFilterParameters(const double& omega, const double& T);
	double calculate(double currentState);

private:
	double cutOffFrequency;
	double samplePeriod;
	double lastState;
	double lastStateDerivative;
	double stateDerivative;
	double wT;
};

#endif //LOW_PASS_FILTER_H
