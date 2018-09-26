#include <iostream>
#include <chrono>
#include <thread>
#include "../include/mpa/mpa.h"
#include "./MiniPID/MiniPID.h"
#include "./matplotlib-cpp/matplotlibcpp.h"
#define USE_PID true
#define STEP_TEST true // commandPressure is a step function in regards to time

namespace plt = matplotlibcpp;

int step_func(int i){
	// step function to send to valve
	if (i%1500 < 750){
		return 900;
	}
	else{
		return 1100;
	}
}

int main() {
	// Create MPA controller.
	MPA mpa("192.168.1.101", "502");
	unsigned int valveNum = 0; //test valves 0 to 15
	unsigned int commandPressure = 1000; //tested 0 to 1000 mbar
	unsigned int endPressure = 0;

	//Ziegler-Nichols method
	double Ku = 2.6;
	double Tu = 0.14;
	double KP=0.6 * Ku;
	double KI=KP/ (Tu / 2.0) * 0.002;
	double KD=KP*Tu/2.0 / 0.002;

	MiniPID pid(KP, KI, KD); // create PID controller
	pid.setOutputLimits(50); // setting a good output limit is important so as not oscillate

	// Connect.
	if (!mpa.connect()) {
		std::cout << "Failed to connect to MPA." << std::endl;
		return -1;
	}

	// Set valve 0 to 1 bar.
	mpa.set_single_pressure(valveNum, commandPressure);
	int sensorvalue;
	double output;

	int cycles = 3000;
	std::vector<double> x(cycles), pressures(cycles), commandpressures(cycles); // for logging pressure profile
	for ( int i=0; i<3000; i++) {
		// Wait 1 ms.
		//std::this_thread::sleep_for(std::chrono::millisecofalsends(1));

		if (STEP_TEST){
			commandPressure = step_func(i);
		}
		commandpressures.at(i) = commandPressure;
		// Read pressure of valve 0.
		sensorvalue = mpa.get_single_pressure(valveNum);
		x.at(i) = i;
		pressures.at(i) = sensorvalue;
		output = commandPressure+pid.getOutput(sensorvalue, commandPressure);

		if (USE_PID){
			mpa.set_single_pressure(valveNum, (int)output);
		}
		else{
			mpa.set_single_pressure(valveNum, commandPressure);
		}



		if (i%50 == 0){
			std::cout << "Valve " << valveNum << " sensor: "
				<< sensorvalue
				<< " mbar\toutput: "
				<< output
				<< " mbar"
				<< std::endl;
		}
	}

	// Set valve 0 to 0 bar (off).
	mpa.set_single_pressure(valveNum, endPressure);

	// Wait 100 ms.
	std::this_thread::sleep_for(std::chrono::milliseconds(500));

	// Read pressure of valve 0.
	std::cout << "Valve " << valveNum << " :"
		<< mpa.get_single_pressure(valveNum)
		<< " mbar"
		<< std::endl;

	// Disconnect.
	mpa.disconnect();

	plt::figure_size(1200, 780);
	if (USE_PID){
		plt::title("P:" + std::to_string(KP)+ ", I:"+ std::to_string(KI)+", D:"+std::to_string(KD));
	}
	else{
		plt::title("w/o PID");
	}
	plt::named_plot("measured pressure",x,pressures);
	plt::named_plot("commanded pressure",x, commandpressures);
	plt::ylim(800,1200);
	plt::legend();
	plt::save("./graph.png");
	std::cout << "graph output to ./graph.png" << '\n';

	return 0;
}
