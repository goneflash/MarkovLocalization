#include <iostream>
#include <vector>
#include <string>
#include "bee-map.h"
#include "loadLog.h"
#include "MonteCarloLocalization.h"

using namespace std;

#define DEBUG

int main(int argc, char **argv){
	// Set parameters
	string map_name = "../data/map/wean.dat";
	string log_name = "../data/log/robotdata1.log";

	vector<log_type> logData;
	map_type map;
	
	read_log_data(log_name.c_str(), logData);
	read_beesoft_map(map_name.c_str(), &map);

	// Initialization
	int num_particles = 1000;
	MonteCarloLocalization localizer;

	// Initialize Particle Filter



	// Main Loop


}