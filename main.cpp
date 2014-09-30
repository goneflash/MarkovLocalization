#include <iostream>
#include <vector>
#include <string>
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

#ifdef DEBUG
	// Check Map information
	cout << "Map information: " << endl;
	cout << "Resolution " << map.resolution << " SizeX " << map.size_x << " SizeY " << map.size_y << endl;
	cout << "Min_Max X " << map.min_x << " " << map.max_x << endl;
	cout << "Min_Max Y " << map.min_y << " " << map.max_y << endl;
	cout << "Offset X " << map.offset_x << " Offset Y " << map.offset_y << endl;
#endif
	// Initialization
	int num_particles = 10;
	MonteCarloLocalization localizer;
	localizer.init_map(map);
	localizer.init_particles(num_particles); 

	// Initialize Particle Filter



	// Main Loop


}