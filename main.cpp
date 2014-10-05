#include <iostream>
#include <vector>
#include <string>
#include "loadLog.h"
#include "MonteCarloLocalization.h"


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;

using namespace std;

#define DEBUG

int main(int argc, char **argv){
	// Set parameters
	string map_name = "../data/map/wean.dat";
	string log_name = "../data/log/robotdata1.log";
	// string log_name = "../data/log/ascii-robotdata2.log";

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
	
	// Check Log Information
	cout << "Log size: " << logData.size() << endl;
	// for (unsigned int i = 0; i < logData.size(); i++)
	// 		cout << logData[i].type << " " << logData[i].ts << " " << logData[i].x << " " << logData[i].y << endl;
#endif

	// Initialization
	int num_particles = 3000;

	MonteCarloLocalization localizer;
	localizer.init_map(map);
	localizer.init_particles(num_particles); 

	// Main Loop
	measurement reading;
	reading.r = new float[RANGE_LEN];
	for (unsigned int i = 1; i < logData.size(); i++){
		cout << "Processing frame at: " << logData[i].ts << " " << i << "/" << logData.size() << endl;
		control ctrl;
		ctrl.x = logData[i-1].x; ctrl.y = logData[i-1].y; ctrl.theta = logData[i-1].theta;
	 	ctrl.x_prime = logData[i].x; ctrl.y_prime = logData[i].y; ctrl.theta_prime = logData[i].theta;
		localizer.update_motion(ctrl);
		
		// cout << "data type " << logData[i].type << endl;
		if (logData[i].type != LASER_DATA)
			continue;
		cout << "It's laser data." << endl;


		for (unsigned int angle = 0; angle < 180; angle++){
			reading.r[angle] = logData[i].r[angle] / 10;
			// cout << "reading " << reading.r[angle] << " log " << logData[i].r[angle] << endl;
		}

		// TODO
		// not always resample

		localizer.update_observation(reading);
	}

	// Clean Up
	free(map.cells);
	// delete reading.r;

	return 0;
}