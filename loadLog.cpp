#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>

#include "loadLog.h"

// #define DEBUG

using namespace std;

int read_log_data(const char *logName, vector<log_type> & logData){
	ifstream logFile(logName); 
	if (!logFile.is_open()){
		fprintf(stderr, "Couldn't open file %s.\n", logName);
		return -1;
	}
	string line;
	logData.clear();
	fprintf(stderr, "# Reading log: %s\n", logName);
	while (getline(logFile, line)){
		log_type data;
		char type;
		istringstream ss(line);
		ss >> type;
		if (line[0] == 'O'){
			data.type = ODO_DATA;
			ss >> data.x >> data.y;
			ss >> data.theta >> data.ts;
		}
		else if (line[0] == 'L'){
			data.r = new float[RANGE_LEN];
			data.type = LASER_DATA;
			ss >> data.x >> data.y >> data.theta;
			ss >> data.xl >> data.yl >> data.thetal;
			for (unsigned int i = 0; i < RANGE_LEN; i++)
				ss >> data.r[i];
			ss >> data.ts;
		}
		else{
			fprintf(stderr, "Wrong data format at line %d\n", (int)logData.size());
			return -1;
		}
		logData.push_back(data);
#ifdef DEBUG
		cout << logData.back().type << " " << logData.back().x << " ";
		cout << logData.back().y << " " << logData.back().ts << endl;
#endif

	}

	logFile.close();
	return 1;
}


// Unit Test
#ifdef LOG_UNIT_TEST

int main(){
	vector<log_type> log_Data;
	string filename = "../data/log/robotdata1.log";
	read_log_data(filename.c_str(), log_Data);
}

#endif