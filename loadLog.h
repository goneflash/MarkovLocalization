#include <iostream>
#include <vector>

using namespace std;

#define RANGE_LEN 180

enum LogType
{
	LASER_DATA = 0, ODO_DATA = 1
};

typedef struct {
	LogType type;
	float x, y, xl, yl;
	float theta, thetal;
	double ts;
	float* r; 
} log_type;

int read_log_data(const char *logName, vector<log_type> & logData);