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
	double x, y, xl, yl;
	double theta, thetal;
	double ts;
	double* r; 
} log_type;

int read_log_data(const char *logName, vector<log_type> & logData);