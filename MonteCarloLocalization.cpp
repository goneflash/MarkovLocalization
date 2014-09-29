#include <iostream>
#include <stdlib.h>
#include "MonteCarloLocalization.h"

using namespace std;

#define DEBUG

MonteCarloLocalization::MonteCarloLocalization(){
	_num_particles = 100;
	_particles = new particle[_num_particles];

}

MonteCarloLocalization::~MonteCarloLocalization(){
	delete _particles;
}

MonteCarloLocalization::MonteCarloLocalization(double** map, int sizeX, int sizeY, 
	int num_particles, double maxX, double minX, double maxY, 
		double minY, double maxTheta, double minTheta){
	_num_particles = num_particles;
	_particles = new particle[_num_particles];
	init_map(map, sizeX, sizeY);

	_maxX = maxX;
	_maxY = maxY;
	_maxTheta = maxTheta;
	_minX = minX;
	_minY = minY;
	_minTheta = minTheta;
	_init_particles();
}

void MonteCarloLocalization::init_map(double** map, int sizeX, int sizeY){
	_map = new double*[sizeX];
	for (unsigned int i = 0; i < sizeX; i++)
		_map[i] = new double[sizeY];

	for (int i = 0; i < sizeX; i++)
		for (int j = 0; j < sizeY; j++)
			_map[i][j] = map[i][j];
}

void MonteCarloLocalization::_init_particles(){

}

void MonteCarloLocalization::update(){

}

void MonteCarloLocalization::add_measurement(double* range){

}

void MonteCarloLocalization::add_control(double x, double y){

}

void MonteCarloLocalization::_update_motion_model(){

}
void MonteCarloLocalization::_update_measurement_weight(){

}

double MonteCarloLocalization::_get_particle_weight(int par_id){

}

void MonteCarloLocalization::_low_variance_sampling(){

}

// Unit Test
#ifdef PF_UNIT_TEST
int main(){
	return 0;
}
#endif