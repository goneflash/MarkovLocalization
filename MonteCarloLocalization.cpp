#include <iostream>
#include <stdlib.h>
#include "MonteCarloLocalization.h"
#include "bee-map.h"

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
}

void MonteCarloLocalization::init_map(double** map, int sizeX, int sizeY){

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