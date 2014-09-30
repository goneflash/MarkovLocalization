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

void MonteCarloLocalization::init_map(map_type map){
	_map = map;
}

void MonteCarloLocalization::init_particles(int num_particles){
	_num_particles = num_particles;
	// Randomly throw particles
	srand ( time(NULL) );
	for (unsigned int i = 0; i < _num_particles; i++){
		do {
		_particles[i].x = rand() / (float)RAND_MAX * (_map.max_x - _map.min_x)  + _map.min_x;
		_particles[i].y = rand() / (float)RAND_MAX * (_map.max_y - _map.min_y)  + _map.min_y;
		_particles[i].theta = rand() / (float)RAND_MAX * 2 * PI;
		} while (_map.cells[_particles[i].x][_particles[i].y] < 0.0);
#ifdef DEBUG
		cout << "Particle: " << _particles[i].x << " " << _particles[i].y << " ";
		cout << _particles[i].theta << " Prob " << _map.cells[_particles[i].x][_particles[i].y] << endl;
#endif
	}
}

// Unit Test
#ifdef PF_UNIT_TEST
int main(){
	return 0;
}
#endif