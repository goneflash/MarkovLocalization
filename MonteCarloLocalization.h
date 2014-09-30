#include <iostream>
#include "bee-map.h"

using namespace std;

#define PI 3.1415926

typedef struct particle{
	int x, y;
	float theta;
	double weight;
} particle;

typedef struct control{
	double x, y;
} control;

typedef struct measurement{
	double* r;
} measurement;

class MonteCarloLocalization{
public:
	MonteCarloLocalization();
	~MonteCarloLocalization();

	// Map
	void init_map(map_type map);
	// Particles
	void init_particles(int num_particles);

protected:
	int _num_particles;
	particle* _particles;

	map_type _map;
	double _cm_per_cell;

	measurement _obs_data;
	control _ctrl_data;


	bool _viz;

};