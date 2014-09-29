#include <iostream>

using namespace std;

typedef struct particle{
	double x, y, theta;
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
	MonteCarloLocalization(double** map, int sizeX, int sizeY, int num_particles,
		double maxX, double minX, double maxY, 
		double minY, double maxTheta, double minTheta);

	void update();

	// Map
	void init_map(double** map, int sizeX, int sizeY);
	// Laser data
	void add_measurement(double* range);
	// Odometry data
	void add_control(double x, double y);

protected:
	int _num_particles;
	particle* _particles;
	double _maxX, _maxY, _maxTheta;
	double _minX, _minY, _minTheta;

	double** _map;
	double _cm_per_cell;

	measurement _obs_data;
	control _ctrl_data;


	bool _viz;

	void _init_particles();

	void _update_motion_model();
	void _update_measurement_weight();

	double _get_particle_weight(int par_id);
	void _low_variance_sampling();

};