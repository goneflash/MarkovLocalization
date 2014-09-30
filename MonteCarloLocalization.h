#include <iostream>
#include "bee-map.h"

#ifdef VIZ
	#include <opencv2/core/core.hpp>
	#include <opencv2/highgui/highgui.hpp>
	using namespace cv;
#endif

using namespace std;

#define PI 3.1415926

typedef struct state{
	float x, y, theta;
} state;

typedef struct particle{
	float x, y, theta;
	double weight;
} particle;

typedef struct control{
	float x, y, theta;
	float x_prime, y_prime, theta_prime;
} control;



class MonteCarloLocalization{
public:
	MonteCarloLocalization();
	~MonteCarloLocalization();

	// Initialize
	void init_map(map_type map);
	void init_particles(int num_particles);
	void init_parameters(float alpha[4]);

	// Update
	void update_motion(control ctrl);

protected:
	int _num_particles;
	particle* _particles;

	map_type _map;

	float _alpha[4];
	state _sample_motion_model_odometry(control ctrl, state old_state);
	float _sample_normal_distribution(float b);

#ifdef VIZ
	Mat _map_image;
	void _visualize_particles();
#endif


};