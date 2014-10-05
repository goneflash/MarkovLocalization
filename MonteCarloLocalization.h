#include <iostream>
#include "bee-map.h"

#ifdef VIZ
	#include <opencv2/core/core.hpp>
	#include <opencv2/highgui/highgui.hpp>
	using namespace cv;
#endif

using namespace std;

#define PI 3.1415926
#define RANGE_LEN 180

typedef struct state{
	float x, y, theta;
} state;

typedef struct particle{
	float x, y, theta;
	float weight;
} particle;

typedef struct control{
	float x, y, theta;
	float x_prime, y_prime, theta_prime;
} control;

typedef struct measurement{
	float* r;
} measurement;

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
	void update_observation(measurement reading);

	// Estimation
	

protected:
	int _num_particles;
	particle* _particles;

	map_type _map;

	float _alpha[4];
	float _threshold;
	float _zhit, _znoise, _zshort, _zmax;
	float _hit_sigma, _lamda_short;
	float _max_laser_range;
	float _min_step;
	int _resampling_freq;
	int _resampling_count;
	float _weight_avg, _weight_fast, _weight_slow;
	float _alpha_fast, _alpha_slow;

	state _sample_motion_model_odometry(control ctrl, state old_state);
	float _cal_observation_weight(measurement reading, state particle_state);
	float _cal_observation_weight_v1(measurement reading, state particle_state);
	float _cal_weight_beam_range_model(measurement reading, state particle_state);

	state _estimated_state();

	float _sample_normal_distribution(float b);
	float _sensor_noise(float laser_data, float dist_exp);
	float _random_noise();
	float _unexpected_object(float laser_data, float dist_exp);
	float _max_noise(float laser_data);
	float _expected_distance(state particle_state, float laser, float angle);
	void _low_variance_sampler();
	void _augmented_low_variance_sampler();
	void _evaluate_convergence();

#ifdef VIZ
	Mat _map_image;
	void _visualize_particles();
	void _visualize_particle_with_log(state , measurement );
#endif


};