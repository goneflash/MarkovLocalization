#include <iostream>
#include <stdlib.h>
#include <cmath>
#include "MonteCarloLocalization.h"

#ifdef VIZ
	#include <opencv/cv.h>
	#include <opencv2/core/core.hpp>
	#include <opencv2/highgui/highgui.hpp>
using namespace cv;
#endif

using namespace std;

#define DEBUG

MonteCarloLocalization::MonteCarloLocalization(){
	_num_particles = 1000;
	_particles = new particle[_num_particles];
	_alpha[0] = 0.02;_alpha[1] = 0.02;
	_alpha[2] = 0.3;_alpha[3] = 0.3;

	_threshold = 0.75;
	_zhit = 0.8;
	_znoise = 0.1;
	_zshort = 0.095;
	_zmax = 0.005;
	_hit_sigma = 20;
	_lamda_short = 0.0005;
	_max_laser_range = 800.0;
	_min_step = 1;
	_resampling_freq = 2;	
	_resampling_count = 0;

	_weight_fast = 0;
	_weight_slow = 0;
	_weight_avg = 0;	
	_alpha_fast = 0.1;
	_alpha_slow = 0.001;

	srand(time(NULL));
}

MonteCarloLocalization::~MonteCarloLocalization(){
	delete[] _particles;
	_map_image = Mat();
}

void MonteCarloLocalization::init_map(map_type map){
	_map = map;
	
	// TODO:
	// smooth map

#ifdef VIZ
	_map_image = Mat::zeros( _map.size_x, map.size_y, CV_32FC1 );

	cout << "Image Property" << endl;
	cout << "Row: " << _map_image.rows << " Col: " << _map_image.cols << endl;
	cout << "Step: " << _map_image.step << " Dim: " << _map_image.dims << endl;
	cout << "ElemSize: " << _map_image.elemSize() << " Depth: " << _map_image.depth() << endl;
	cout << "Channels: " << _map_image.channels() << endl;

	// unsigned char *imgMat = (unsigned char*)(image.data);
	float max_value = -10, min_value = 10;
	for (unsigned int i = 0; i < _map_image.rows; i++)
		for (unsigned int j = 0; j < _map_image.cols; j++){
			if (_map.cells[i][j] >= 0.0 && _map.cells[i][j] <= 1.0)
				_map_image.at<float>(i, j) = _map.cells[i][j];
			else
				_map_image.at<float>(i, j) = 0;
			max_value = max_value >  _map.cells[i][j] ? max_value : _map.cells[i][j];
			min_value = min_value <  _map.cells[i][j] ? min_value : _map.cells[i][j];
		}
#ifdef DEBUG
		cout << "Max Value " << max_value << " Min Value " << min_value << endl;
		imshow("Image", _map_image);
		waitKey( 0 );
#endif
#endif	
}

void MonteCarloLocalization::init_particles(int num_particles){
		_num_particles = num_particles;
		delete[] _particles;
		_particles = new particle[_num_particles];
	// Randomly throw particles
		for (unsigned int i = 0; i < _num_particles; i++){
			do {
				_particles[i].x = rand() / (float)RAND_MAX * (_map.max_x - _map.min_x)  + _map.min_x;
				_particles[i].y = rand() / (float)RAND_MAX * (_map.max_y - _map.min_y)  + _map.min_y;
				_particles[i].theta = rand() / (float)RAND_MAX * 2 * PI;
				// _particles[i].weight = 1.0 / _num_particles;
			} while (_map.cells[(int)_particles[i].x][(int)_particles[i].y] == -1 || 
				_map.cells[(int)_particles[i].x][(int)_particles[i].y] <= 0.9);
#ifdef DEBUG
		// cout << "Particle: " << (int)_particles[i].x << " " << (int)_particles[i].y << " ";
		// cout << _particles[i].theta << " Prob " << _map.cells[(int)_particles[i].x][(int)_particles[i].y] << endl;
#endif
		}
		_visualize_particles();
}

void MonteCarloLocalization::init_parameters(float alpha[4]){
		for (unsigned int i = 0; i < 4; i++)
			_alpha[i] = alpha[i];
}

void MonteCarloLocalization::update_motion(control ctrl){
		for (unsigned int i = 0; i < _num_particles; i++){
			state s = {_particles[i].x, _particles[i].y, _particles[i].theta};
			s = _sample_motion_model_odometry(ctrl, s);
			_particles[i].x = s.x; _particles[i].y = s.y; _particles[i].theta = s.theta;

		// TODO:
		// If out of range or at -1 prob cell
		}

		_visualize_particles();
}

void MonteCarloLocalization::update_observation(measurement reading){
		float weight_sum = 0;
		for (unsigned int i = 0; i < _num_particles; i++){
			state s = {_particles[i].x, _particles[i].y, _particles[i].theta}; 
			_particles[i].weight = _cal_observation_weight(reading, s);

			// cout << "Particle " << i << " weight is: " << _particles[i].weight << endl;
			
			weight_sum += _particles[i].weight;		
		}
		_weight_avg = weight_sum / _num_particles;
	// Normalize weights
		for (unsigned int i = 0; i < _num_particles; i++)
			_particles[i].weight /= weight_sum;

		_resampling_count++;
		if (_resampling_count == _resampling_freq){
			_resampling_count = 0;
			_low_variance_sampler();
			// _augmented_low_variance_sampler();
		}

		_visualize_particle_with_log(_estimated_state(), reading);

	// TODO:
	// Try other resampling method
}

float MonteCarloLocalization::_cal_observation_weight(measurement reading, state s){
	// 25 cm offset
		float x = s.x + 2.5 * cos(s.theta), y = s.y + 2.5 * sin(s.theta);
	// If wrong position
		if (x < _map.min_x || x > _map.max_x || 
			y < _map.min_y || y > _map.max_y || 
			_map.cells[(int)x][(int)y] == -1 || _map.cells[(int)x][(int)y] < _threshold)
			return 0.0;	
	// process each angle
		float match_score = 0;
		for (unsigned int i = 0; i < RANGE_LEN; i++){
			float angle = (float)i * PI / 180 + s.theta;
			float x_end = reading.r[i] * cos(angle - PI / 2) + x;
			float y_end = reading.r[i] * sin(angle - PI / 2) + y;

			if (x_end < _map.min_x || x_end > _map.max_x || 
				y_end < _map.min_y || y_end > _map.max_y || _map.cells[(int)x_end][(int)y_end] < 0)
				continue;

		// TODO:
		// Try other methods for example sum of log
		// sum of 1 - weight 
			match_score += _map.cells[(int)x_end][(int)y_end] < _threshold ? 1 : 0;
		}
		return match_score;
}

float MonteCarloLocalization::_cal_observation_weight_v1(measurement reading, state s){
	// 25 cm offset
	float x = s.x + 2.5 * cos(s.theta), y = s.y + 2.5 * sin(s.theta);
	float weight = 0;

	// If wrong position
	if (x < _map.min_x || x > _map.max_x || 
		y < _map.min_y || y > _map.max_y || 
		_map.cells[(int)round(x)][(int)round(y)] == -1 ||
		_map.cells[(int)round(x)][(int)round(y)] < 0.9)
		return 0;	
	// process each angle
	
	for (unsigned int i = 0; i < RANGE_LEN; i++){
		float angle = (float)i * PI / 180 + s.theta;
		float x_end = reading.r[i] * cos(angle - PI / 2) + x;
		float y_end = reading.r[i] * sin(angle - PI / 2) + y;
		// cout << "laser end " << x_end << " " << y_end << endl;
		if (x_end < _map.min_x || x_end > _map.max_x || 
			y_end < _map.min_y || y_end > _map.max_y ||
			_map.cells[(int)round(x)][(int)round(y)] <= 0)
			continue;
		// TODO:
		// Try other methods for example sum of log
		// sum of 1 - weight 
		if (_map.cells[(int)round(x)][(int)round(y)] < 0.5)
			weight++;
	}
	return weight;
}

float MonteCarloLocalization::_cal_weight_beam_range_model(measurement reading, state particle_state){

		float weight = 0;
		float x = particle_state.x + 2.5 * cos(particle_state.theta);
		float y = particle_state.y + 2.5 * sin(particle_state.theta);

	// If wrong position
		if (x < _map.min_x || x > _map.max_x || 
			y < _map.min_y || y > _map.max_y || 
			_map.cells[(int)x][(int)y] <= 0 || _map.cells[(int)x][(int)y] < _threshold)
			return 0.0;	

		for (unsigned int i = 0; i < RANGE_LEN; i++){
			float angle = (float)i * PI / 180 + particle_state.theta;
			float x_end = reading.r[i] * cos(angle - PI / 2) + x;
			float y_end = reading.r[i] * sin(angle - PI / 2) + y;
			if (x_end < _map.min_x || x_end > _map.max_x || 
				y_end < _map.min_y || y_end > _map.max_y || _map.cells[(int)x_end][(int)y_end] <= 0)
				continue;

			float dist_exp = _expected_distance(particle_state, reading.r[i], angle);

			float likelihood = (_zhit * _sensor_noise(reading.r[i], dist_exp) +
				_znoise * _random_noise() +
				_zshort * _unexpected_object(reading.r[i], dist_exp) +
				_zmax * _max_noise(reading.r[i]));

			weight += log(likelihood);//	
			// weight *= likelihood * (1./len(expected_distances));
		}
	return weight;
}

float MonteCarloLocalization::_expected_distance(state s, float laser, float angle){
	// use ray tracing
	// 25 cm offset
	float x = s.x + 2.5 * cos(s.theta), y = s.y + 2.5 * sin(s.theta);
	float max_laser = laser, min_laser = 0;

	// cout << " actual " << laser;

	while (max_laser - min_laser > _min_step){
		float dist = (max_laser + min_laser) / 2;

		// cout << " mid " << dist; 

		float x_end = dist * cos(angle - PI / 2) / 10 + x;
		float y_end = dist * sin(angle - PI / 2) / 10 + y;
		if (_map.cells[(int)x_end][(int)y_end] < _threshold)
			max_laser = dist;
		else
			min_laser = dist;
	}

	// cout << endl;

	return (max_laser + min_laser) / 2;
}

float MonteCarloLocalization::_sensor_noise(float laser_data, float dist_exp){
	float normalizer = 1.0 / (_hit_sigma * sqrt(2 * PI));
	return normalizer * exp(-0.5 * ((laser_data - dist_exp) / _hit_sigma) * ((laser_data - dist_exp) / _hit_sigma));
}
float MonteCarloLocalization::_random_noise(){
	return 1.0 / _max_laser_range;
}
float MonteCarloLocalization::_unexpected_object(float laser_data, float dist_exp){
	float normalizer = 1.0 / (1 - exp(-_lamda_short * dist_exp));
	if (laser_data < dist_exp)
		return (_lamda_short * normalizer * exp(-_lamda_short * laser_data));
	return 0;
}
float MonteCarloLocalization::_max_noise(float laser_data){
	return (fabs(laser_data - _max_laser_range) <= 0.001 ? 1 : 0);
}

void MonteCarloLocalization::_low_variance_sampler(){
	particle* new_pars = new particle[_num_particles];
	// create a random value to start
	float r = rand() / (float)RAND_MAX / _num_particles; 
	int idx = 0;
	float current_weight = _particles[0].weight;
	for (unsigned int i = 0; i < _num_particles; i++){
		float now_total_weight = r + (float)i / _num_particles;
		while (current_weight < now_total_weight){
			idx++;
			current_weight += _particles[idx].weight; 
		}
		new_pars[i] = _particles[idx];
	}
	for (unsigned int i = 0; i < _num_particles; i++)
		_particles[i] = new_pars[i];
}

void MonteCarloLocalization::_augmented_low_variance_sampler(){
	particle* new_pars = new particle[_num_particles];
	// create a random value to start
	float r = rand() / (float)RAND_MAX / _num_particles; 
	int idx = 0;
	float current_weight = _particles[0].weight;
	
	_weight_slow = _weight_slow * (1 - _alpha_slow) + _alpha_slow * (_weight_avg - _weight_slow);
	_weight_fast = _weight_fast * (1 - _alpha_fast) + _alpha_fast * (_weight_avg - _weight_fast); 

	float randomize_threshold = 0;
	if (_weight_slow != 0)
		randomize_threshold = 1.0 - _weight_fast / _weight_slow;

	cout << "randomize_threshold is " << randomize_threshold;
	cout << " weight fast " << _weight_fast << " weight slow" << _weight_slow << endl;

	randomize_threshold = -1;

	for (unsigned int i = 0; i < _num_particles; i++){
		float now_total_weight = r + (float)i / _num_particles;
		while (current_weight < now_total_weight){
			idx++;
			current_weight += _particles[idx].weight; 
		}

		if (randomize_threshold > 0)
			if ((rand() / (float)RAND_MAX) < randomize_threshold){
				
			do {
				new_pars[i].x = rand() / (float)RAND_MAX * (_map.max_x - _map.min_x)  + _map.min_x;
				new_pars[i].y = rand() / (float)RAND_MAX * (_map.max_y - _map.min_y)  + _map.min_y;
				new_pars[i].theta = rand() / (float)RAND_MAX * 2 * PI;
				new_pars[i].weight = 1.0 / _num_particles;
			} while (_map.cells[(int)new_pars[i].x][(int)new_pars[i].y] == -1 || 
				_map.cells[(int)new_pars[i].x][(int)new_pars[i].y] <= 0.9);
				continue;
		}

		new_pars[i] = _particles[idx];
	}
	for (unsigned int i = 0; i < _num_particles; i++)
		_particles[i] = new_pars[i];
}

state MonteCarloLocalization::_sample_motion_model_odometry(control ctrl, state old_state){
	float d_rot1 = atan2(ctrl.y_prime - ctrl.y, ctrl.x_prime - ctrl.x) - ctrl.theta;
	float d_tran = sqrt((ctrl.x - ctrl.x_prime) * (ctrl.x - ctrl.x_prime) + (ctrl.y - ctrl.y_prime) * (ctrl.y - ctrl.y_prime));
	float d_rot2 = ctrl.theta_prime - ctrl.theta - d_rot1; 

	float d_rot1_prime = d_rot1 - _sample_normal_distribution(_alpha[0]*d_rot1 + _alpha[1]*d_tran);
	float d_tran_prime = d_tran - _sample_normal_distribution(_alpha[2]*d_tran + _alpha[3]*(d_rot1 + d_rot2));
	float d_rot2_prime = d_rot2 - _sample_normal_distribution(_alpha[0]*d_rot2 + _alpha[1]*d_tran);

	// TODO:
	// Try other distribution, triangle distribution

	state new_state;
	new_state.x = old_state.x + (d_tran_prime / 10) * cos(old_state.theta + d_rot1_prime);
	new_state.y = old_state.y + (d_tran_prime / 10) * sin(old_state.theta + d_rot1_prime);
	// new_state.x = old_state.x + d_tran_prime * cos(old_state.theta + d_rot1_prime);
	// new_state.y = old_state.y + d_tran_prime * sin(old_state.theta + d_rot1_prime);
	new_state.theta = old_state.theta + d_rot1_prime + d_rot2_prime;

	return new_state;
}

float MonteCarloLocalization::_sample_normal_distribution(float b){
	// sample from zero mean, b variance.
	float result = 0;
	for (unsigned int i = 0; i < 12; i++)
		result += (rand() - RAND_MAX / 2) / (float)RAND_MAX * 2;
	result *= b;
	result /= 6;
	return result;
}

#ifdef VIZ

void MonteCarloLocalization::_visualize_particles(){
  	// Draw circles
	Mat new_image;
	// _map_image.clone();
	cvtColor(_map_image, new_image, CV_GRAY2RGB);
	for (unsigned int i = 0; i < _num_particles; i++){
  		// circle(Mat& img, Point center, int radius, const Scalar& color, int thickness=1, int lineType=8, int shift=0)
		circle(new_image, Point(_particles[i].y, _particles[i].x), 2, Scalar(0, 0, 255), 2, 8);
	}
  	// circle( image, Point( 200, 200 ), 32.0, Scalar( 0, 0, 255 ), 1, 8 );
	imshow("Image", new_image);
	waitKey( 10 );
	new_image = Mat();
}

void MonteCarloLocalization::_visualize_particle_with_log(state particle_state, measurement r){
	// Draw circles
	Mat new_image;
	Mat tmp_img = _map_image.clone();
	cvtColor(_map_image, new_image, CV_GRAY2RGB);

	float x = particle_state.x + 2.5 * cos(particle_state.theta);
	float y = particle_state.y + 2.5 * sin(particle_state.theta);
	circle(new_image, Point(y, x), 3, Scalar(0, 0, 255), 2, 8);

	for (unsigned int i = 0; i < RANGE_LEN; i++){
		float angle = (float)i * PI / 180 + particle_state.theta;
		for (unsigned int d = 0; d < (int)(r.r[i]); d++){
			int x_end = (int)(d * cos(angle - PI / 2) + x);
			int y_end = (int)(d * sin(angle - PI / 2) + y);
			if (x_end < _map.min_x || x_end > _map.max_x || 
				y_end < _map.min_y || y_end > _map.max_y)
				continue;
			circle(new_image, Point(y_end, x_end), 1, Scalar(255, 0, 255), 2, 8);
		}
	}

	imshow("Estimation", new_image);
	waitKey( 10 );
}

state MonteCarloLocalization::_estimated_state(){
	float x_est = 0, y_est = 0, theta_est = 0, weight_sum = 0;
	for (unsigned int i = 0; i < _num_particles; i++){
		x_est += _particles[i].x * _particles[i].weight;
		y_est += _particles[i].y * _particles[i].weight;
		theta_est += _particles[i].theta * _particles[i].weight;
		weight_sum += _particles[i].weight;
	}
	x_est /= weight_sum;
	y_est /= weight_sum;
	theta_est /= weight_sum;
	state s = {x_est, y_est, theta_est}; 
	return s;
}

void MonteCarloLocalization::_evaluate_convergence(){
	float mean_x = 0, mean_y = 0;
	for (unsigned int i = 0; i < _num_particles; i++){
		mean_x += _particles[i].x;
		mean_y += _particles[i].y;
	}
	mean_x /= _num_particles;
	mean_y /= _num_particles;
	float std_x = 0, std_y = 0;
	for (unsigned int i = 0; i < _num_particles; i++){
		std_x += (mean_x - _particles[i].x) * (mean_x - _particles[i].x);
		std_y += (mean_y - _particles[i].y) * (mean_y - _particles[i].y);
	}
	cout << "Std of x is: " << std_x << "Std of y is: " << std_y << endl;
}


#endif

// Unit Test
#ifdef PF_UNIT_TEST
int main(){



	return 0;
}
#endif