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
	_num_particles = 100;
	_particles = new particle[_num_particles];
	_alpha[0] = 0.1;_alpha[1] = 0.1;
	_alpha[2] = 0.1;_alpha[3] = 0.1;

	srand ( time(NULL) );
}

MonteCarloLocalization::~MonteCarloLocalization(){
	delete[] _particles;
	_map_image = Mat();
}

void MonteCarloLocalization::init_map(map_type map){
	_map = map;

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
			if (_map.cells[i][j] >= 0.0)
				_map_image.at<float>(i, j) = 1 - _map.cells[i][j];
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
		} while (_map.cells[(int)_particles[i].x][(int)_particles[i].y] == -1);// || 
			// _map.cells[(int)_particles[i].x][(int)_particles[i].y] >= 0.5);
#ifdef DEBUG
		cout << "Particle: " << (int)_particles[i].x << " " << (int)_particles[i].y << " ";
		cout << _particles[i].theta << " Prob " << _map.cells[(int)_particles[i].x][(int)_particles[i].y] << endl;
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
		weight_sum += _particles[i].weight;		
	}
	// Normalize weights
	for (unsigned int i = 0; i < _num_particles; i++)
		_particles[i].weight /= weight_sum;

	_low_variance_sampler();

	// TODO:
	// Try other resampling method

	_evaluate_convergence();
}

float MonteCarloLocalization::_cal_observation_weight(measurement reading, state s){
	// 25 cm offset
	float x = s.x + 2.5 * cos(s.theta), y = s.y + 2.5 * sin(s.theta);
	// process each angle
	float match_score = 0;
	for (unsigned int i = 0; i < RANGE_LEN; i++){
		float angle = (float)i * PI / 180 + s.theta;
		float x_end = reading.r[i] * cos(angle - PI / 2) / 10 + x;
		float y_end = reading.r[i] * sin(angle - PI / 2) / 10 + y;
		if (x_end < _map.min_x || x_end > _map.max_x || 
			y_end < _map.min_y || y_end > _map.max_y)
			continue;

		// TODO:
		// Try other methods for example sum of log
		// sum of 1 - weight 
		float threshold = 0.5;
		match_score += _map.cells[(int)x_end][(int)y_end] > threshold ? 1 : 0;
	}
	return match_score;
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

state MonteCarloLocalization::_sample_motion_model_odometry(control ctrl, state old_state){
	float d_rot1 = atan2(ctrl.y_prime - ctrl.y, ctrl.x_prime - ctrl.x) - ctrl.theta;
	float d_tran = sqrt((ctrl.x - ctrl.x_prime) * (ctrl.x - ctrl.x_prime) + (ctrl.y - ctrl.y_prime) * (ctrl.y - ctrl.y_prime));
	float d_rot2 = ctrl.theta_prime - ctrl.theta - d_rot1; 

	float d_rot1_prime = d_rot1 - _sample_normal_distribution(_alpha[1]*d_rot1 + _alpha[2]*d_tran);
	float d_tran_prime = d_tran - _sample_normal_distribution(_alpha[3]*d_tran + _alpha[4]*(d_rot1 + d_rot2));
	float d_rot2_prime = d_rot2 - _sample_normal_distribution(_alpha[1]*d_rot2 + _alpha[2]*d_tran);

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
  	_map_image.clone();
  	cvtColor(_map_image, new_image, CV_GRAY2RGB);
  	for (unsigned int i = 0; i < _num_particles; i++){
  		// circle(Mat& img, Point center, int radius, const Scalar& color, int thickness=1, int lineType=8, int shift=0)
  		circle(new_image, Point(_particles[i].y, _particles[i].x), 2, Scalar(0, 0, 255), 2, 8);
  	}
  	// circle( image, Point( 200, 200 ), 32.0, Scalar( 0, 0, 255 ), 1, 8 );
  	imshow("Image", new_image);
	waitKey( 0 );
	new_image = Mat();
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