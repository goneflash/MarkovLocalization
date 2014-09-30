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
	delete _particles;
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
	for (unsigned int i = 0; i < _map_image.rows; i++)
		for (unsigned int j = 0; j < _map_image.cols; j++){
			if (_map.cells[i][j] > 0.0)
				_map_image.at<float>(i, j) = _map.cells[i][j]; 
		}
#ifdef DEBUG
	imshow("Image", _map_image);
	waitKey( 0 );
#endif
#endif	
}

void MonteCarloLocalization::init_particles(int num_particles){
	_num_particles = num_particles;
	delete _particles;
	_particles = new particle[_num_particles];
	// Randomly throw particles
	for (unsigned int i = 0; i < _num_particles; i++){
		do {
		_particles[i].x = rand() / (float)RAND_MAX * (_map.max_x - _map.min_x)  + _map.min_x;
		_particles[i].y = rand() / (float)RAND_MAX * (_map.max_y - _map.min_y)  + _map.min_y;
		_particles[i].theta = rand() / (float)RAND_MAX * 2 * PI;
		} while (_map.cells[(int)_particles[i].x][(int)_particles[i].y] < 0.2);
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
		state s;
		s.x = _particles[i].x; s.y = _particles[i].y; s.theta = _particles[i].theta;
		s = _sample_motion_model_odometry(ctrl, s);
		_particles[i].x = s.x; _particles[i].y = s.y; _particles[i].theta = s.theta;
	}
}

state MonteCarloLocalization::_sample_motion_model_odometry(control ctrl, state old_state){
	float d_rot1 = atan2(ctrl.y_prime - ctrl.y, ctrl.x_prime - ctrl.x) - ctrl.theta;
	float d_tran = sqrt((ctrl.x - ctrl.x_prime) * (ctrl.x - ctrl.x_prime) + (ctrl.y - ctrl.y_prime) * (ctrl.y - ctrl.y_prime));
	float d_rot2 = ctrl.theta_prime - ctrl.theta - d_rot1; 

	float d_rot1_prime = d_rot1 - _sample_normal_distribution(_alpha[1]*d_rot1 + _alpha[2]*d_tran);
	float d_tran_prime = d_tran - _sample_normal_distribution(_alpha[3]*d_tran + _alpha[4]*(d_rot1 + d_rot2));
	float d_rot2_prime = d_rot2 - _sample_normal_distribution(_alpha[1]*d_rot2 + _alpha[2]*d_tran);

	state new_state;
	// new_state.x = old_state.x + (d_tran_prime / 10) * cos(old_state.theta + d_rot1_prime);
	// new_state.y = old_state.y + (d_tran_prime / 10) * sin(old_state.theta + d_rot1_prime);
	new_state.x = old_state.x + d_tran_prime * cos(old_state.theta + d_rot1_prime);
	new_state.y = old_state.y + d_tran_prime * sin(old_state.theta + d_rot1_prime);
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
  	// Draw a circle
  	Mat new_image;
  	cvtColor(_map_image, new_image, CV_GRAY2RGB);
  	for (unsigned int i = 0; i < _num_particles; i++){
  		// circle(Mat& img, Point center, int radius, const Scalar& color, int thickness=1, int lineType=8, int shift=0)
  		circle(new_image, Point(_particles[i].y, _particles[i].x), 2, Scalar(0, 0, 255), 2, 8);
  	}
  	// circle( image, Point( 200, 200 ), 32.0, Scalar( 0, 0, 255 ), 1, 8 );
  	imshow("Image", new_image);
	waitKey( 0 );
}

#endif

// Unit Test
#ifdef PF_UNIT_TEST
int main(){

	return 0;
}
#endif