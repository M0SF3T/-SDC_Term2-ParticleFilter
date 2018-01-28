/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	default_random_engine gen;

	//Number of particles
	num_particles = 100;

	// Creating normal gaussians for each variable
	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);

	// Asign length of particles and weights vectors
	particles.resize(num_particles);
	weights.resize(num_particles);

	for(int i = 0; i < num_particles; i++)
	{
		// Iterates over particles assigning a sample of the gaussian generated with GPS measurements
		particles[i].id = i;
		particles[i].x = dist_x(gen);
		particles[i].y = dist_y(gen);
		particles[i].theta = dist_theta(gen);
		particles[i].weight = 1;


		// Iterates over weight and intialitze its value to 1
		weights[i] = 1;

	}
	//std::cout << "particles initialized" << endl;
	is_initialized = true;
	//std::cout << "Initialization complete" << endl;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution 
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	default_random_engine gen;

	double pred_x = 0, pred_y = 0, pred_theta = 0;

	//normal_distribution<double> dist_x(pred_x, std_pos[0]);
	//normal_distribution<double> dist_y(pred_y, std_pos[1]);
	//normal_distribution<double> dist_theta(pred_theta, std_pos[2]);

	double sin_theta_plus_thetaDot = 0, cos_theta_plus_thetaDot = 0, sin_theta = 0, cos_theta = 0, vel_over_theta = 0;

	vel_over_theta = velocity / yaw_rate;

	for(int i = 0; i < num_particles; i++)
	{

		sin_theta = sin(particles[i].theta);
		cos_theta = cos(particles[i].theta);

		if(!(fabs(yaw_rate) > 0.001))
		{
			pred_x = particles[i].x + velocity * delta_t * cos_theta;
			pred_y = particles[i].y + velocity * delta_t * sin_theta;
			pred_theta = particles[i].theta;
		}
		// Iterates over particles assigning a sample of the gaussian generated with GPS measurements
		else
		{
			sin_theta_plus_thetaDot = sin(particles[i].theta + (yaw_rate*delta_t));

			cos_theta_plus_thetaDot = cos(particles[i].theta + (yaw_rate*delta_t));

			pred_x = particles[i].x + vel_over_theta * (sin_theta_plus_thetaDot - sin_theta);
			pred_y = particles[i].y + vel_over_theta * (cos_theta - cos_theta_plus_thetaDot);
			pred_theta = particles[i].theta + yaw_rate*delta_t;
			//particles[i].weight = 1;
		}

		normal_distribution<double> dist_x(pred_x, std_pos[0]);
		normal_distribution<double> dist_y(pred_y, std_pos[1]);
		normal_distribution<double> dist_theta(pred_theta, std_pos[2]);

		particles[i].x = dist_x(gen);
		particles[i].y = dist_y(gen);
		particles[i].theta = dist_theta(gen);


		// Iterates over weight and intialitze its value to 1
		//weights[i] = 1;
	}
	//std::cout << "A prediction step was completed" << endl;

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
//list<double> ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, LandmarkObs current_k_observation) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	

	double x_dist = 0; //abs(observations[0].x - predicted[0].x);
	double y_dist = 0; // abs(observations[0].y - predicted[0].y);
	//observations[0].id = 0;

	//best_distance = sqrt(pow(x_dist, 2) + pow(y_dist, 2));

	for (int i = 0; i < observations.size(); i++)
	{
		
		double best_distance = 50;
		double current_distance = best_distance;
		//int k = 0;
		//x_dist = abs(observations[i].x - predicted[k].x);
		//y_dist = abs(observations[i].y - predicted[k].y);

		for(int k = 0; k < predicted.size(); k++)
		{
			// @
			double x_dist = abs(observations[i].x - predicted[k].x);
			double y_dist = abs(observations[i].y - predicted[k].y);

			current_distance = sqrt(pow(x_dist, 2) + pow(y_dist, 2));

			if (current_distance < best_distance)
			{
				best_distance = current_distance;
				observations[i].id = k;
			}
		}
	}
	//std::cout << "Finised an association process" << endl;
	//returns the values of (x- Mu_x) and (y - Mu_y) for the best association found
	//list<double> best_association_values[2];
	//best_association_values[0] = best_association.x;
	//best_association_values[1] = best_association.y;

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html
	std::vector<LandmarkObs> observations_map_cord;
	//std::cout << "Update started" << endl;
	std::cout << "Particles size = " << particles.size() << endl;

	//Calculation of the normalization term to use it during weight balancing
	double norm_term = 1 / (2* M_PI * std_landmark[0] * std_landmark[1]);

	//Calculation of Sigma x and Sigma y, respectively squared
	double sqr_sigma_x = std_landmark[0] * std_landmark[0];
	double sqr_sigma_y = std_landmark[1] * std_landmark[1];

	for (int i = 0; i < particles.size(); i++)
	{
		//std::cout << "Entering an update cycle" << endl;
		//Vector to store landmarks predicted within sensor range
		std::vector<LandmarkObs> predicted;

		//Extract particle parameters for readability
		double _theta = particles[i].theta;
		double _xp = particles[i].x;
		double _yp = particles[i].y;

		std::cout << "There are " << map_landmarks.landmark_list.size() << " landmarks to look for" << endl;

		for (int m = 0; m <map_landmarks.landmark_list.size(); m++)
		{ 
			//Extract map landmarks parameters for readability
			LandmarkObs current_landmark;

			current_landmark.id = m;
			current_landmark.x = map_landmarks.landmark_list[m].x_f;
			current_landmark.y = map_landmarks.landmark_list[m].y_f;

			if ( sqrt(pow(_xp - current_landmark.x, 2) + pow(_yp - current_landmark.y, 2) ) <= sensor_range )
			{
				predicted.push_back(current_landmark);
			}
			//std::cout << "Found " << predicted.size() <<" landmarks withing sensor range" << endl;
			//std::cout << "M is currently = " << m << endl;

		}
		//std::cout << "Done finding landmarks within sensor range" << endl;
		//std::cout << "There are " << observations.size() << " observations in the current step" << endl;

		for(int j = 0; j < observations.size(); j++)
		{
			LandmarkObs current_observation; // struct used to temporarily store current cycle parameter

			//Extract observation parameters for readability
			double _xc = observations[j].x;
			double _yc = observations[j].y;
			//std::cout << "Parameters x and y of observations extracted" << endl;

			//calculating observations map coordinate
			current_observation.x = _xp + (cos(_theta) * _xc) - (sin(_theta) * _yc);
			current_observation.y = _yp + (sin(_theta) * _xc) + (cos(_theta) * _yc);

			observations_map_cord.push_back(current_observation);
			//observations[j].x = current_observation.x;
			//observations[j].y = current_observation.y;

			//std::cout << "Finished " << predicted.size() <<" landmarks withing sensor range" << endl;
			//std::cout << "J is currently = " << j << endl;
		}

		dataAssociation(predicted, observations_map_cord);

		for (int k = 0; k < observations_map_cord.size(); k++)
		{
			double x_obs = observations_map_cord[k].x;
			double y_obs = observations_map_cord[k].y;

			double mu_x = predicted[observations_map_cord[k].id].x;
			double mu_y = predicted[observations_map_cord[k].id].y;

			double x_term_sqr = (x_obs - mu_x) * (x_obs - mu_x);
			double y_term_sqr = (y_obs - mu_y) * (y_obs - mu_y);

			//particles[i].weight *= norm_term * exp(-1 * ( (x_term_sqr / sqr_sigma_x) + (y_term_sqr / sqr_sigma_y) ) );
			particles[i].weight *= norm_term * exp(-1 * ( (x_term_sqr / sqr_sigma_x) + (y_term_sqr / sqr_sigma_y) ) );
			//std::cout << weights[i] << endl;

		}
		//std::cout << weights << endl;

	}
	//std::cout << "An update step was completed" << endl;
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	//default_random_engine gen;
	std::vector <float> local_weights;

	for (int i = 0; i < num_particles; i++)
	{
		local_weights.push_back(particles[i].weight);
	}

	discrete_distribution<int> distribution(local_weights.begin(), local_weights.end());

	vector<Particle> resample_particles;

	std::random_device rd;
	std::mt19937 gen(rd());

	for (int j = 0; j < num_particles; j++)
	{
		resample_particles.push_back(particles[distribution(gen)]);
	}

	particles = resample_particles;

}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations= associations;
 	particle.sense_x = sense_x;
 	particle.sense_y = sense_y;

 	return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
