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

	//generator
	default_random_engine gen;

	// Vatiables used to temporarily store the final calculated values
	double pred_x = 0, pred_y = 0, pred_theta = 0;

	// Variable used to store calculations in blocks (for readability purpose)
	double sin_theta_plus_thetaDot = 0, cos_theta_plus_thetaDot = 0, sin_theta = 0, cos_theta = 0, vel_over_theta = 0;
	vel_over_theta = velocity / yaw_rate;

	//Cycle to start perform the calculations
	for(int i = 0; i < num_particles; i++)
	{

		sin_theta = sin(particles[i].theta);
		cos_theta = cos(particles[i].theta);

		// calculatoions when yaw rate is 0
		if(!(fabs(yaw_rate) > 0.001))
		{
			pred_x = particles[i].x + velocity * delta_t * cos_theta;
			pred_y = particles[i].y + velocity * delta_t * sin_theta;
			pred_theta = particles[i].theta;
		}

		// calculations for yaw rate different from 0
		else
		{
			sin_theta_plus_thetaDot = sin(particles[i].theta + (yaw_rate*delta_t));

			cos_theta_plus_thetaDot = cos(particles[i].theta + (yaw_rate*delta_t));

			pred_x = particles[i].x + vel_over_theta * (sin_theta_plus_thetaDot - sin_theta);
			pred_y = particles[i].y + vel_over_theta * (cos_theta - cos_theta_plus_thetaDot);
			pred_theta = particles[i].theta + yaw_rate*delta_t;
		}

		normal_distribution<double> dist_x(pred_x, std_pos[0]);
		normal_distribution<double> dist_y(pred_y, std_pos[1]);
		normal_distribution<double> dist_theta(pred_theta, std_pos[2]);

		particles[i].x = dist_x(gen);
		particles[i].y = dist_y(gen);
		particles[i].theta = dist_theta(gen);

	}
	//std::cout << "A prediction step was completed" << endl;

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
//list<double> ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, LandmarkObs current_k_observation) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

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

	// Cycle for weight calculation of each particle
	// 1. Transforms current observations from car coordinatios to map coordinates
	// 2. Associates observations in map coordinates to a landmark using nearest neighbor approach
	// 3. Calulates current particle weight using a Multivariate Gaussian probability
	for (int i = 0; i < num_particles; i++)
	{

		// Declares (and restarts to 1 each cycle) a Variable used to store accumulated weights for current particle
		double P_weight_acc = 1;

		// Variables for readability
		double theta_particle = particles[i].theta;
		double x_particle = particles[i].x;
		double y_particle = particles[i].y;

		//

		//Landmark vector used to store observations in map coordinates
		std::vector<LandmarkObs> observations_map_cord;

		//std::cout << "There are " << map_landmarks.landmark_list.size() << " landmarks to look for" << endl;

		//std::cout << "Total number of observations = " << observations.size() << endl;

		// Step 1 : Transfrom observations coordinates from car to map
		for(int j = 0; j < observations.size(); j++)
		{
			LandmarkObs obs_map_cord; // struct used to temporarily store current cycle parameter

			//Extract observation parameters for readability
			double x_map = observations[j].x;
			double y_map = observations[j].y;

			//calculating observations map coordinate
			obs_map_cord.id = j;
			obs_map_cord.x = x_particle + (cos(theta_particle) * x_map) - (sin(theta_particle) * y_map);
			obs_map_cord.y = y_particle + (sin(theta_particle) * x_map) + (cos(theta_particle) * y_map);

			observations_map_cord.push_back(obs_map_cord);
			//std::cout << "Particle coordinates, x = " << x_particle << ", y =" << y_particle << ", theta =" << theta_particle << endl;
			//std::cout << "Obs in car coordinates, x = " << x_map << ", y = " << y_map << endl;
			//std::cout << "Obs in map coordinates, x = " << obs_map_cord.x << ", y = " << obs_map_cord.y << endl;
		}

		// Declares (and restarts to 0) variables used for readability
		double x_dist = 0;
		double y_dist = 0;

		// Step 2 : Associate a landmark to each observation using nearest neighbor
		for (int k = 0; k < observations_map_cord.size(); k++)
		{
			
			//std::cout << "Number of observations in map coordinates = " << observations_map_cord.size() << endl;

			// Variables for radability
			double best_distance = sensor_range;
			double current_distance;

			//Compares current observation with every landmark
			for(int l = 0; l < map_landmarks.landmark_list.size(); l++)
			{
				// variables for readablity
				double x_dist = observations_map_cord[k].x - map_landmarks.landmark_list[l].x_f;
				double y_dist = observations_map_cord[k].y - map_landmarks.landmark_list[l].y_f;

				current_distance = sqrt(pow(x_dist, 2) + pow(y_dist, 2));

				// When a new shortest distance is found updates the value of best distance and assign the corresponding landmark id to the id of the observation
				if (current_distance < best_distance)
				{
					best_distance = current_distance;
					observations_map_cord[k].id = l;
				}
			}
			//std::cout << "Best distance found = " << best_distance << endl;


			//Calculation of the normalization term to use it during weight balancing
			double norm_term = 1 / (2* M_PI * std_landmark[0] * std_landmark[1]);

			//Calculation of Sigma x and Sigma y, respectively squared
			double sqr_sigma_x = std_landmark[0] * std_landmark[0];
			double sqr_sigma_y = std_landmark[1] * std_landmark[1];

			// Variables for readablity of observation coordinates(map)
			double x_obs = observations_map_cord[k].x;
			double y_obs = observations_map_cord[k].y;

			// Variables used for readability of landmark associated to current observation
			double mu_x = map_landmarks.landmark_list[observations_map_cord[k].id].x_f;
			double mu_y = map_landmarks.landmark_list[observations_map_cord[k].id].y_f;

			double x_term_sqr = pow(x_obs - mu_x, 2);
			double y_term_sqr = pow(y_obs - mu_y, 2);

			// Calculation of the exponential term
			double exponential_term = exp(-( (x_term_sqr / sqr_sigma_x) + (y_term_sqr / sqr_sigma_y) ) );

			//Accumulates temporary weight variable for each observation
			P_weight_acc *= norm_term * exponential_term;
		}

		// Stores final accumulated weight into the particles weight location
		particles[i].weight = P_weight_acc;

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

	// Random device used instead of random generator as recommended in the forum oppinio s
	//  ※ Apparently the random generators values do not correspond to an even distribution
	std::random_device rd;
	std::mt19937 gen(rd());

	for (int j = 0; j < num_particles; j++)
	{
		resample_particles.push_back(particles[distribution(gen)]);
	}
	// Upating the particles with the new resampled ones
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
