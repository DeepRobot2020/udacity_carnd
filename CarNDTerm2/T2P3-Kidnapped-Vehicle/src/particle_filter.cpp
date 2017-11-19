/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <random>
#include <iostream>
#include <tuple>
#include <numeric>
#include <math.h>
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

void ParticleFilter::init(double x, double y, double theta, double std[])
{
	if(is_initialized)
		return;
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of
	//   x, y, theta and their uncertainties from GPS) and all weights to 1.
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	std::default_random_engine gen;
	double std_x     = std[0];
	double std_y     = std[1];
	double std_theta = std[2];
	
	std::normal_distribution<double> norm_x(x, std_x);
	std::normal_distribution<double> norm_y(y, std_y);
	std::normal_distribution<double> norm_theta(theta, std_theta);

	num_particles = 250;
	for(int i = 0; i < num_particles; i++)
	{
		Particle p;		
		p.id = i;
		p.x = norm_x(gen);
		p.y = norm_y(gen);
		p.theta = norm_theta(gen);
		p.weight = 1.0f;
		particles.push_back(p);
	}
	// initialize the weights vector to be all one
	weights.resize(num_particles, 1.0);
	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate)
{
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	std::default_random_engine gen;
	double std_x     = std_pos[0];
	double std_y     = std_pos[1];
	double std_theta = std_pos[2];

	for (auto &p : particles)
	{
		// avoid division by zero
		double xf, yf, thetaf;
		double yaw_rate_prod_dt = delta_t * yaw_rate;
		if (fabs(yaw_rate) < 0.0001)
		{
			xf = p.x + velocity * delta_t * cos(p.theta);
			yf = p.y + velocity * delta_t * sin(p.theta);
		}
		else
		{
			double v_div_yaw_rate = velocity / yaw_rate;
			xf = p.x + v_div_yaw_rate * (sin(p.theta + yaw_rate_prod_dt) - sin(p.theta));
			yf = p.y + v_div_yaw_rate * (cos(p.theta) - cos(p.theta + yaw_rate_prod_dt));
		}
		thetaf = p.theta + yaw_rate_prod_dt;
		std::normal_distribution<double> norm_x(xf, std_x);
		std::normal_distribution<double> norm_y(yf, std_y);
		std::normal_distribution<double> norm_theta(thetaf, std_theta);
		// Predict the x,y, theta value of each particle filter with random gaussin noise. 
		p.x = norm_x(gen);
		p.y = norm_y(gen);
		p.theta = norm_theta(gen);
	}
}

// Note: all the items are in map coordinates
void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, 
									 std::vector<LandmarkObs> &observations)
{
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
	//   implement this method and use it as a helper during the updateWeights phase.
	for(auto &obs : observations)
	{
		double min_distance = std::numeric_limits<double>::max();
		const double obs_x = obs.x;
		const double obs_y = obs.y;
		obs.id = -1;
		for(int i = 0; i < predicted.size(); i++)
		{
			const double pred_x = predicted[i].x;
			const double pred_y = predicted[i].y;
			const double cur_distance = dist(obs_x, obs_y, pred_x, pred_y);
			if(cur_distance < min_distance)
			{
				obs.id = i;
				min_distance = cur_distance;
			}
		}
		if(obs.id != -1)
		{
			obs.x -= predicted[obs.id].x;
			obs.y -= predicted[obs.id].y;
		}
	}
} 

static double normal_2d(const double std_landmark[], double dist_x, double dist_y)
{
	double std_landmark_x = std_landmark[0];
	double std_landmark_y = std_landmark[1];
	double normalizer = (2.0*M_PI*std_landmark_x*std_landmark_y);

	double x_part = (-1)*dist_x * dist_x / (2*std_landmark_x*std_landmark_x);
	double y_part = (-1)*dist_y * dist_y / (2*std_landmark_y*std_landmark_y);

	return exp(x_part + y_part) / normalizer;
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
								   const std::vector<LandmarkObs> &observations, const Map &map_landmarks)
{
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
	int p_index = 0;
	for (auto &p : particles)
	{
		// auto p = particles[p_idx];
		double p_x = p.x;
		double p_y = p.y;
		double p_theta = p.theta;

		std::vector<LandmarkObs> predicted_measurements;
		// assume the car is in current particle location, calculate predict measurements
		for (const auto lm : map_landmarks.landmark_list)
		{
			double lm_x = lm.x_f;
			double lm_y = lm.y_f;
			int lm_id = lm.id_i;
			double distance_to_particle = dist(lm_x, lm_y, p_x, p_y);
			if (distance_to_particle > sensor_range)
				continue;
			LandmarkObs lm_pred;
			lm_pred.x = lm_x;
			lm_pred.y = lm_y;
			lm_pred.id = lm_id;
			predicted_measurements.push_back(lm_pred);
		}

		std::vector<LandmarkObs> observations_particle;

		for (const auto &ob : observations)
		{
			double ob_x = ob.x;
			double ob_y = ob.y;
			// map the observation to map cooridnates
			double ob_x_part = p_x + cos(p_theta) * ob_x - sin(p_theta) * ob_y;
			double ob_y_part = p_y + sin(p_theta) * ob_x + cos(p_theta) * ob_y;
			// remove the observations out of sensor range
			double distance_to_particle = dist(ob_x_part, ob_y_part, p_x, p_y);
			if(distance_to_particle > sensor_range)
				continue;

			LandmarkObs lm_obs;
			lm_obs.x = ob_x_part;
			lm_obs.y = ob_y_part;
			lm_obs.id = -1;
			observations_particle.push_back(lm_obs);
		}
		dataAssociation(predicted_measurements, observations_particle);
		// update weight for this partcile
		// assume each measurement to be independent
		double weight_updated = 1.0f;
		for(const auto &ob : observations_particle)
		{
			if(ob.id == -1)
				continue;
			double weight_i = normal_2d(std_landmark, ob.x, ob.y);
			weight_updated *= weight_i;
		}

		p.weight = weight_updated;
		weights[p_index++] = p.weight;
	}
}

void ParticleFilter::resample()
{
	// TODO: Resample particles with replacement with probability proportional to their weight.
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	std::default_random_engine gen;
	std::discrete_distribution<> d(weights.begin(), weights.end());
	std::vector<Particle> particles_resampled;
	std::vector<double> weights_resampled;
	
	for(int i = 0; i < num_particles; i++)
	{
		// randomly generated one id and moduble it into range [0, num_particles)
		int id = d(gen) % num_particles; 
		particles_resampled.push_back(particles[id]);
		weights_resampled.push_back(particles[id].weight);
	}
	// update weights and particles vector
	particles = particles_resampled;
	weights = weights_resampled;	
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

std::string ParticleFilter::getAssociations(Particle best)
{
	std::vector<int> v = best.associations;
	std::stringstream ss;
    copy( v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
    std::string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}

std::string ParticleFilter::getSenseX(Particle best)
{
	std::vector<double> v = best.sense_x;
	std::stringstream ss;
    copy( v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
    std::string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}

std::string ParticleFilter::getSenseY(Particle best)
{
	std::vector<double> v = best.sense_y;
	std::stringstream ss;
    copy( v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
    std::string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
