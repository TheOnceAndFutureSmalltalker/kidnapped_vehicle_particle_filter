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

	// initialize number of particles
	num_particles = 200;

    // This creates a normal (Gaussian) distribution for x, y, and theta
	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);

	// create the Gaussian random number generator
	default_random_engine gen;

	// initialize the particles at current GPS location with some random noise and weight of 1.0
	for(int i=0;i<num_particles;i++)
    {
        Particle particle;
        particle.id = i;
        particle.x = dist_x(gen);
        particle.y = dist_y(gen);
        particle.theta = dist_theta(gen);
        particle.weight = 1.0;
        particles.push_back(particle);
    }

    // make sure we don't do this step again!
	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {

    // find new position and direction for each of the particles based on time, velocity, yaw rate,
    // and some noise parameters
    default_random_engine gen;
    for(int i=0;i<num_particles;i++)
    {
        // predict new position of particle based on time, velocity and theta dot
        double x_pred = 0.0;
        double y_pred = 0.0;
        if(abs(yaw_rate) < 0.001)
        {
            x_pred = particles[i].x + velocity * delta_t * cos(particles[i].theta);
            y_pred = particles[i].y + velocity * delta_t * sin(particles[i].theta);
        }
        else
        {
            x_pred = particles[i].x + velocity/yaw_rate*(sin(particles[i].theta + yaw_rate*delta_t) - sin(particles[i].theta));
            y_pred = particles[i].y + velocity/yaw_rate*(cos(particles[i].theta) - cos(particles[i].theta + yaw_rate*delta_t));
        }
        double theta_pred = particles[i].theta + yaw_rate * delta_t;

        // add some random noise for each particle
        normal_distribution<double> dist_x(x_pred, std_pos[0]);
        normal_distribution<double> dist_y(y_pred, std_pos[1]);
        normal_distribution<double> dist_theta(theta_pred, std_pos[2]);

        particles[i].x = dist_x(gen);
        particles[i].y = dist_y(gen);
        particles[i].theta = dist_theta(gen);
    }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {

    // for each observation, find the landmark (predicted) closest to it and associate the two
	for(int i=0;i<observations.size();i++)
    {
        double cur_dist = 10000.0;
        for(int j=0;j<predicted.size();j++)
        {
            double new_dist = dist(observations[i].x, observations[i].y, predicted[j].x, predicted[j].y);
            if(new_dist < cur_dist)
            {
                cur_dist = new_dist;
                observations[i].id = predicted[j].id;
            }
        }
    }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {

	// define some intermediate values used in subsequent calculations
	double sigma_x = std_landmark[0];
	double sigma_y = std_landmark[1];
	double gauss_norm = 1.0 / (2 * M_PI * sigma_x * sigma_y);
	double weight_sum = 0.0;

	// for each particle, do the following...
	for(int i=0;i<num_particles;i++)
    {
        // get particle's predicted map location and heading
        double p_x = particles[i].x;
        double p_y = particles[i].y;
        double p_theta = particles[i].theta;

        // find out which map landmarks are within sensor range of particle's map location
        std::vector<LandmarkObs> sensor_range_landmarks;
        for(int j=0;j<map_landmarks.landmark_list.size();j++)
        {
            double lm_x = map_landmarks.landmark_list[j].x_f;
            double lm_y = map_landmarks.landmark_list[j].y_f;
            double distance = dist(p_x, p_y, lm_x, lm_y);
            if(distance < sensor_range)
            {
                LandmarkObs landmark;
                landmark.id = map_landmarks.landmark_list[j].id_i;
                landmark.x = lm_x;
                landmark.y = lm_y;
                sensor_range_landmarks.push_back(landmark);
            }
        }

        // translate/rotate observations from car coordinate system to map coordinate system
        std::vector<LandmarkObs> map_coord_observations;
        for(int j=0;j<observations.size();j++)
        {
            LandmarkObs map_coord_observation;
            map_coord_observation.x = p_x + observations[j].x * cos(p_theta) - observations[j].y * sin(p_theta);
            map_coord_observation.y = p_y + observations[j].x * sin(p_theta) + observations[j].y * cos(p_theta);
            map_coord_observations.push_back(map_coord_observation);
        }

        // associate map coordinate observations with map landmarks that are in range of particle
        dataAssociation(sensor_range_landmarks, map_coord_observations);

        // calculate particle's weight
        double w = 1.0;
        for(int j=0;j<map_coord_observations.size();j++)
        {
            // determine a weight for each of the particle's observations
            // by using a Gaussian multivariate probability density function
            // to determine the likelihood that the observation is actually observing
            // the associated landmark
            double x = map_coord_observations[j].x;
            double y = map_coord_observations[j].y;
            int id = map_coord_observations[j].id;
            double mu_x = map_landmarks.landmark_list[id-1].x_f;
            double mu_y = map_landmarks.landmark_list[id-1].y_f;
            double exponent = ((x-mu_x)*(x-mu_x)/(2*sigma_x*sigma_x)) + ((y-mu_y)*(y-mu_y)/(2*sigma_y*sigma_y));
            double w_obs = gauss_norm * exp(-exponent);
            // particle total weight is product of all observation weights
            w *= w_obs;
        }
        particles[i].weight = w;

        // accumulate weights across all particles for normalization
        weight_sum += w;
    }

    // now normalize weights
    for(int i=0;i<num_particles;i++)
    {
        particles[i].weight = particles[i].weight / weight_sum;
    }
}

void ParticleFilter::resample() {
	// NOTE:  this code uses Sebastian's re-sampling wheel algorithm from exercise!

	// initialize beta, index, max weight, and new samples vector
	double beta = 0;
	double w_max = 0.0;
	std::default_random_engine generator;
	std::discrete_distribution<int> index_gen(0,num_particles);
	int index = index_gen(generator);
	std::vector<Particle> samples;

	// find max_weight
	for(int i=0;i<num_particles;i++)
    {
        if(particles[i].weight > w_max) w_max = particles[i].weight;
    }

    // for each particle, sample a replacement particle
    for(int i=0;i<num_particles;i++)
    {
        // calculate new beta
        double rand_0_1 = (double)rand()/((double)RAND_MAX+1);
        double rand_0_2wmax = rand_0_1 * 2 * w_max;
        beta += rand_0_2wmax;
        // use beta to find index of new sample
        while(beta > particles[index].weight)
        {
            beta -= particles[index].weight;
            index += 1;
            if(index >= num_particles) index = 0;
        }
        // copy sampled particle and add it to samples
        Particle sample;
        sample.id = i;
        sample.x = particles[index].x;
        sample.y = particles[index].y;
        sample.theta = particles[index].theta;
        sample.weight = particles[index].weight;
        samples.push_back(sample);
    }
    // now make the samples the new particles list
    particles = samples;
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations,
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
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
