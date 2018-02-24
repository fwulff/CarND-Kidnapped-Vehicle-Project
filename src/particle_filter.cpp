/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>

#include "particle_filter.h"

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// Set the number of particles. Initialize all particles to first position (based on estimates of
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

	// Set number of particles to be used, more particles can result in lower error, but higher runtime
    // num_particles = 10; // Cumulative mean weighted error: x 0.190679 y 0.15542 yaw 0.00606433, Runtime (sec): 0.614531, Success! Your particle filter passed!
    // num_particles = 50; // Cumulative mean weighted error: x 0.123841 y 0.126784 yaw 0.00432963, Runtime (sec): 2.23674, Success! Your particle filter passed!

    num_particles = 100; // Cumulative mean weighted error: x 0.120595 y 0.12257 yaw 0.00400091, Runtime (sec): 4.29132, Success! Your particle filter passed!

    // num_particles = 500; // Cumulative mean weighted error: x 0.123069 y 0.119063 yaw 0.00394135, Runtime (sec): 20.5638, Success! Your particle filter passed!
    // num_particles = 1000; // Cumulative mean weighted error: x 0.124452 y 0.116551 yaw 0.00394275, Runtime (sec): 40.4797, Success! Your particle filter passed!


    std::default_random_engine gen;
    double std_x, std_y, std_theta; // Standard deviations for x, y, and theta

    // Set standard deviations for x, y, and theta
    std_x = std[0];
    std_y = std[1];
    std_theta = std[2];


    // This line creates a normal (Gaussian) distribution for x
    std::normal_distribution<double> dist_x(x, std_x);
    std::normal_distribution<double> dist_y(y, std_y);
    std::normal_distribution<double> dist_theta(theta, std_theta);


    for (int i = 0; i < num_particles; ++i) {
		double sample_x, sample_y, sample_theta;

        // create a new particle struct
        Particle new_particle;

        // Sample  and from these normal distrubtions:
        sample_x = dist_x(gen);
        sample_y = dist_y(gen);
        sample_theta = dist_theta(gen);

        // set struct
        new_particle.id = i;
        new_particle.x = sample_x;
        new_particle.y = sample_y;
        new_particle.theta = sample_theta;

        // weights need to sum to 1.0
        new_particle.weight = 1.0/num_particles;

        // push back particle into particles
        particles.push_back(new_particle);
        weights.push_back(new_particle.weight);

		// Print new particle
		//std::cout << "Initialised particle " << i + 1 << " with x: " << sample_x << " y: " << sample_y << " theta: " << sample_theta << std::endl;
    }

    is_initialized = true;

    std::cout << "Initialised particle filter with number of particles: " << num_particles << std::endl;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/


    // get motion noise generator
    std::default_random_engine gen;
    double std_x, std_y, std_theta; // Standard deviations for x, y, and theta

    // Set standard deviations for x, y, and theta
    std_x = std_pos[0];
    std_y = std_pos[1];
    std_theta = std_pos[2];

    // for all particles
    for (int i = 0; i < num_particles; ++i) {

        double predicted_x, predicted_y, predicted_theta;

        // Motion model with yaw_rate = 0
        // x_new = x_old + velocity * dt * cos(theta_old)
        // y_new = y_old + velocity * dt * sin(theta_old)
        // theta_new = theta_old

        // Motion model with yaw_rate != 0
        // x_new = x_old + (velocity/yaw_rate) * (sin(theta_old + yaw_rate(dt) - sin(theta_old))
        // y_new = y_old + (velocity/yaw_rate) * (cos(theta_old - cos(theta_old + yaw_rate(dt))
        // theta_new = theta_old + yaw_rate(dt)

        // Predict new  if yaw_rate != 0
        if(yaw_rate != 0.) {
            predicted_x = particles[i].x + ((velocity / yaw_rate) *
                                            (sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta)));
            predicted_y = particles[i].y + ((velocity / yaw_rate) *
                                            (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t)));
            predicted_theta = particles[i].theta + yaw_rate * delta_t;
        }
        else{
            predicted_x = particles[i].x + velocity * delta_t * cos(particles[i].theta);
            predicted_y = particles[i].y + velocity * delta_t * sin(particles[i].theta);
            predicted_theta = particles[i].theta;
        }

        // This line creates a normal (Gaussian) distribution for each particles new value x, y, theta
        std::normal_distribution<double> dist_x(predicted_x, std_x);
        std::normal_distribution<double> dist_y(predicted_y, std_y);
        std::normal_distribution<double> dist_theta(predicted_theta, std_theta);

        double noisy_x, noisy_y, noisy_theta;

        // Sample  and from these normal distributions to add movement noise to motion model
        noisy_x = dist_x(gen);
        noisy_y = dist_y(gen);
        noisy_theta = dist_theta(gen);

        // set particle position
        particles[i].x = noisy_x;
        particles[i].y = noisy_y;
        particles[i].theta  = noisy_theta;

        // Print new particle
        //std::cout << "Updated particle " << i + 1 << " to x: " << noisy_x << " y: " << noisy_y << " theta: " << noisy_theta << std::endl;
    }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

    /* NOT USED, done directly and individually for each particle and observations in updateWeights*/

}


void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], std::vector<LandmarkObs> observations, Map map_landmarks) {
	// Update the weights of each particle using a multi-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33. Note that you'll need to switch the minus sign in that equation to a plus to account 
	//   for the fact that the map's y-axis actually points downwards.)
	//   http://planning.cs.uiuc.edu/node99.html

    // transform all coordinates of observations according to the individual "vehicle = particle" position
    for (int i = 0; i < num_particles; i++){

        //std::cout << "updating weights of particle " << i << std::endl;

        // individual observations vector for all particles
        std::vector<LandmarkObs> particle_observations;

        for (int obs = 0; obs < observations.size(); obs++){
            // individual observation of this particle
            LandmarkObs particle_observation;

            // transform observations (vehicle coordinates) into map coordinate
            // http://planning.cs.uiuc.edu/node99.html 3.3
            // x_transformed = (x * cos(theta) - y * sin(theta) + x_translation)
            // y_transformed = (x * sin(theta) + y * cos(theta) + y_translation)
            particle_observation.x = observations[obs].x * cos(particles[i].theta) - observations[obs].y * sin(particles[i].theta) + particles[i].x;
            particle_observation.y = observations[obs].x * sin(particles[i].theta) + observations[obs].y * cos(particles[i].theta) + particles[i].y;

            // add transformed observation to vector
            particle_observations.push_back(particle_observation);

            //std::cout << "Converted observation " << obs << " to x y " << particle_observation.x << " " << particle_observation.y << std::endl;
        }

        // iterate all transformed observations to associate landmarks and update weights
        for (int obs = 0; obs < particle_observations.size(); obs++) {

            double distance_to_landmark = sensor_range;
            int associated_landmark = -1;

            // iterate all landmarks and calculate distance to observation
            for (int landmark = 0; landmark < map_landmarks.landmark_list.size(); landmark++) {
                double landmark_pos_x = map_landmarks.landmark_list[landmark].x_f;
                double landmark_pos_y = map_landmarks.landmark_list[landmark].y_f;

                // Euclidean distance
                double current_distance = std::sqrt(pow(particle_observations[obs].x - landmark_pos_x, 2.) + pow(particle_observations[obs].y - landmark_pos_y, 2.));

                // associate observation, if closest distance found
                if (current_distance < distance_to_landmark) {
                    distance_to_landmark = current_distance;
                    associated_landmark = landmark;
                }
            }
            //std::cout << "Associate observation " << obs << " to landmark " << associated_landmark << std::endl;

            // check if associated landmark was found
            if(associated_landmark != -1){

                // update weight of particles using a multivariate gaussian probability density function

                // distances in 1D
                double distance_x = sqrt(pow(particle_observations[obs].x - map_landmarks.landmark_list[associated_landmark].x_f, 2.0));
                double distance_y = sqrt(pow(particle_observations[obs].y - map_landmarks.landmark_list[associated_landmark].y_f, 2.0));

                // Probability density function 2D
                double denominator_factor = 1/(2*M_PI*std_landmark[0]*std_landmark[1]);
                double exponential_factor = exp(-2*(distance_x/(2*pow(std_landmark[0],2.0))+distance_y/(2*pow(std_landmark[1],2.0))));
                double weight_factor = denominator_factor*exponential_factor;

                particles[i].weight *= weight_factor;

                //std::cout << "Updated particle " << i << " with weight factor " << multiplier << " to weight " << particles[i].weight << std::endl;
            }
	    else{
		// if no landmark is associated, set particle weight to zero (impossible)
		particles[i].weight = 0.;	
	    }
        }


        // update weights vector for resampling
        weights[i] = particles[i].weight;
    }
}

void ParticleFilter::resample() {
	// Resample particles with replacement with probability proportional to their weight.
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
    // create new particles with equal weight 1.0

    // first normalise weights to sum of 1.0
    double sum_of_weights = std::accumulate(weights.begin(), weights.end(), 0.0);
    for (int i = 0; i < weights.size(); i++){
        weights[i] = weights[i]/sum_of_weights;
    }

    // produces random integers on the interval [0, n), where the probability of each individual integer by the weight of the ith integer divided by the sum of all n weights.
    std::default_random_engine gen;
    std::discrete_distribution<int> resampled_distribution(weights.begin(), weights.end());

    // vector to hold the new resampled particle set
    std::vector<Particle> new_particles;

    // iterate all particles 
    for (int i = 0; i < num_particles; i++)
    {
        // choose particle from by random integer from discrete distribution generator
        Particle new_particle = particles[resampled_distribution(gen)];

        // re-initialise equal weights for all particles (to sum of 1.0)
        new_particle.weight = 1.0/num_particles;
        new_particles.push_back(new_particle);
    }

    // create a new set of particles distributed according to the probability of their previous position
    particles = new_particles;
}

void ParticleFilter::write(std::string filename) {
	// You don't need to modify this file.
	std::ofstream dataFile;
	dataFile.open(filename, std::ios::app);
	for (int i = 0; i < num_particles; ++i) {
		dataFile << particles[i].x << " " << particles[i].y << " " << particles[i].theta << "\n";
	}
	dataFile.close();
}
