/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */

  // check if already init is done
  if (is_initialized) {
    return;
  }

  num_particles = 500;  // TODO: Set the number of particles

  // allocate memory for weights.
  weights.resize(num_particles);
  // assign default values.
  weights.assign(num_particles, 1.0);

  // init values for random gaussiaan noise to init values.
  std::default_random_engine gen;
  std::normal_distribution<double> dist_x(x, std[0]);
  std::normal_distribution<double> dist_y(y, std[1]);
  std::normal_distribution<double> dist_theta(theta, std[2]);

  // temporary varibale to store values while same is added to list.
  Particle temp_created;

  for (int i = 0; i < num_particles; i++) {

	temp_created.id = i;
    temp_created.x = dist_x(gen);
    temp_created.y = dist_y(gen);
    temp_created.theta = dist_theta(gen);
    temp_created.weight = 1;
    particles.push_back(temp_created);

  }

  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */

    // init values for random Gaussian noise to init values.
    std::default_random_engine gen;
    std::normal_distribution<double> dist_x(0, std_pos[0]);
    std::normal_distribution<double> dist_y(0, std_pos[1]);
    std::normal_distribution<double> dist_theta(0, std_pos[2]);

    /**
     * Prediction is called to get next position based on current
     * velocity and yaw rate. Below formulas are used for that
     *
     * theta_pred = old_thetha + yar_rate * delta_time
     * x_pred = x_prev + (velocity/yaw rate) * (sin(delta_time) - sin(old_thetha))
     * y_pred = y_prev + (velocity/yaw rate) * (cos(old_thetha) + cos(delta_time))
     *
     * Used temp variables to store values used multiple times.
     */

    double theta_del_t = yaw_rate * delta_t;
    double temp_theta = 0.0;

    for (int i = 0; i < num_particles; i++) {

      if (fabs(yaw_rate)>0.001) {

        temp_theta = particles[i].theta + theta_del_t;
        particles[i].x = particles[i].x + (velocity / yaw_rate) * (sin(temp_theta) - sin(particles[i].theta)) + dist_x(gen);
        particles[i].y = particles[i].y + (velocity / yaw_rate) * (cos(particles[i].theta) - cos(temp_theta)) + dist_y(gen);
        particles[i].theta = temp_theta + dist_theta(gen);

      } else {

        particles[i].x = particles[i].x + velocity * delta_t * cos(particles[i].theta) + dist_x(gen);
        particles[i].y = particles[i].y + velocity * delta_t * sin(particles[i].theta) + dist_y(gen);

      }

    }

}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian 
   *   distribution. You can read more about this distribution here: 
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */

  /**
   * This function is heart of this project. Here weights are updates for
   * positions and that depicts our current position on Map.
   *
   * Below are main steps for this function.
   * 1: Car observes everything from its sensors, which are
   *    based on local coordinates axis. This needs to be transformed
   *    to map coordinates so that it can check for obstacles in path.
   * 2: Once we have converted local coordinates to map coordinates,
   *    we can check for obstacles in our path that are in range of
   *    sensors. Keep a list of all obstacles that are in our current range.
   * 3: Get nearest obstacles from current position. Use multivariate Gaussian
   *    probability density having X and Y direction. It does not uses Theta
   *    as of now.
   *
   * TODO: This is quite un-optimized as of now. It uses 3 for loops
   *       for number of particles, number of observations and number of
   *       landmarks.
   *
   * It use temporary variable to store values in repeated computations.
   *
   */

  double gauss_norm = 1 / (2 * M_PI * std_landmark[0] * std_landmark[1]);
  double x_demon = 1 / (2 * std_landmark[0] * std_landmark[0]);
  double y_demon = 1 / (2 * std_landmark[1] * std_landmark[1]);
  double gaus_dist = 1.0;
  double map_x,map_y;
  vector<double> dist_list(map_landmarks.landmark_list.size());
  double current_dist = 0;
  double dist_max = std::numeric_limits<float>::max();
  int min_index = 0;
  double x_min, y_min;
  double exponent;

  for (int i = 0; i < num_particles; i++) {

      gaus_dist = 1.0;

    for (int j = 0; j < observations.size(); j++) {

      map_x = particles[i].x + cos (particles[i].theta) * observations[j].x
                    - sin(particles[i].theta) * observations[j].y;
      map_y = particles[i].y + cos (particles[i].theta) * observations[j].y
                +  sin(particles[i].theta) * observations[j].x;

      for (int k = 0; k < map_landmarks.landmark_list.size(); k++) {

        current_dist = dist(map_x,map_y,
                        map_landmarks.landmark_list[k].x_f,
                        map_landmarks.landmark_list[k].y_f);

        if (current_dist < sensor_range) {
          dist_list[k] = current_dist;
        } else {
          dist_list[k] = dist_max;
        }
      }

      min_index = std::min_element(dist_list.begin(),dist_list.end()) - dist_list.begin();
      x_min = map_landmarks.landmark_list[min_index].x_f;
      y_min = map_landmarks.landmark_list[min_index].y_f;
      exponent = (((pow((map_x - x_min),2) / x_demon) +
                      (pow((map_y - y_min),2)/ y_demon)));

      gaus_dist = gaus_dist * gauss_norm * exp(-exponent);
    }

    particles[i].weight = gaus_dist;
    weights[i] = gaus_dist;
//    std::cout<< gaus_dist << std::endl;
  }

}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */

  /**
   *  Once all weights are calculated we need to draw samples based on
   *  their weights. High weight will lead to high probability of get it
   *  resampled. This uses Roulette Wheel algorithm to draw sampled from
   *  weight list. This new sample is made as old sample for next iteration
   *  and process continues in loop.
   */
    double max_weight = *std::max_element(weights.begin(), weights.end());
    std::default_random_engine generator;
    std::uniform_int_distribution<int> distribution(0,(num_particles -1));
    double beta;
    int index = distribution(generator);
    vector<Particle> new_particles;

    for(int i = 0; i < num_particles ; i++) {

      beta = weights[index] + 2 * max_weight;
      while(weights[index] < beta) {

         beta = beta - weights[index];
         index = (index + 1) % num_particles;

      }

      new_particles.push_back(particles[index]);
    }

    particles = new_particles;
}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}
