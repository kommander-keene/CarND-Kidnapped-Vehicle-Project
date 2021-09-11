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

static int NUM_PARTICLES = 128;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
    std::normal_distribution<double> x_normal(x, std[0]);
    std::normal_distribution<double> y_normal(y, std[1]);
    std::normal_distribution<double> theta_normal(theta, std[2]);
    std::default_random_engine random;
    num_particles = NUM_PARTICLES;
    particles.resize(num_particles);
    
    for (auto& p: particles) {
        //set equals
        p.x = x_normal(random);
        p.y = y_normal(random);
        p.theta = theta_normal(random);
        p.weight = 1.0;
        
    }

    is_initialized = true;

}
void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
    std::default_random_engine random;
    std::normal_distribution<double> x_N(0, std_pos[0]);
    std::normal_distribution<double> y_N(0, std_pos[1]);
    std::normal_distribution<double> theta_N(0, std_pos[2]);
    
    for (auto& p: particles) {
        double th = p.theta;
        //Add measurements
        if (fabs(yaw_rate) < 0.0001) { //returns if yaw_rate is about 0
            p.x += velocity * delta_t * cos(th); //x = vcos(0)*t
            p.y += velocity * delta_t * sin(th); //y = vsin(0)*t
        } else { //use the formulas
            p.x += (velocity/yaw_rate) * (sin(th + delta_t*yaw_rate -sin(th)));
            p.y += (velocity/yaw_rate) * (cos(th)-cos(th+yaw_rate*delta_t));
            p.theta += (yaw_rate*delta_t);
        }
        //Add noise
        p.x += x_N(random);
        p.y += y_N(random);
        p.theta += theta_N(random);
    }

}
//generates associations between observations and particles
void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, vector<LandmarkObs>& observations) {
    //predicted landmarks
    for (const auto p: predicted) {
        double min = std::numeric_limits<double>::max();
        //current observations
        for (auto& o: observations) {
            double d = dist(p.x, p.y, o.x, o.y);
            if (d < min) {
                min = d;
                o.id = p.id;
            }
        }
    }
}
std::vector<double> transform(std::vector<double> x, std::vector<double> y, double theta) {
    // x[0] is xp
    // x[1] is xc
    // transforms coordinates with a -90º rotation.
    
    double xm = x[0] + (cos(theta)*x[1]) - (sin(theta) * y[1]);
    double ym = x[0] + (cos(theta)*x[1]) - (sin(theta) * y[1]);;
    
    return std::vector<double>(xm,ym);
}
std::vector<LandmarkObs> nearest_neighbor_association(double sensor_range, const Particle particle, const Map &map_landmarks) {
    std::vector<LandmarkObs> nna;
    
    //for all of the map landmarks
    for (const auto map: map_landmarks.landmark_list) {
        double map_x = map.x_f;
        double map_y = map.y_f;
        int map_id = map.id_i;

        double p_x = particle.x;
        double p_y = particle.y;
        //Record all of the valid landmarks that apply to this one particle as LandMarkObs types
        if (dist(map_x, map_y, p_x, p_y) <= sensor_range) {
            LandmarkObs copy_mark = LandmarkObs{map_id, map_x, map_y};
            nna.push_back(copy_mark);
        }
    }
    
    return nna;
}
std::vector<LandmarkObs> transformObs(const vector<LandmarkObs> &obs, Particle p) {
    vector<LandmarkObs> to_return;
    for (const auto o: obs) {
        vector<double>x_set(p.x, o.x);
        vector<double>y_set(p.y, o.y);
        transform(x_set, y_set, p.theta);
    }
    return to_return;
}
double multivariate_weightUpdates(double x, double y, double mean_x, double mean_y, double s_x, double s_y) {
    double x_stuff = (pow((x-mean_x), 2)/(2*pow(s_x, 2)));
    double y_stuff = (pow((y-mean_y), 2)/(2*pow(s_y, 2)));
    double power_op = -(x_stuff + y_stuff);
    double P = (1.0/(2.0*M_PI*s_x*s_y) * (exp(power_op)));
    return P;
}

LandmarkObs findCurrentLandmark(int id, const std::vector<LandmarkObs> obs) {
    for (auto m: obs) {
        if (m.id == id) {
            return m;
        }
    }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], const vector<LandmarkObs> &observations, const Map &map_landmarks) {
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
    
    
    for (Particle p : particles) {
        //valid map landmarks
        std::vector<LandmarkObs> valid;
        //transformed observations
        std::vector<LandmarkObs> t_obs;
        p.weight = 1.0;
        
        valid = nearest_neighbor_association(sensor_range, p, map_landmarks);
        t_obs = transformObs(observations, p);
        //associates valid results and observations together
        dataAssociation(valid, t_obs);
        
        //factor in that observation's gaussian in order to greater weight different things
        //think of the mean as the expected value, or the actual value of the landmark.
        int index = 0;
        for (int i = 0; i < observations.size(); i++) {
        
        LandmarkObs obs = observations[i];
        //get current landmark associated with this observation
        LandmarkObs landmark = findCurrentLandmark(obs.id, valid);
        
        p.weight *= multivariate_weightUpdates(obs.x, obs.y, landmark.x, landmark.y, std_landmark[0], std_landmark[1]);
        }
    }
}
double maximumWeights(std::vector<double>& weights, const std::vector<Particle> particles) {
    double mini = std::numeric_limits<double>::min();
    for (const Particle p: particles) {
        weights.push_back(p.weight);
        if (p.weight > mini) {
            mini = p.weight;
        }
    }
    return mini;
}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional
   *   to their weight.
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
    //random vars
    std::random_device random;
    std::mt19937 wtf(random());

    //a random selection of the indicides
    std::vector<Particle> resampled;
    
    int index = rand() % (num_particles-1);
    double beta = 0.0;
    double max = maximumWeights(weights, particles);

    //a random selection of a distribution of weights
    std::discrete_distribution<double> N_w(weights.begin(), weights.end());


    for (int i=0; i < num_particles; i++) {
        beta += N_w(wtf)*2*max;

        while (weights[index] < beta) {
            beta -= weights[index];
            index = (index + 1);
        }
        resampled.push_back(particles[index]);
    }
    particles = resampled;

    weights.clear();

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
