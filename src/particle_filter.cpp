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
        std::default_random_engine generator;  
        std::normal_distribution<double> distributionx(0, std[0]);  
	std::normal_distribution<double> distributiony(0, std[1]);  
	std::normal_distribution<double> distributiontheta(0, std[2]); 
       x = distributionx(generator)   ;
       y = distributiony(generator);
       theta = distributiontheta(generator);
       
       //std::cout<<"x:"<<x<<";y:"<<y<<";theta:"<<theta;
      // std::cout<<std;
      

	for(int i =0; i <num_particles ; i++)
	{
	  weights.push_back(1);
	}

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
       std::default_random_engine generator;  
       std::normal_distribution<double> distributionx(0, std_pos[0]);  
	std::normal_distribution<double> distributiony(0, std_pos[1]);  
	std::normal_distribution<double> distributiontheta(0, std_pos[2]);  
     
        
	for(int i =0; i <num_particles ; i++)
	{
		double  noise_x = distributionx(generator)   ;
		double  noise_y = distributiony(generator);
		double  noise_theta = distributiontheta(generator);

		particles[i].x = particles[i].x +noise_x + velocity/yaw_rate*(sin( particles[i].theta+yaw_rate*delta_t+ noise_theta)-sin( particles[i].theta));

		particles[i].y = particles[i].y + noise_y + velocity/yaw_rate*(cos( particles[i].theta)-cos( particles[i].theta+yaw_rate*delta_t+ noise_theta));

		particles[i].theta = particles[i].theta + noise_theta + yaw_rate*delta_t;
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
        double dist = 1000;
       int index =0;
       for(int j =0; j <num_particles ; j++)
	{
          dist = 1000;
	   for(int i =0; i <num_particles ; i++)
		{
	            double tempdist = sqrt((particles[i].y-observations[j].y)*(particles[i].y-observations[j].y)+(particles[i].x -observations[j].x)*(particles[i].x -observations[j].x));
                   if(dist > tempdist)
                    {
                      dist = tempdist;
                      index = i;
                      }
		}
            predicted[j] = particles[i];
            
	}

       
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
        double x=0;
        double y =0;
        for(int i =0; i <num_particles ; i++)
	{
	  x = observations[i].x*cos(observations[i].theta)-observations[i].y*sin(observations[i].theta)+std_landmark[0];
          y = observations[i].x*sin(observations[i].theta) + observations[i].y*cos(observations[i].theta)+std_landmark[1];
         dist < sensor_range
         weight[i] *=  exp(- ((mu - x) ** 2) / (std_landmark[0]** 2) / 2.0) / sqrt(2.0 * pi * (std_landmark[0] ** 2))
	}
     

       
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

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
