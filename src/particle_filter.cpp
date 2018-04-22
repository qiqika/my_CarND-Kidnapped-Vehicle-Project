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
        if (is_initialized) {
    return;
  } else {
        std::default_random_engine generator;  
        std::normal_distribution<double> distributionx(x, std[0]);  
	std::normal_distribution<double> distributiony(y, std[1]);  
	std::normal_distribution<double> distributiontheta(theta, std[2]); 
      
       //std::cout<<"sssssss"<<num_particles<<std::endl;
       //std::cout<<"x:"<<x<<";y:"<<y<<";theta:"<<theta;
      // std::cout<<std;
      
        num_particles =100;
	for(int i =0; i <num_particles ; i++)
	{
            Particle p;
            p.id = i;
            p.x = distributionx(generator)   ;
            p.y = distributiony(generator);
            p.theta = distributiontheta(generator);

            
            p.weight = 1;

            particles.push_back(p);
            weights.push_back(1);
	  
	}
       is_initialized = true;
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
                
                if (fabs(yaw_rate) < 0.00001) {  
		      particles[i].x += velocity * delta_t * cos(particles[i].theta) + noise_x;
		      particles[i].y += velocity * delta_t * sin(particles[i].theta) + noise_y;
                      particles[i].theta +=  noise_theta ;
               } 
              else {
		particles[i].x += noise_x + velocity/yaw_rate*(sin( particles[i].theta+yaw_rate*delta_t)-sin( particles[i].theta));

		particles[i].y += noise_y + velocity/yaw_rate*(cos( particles[i].theta)-cos( particles[i].theta+yaw_rate*delta_t));

		particles[i].theta += noise_theta + yaw_rate*delta_t;
            }
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
        double dist = numeric_limits<double>::max();
       int index =0;
       
       for(int j =0; j <observations.size() ; j++)
	{
          dist = numeric_limits<double>::max();
          index = 0;
	   for(int i =0; i <predicted.size() ; i++)
		{
                    double dx = predicted[i].x -observations[j].x;
                    double dy = predicted[i].y-observations[j].y;
	            double tempdist = dx*dx + dy*dy;
                   if(dist > tempdist)
                    {
                      dist = tempdist;
                      index = i;
                      }
		}
             
             observations[j].id = index;//predicted(standard) is the key and id is the index of predicted with observation ****************
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

        
        
        
        
        for(int i =0; i <  num_particles; i++)
        {
           
           
           double p_x = particles[i].x;
           double p_y = particles[i].y;
           double p_theta = particles[i].theta;

          //transform-------from observation(vehicle) to map with particles
	   std::vector<LandmarkObs> temp_trans;

	   for(int s =0; s <observations.size() ; s++)
	   {
              LandmarkObs temp;
	      double x = observations[s].x*cos(p_theta)-observations[s].y*sin(p_theta)+p_x;
	      double y = observations[s].x*sin(p_theta) + observations[s].y*cos(p_theta)+p_y;

              temp.id = observations[s].id;
              temp.x = x;
              temp.y = y;
	      temp_trans.push_back(temp); //real sensor map datas      
	   }
         //sensor_range -----candidate map with particles and map datas
           std::vector<LandmarkObs> predicted;
           for(int j =0; j < map_landmarks.landmark_list.size(); j++){
                  
              int landmark_id = map_landmarks.landmark_list[j].id_i;
	      double landmark_x = map_landmarks.landmark_list[j].x_f;
	      double landmark_y = map_landmarks.landmark_list[j].y_f;

	      double distance_x = landmark_x - p_x;
	      double distance_y = landmark_y - p_y;
	      double distance = sqrt(distance_x * distance_x + distance_y * distance_y);
              
              LandmarkObs temp;
		  if(distance < sensor_range )
		  {
                     temp.id = landmark_id;
                     temp.x = landmark_x;
                     temp.y = landmark_y;
		     predicted.push_back(temp);//reduced standard position
                  // cout<<"id"<<landmark_id<<endl;
                   // std::cout<<predicted[j].id<<endl;
		  }
         }
           

        
                
                //find closed with map_observation and predicted(candidate map)
               dataAssociation( predicted, temp_trans); 
         
                 
               //calcuate weight with predicted and temp_trans
               particles[i].weight =1; 
               for(int  k =0; k < temp_trans.size(); k++)
               {

                  double p_x =0;
                  double p_y =0;
                  double m_x = temp_trans[k].x;
                  double m_y = temp_trans[k].y;
                  
                  int associated_prediction = temp_trans[k].id;
                  cout<<"id"<<temp_trans[k].id<<endl;
		      // get the x,y coordinates of the prediction associated with the current observation
	        
                     
		     p_x = predicted[temp_trans[k].id].x;
		     p_y = predicted[temp_trans[k].id].y;
		 
                  cout<<"p_x"<<p_x<<endl;
                  cout<<"p_y"<<p_y<<endl;
		  double dx = p_x-m_x;
                  double dy = p_y-m_y;
                  double dx2= pow(dx,2)/(2*pow(std_landmark[0], 2));
                  double dy2= pow(dy,2)/(2*pow(std_landmark[1], 2));
                 
		  double obs_w = sqrt( 1/(2*M_PI*std_landmark[0]*std_landmark[1])) * exp(-(dx2+dy2) );
		      
		  particles[i].weight *=obs_w ;
                   
               }
                if(particles[i].weight == 0)
                {
                     particles[i].weight =0.00001 ;
                     weights[i] = 0.00001;
 
                }
                else
                {
                  particles[i].weight =particles[i].weight ;
                  weights[i] = particles[i].weight;
                }
/*
		 double std_x = std_landmark[0];
	    double std_y = std_landmark[1];
	    double denominator = sqrt(2.0 * M_PI * std_x * std_y);
	    double a_denominator = 2 * std_x * std_x;
	    double b_denominator = 2 * std_y * std_y;

	    double weight = 1;
	    for (int j = 0; j < temp_trans.size(); j++) {
	      int obs_id = temp_trans[j].id;
	      double obs_x = temp_trans[j].x;
	      double obs_y = temp_trans[j].y;

	      double predicted_x = predicted[obs_id].x;
	      double predicted_y = predicted[obs_id].y;

	      double delta_x = obs_x - predicted_x;
	      double delta_y = obs_y - predicted_y;

	      double a = delta_x * delta_x / a_denominator;
	      double b = delta_y * delta_y / b_denominator;

	      weight *= exp(-(a + b)) / denominator;
	    }
	    if (weight == 0) {
	      particles[i].weight = 0.00001;
	      weights[i] = 0.00001;
	    } else {
	      particles[i].weight = weight;
	      weights[i] = weight;
	    }
        */
     }

       
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
       
      

        vector<Particle> new_particles;
      
         
        std::default_random_engine gen;  
         discrete_distribution<int> index(weights.begin(), weights.end());
        
        
        for(int i =0; i < num_particles; i++)
        {
            
           const int  idx = index(gen);
           Particle p;

           p.id = p.x = particles[idx].id;
           p.x = particles[idx].x;
           p.y = particles[idx].y;
          
           
           p.theta = particles[idx].theta;
           p.weight = 1;
              
           new_particles.push_back(p);
        }
         particles = new_particles;
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
