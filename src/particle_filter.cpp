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
	num_particles=40;

	// TODO: Set standard deviations for x, y, and theta.
	  double std_x=std[0];
	  double std_y=std[1];
	  double std_theta=std[2];
	
	
	// This line creates a normal (Gaussian) distribution for x.
	normal_distribution<double> dist_x(x, std_x);
	
	// TODO: Create normal distributions for y and theta.
	normal_distribution<double> dist_y(y, std_y);
	
	normal_distribution<double> dist_theta(theta, std_theta);
	
	

	for (int i = 0; i < num_particles; ++i) {
		double sample_x, sample_y, sample_theta;
		
		Particle P=Particle();
		P.x = dist_x(gen);
		P.y = dist_y(gen);
		P.theta = dist_theta(gen);
		P.weight=1;
		// std::cout<<"x:"<<P.x<<endl;
		// std::cout<<"y:"<<P.y<<endl;
		// std::cout<<"theta:"<<P.theta<<endl;
		weights.push_back(1);
		particles.push_back(P);

}
is_initialized=true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	default_random_engine gen;
	
	double pred_x,pred_y,pred_theta;
	for(int i=0;i<num_particles;i++)
	{
		
		if(yaw_rate!=0){
			pred_x=particles[i].x+(velocity/yaw_rate)*(sin(particles[i].theta+yaw_rate*delta_t)-sin(particles[i].theta));
			pred_y=particles[i].y+(velocity/yaw_rate)*(cos(particles[i].theta)-cos(particles[i].theta+yaw_rate*delta_t));
			pred_theta=particles[i].theta+yaw_rate*delta_t;
		}
		else{
			pred_x=particles[i].x+velocity*delta_t*cos(particles[i].theta);
			pred_y=particles[i].y+velocity*delta_t*sin(particles[i].theta);
			pred_theta=particles[i].theta;

		}
		
	
	// TODO: Create normal distributions for y and theta.
		normal_distribution<double> dist_x(pred_x, std_pos[0]);
		normal_distribution<double> dist_y(pred_y, std_pos[1]);
	
		normal_distribution<double> dist_theta(pred_theta, std_pos[2]);
		particles[i].x = dist_x(gen);
		particles[i].y = dist_y(gen);
		particles[i].theta = dist_theta(gen);
		particles[i].weight=1;
		weights[i]=1;
		


	}

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
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
	std::vector<LandmarkObs> Map_Observations;

	
	// std::cout<<map_landmarks.landmark_list.size()<<endl;
	for (int t=0;t<num_particles;t++)
	{
		
		double probabil=1;
		for(int k=0;k<observations.size();k++)
		{
			double x_map,y_map;
			LandmarkObs l=LandmarkObs();
			l.x=particles[t].x+cos(particles[t].theta)*observations[k].x-sin(particles[t].theta)*observations[k].y;
			l.y=particles[t].y+sin(particles[t].theta)*observations[k].x+cos(particles[t].theta)*observations[k].y;
		
			Map_Observations.push_back(l);
		}
		
		int index;
		for (int i=0;i<Map_Observations.size();i++)
		{
			double x1,y1,x2,y2;
			double dist=100;
		
		
		
			x1=Map_Observations[i].x;
			y1=Map_Observations[i].y;
			probabil=1;
			for(int j=0;j<map_landmarks.landmark_list.size();j++)
			{
			

				x2=double(map_landmarks.landmark_list[j].x_f);
				y2=double(map_landmarks.landmark_list[j].y_f);
				double dist1=sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
				if(dist1>sensor_range)
				{
					
				}
				if(dist1<dist)
				{
					dist=dist1;
					index=j;
					

				}
			}
			x2=double(map_landmarks.landmark_list[index].x_f);
			y2=double(map_landmarks.landmark_list[index].y_f);
			// std::cout<<x1<<endl;
			// std::cout<<y1<<endl;
			// std::cout<<map_landmarks.landmark_list[index].x_f<<endl;
			// std::cout<<map_landmarks.landmark_list[index].y_f<<endl;
			double gauss_norm= (1/(2 * M_PI * std_landmark[0]*std_landmark[1]));
			probabil=probabil*gauss_norm*exp(-(((x1-x2)*(x1-x2))/(2*std_landmark[0]*std_landmark[0]))-(((y1-y2)*(y1-y2))/(2*std_landmark[1]*std_landmark[1])));


		}
		

		weights[t]=probabil;
		particles[t].weight=probabil;
		

	



		}
		}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	default_random_engine gen;
	discrete_distribution<int> distribution(weights.begin(),weights.end());	

	vector<Particle> resample_particles;

	for(int i=0;i<num_particles;i++)
	{
		resample_particles.push_back(particles[distribution(gen)]);
	}
	
	particles=resample_particles;

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
