#include "ParticleFilter.hpp"
// Used for simulateRangeScan
#include "occupancy_grid_utils/ray_tracer.h"
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <cstdlib>
#include "vector"



 /**  
  * Just place all particles on a line along x=y. This should be
  * replaced with something more sensible, like drawing particles
  * from a Gaussian distribution with large variance centered on
  * the supplied initial pose, or just placing them in a regular
  * grid across the map.
  */
  void MyLocaliser::initialisePF( const geometry_msgs::PoseWithCovarianceStamped& initialpose )
  {
      ROS_INFO("A");
    for (unsigned int i = 0; i < particleCloud.poses.size(); ++i)
    {
      particleCloud.poses[i].position.x = map.info.width * map.info.resolution * ((float)rand()/(float)RAND_MAX);
      particleCloud.poses[i].position.y = map.info.height * map.info.resolution * ((float)rand()/(float)RAND_MAX);
      geometry_msgs::Quaternion odom_quat =
        tf::createQuaternionMsgFromYaw(i*0.01);
      particleCloud.poses[i].orientation = odom_quat;
    }

  }
  
  double sample(double b) {
    b = sqrt(b);
    double s = 0;
    for(int i = 0 ; i < 12 ; i++) {
        float random = ((float) rand()) / (float) RAND_MAX;
        float diff = 2 * b;
        float r = random * diff;
        s+=-b + r;
    }
    return s/2;
  }

  /**
   * Your implementation of this should sample from a random
   * distribution instead of blindly adding the odometry increment. It
   * should also update the angle of the particles.
   */
  void MyLocaliser::applyMotionModel( double deltaX, double deltaY, double deltaT )
  {
    if (deltaX > 0 or deltaY > 0)
      ROS_DEBUG( "applying odometry: %f %f %f", deltaX, deltaY, deltaT );
    for (unsigned int i = 0; i < particleCloud.poses.size(); ++i)
    {
        double a1, a2, a3, a4 = 0.1;
        double x = particleCloud.poses[i].position.x;
        double y = particleCloud.poses[i].position.y;
        double t = particleCloud.poses[i].orientation.w;
        double xPrime = x + deltaX;
        double yPrime = y + deltaY;
        double tPrime = t + deltaT;
        double deltarot1 = atan2 ((yPrime-y),(xPrime - x));
        double deltatrans = sqrt (((x - xPrime) * (x - xPrime))+((y-yPrime)*(y-yPrime)));
        double deltarot2 = tPrime - t - deltarot1;
        double deltahatrot1 = deltarot1 - sample(a1 * pow(deltarot1, 2) + a2 * pow(deltatrans, 2));
        double deltahattrans = deltatrans - sample(a3 * pow(deltarot2, 2) + a4 * pow(deltarot1, 2) + a4 * pow(deltarot2, 2));
        double deltahatrot2 = deltarot2 - sample(a1 * pow(deltarot2, 2) + a2 * pow(deltatrans, 2));
        xPrime = x + deltahattrans * cos(t + deltahatrot1);
        yPrime = y + deltahattrans * sin(t + deltahatrot1);
        tPrime = t + deltahatrot1 + deltahatrot2;

      //particleCloud.poses[i].position.x += deltaX;
      //particleCloud.poses[i].position.y += deltaY;
        particleCloud.poses[i].position.x = xPrime;
        particleCloud.poses[i].position.y = yPrime;
        particleCloud.poses[i].orientation.w = tPrime;
	}
}

  // Rik
  

  double MyLocaliser::beamRangeFinderModel(
      const sensor_msgs::LaserScan& scan, 
      const sensor_msgs::LaserScan::Ptr& simulatedScan, 
      geometry_msgs::Pose sensor_pose,
      std::vector<double> probs)
  {
    // q = 1
    double q = 1.0; // rik
    // for k = 1 to K do
    for (unsigned int k = 0; k < simulatedScan->ranges.size(); k++)
    {
      // compute zkt* for the measurement zkt using ray casting
      // actual measurement
      float zkt       = scan.ranges[k];
      // true measurement from robots pose
      float zkt_star  = simulatedScan->ranges[k];
      // p = zHit * pHit(zkt|xt,m) + zShort * pShort(zkt|xt,m) + zMax * pMax(zkt|xt,m) + zRand * pRand(zkt|xt,m)
      double zHit   = probs[0];
      double zShort = probs[1];
      double zMax   = probs[2];
      double zRand  = probs[3];
      double sHit   = probs[4];
      double lShort = probs[5];

      double p =   
            zHit    * calculate_pHit(zkt, zkt_star, sHit)
          + zShort  * calculate_pShort(zkt, zkt_star, lShort)
          + zMax    * calculate_pMax(zkt, zMax)
          + zRand   * calculate_pRand(zkt, zMax);
      // q = q * p;
      q = q * p;
    }
    // return q
    return q;
  }

  double MyLocaliser::calculate_pHit(float zkt, float zkt_star, double sHit)
  {
    return (1/(sqrt(2 * M_PI * pow(sHit,2)))) * exp(-0.5*((pow((zkt-zkt_star),2))/pow(sHit,2)));
  }

  double MyLocaliser::calculate_pShort(float zkt, float zkt_star, double lShort)
  {
    if (zkt >= 0 && zkt <= zkt_star)
    {
      return (1 / (1 - (exp(-lShort * zkt_star)))) * lShort * exp(-lShort * zkt);
    }
    return 0;
  }

  double MyLocaliser::calculate_pMax(float zkt, double zMax)
  {
    if (zkt == zMax)
    {
      return 1;
    }
    return 0;
  }

  double MyLocaliser::calculate_pRand(float zkt, double zMax)
  {
    if (zkt >= 0 && zkt < zMax)
    {
      return (1.0 / zMax);
    }
    return 0;
  }

  std::vector<double> MyLocaliser::learnIntrinsicParameters()
  {
    std::vector<double> probs;
    double zHit   = 0.1;
    double zShort = 0.1;
    double zMax   = 0.1;
    double zRand  = 0.1;
    double sHit   = 0.1;
    double lShort = 0.1;

    // TODO



    probs.push_back(zHit);
    probs.push_back(zShort);
    probs.push_back(zMax);
    probs.push_back(zRand);
    probs.push_back(sHit);
    probs.push_back(lShort);

    return probs;
  }
  // End Rik


  /**
   * After the motion model moves the particles around, approximately
   * according to the odometry readings, the sensor model is used to
   * weight each particle according to how likely it is to get the
   * sensor reading that we have from that pose.
   */
  void MyLocaliser::applySensorModel( const sensor_msgs::LaserScan& scan )
  {
    /* This method is the beginning of an implementation of a beam
     * sensor model */  
    
    std::vector<double> probs; // rik
    probs = learnIntrinsicParameters(); // rik
    for (unsigned int i = 0; i < particleCloud.poses.size(); ++i)
    {
      weights.push_back(1.0 / particleCloud.poses.size());
      geometry_msgs::Pose sensor_pose;      
      sensor_pose = particleCloud.poses[i];
      /* If the laser and centre of the robot weren't at the same
       * position, we would first apply the tf from /base_footprint
       * to /base_laser here. */

      // compute Zkt* for the measurement zkt using ray casting - rik
      sensor_msgs::LaserScan::Ptr simulatedScan;
      try{
        simulatedScan
          = occupancy_grid_utils::simulateRangeScan
          ( this->map, sensor_pose, scan, true );
      }
      catch (occupancy_grid_utils::PointOutOfBoundsException)
      {
        continue;
      }

      // Rik
      double q = beamRangeFinderModel(scan, simulatedScan, sensor_pose, probs);

      // End Rik

      /* Now we have the actual scan, and a simulated version ---
       * i.e., how a scan would look if the robot were at the pose
       * that particle i says it is in. So now we should evaluate how
       * likely this pose is; i.e., the actual sensor model. */
      
      //   if (i == 0)
      //   {
      //     for (unsigned int k = 0; k < simulatedScan->ranges.size(); ++k)
      //       ROS_INFO_STREAM(simulatedScan->ranges[k]);
      //     std::cerr << "\n\n";
      //   }
    }
  }
/*

  std::vector <double> weights;
  double normalize;
  void normalizeWeights() {
      for(unsigned int i = 0 ; i < weights.size() ; i++) {
          normalize += weights[i];
      }
      for(unsigned int i = 0 ; i < weights.size() ; i++) {
          weights[i] /= normalize;
      }
  }*/
  /**
   * This is where resampling should go, after applying the motion and
   * sensor models.
   */
  geometry_msgs::PoseArray MyLocaliser::updateParticleCloud
  ( const sensor_msgs::LaserScan& scan,
    const nav_msgs::OccupancyGrid& map,
    const geometry_msgs::PoseArray& particleCloud )
  {
      ROS_INFO("A");
      /*
      int size = particleCloud.poses.size();
        geometry_msgs::PoseArray resampled;
        normalizeWeights();
        double remain = 0;
        int index = 0;
        
        Stochastic Universal Sampling

        for(int i = 0 ; i < size ; i++) {
            double part = normalize/size;
            if(weights[i] >= part+remain) {
                geometry_msgs::Pose pose;
                pose.position = particleCloud.poses[i].position;
                pose.orientation = particleCloud.poses[i].orientation;
                resampled.poses[index] = pose;
                if(index < size) {
                    index++;
                }
                weights[i]-=part;
                remain = 0;
                i--;
            } else {
                remain = weights[i];

            }
        }
*/
    //return resampled;
    return this->particleCloud;
  }


  /**
   * Update and return the most likely pose. 
   */
  geometry_msgs::PoseWithCovariance MyLocaliser::updatePose()
  {
    this->estimatedPose.pose.pose = particleCloud.poses[1];
    //TODO: also update the covariance
    return this->estimatedPose.pose;
  }

  



