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
      ROS_INFO("ParticleCloud has size [%d]",particleCloud.poses.size());
    for (unsigned int i = 0; i < particleCloud.poses.size(); ++i)
    {
      particleCloud.poses[i].position.x = map.info.width * map.info.resolution * ((float)rand()/(float)RAND_MAX);
      particleCloud.poses[i].position.y = map.info.height * map.info.resolution * ((float)rand()/(float)RAND_MAX);
      geometry_msgs::Quaternion odom_quat =
        tf::createQuaternionMsgFromYaw(i*0.01);
      particleCloud.poses[i].orientation = odom_quat;
      weights.push_back(1.0 / particleCloud.poses.size());
      ROS_INFO("Init weight [%d] to [%f]",i,weights[i]);
    }
    ROS_INFO("Weights got init to size [%d]",weights.size());

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
    ROS_INFO("Start motion model");
    if (deltaX > 0 or deltaY > 0)
      ROS_DEBUG( "applying odometry: %f %f %f", deltaX, deltaY, deltaT );
    for (unsigned int i = 0; i < particleCloud.poses.size(); ++i)
    {
        double a1 = 0.1, a2 = 0.1, a3 = 0.1, a4 = 0.1;


        double x = particleCloud.poses[i].position.x;
        double y = particleCloud.poses[i].position.y;
        double t = particleCloud.poses[i].orientation.w;
         ROS_INFO("Old orientation = %d", t);
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
        particleCloud.poses[i].orientation = tf::createQuaternionMsgFromYaw(tPrime);
        ROS_INFO("New orientation = %d", particleCloud.poses[i].orientation.w);
	}
  ROS_INFO("End motion model");
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
    // number of laserscans to consider:
    int numberOfScans = 20;
    for (unsigned int k = 0; k < simulatedScan->ranges.size(); k+=(int) (scan.ranges.size() / numberOfScans))
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
      // sROS_INFO("probs[%f,%f,%f,%f,%f,%f], zkt = [%f], zkt_star = [%f]",zHit,zShort,zMax,zRand,sHit,lShort,zkt,zkt_star);
      double p =   
            (zHit    * calculate_pHit(zkt, zkt_star, sHit, zMax))
          + (zShort  * calculate_pShort(zkt, zkt_star, lShort))
          + (zMax    * calculate_pMax(zkt, zMax))
          + (zRand   * calculate_pRand(zkt, zMax));
          
      // q = q * p;
      if (p > 0)
      {
        q = q * p;
      }
     // ROS_INFO("k: [%d], \tp=[%f], \tq=[%f]",k, p, q);
    }
    // return q
   // ROS_INFO("Return q = [%f]",q);
    return q;
  }

  double MyLocaliser::calculate_pHit(float zkt, float zkt_star, double sHit, double zMax)
  {
    if (zkt >= 0 && zkt <= zMax)
    {
      double squareRoot = (1/(sqrt(2 * M_PI * pow(sHit,2))));
      double breaker = -0.5 * (pow((zkt - zkt_star),2)/pow(sHit,2));
      double exponent = exp(breaker);
      double pHit = (1/(sqrt(2 * M_PI * pow(sHit,2)))) * exp(-0.5*((pow((zkt-zkt_star),2))/pow(sHit,2)));
       ROS_INFO("[%f], [%f], [%f], pHit = [%f]",squareRoot, breaker, exponent, pHit);
      return pHit;
    }
    // ROS_INFO("pHit = 0");
    return 0;
  }

  double MyLocaliser::calculate_pShort(float zkt, float zkt_star, double lShort)
  {
    if (zkt >= 0 && zkt <= zkt_star)
    {
      double pShort = (1 / (1 - (exp(-lShort * zkt_star)))) * lShort * exp(-lShort * zkt);
      // ROS_INFO("pShort = [%f]",pShort);
      return pShort;
    }
    // ROS_INFO("pShort = 0");
    return 0;
  }

  double MyLocaliser::calculate_pMax(float zkt, double zMax)
  {
    if (zkt == zMax)
    {
      //ROS_INFO("pMax = 1");
      return 1;
    }
    //ROS_INFO("pMax = 0");
    return 0;
  }

  double MyLocaliser::calculate_pRand(float zkt, double zMax)
  {
    if (zkt >= 0 && zkt < zMax)
    {
      //ROS_INFO("pRand = 1");
      return (1.0 / zMax);
    }
    //ROS_INFO("pRand = 0");
    return 0;
  }

  std::vector<double> MyLocaliser::learnIntrinsicParameters()
  {
    std::vector<double> probs;
    double zHit   = 5.0;
    double zShort = 1.0;
    double zMax   = 4.0;
    double zRand  = 1.0;
    double sHit   = 3.0;
    double lShort = 1.0;

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
    ROS_INFO("Start sensor model");
    /* This method is the beginning of an implementation of a beam
     * sensor model */  
    
    std::vector<double> probs; // rik
    probs = learnIntrinsicParameters(); // rik
    for (unsigned int i = 0; i < particleCloud.poses.size(); ++i)
    {
      geometry_msgs::Pose sensor_pose;  
      //ROS_INFO("Get pose");    
      sensor_pose = particleCloud.poses[i];

      //ROS_INFO("Pose [x,y,z,w]: [%f,%f,%f,%f]",sensor_pose.orientation.x,sensor_pose.orientation.y,sensor_pose.orientation.z,sensor_pose.orientation.w);
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
      weights[i] = beamRangeFinderModel(scan, simulatedScan, sensor_pose, probs);
      ROS_INFO("Sensor model updated weight [%d] to [%f]",i,weights[i]);
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
    normalizeWeights();
    ROS_INFO("End sensor model");
  }


  
  void MyLocaliser::normalizeWeights() {
    ROS_INFO("B");
    // ROS_INFO("Weights has size [%d]",weights.size());
    normalize = 0;
      for(unsigned int i = 0 ; i < weights.size() ; i++) {
          normalize += weights[i];
      }
      for(unsigned int i = 0 ; i < weights.size() ; i++) {
        // ROS_INFO("Weight %d = %f",i,weights[i]);
          weights[i] /= normalize;
          // ROS_INFO("Weight %d = %f",i,weights[i]);
      }
      ROS_INFO("C");
  }


//Source : http://www.dreamincode.net/code/snippet1446.htm

  double normal(double mu, double sigma) {
                 //        deviate from previous calculation
        double polar, rsquared, var1, var2;
       

               
                //        choose pairs of uniformly distributed deviates, discarding those
                //        that don't fall within the unit circle
                do {
                        var1=2.0*( double(rand())/double(RAND_MAX) ) - 1.0;
                        var2=2.0*( double(rand())/double(RAND_MAX) ) - 1.0;
                        rsquared=var1*var1+var2*var2;
                } while ( rsquared>=1.0 || rsquared == 0.0);
               
                //        calculate polar tranformation for each deviate
                polar=sqrt(-2.0*log(rsquared)/rsquared);
               
                return var2*polar*sigma + mu;
  }


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
      
      int size = particleCloud.poses.size();
        geometry_msgs::PoseArray resampled;
        normalizeWeights();
        double remain = 0;
        int index = 0;
        
        // Stochastic Universal Sampling

        for(int i = 0 ; i < size ; i++) {
          
            double part = normalize/size;
            //ROS_INFO("weight [%d]: [%f], part: [%f], n: [%f]",i,weights[i],part,normalize);
            if(weights[i] >= part+remain) {
                geometry_msgs::Pose pose;
                pose.position = particleCloud.poses[i].position;
                pose.orientation = particleCloud.poses[i].orientation;
                resampled.poses.push_back(pose);
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
        for (int i = 0; i < size; i++)
        {

      //ROS_INFO("Updated weight [%d] to [%f]",i,weights[i]);
        }
    return resampled;
    //return this->particleCloud;
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

  



