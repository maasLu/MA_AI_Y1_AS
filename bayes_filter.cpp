/* Autonomous Systems : Bayes filter
// For this assignment you are going to implement a bayes filter

- uniformly distibute the belief over all beliefstates
- implement a representation of the world for every beliefstate
- implement the measurement model
- implement the motion model (forward & turn)
*/

#include "ros/ros.h"
#include "ros/console.h"
#include "geometry_msgs/Twist.h"
#include "laser_to_wall/WallScan.h"
#include "std_msgs/Int32.h"
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"
#include <boost/lexical_cast.hpp>
#include <cstdlib> // Needed for rand()
#include <ctime> // Needed to seed random number generator with a time value
#include <sstream>
#include <vector>


class BayesFilter {


public:
  // Construst a new BayesFilter object and hook up this ROS node
  // to the simulated robot's velocity control and laser topics
  BayesFilter(ros::NodeHandle& nh) : rotateStartTime(ros::Time::now()),rotateDuration(1.8f), moveDuration(0.75f) {
    // Initialize random time generator
    srand(time(NULL));
    // Advertise a new publisher for the simulated robot's velocity command topic
    // (the second argument indicates that if multiple command messages are in
    //  the queue to be sent, only the last command will be sent)
    commandPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    // Subscribe to the simulated robot's wall scan topic and tell ROS to call
    // this->commandCallback() whenever a new message is published on that topic
    wallSub = nh.subscribe("wall_scan", 1, &BayesFilter::commandCallbackWallScan, this);
    actionSub = nh.subscribe("action", 1, &BayesFilter::commandCallbackAction, this);
    markerPub = nh.advertise<visualization_msgs::MarkerArray>("beliefs",1);
    movenoise = false;
    measnoise = false;

    /*=TODO - INSERT-CHANGE CODE HERE IF NEEDED=*/
    // Initial belief distribution
    for (int i = 0; i<NUM_STATES; i++) { 
      beliefStates.push_back(1.0/NUM_STATES); 
      predictions.push_back(1.0/NUM_STATES);
    };
    world[0][0] = 1;     world[0][1] = 0;     world[0][2] = 1;
    world[1][0] = 0;     world[1][1] = 0;     world[1][2] = 1;
    world[2][0] = 1;     world[2][1] = 0;     world[2][2] = 1;
    world[3][0] = 1;     world[3][1] = 0;     world[3][2] = 0;
    world[4][0] = 1;     world[4][1] = 0;     world[4][2] = 1;
    world[5][0] = 1;     world[5][1] = 0;     world[5][2] = 1;
    world[6][0] = 1;     world[6][1] = 0;     world[6][2] = 1;
    world[7][0] = 0;     world[7][1] = 0;     world[7][2] = 1;
    world[8][0] = 1;     world[8][1] = 0;     world[8][2] = 1;
    world[9][0] = 1;     world[9][1] = 1;     world[9][2] = 1;

    world[10][0] = 1;    world[10][1] = 0;    world[10][2] = 1;
    world[11][0] = 1;    world[11][1] = 0;    world[11][2] = 1;
    world[12][0] = 1;    world[12][1] = 0;    world[12][2] = 0;
    world[13][0] = 1;    world[13][1] = 0;    world[13][2] = 1;
    world[14][0] = 1;    world[14][1] = 0;    world[14][2] = 1;
    world[15][0] = 1;    world[15][1] = 0;    world[15][2] = 1;
    world[16][0] = 0;    world[16][1] = 0;    world[16][2] = 1;
    world[17][0] = 1;    world[17][1] = 0;    world[17][2] = 1;
    world[18][0] = 1;    world[18][1] = 0;    world[18][2] = 0;
    world[19][0] = 1;    world[19][1] = 1;    world[19][2] = 1;

    world[20][0] = 0;    world[20][1] = 1;    world[20][2] = 1;
    world[21][0] = 0;    world[21][1] = 1;    world[21][2] = 0;
    world[22][0] = 0;    world[22][1] = 1;    world[22][2] = 0;
    world[23][0] = 0;    world[23][1] = 0;    world[23][2] = 0;
    world[24][0] = 0;    world[24][1] = 1;    world[24][2] = 0;
    world[25][0] = 0;    world[25][1] = 1;    world[25][2] = 0;
    world[26][0] = 0;    world[26][1] = 1;    world[26][2] = 0;
    world[27][0] = 0;    world[27][1] = 1;    world[27][2] = 0;
    world[28][0] = 0;    world[28][1] = 1;    world[28][2] = 0;
    world[29][0] = 1;    world[29][1] = 1;    world[29][2] = 0;

    world[30][0] = 0;    world[30][1] = 1;    world[30][2] = 1;
    world[31][0] = 0;    world[31][1] = 1;    world[31][2] = 0;
    world[32][0] = 0;    world[32][1] = 0;    world[32][2] = 0;
    world[33][0] = 0;    world[33][1] = 1;    world[33][2] = 0;
    world[34][0] = 0;    world[34][1] = 1;    world[34][2] = 0;
    world[35][0] = 0;    world[35][1] = 1;    world[35][2] = 0;
    world[36][0] = 0;    world[36][1] = 1;    world[36][2] = 0;
    world[37][0] = 0;    world[37][1] = 1;    world[37][2] = 0;
    world[38][0] = 0;    world[38][1] = 0;    world[38][2] = 0;
    world[39][0] = 1;    world[39][1] = 1;    world[39][2] = 0;

    ROS_INFO("Bayes Filter running");
   /*============================================*/
   }; 

 
 
  // publish visual information to RVIZ of the beliefstates 
  void publishBeliefMarkers() {
     visualization_msgs::MarkerArray beliefs;
     for (int i = 0; i < NUM_STATES; i++) {
	visualization_msgs::Marker marker;	
	marker.header.frame_id = "/map";
	marker.header.stamp = ros::Time();
	marker.ns = "beliefs";
	marker.type = visualization_msgs::Marker::CUBE;
	marker.action = visualization_msgs::Marker::ADD;
	if (i >= 10) {	
		marker.pose.position.x = -0.8;
    		marker.pose.position.y =  4.5 -i%10;
	}	
	else{
		marker.pose.position.x = 0.8;
    		marker.pose.position.y = -4.5 +i;
	}
    	marker.pose.position.z = 0.2;
    	marker.pose.orientation.x = 0.0;
    	marker.pose.orientation.y = 0.0;
    	marker.pose.orientation.z = 0.0;
   	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.5;
  	marker.scale.y = 1.0;
  	marker.scale.z = 1.0;
    	// Set the color -- be sure to set alpha to something non-zero!
    	marker.color.r = 1.0f;
    	marker.color.g = 0.0f;
    	marker.color.b = 0.0f;
    	marker.color.a = 1.0  * beliefStates[i]		;
	marker.id = i;
	beliefs.markers.push_back(marker); 

	//Text
	visualization_msgs::Marker marker2;
	marker2.header.frame_id = "/map";
	marker2.header.stamp = ros::Time();
	marker2.ns = "beliefs";
	marker2.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	marker2.action = visualization_msgs::Marker::ADD;
	if (i >= 10) {	
		marker2.pose.position.x = -0.8;
    		marker2.pose.position.y =  4.5 -i%10;
	}	
	else{
		marker2.pose.position.x = 0.8;
    		marker2.pose.position.y = -4.5 +i;
	}
    	marker2.pose.position.z = 0.2;
    	marker2.pose.orientation.x = 0.0;
    	marker2.pose.orientation.y = 0.0;
    	marker2.pose.orientation.z = 0.0;
   	marker2.pose.orientation.w = 1.0;
	marker2.scale.x = 0.5;
  	marker2.scale.y = 1.0;
  	marker2.scale.z = 0.15;
    	// Set the color -- be sure to set alpha to something non-zero!
    	marker2.color.r = 1.0f;
    	marker2.color.g = 1.0f;
    	marker2.color.b = 1.0f;
    	marker2.color.a = 1.0;
	//std::string text = boost::lexical_cast<string>(i);
	std::ostringstream oss;
	oss << i;	
	std::ostringstream oss2;
	oss2 << beliefStates[i];	
	marker2.text = "State: " + oss.str() + "\nBelief:\n" + oss2.str();
	marker2.id = NUM_STATES + i;
	beliefs.markers.push_back(marker2);    
     }
     markerPub.publish(beliefs);
  };



  /*=TODO - INSERT-CHANGE CODE HERE IF NEEDED=*/
  void updateMove() {
    // Movement Model
    // p(Xi | Xi) = 0.1
    // p(Xi+1 | Xi) = 0.8
    // p(Xi+2 | Xi) = 0.1

    // ___
    // bel(Xt) = S p(Xt | Ut, Xt-1)bel(Xt-1)dXt-1
    // Ut from movement model
    // var [movenoise] bool for movement noise

    double newPredictions[NUM_STATES];
    double transProbs[NUM_STATES][NUM_STATES];
    for (int i = 0; i < NUM_STATES; i++) 
    {
      // determine state transition probabilities 
      // p(Xt | Ut, Xt-1)
      for (int j = 0; j < NUM_STATES; j++)
      {
        transProbs[i][j] = 0;    
        if(i < NUM_STATES / 2 && j < NUM_STATES / 2) // transition prob = 0 if states > 19 are involved (can only be reached by turning 90 degrees)
        {
          // P(Xi | Xi) = 0.1
          if (j == i)
              transProbs[i][j] = 0.1;
          // P(Xi+1 | Xi) = 0.8
          else if (j == (i+1))
              transProbs[i][j] = 0.8;
          // P(Xi+2 | Xi) = 0.1
          else if (j == (i+2))
              transProbs[i][j] = 0.1;
        }
        else if (i == j) // the model states that when the robot is in a 90* turned position it is unable to move, only turn
        {
          transProbs[i][j] = 1;
        }
        // can't face other direction after a move
        if (
              (i == 9   && j == 10)
          ||  (i == 9   && j == 11)
          ||  (i == 19  && j == 0 )
          ||  (i == 19  && j == 1 )
          ||  (i == 8   && j == 10)
          ||  (i == 18  && j == 0 ) 
          )
        {
          transProbs[i][j] = 0;
        }
      }
      // normalize movement probabilities
      // in state 9 for example, you can never reach another state according to the model
      // so transprobs[9][9] = 1
      double normalize = 0;
      for (int j = 0; j < NUM_STATES; j++)
      {
        normalize += transProbs[i][j];
      }
      // ROS_INFO("For state %d, normalize = %f",i,normalize);
      for (int j = 0; j < NUM_STATES; j++)
      {
        if (normalize > 0)
          transProbs[i][j] = transProbs[i][j] / normalize;
      }
      // end transition probabilities
    }

    // calculate prediction
    // ___
    // bel(Xt) = S p(Xt | Ut, Xt-1)bel(Xt-1)dXt-1
    // with p(Xt | Ut, Xt-1) = transProbs[j]
    for (int i = 0; i < NUM_STATES; i++)
    {
      double sum = 0;
      for (int j = 0; j < NUM_STATES; j++)
      {
        sum += transProbs[j][i] * beliefStates[j];
      }
      newPredictions[i] = sum;
      // ROS_INFO("Prediction for state [%d] = [%f], current belief=[%f]",i,sum,beliefStates[i]);
    }
    // update the predictions with the new calculated values
    for (int i = 0; i < NUM_STATES; i++)
    {
      predictions[i] = newPredictions[i];
    }     
  }
  /*==========================================*/




  /*=TODO - INSERT-CHANGE CODE HERE IF NEEDED=*/
  void updateTurn() {
    // Turn Model
    // p(N-Xi-1 | Xi) = 0.9 Added the minus 1, I think that's an error in the assignment, Rik
    // p(Xi | Xi) = 0.1

    // ___
    // bel(Xt) = S p(Xt | Ut, Xt-1)bel(Xt-1)dXt-1
    // Ut from turn model
    // var [movenoise] bool for movement noise

    double newPredictions[NUM_STATES];
    double transProbs[NUM_STATES][NUM_STATES];
    for (int i = 0; i < NUM_STATES; i++) 
    {
      // determine state transition probabilities 
      // p(Xt | Ut, Xt-1)
      for (int j = 0; j < NUM_STATES; j++)
      {
        transProbs[i][j] = 0;
        if(j < NUM_STATES / 2 && i < NUM_STATES / 2) {  // for states 0 - 19
           // P(Xi | Xi) = 0.1
           if (j == i)
             transProbs[i][j] = 0.1;
           // P((N/2)-Xi-1 | Xi) = 0.9
           else if (j == (NUM_STATES / 2) - i - 1)
             transProbs[i][j] = 0.9;
        } else {                                        // for states 20 - 39
            // P(Xi | Xi) = 0.1
            if (j == i)
              transProbs[i][j] = 0.1;
            // P(N-Xi-1 | Xi) = 0.9
            else if ((j - NUM_STATES / 2) == (NUM_STATES / 2) - (i - NUM_STATES / 2) - 1)
              transProbs[i][j] = 0.9;
        }
      }
      // end transition probabilities
    }

    // calculate prediction
    // ___
    // bel(Xt) = S p(Xt | Ut, Xt-1)bel(Xt-1)dXt-1
    // with p(Xt | Ut, Xt-1) = transProbs[j]
    for (int i = 0; i < NUM_STATES; i++)
    {
      double sum = 0;
      for (int j = 0; j < NUM_STATES; j++)
      {
        sum += transProbs[j][i] * beliefStates[j];
      }
      newPredictions[i] = sum;
      // ROS_INFO("Prediction for state [%d] = [%f], current belief=[%f]",i,sum,beliefStates[i]);
    }
    // update the predictions with the new calculated values
    for (int i = 0; i < NUM_STATES; i++)
    {
      predictions[i] = newPredictions[i];
    }     
  }

  void updateSmallTurn() {
      // 90 degree turn model

      double newPredictions[NUM_STATES];
      double transProbs[NUM_STATES][NUM_STATES];
      for (int i = 0; i < NUM_STATES; i++)
      {
        // determine state transition probabilities
        // p(Xt | Ut, Xt-1)
        for (int j = 0; j < NUM_STATES; j++)
        {
          transProbs[i][j] = 0;
          if(i < NUM_STATES / 2) {       // transition from states 0 - 19
             // P(Xi | Xi) = 0.1
             if (j == i)
               transProbs[i][j] = 0.1;
             // P((N/2)-Xi-1 | Xi) = 0.9
             else if (j == i + (NUM_STATES / 2))
               transProbs[i][j] = 0.9;
          } else {                         // transition from states 20 - 39
              // P(Xi | Xi) = 0.1
              if (j == i)
                transProbs[i][j] = 0.1;
              // P(N-Xi-1 | Xi) = 0.9
              else if (i + j == NUM_STATES)
                transProbs[i][j] = 0.9;
          }
        }
        // end transition probabilities
      }

      // calculate prediction
      // ___
      // bel(Xt) = S p(Xt | Ut, Xt-1)bel(Xt-1)dXt-1
      // with p(Xt | Ut, Xt-1) = transProbs[j]
      for (int i = 0; i < NUM_STATES; i++)
      {
        double sum = 0;
        for (int j = 0; j < NUM_STATES; j++)
        {
          sum += transProbs[j][i] * beliefStates[j];
        }
        newPredictions[i] = sum;
        // ROS_INFO("Prediction for state [%d] = [%f], current belief=[%f]",i,sum,beliefStates[i]);
      }
      // update the predictions with the new calculated values
      for (int i = 0; i < NUM_STATES; i++)
      {
        predictions[i] = newPredictions[i];
      }
  }
  /*==========================================*/



  /*=TODO - INSERT-CHANGE CODE HERE IF NEEDED=*/

  /*
      ROS_INFO("=========================");
      ROS_INFO("Wall left: [%d]", wall_left);
      ROS_INFO("Wall front: [%d]", wall_front);
      ROS_INFO("Wall right: [%d]", wall_right);
      int sensor_left  = (wall_left)?1:0;
      int sensor_front = (wall_front)?1:0;
      int sensor_right = (wall_right)?1:0;
  */

  
  void updateSensing() {
    // Measurement model
    // p(Zt = sense_wall | Xt = is_wall ) = 0.8
    // p(Zt = sense_door | Xt = is_wall ) = 0.2
    // p(Zt = sense_wall | Xt = is_door ) = 0.3
    // p(Zt = sense_door | Xt = is_door ) = 0.7

    //                        ___
    // bel(Xt) = n p(Zt | Xt) bel(Xt)
    // var [measnoise] bool for measurement noise

    // the sensor observations
    int sensorObs[3] = {(wall_left) ? 1 : 0, (wall_front) ? 1 : 0, (wall_right) ? 1 : 0};

    // ROS_INFO("Current observation:[L,F,R]=[%d,%d,%d]",sensorObs[0],sensorObs[1],sensorObs[2]);

    // determine observation probabilities 
    // p(Zt | Xt)
    double obsProbs[NUM_STATES];
    double newBeliefStates[NUM_STATES];
    for (int i = 0; i < NUM_STATES; i++) 
    { 
      // the expected observation in state i
      int worldObs[3] = {world[i][0], world[i][1], world[i][2]};

      // Calculate probabilities separately for left, front right
      double obsProb[3];
      for (int j = 0; j < 3; j++) // left, front, right
      {
        if (worldObs[j] == 1)     // state has a wall
        {
          if (sensorObs[j] == 1)  // sensor senses wall
            obsProb[j] = 0.8;     // p(Zt = sense_wall | Xt = is_wall ) = 0.8
          else                    // sensor senses door
            obsProb[j] = 0.2;     // p(Zt = sense_door | Xt = is_wall ) = 0.2
        } 
        else 
        {
          if (sensorObs[j] == 1)  // sensor senses wall
            obsProb[j] = 0.3;     // p(Zt = sense_wall | Xt = is_door ) = 0.3
          else                    // sensor senses door                    
            obsProb[j] = 0.7;     // p(Zt = sense_door | Xt = is_door ) = 0.7
        }
      }
      obsProbs[i] = obsProb[0] * obsProb[1] * obsProb[2];
        
      // ROS_INFO("obsProbs[i]=[%f]=[%f]*[%f]*[%f] for state [%d]=[%d,%d,%d]",
      //   obsProbs[i],obsProb[0],obsProb[1],obsProb[2],i,world[i][0],world[i][1],world[i][2]);
      // end observation probabilities
    } 
    // calculate beliefstate
    //                        ___
    // bel(Xt) = n p(Zt | Xt) bel(Xt)
    //                                   ___
    // with p(Zt | Xt) = obsProbs[i] and bel(Xt) = predictions[i]
    double n = 0;
    for (int i = 0; i < NUM_STATES; i++)
    {
      n += obsProbs[i] * predictions[i];
      // ROS_INFO("Calculating n = obsProbs[i] * predictions[i]: [%f]=[%f]*[%f]",n,obsProbs[i],predictions[i]);
    }
    // normalizing factor
    // ROS_INFO("Calculated normalizing factor of: [%f]",n);
    n = 1/n;
    for (int i = 0; i < NUM_STATES; i++)
    {
      beliefStates[i] = n * obsProbs[i] * predictions[i];
      ROS_INFO("Updated belief for state [%d] = [%f]",i,beliefStates[i]);
    }
  }
  /*==========================================*/


  // Send a velocity command 
  void move(double linearVelMPS, double angularVelRadPS) {
    geometry_msgs::Twist msg; // The default constructor will set all commands to 0
    msg.linear.x = linearVelMPS;
    msg.angular.z = angularVelRadPS;
    commandPub.publish(msg);
  }

  // introduce discrete movement noise
  int movementNoise()
  {
	if (movenoise)
	{ 
	 int val = rand()%100;
	 if (val<LOWER_NOISE_THRESHOLD) return 0;
	 if (val>=UPPER_NOISE_THRESHOLD) return 2;
	}
	return 1;
  }

  // Introduce measurement noise 
  bool measurementNoise(bool measurement)
  {
	if (measnoise)
	{    
	  int val = rand()%100;
	  if (measurement) {
            if (val>=80)
		return !measurement; 
	  }
          else 
	    if (val>=70) 
	      return !measurement;
    	}
	return measurement;
  }

  // Process the incoming action message
  void commandCallbackAction(const std_msgs::Int32::ConstPtr& msg) {
	int steps = movementNoise();
	if (msg->data == 0) {
    	  for (int i = 0; i<steps; i++) {
	    if (!obstacle) {
	      moveStartTime = ros::Time::now();
	      while (ros::Time::now() - moveStartTime <= moveDuration) 
		move(FORWARD_SPEED_MPS,0);
	    }
	    ros::Duration(0.2).sleep();
	  }
	  updateMove();
	}
	if (msg->data == 1) {
    	  for (int i = 0; i<std::min(steps,1); i++) {
	    rotateStartTime = ros::Time::now();
 	    while (ros::Time::now() - rotateStartTime <= rotateDuration) 
  		move(0,ROTATE_SPEED_RADPS);
	  }
	  updateTurn();
	}
	if (msg->data == 2) {	
	  updateSensing();
	}
	if (msg->data == 3) {
	  if (movenoise==false) movenoise = true;
	  else movenoise = false;
	  ROS_INFO_STREAM("movementnoise: " << movenoise);
	}
	if (msg->data == 4) {
	  if (measnoise==false) measnoise = true;
	  else measnoise = false;
	  ROS_INFO_STREAM("measurementnoise: " << measnoise);
    }
// Controller needs to send message '5' for 90 degree turns
    if (msg->data == 5) {
          for (int i = 0; i<std::min(steps,1); i++) {
        rotateStartTime = ros::Time::now();
        while (ros::Time::now() - rotateStartTime <= rotateDuration)
        move(0,ROTATE_SPEED_RADPS/2);
      }
      updateSmallTurn();
    }
	publishBeliefMarkers();
  }

  // Process the incoming wall scan message
  void commandCallbackWallScan(const laser_to_wall::WallScan::ConstPtr& msg) {
	wall_left  =  measurementNoise((bool)msg->wall_left);
	wall_right =  measurementNoise((bool)msg->wall_right);
	wall_front =  measurementNoise((bool)msg->wall_front);
	obstacle =  (bool)msg->wall_front;
  }


protected:
  ros::Publisher commandPub; // Publisher to the simulated robot's velocity command topic
  ros::Publisher markerPub;
  ros::Subscriber wallSub; // Subscriber to the simulated robot's wall scan topic
  ros::Subscriber actionSub; // Subscriber to the action topic
  ros::Time rotateStartTime; // Start time of the rotation
  ros::Duration rotateDuration; // Duration of the rotation
  ros::Time moveStartTime; // Start time of the rotation
  ros::Duration moveDuration; // Duration of the rotation
  std::vector<double> beliefStates;
  bool wall_front, wall_left, wall_right, movenoise, measnoise, obstacle;
  const static int NUM_STATES = 40;
  const static double FORWARD_SPEED_MPS = 1.0;
  const static double ROTATE_SPEED_RADPS = M_PI/2;
  const static int UPPER_NOISE_THRESHOLD = 90;
  const static int LOWER_NOISE_THRESHOLD = 10;
  /*=TODO - INSERT-CHANGE CODE HERE IF NEEDED=*/
  int world[NUM_STATES][3];
  std::vector<double> predictions;
  /*==========================================*/
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "bayes_filter"); // Initiate new ROS node named "bayes_filter"
  ros::NodeHandle n;
  BayesFilter filter(n); // Create new filter object
  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();
//  ros::spin();
  return 0;
};
