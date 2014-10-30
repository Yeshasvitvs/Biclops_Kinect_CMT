#include <ros/ros.h>
#include <cstdio>          // for FILE defn
#include <iostream>         // for cout, etc
#include <time.h>           // for clock_t and clock
#include <list>
#include <math.h>
#include <string>
#include <cstring>
#include <std_srvs/Empty.h>
#include <sensor_msgs/JointState.h>
#include "turtlesim/Velocity.h"
#include <tf/transform_broadcaster.h>


using namespace std;
using namespace ros;

#include <Biclops.h>
#include <PMDUtils.h>


char *config_path="/home/yeshi/catkin_ws/src/biclops/BiclopsDefault.cfg";

// Defines which axes we want to use.
int axisMask=Biclops::PanMask
        + Biclops::TiltMask;;

// Pointers to each axis (populated once controller is initialized).
  PMDAxisControl *panAxis = NULL;
  PMDAxisControl *tiltAxis = NULL;

  // THE interface to Biclops
        Biclops biclops_obj;




double pan_pos_value=0;
double tilt_pos_value=0;

double pan_joint_pos=0;
double tilt_joint_pos=0;

PMDAxisControl::Profile panProfile,tiltProfile;


ros::Subscriber key_sub;
ros::Publisher joint_pub;
ros::ServiceServer homing_service;
sensor_msgs::JointState joint_state;


void keyCallback(turtlesim::Velocity vel){

    cout<<"Inside The Callback"<<endl;



    if(vel.angular==-2){


		pan_pos_value=pan_pos_value+5;

		panProfile.pos = PMDUtils::DegsToRevs(pan_pos_value);
		cout<<"Pan POSITION in Degrees : "<<pan_pos_value<<endl;
		cout<<"Pan POSITION in revs : "<<panProfile.pos<<endl;
		panAxis->SetProfile(panProfile);
		panAxis->Move();

	}

	if(vel.angular==2){

			pan_pos_value=pan_pos_value-5;
			panProfile.pos = PMDUtils::DegsToRevs(pan_pos_value);
			cout<<"Pan POSITION in Degrees : "<<pan_pos_value<<endl;
			cout<<"Pan POSITION in revs : "<<panProfile.pos<<endl;
			panAxis->SetProfile(panProfile);
			panAxis->Move();

		}

	if(vel.linear==2){

			tilt_pos_value=tilt_pos_value+5;
			tiltProfile.pos = PMDUtils::DegsToRevs(tilt_pos_value);
			cout<<"Tilt POSITION in Degrees : "<<tilt_pos_value<<endl;
			cout<<"Tilt POSITION in revs : "<<tiltProfile.pos<<endl;
			tiltAxis->SetProfile(tiltProfile);
			tiltAxis->Move();

		}

	if(vel.linear==-2){

				tilt_pos_value=tilt_pos_value-5;
				tiltProfile.pos = PMDUtils::DegsToRevs(tilt_pos_value);
				cout<<"Tilt POSITION in Degrees : "<<tilt_pos_value<<endl;
				cout<<"Tilt POSITION in revs : "<<tiltProfile.pos<<endl;
				tiltAxis->SetProfile(tiltProfile);
				tiltAxis->Move();

			}


	//Assign the joint positions to the position values variables
	
	pan_joint_pos=(pan_pos_value/180)*3.14;
	tilt_joint_pos=(tilt_pos_value/180)*3.14;
	cout<<"Current Joint Values in Radians : "<<"Pan "<< pan_joint_pos<<"; Tilt "<<tilt_joint_pos<<endl;

	    joint_state.header.stamp = ros::Time::now();
	    joint_state.name.resize(2);
	    joint_state.position.resize(2);
	    joint_state.name[0] ="PAN_JOINT";
	    joint_state.position[0] = pan_joint_pos;
	    joint_state.name[1] ="TILT_JOINT";
	    joint_state.position[1] = -tilt_joint_pos;

	    //send the joint state
	    joint_pub.publish(joint_state);


}


bool Homing(std_srvs::Empty::Request &req,
		     std_srvs::Empty::Response  &res){

	ROS_INFO("Performing Homing Sequence");
	biclops_obj.HomeAxes(axisMask,true);

	pan_pos_value=0;
	tilt_pos_value=0;

	pan_joint_pos=0;
	tilt_joint_pos=0;

	return true;
}


//----------------------------------------------------------------------------
int main (int argc, char *argv[]) {

   ros::init(argc,argv,"Biclops");
   ROS_INFO("node creation");
   ros::NodeHandle nh;
   ros::Rate loop_rate(10);

  
      // ROS_INFO("ARGV : %s ",argv[1]);
    //argv[1]="/home/yeshi/catkin_ws/src/biclops/BiclopsDefault.cfg";
      argv[1]=config_path;
    ROS_INFO( "\n\nBasic Biclops Running Example\n\n" );
	//Initializing

        cout<<"Initialization Routine"<<biclops_obj.Initialize(argv[1])<<endl;

	if (!biclops_obj.Initialize(argv[1]))
	{

		ROS_INFO( "Failed to open Biclops Communication\n" );
		ROS_INFO( "Press Enter key to terminate...\n" );
		getchar();
		//return 0;
	}
	else
    		ROS_INFO( "Succeed to open Biclops Communication\n" );



    //Subscribing
    key_sub=nh.subscribe<turtlesim::Velocity>("/turtle1/command_velocity",1,keyCallback);
	
    //Publishing
	joint_pub=nh.advertise<sensor_msgs::JointState>("joint_states",1);

	//Homing sequence Service
	homing_service =nh.advertiseService("homing_sequence",Homing);

    
	//Homing sequnce at the bigining of the application
	biclops_obj.HomeAxes(axisMask,true);

	// Get shortcut references to each axis.
	panAxis = biclops_obj.GetAxis(Biclops::Pan);
	tiltAxis = biclops_obj.GetAxis(Biclops::Tilt);


   // Get the currently defined (default) motion profiles.
	panAxis->GetProfile(panProfile);
	tiltAxis->GetProfile(tiltProfile);


	while(ros::ok() ){
    cout<<"Inside ROS OK while loop"<<endl;
    spin();



    loop_rate.sleep();

	}

	panAxis->DisableAmp();
	tiltAxis->DisableAmp();

    // Something went wrong. Return failure indication.
    return 0;

}
