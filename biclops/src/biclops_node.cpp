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
#include <vision/center.h>//Custom Msg type1 with only center
#include <vision/vision.h>//Custom Msg type1 with all values of image processing

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


double pan_max_step=320;
double pan_max_angle=28.5;

double pan_pos_value=0;
double tilt_pos_value=0;

double pan_joint_pos=0;
double tilt_joint_pos=0;


PMDAxisControl::Profile panProfile,tiltProfile;

//Topics definition
ros::Subscriber center_sub;
ros::Subscriber values_sub;
ros::Publisher joint_pub;

//Service Definition
ros::ServiceServer homing_service;

//Message types
sensor_msgs::JointState joint_state;
vision::center center_point;


void centerCallback(vision::center cen){

    cout<<"Inside The Callback"<<endl;
    center_point=cen;
    pan_pos_value=int (center_point.CenterX)-320;
   
    pan_pos_value= ( pan_pos_value / pan_max_step ) * pan_max_angle;//Rough calibration for PAN joint



    tilt_pos_value=int (center_point.CenterY)-240;//Calibration has to be done for TILT joint
  
    cout<<"Pan POSITION in  : "<<pan_pos_value<<endl;
    cout<<"Tilt POSITION in  : "<<tilt_pos_value<<endl;

   
    panProfile.pos = PMDUtils::DegsToRevs(pan_pos_value);
    cout<<"Pan POSITION in Degrees : "<<pan_pos_value<<endl;
    cout<<"Pan POSITION in revs : "<<panProfile.pos<<endl;
    
   tiltProfile.pos = PMDUtils::DegsToRevs(tilt_pos_value);
   cout<<"Tilt POSITION in Degrees : "<<tilt_pos_value<<endl;
   cout<<"Tilt POSITION in revs : "<<tiltProfile.pos<<endl;

   panAxis->SetProfile(panProfile);
   panAxis->Move();

   tiltAxis->SetProfile(tiltProfile);
   tiltAxis->Move();

		
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

	    //Publish the joint state
	    joint_pub.publish(joint_state);


}

//Homing Service Callback function
bool Homing(std_srvs::Empty::Request &req,
		     std_srvs::Empty::Response  &res){

	ROS_INFO("Performing Homing Sequence");
	biclops_obj.HomeAxes(axisMask,true);

	pan_pos_value=0;
	tilt_pos_value=0;

	pan_joint_pos=0;
	tilt_joint_pos=0;

	joint_state.header.stamp = ros::Time::now();
	joint_state.name.resize(2);
        joint_state.position.resize(2);
	joint_state.name[0] ="PAN_JOINT";
	joint_state.position[0] = pan_joint_pos;
	joint_state.name[1] ="TILT_JOINT";
	joint_state.position[1] = -tilt_joint_pos;

	//Publish the joint state
	joint_pub.publish(joint_state);

	return true;
}


//----------------------------------------------------------------------------
int main (int argc, char *argv[]) {

   ros::init(argc,argv,"Biclops");
   ROS_INFO("node creation");
   ros::NodeHandle nh;
   ros::Rate loop_rate(10);

    
	
    //Publishing
    joint_pub=nh.advertise<sensor_msgs::JointState>("joint_states",1);

    //Homing sequence Service
    homing_service =nh.advertiseService("homing_sequence",Homing);

  
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
	else{
    		ROS_INFO( "Succeed to open Biclops Communication\n" );
                //Subscribing
   		 center_sub=nh.subscribe<vision::center>("/center",1,centerCallback);

	   }

    //Initial Joint States	
    joint_state.header.stamp = ros::Time::now();
    joint_state.name.resize(2);
    joint_state.position.resize(2); 
    joint_state.name[0] ="PAN_JOINT";
    joint_state.position[0] = pan_joint_pos;
    joint_state.name[1] ="TILT_JOINT";
    joint_state.position[1] = -tilt_joint_pos;

    //Publish the initial joint state
    joint_pub.publish(joint_state);

    
   //Homing sequnce at the bigining of the application
   biclops_obj.HomeAxes(axisMask,true);

   // Get shortcut references to each axis.
   panAxis = biclops_obj.GetAxis(Biclops::Pan);
   tiltAxis = biclops_obj.GetAxis(Biclops::Tilt);


   // Get the currently defined (default) motion profiles.
   panAxis->GetProfile(panProfile);
   tiltAxis->GetProfile(tiltProfile);


	while(ros::ok() ){
	
    		cout<<"Do you  want to give the ROI ? "<<endl;
		cout<<"Press y for YES and n for NO"<<endl;
    		spin();
   		loop_rate.sleep();

	}

	//Cut the power to the motors in standby
	panAxis->DisableAmp();
	tiltAxis->DisableAmp();

    // Something went wrong. Return failure indication.
    while(!ros::ok() )
    return 0;

}
