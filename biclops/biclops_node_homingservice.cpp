#include <ros/ros.h>
#include <cstdio>          // for FILE defn
#include <iostream>         // for cout, etc
#include <time.h>           // for clock_t and clock
#include <list>
#include <math.h>
#include <string>
#include <cstring>
#include <std_srvs/Empty.h>


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


PMDAxisControl::Profile panProfile,tiltProfile;


//Service Definition
ros::ServiceServer homing_service;



//Homing Service Callback function
bool Homing(std_srvs::Empty::Request &req,
		     std_srvs::Empty::Response  &res){

	ROS_INFO("Performing Homing Sequence");
	biclops_obj.HomeAxes(axisMask,true);

	return true;
}


//----------------------------------------------------------------------------
int main (int argc, char *argv[]) {

   ros::init(argc,argv,"Biclops");
   ROS_INFO("node creation");
   ros::NodeHandle nh;
   ros::Rate loop_rate(10);

    
	
    
    //Homing sequence Service
    homing_service =nh.advertiseService("homing_sequence",Homing);

  
      
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


    
   //Homing sequnce at the bigining of the application
   biclops_obj.HomeAxes(axisMask,true);

   // Get shortcut references to each axis.
   panAxis = biclops_obj.GetAxis(Biclops::Pan);
   tiltAxis = biclops_obj.GetAxis(Biclops::Tilt);


   // Get the currently defined (default) motion profiles.
   panAxis->GetProfile(panProfile);
   tiltAxis->GetProfile(tiltProfile);


	while(ros::ok() ){
	
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
