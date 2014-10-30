#include <vision/CMT.h>
#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <vision/center.h>//Custom Msg type1 with only center
#include <vision/vision.h>//Custom Msg type1 with all values of image processing



using namespace cv;
using namespace std;
using namespace ros;

//Topics published by vision node
ros::Publisher center_pub;
ros::Publisher scale_pub; 
ros::Publisher orientation_pub;

//Topic for publishing all values of image processing
ros::Publisher value_pub;

//Msg types for publishing 
vision::center Center;
std_msgs::Float32 Scale;
std_msgs::Float32 Orientation;

//Msg type for publishing all values of image processing
vision::vision value;

cv::Mat img ;
cv::Mat im_gray ;

cv::Point2f initTopLeft;//bounding box positions
cv::Point2f initBottomDown;

bool rect_status=false;
int drag=0;

void getRect(int event, int x, int y, int flags, void* userdata){

	//cout<<"Inside the Mouse callback"<<endl;
	CvPoint point;
	if(event == CV_EVENT_LBUTTONDOWN && !drag)
	    {
	        point = cvPoint(x, y);
	        drag = 1;
	        cout<<"Inside LBDOWN"<<endl;
	        cout<<"x :"<<x<<" y :"<<y<<endl;
	        initTopLeft.x=point.x,
	        initTopLeft.y=point.y;

	    }

	 /* user drag the mouse */
	 if(event == CV_EVENT_MOUSEMOVE && drag)
	    {
			Mat copy_img;//Temp MAT to display the rectanglular selection
			copy_img=img.clone();
		 	cout<<"Inside Mouse Drag"<<endl;
	    	rectangle( copy_img, initTopLeft, Point(x,y), Scalar( 0, 55, 255 ), +1, 8,0 );
		 	imshow("Initial frame",copy_img);
		 	


	    }

	 /* user release left button */
	 if(event == CV_EVENT_LBUTTONUP && drag)
	    {
		    cout<<"Inside LBUP "<<endl;
		    cout<<"x :"<<x<<" y :"<<y<<endl;
	        initBottomDown.x=x;
	        initBottomDown.y=y;
	        rect_status=true;
	        cout<<"Top Left"<<initTopLeft<<endl;
	        cout<<"Bottom Right"<<initBottomDown<<endl;
	        drag = 0;
	    }
}


int main(int argc, char *argv[])
{
   ros::init(argc,argv,"Vision");
   ROS_INFO("node creation");
   ros::NodeHandle nh_v;
   ros::Rate loop_rate(10);

   //ROS Topics Publishing
	center_pub=nh_v.advertise<vision::center>("center",1);
	scale_pub=nh_v.advertise<std_msgs::Float32>("scale",1);
	orientation_pub=nh_v.advertise<std_msgs::Float32>("orientation",1);  

	value_pub=nh_v.advertise<vision::vision>("values",1);  

    int start = 0;
    bool SelectROI=false;//For bounding box
    cout<<"Do you  want to give the ROI ?"<<endl<<"Press y for YES and n for NO"<<endl;
    char input='n';
    cin>>input;
    if(input!='n')
	SelectROI=true; 

    cout<<"SelectROI : "<<SelectROI<<endl;
	
    CMT cmt;
    
    //Code for normal camera using OpenCV
    /*VideoCapture cap(0);
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480); 
    bool open=true;
    if (!cap.isOpened())  // if not success, exit program
    {
        cout << "Cannot open the video cam" << endl;
        return -1;
        open=false;
    }
    cvNamedWindow("Initial frame",CV_WINDOW_AUTOSIZE );*/
 

    //Code for using Kinect input using OpenCV
    bool open=true;
    VideoCapture cap;
    cap.open(CV_CAP_OPENNI);
    cap.set( CV_CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE, CV_CAP_OPENNI_VGA_30HZ );
    if( !cap.isOpened() )
    {
        cout << "Can not open a capture object." << endl;
        return -1;
    }
    cout << " Device Open " << endl;
 
    //capture.set(CV_CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE, CV_CAP_OPENNI_VGA_30HZ);
    if( !cap.grab() )
    	      {
    	              cout << "Can not grab images." << endl;
    	              return -1;
    	      }
   

    cvNamedWindow("Initial frame",CV_WINDOW_AUTOSIZE ); 
    	
    	
 
    
    int Keyhit=-1;//Used to get Enter input from the keyboard for ROI selection
    
    while(Keyhit==-1){

    	//cap.read(img);//used for normal cameras
	cap.grab();
	cap.retrieve(img,CV_CAP_OPENNI_BGR_IMAGE);
	
	if(SelectROI==true)
		putText(img, "Press enter when the object is in the field of view ", cvPoint(20,30),FONT_HERSHEY_COMPLEX_SMALL, 0.6, cvScalar(255,255,255), 1, CV_AA);
	else {
		putText(img, "Using Default Bounding box at the center of frame ", cvPoint(20,30),FONT_HERSHEY_COMPLEX_SMALL, 0.6, cvScalar(255,255,255), 1, CV_AA);
		putText(img, "Press enter to start tracking ", cvPoint(20,60),FONT_HERSHEY_COMPLEX_SMALL, 0.6, cvScalar(255,255,255), 1, CV_AA);
	
	     }

	imshow("Initial frame",img);
	Keyhit=cv::waitKey(1);


    }

    while(open && ros::ok())
    {

    	//Capturing images from the  Kinect OpenCV
	//cap.read(img);//used for normal cameras
	cap.grab();
	cap.retrieve(img,CV_CAP_OPENNI_BGR_IMAGE);
	cv::cvtColor(img, im_gray, CV_RGB2GRAY);
	//imshow("Initial frame",img);
	//cv::waitKey(0);
	

	while(start==0 && SelectROI==true){

		cout<<"Inside ROI selection"<<endl;
	    //set the callback function for any mouse event
		static CvFont font;
		cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5, 0, 1, 8);
		
		//cout<<"Starting CMT initialization"<<endl;
		setMouseCallback("Initial frame", getRect, NULL); // Mouse Callback function with image as param
 			
		putText(img, "Use mouse to select the rectangle around object to track ", cvPoint(20,30),FONT_HERSHEY_COMPLEX_SMALL, 0.6, cvScalar(255,255,255), 1, CV_AA);
		putText(img, "You can reselect till you press Enter ", cvPoint(20,50),FONT_HERSHEY_COMPLEX_SMALL, 0.6, cvScalar(255,255,255), 1, CV_AA);
		putText(img, "After selecting PRESS ENTER", cvPoint(20,70),FONT_HERSHEY_COMPLEX_SMALL, 0.6, cvScalar(255,255,255), 1, CV_AA);
		
		imshow("Initial frame",img);
		cv::waitKey(0);
		start=1;
		cout<<"ROI Selected Manually"<<endl;
		cout<<"Rectangular Edge Points : "<<initBottomDown<<","<<initTopLeft<<endl;
		cout<<"CMT Initialising "<<endl;
		cmt.initialise(im_gray, initTopLeft, initBottomDown);//Using the Initialise Method with predefined bounding box
		setMouseCallback("Initial frame", NULL, NULL);//Disabling the Callback function
			}

	if(start==0 && SelectROI==false	){//If manual ROI is not selected the default case 
		
		initTopLeft.x=270;
		initTopLeft.y=190;	//Default bounding box positions - Frame Center
		initBottomDown.x=370;
		initBottomDown.y=290;

		cout<<"Rectangular Edge Points : "<<initBottomDown<<","<<initTopLeft<<endl;
		cout<<"CMT Initialising "<<endl;
		cmt.initialise(im_gray, initTopLeft, initBottomDown);//Using the Initialise Method with default bounding box
		start=1;
	}


	//cout<<endl<<"Starting CMT frame processing"<<endl;
    	cmt.processFrame(im_gray);//Using the process frame method
	
	cout<<"The Center is : "<<cmt.CENTER<<endl;
	Center.CenterX=cmt.CENTER.x;
	Center.CenterY=cmt.CENTER.y;
	center_pub.publish(Center);
	

	cout<<"The scaling is : "<<cmt.SCALE<<endl;
	Scale.data=cmt.SCALE;
	scale_pub.publish(Scale);
	
	cout<<"The orientation is :"<<cmt.ROTATION<<endl;
	Orientation.data=cmt.ROTATION;	
	orientation_pub.publish(Orientation);	

	//Publishing all values of image processing on topic /values
	value.CenterX=cmt.CENTER.x;
	value.CenterY=cmt.CENTER.y;
	value.Scale=cmt.SCALE;
	value.Orientation=cmt.ROTATION;
	
	value_pub.publish(value);

	

	

    //cout<<"CMT Frame Processing Done"<<endl;

    for(int i = 0; i<cmt.trackedKeypoints.size(); i++)
    	cv::circle(img, cmt.trackedKeypoints[i].first.pt, 3, cv::Scalar(255,255,255));//draw circles around track points
        
    cv::line(img, cmt.topLeft, cmt.topRight, cv::Scalar(255,255,255));
    cv::line(img, cmt.topRight, cmt.bottomRight, cv::Scalar(255,255,255));
    cv::line(img, cmt.bottomRight, cmt.bottomLeft, cv::Scalar(255,255,255));
    cv::line(img, cmt.bottomLeft, cmt.topLeft, cv::Scalar(255,255,255));

    putText(img, "Tracking ", cvPoint(20,30),FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(255,255,255), 1, CV_AA);
    imshow("Initial frame", img);//Show the image frame with circles of key points
    cv::waitKey(1);

    }//close for open while loop

   
    return 0;
}
