//testing
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <ros/console.h>
#include <iostream>
#include <fstream>
#include <ctime>

//callback functions
void myCallback(const ros::TimerEvent& e);  // for publisher
void mySubCallback(const std_msgs::Float64::ConstPtr &msg); // for subscriber

ros::Subscriber sub = mNH.subscribe(publishedName + "/myTimerTick", 10, mySubCallback);  // for subscriber

// create timer
ros::Timer myTimer= mNH.createTimer(ros::Duration(0.1), myCallback);

void myCallback(const ros::TimerEvent& e) {
	ros::NodeHandle mNH;
	ros::Publisher pub = mNH.advertise<std_msgs::Float64>(publishedName + "/myTimerTick", 10);
	
	ros::Rate rate(1);
	float timeTick = 0.1;
  while (ros::ok()) {  
	std_msgs::Float64 msg;
    	msg.data = timeTick;
    	pub.publish(msg);  //publish messages

	// writing to a file for testing
   	time_t now = time(0); // current date/time based on current system
   	char* dt = ctime(&now); // convert now to string form

	ofstream myFilePub;
	myFilePub.open ("myTimerPubTesing.txt",ios::out | ios::app );	
	myFilePub << dt <<endl;
	myFilePub << " Publisher: myTimerTesing " << timeTick <<endl;

	ros::spinOnce();
    	rate.sleep();  
  }
 
}

void mySubCallback(const std_msgs::Float64::ConstPtr &msg) {
  	
	// writing to a file for testing
	time_t now = time(0); // current date/time based on current system
   	char* dt = ctime(&now); // convert now to string form
	ofstream myFileSub;
	myFileSub.open ("myTimerSubTesing.txt",ios::out | ios::app );	
	myFileSub << dt <<endl;
	myFileSub << " Subscriber: myTimerTesing " << msg->data <<endl;
 	
}


/*
Notes:
To check for publisher and subscriber, use the rostopic command or use rqt, see below.

rostopic info topic_name
For Example: rostopic info /ajax/myTimerTick

The output files will be generated under ~/.ros directory.
*/
