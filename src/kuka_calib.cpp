/*
 * kuka_calib.cpp
 *
 *  Created on: Apr 23, 2016
 *      Author: haitham
 */
#include <ros/ros.h>
#include <ros/package.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
// Image
#include <opencv/cv.h>
#include <image_geometry/pinhole_camera_model.h>
#include <boost/foreach.hpp>
//Use image_transport for publishing and subscribing to images in ROS
#include <image_transport/image_transport.h>
//Use cv_bridge to convert between ROS and OpenCV Image formats
#include <cv_bridge/cv_bridge.h>
//Include some useful constants for image encoding. Refer to: http://www.ros.org/doc/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html for more info.
#include <sensor_msgs/image_encodings.h>
//Include headers for OpenCV Image processing
#include <opencv2/imgproc/imgproc.hpp>
//Include headers for OpenCV GUI handling
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/video/video.hpp>
// basic file operations
#include <iostream>
#include <fstream>
// Socket communication

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <fcntl.h> /* Added for the nonblocking socket */
 #include <errno.h>

using namespace std;
using namespace cv;

cv::Mat chess_image;
bool savePose=false;
int poseNumber=0;
char subbuff[3];
std::string calib_data_file;
double fileID=0;
ofstream myfile;
char buffer[4095];
void imageCallback(const sensor_msgs::ImageConstPtr& original_image);


int main(int argc, char** argv)
{
ros::init(argc, argv, "kuka_calib_node");
    ros::NodeHandle nh;
    std::string chess_image_topic;

    nh.param<std::string>("chess_image_topic", chess_image_topic, "/kinect2/sd/image_ir");
    nh.param<std::string>("calib_data_file", calib_data_file, "calib");
    fileID=ros::Time::now().toSec();

    image_transport::ImageTransport it(nh);
     image_transport::Subscriber image_sub = it.subscribe(chess_image_topic,1, imageCallback);
     namedWindow("KUKA Calibration", 1);
     cv::moveWindow("KUKA Calibration",(int)(0.3*1366),(int)(0.3*768));

   // Socket

     struct sockaddr_in serv_addr, cli_addr;
     int n;
     int sockfd, newsockfd, calib_port;
     socklen_t clilen;
     nh.param<int>("calib_port", calib_port, 5559);
 	sockfd = socket(AF_INET, SOCK_STREAM, 0);
 	     if (sockfd < 0)
 	    	 perror("ERROR opening socket");
 	     bzero((char *) &serv_addr, sizeof(serv_addr));

 	     serv_addr.sin_family = AF_INET;
 	     serv_addr.sin_addr.s_addr = INADDR_ANY;
 	     serv_addr.sin_port = htons(calib_port);

 	     if (bind(sockfd, (struct sockaddr *) &serv_addr,
 	              sizeof(serv_addr)) < 0)
 	    	 perror("ERROR on binding");
 	     listen(sockfd,5);
 	     clilen = sizeof(cli_addr);
 	     int status = fcntl(sockfd, F_SETFL, fcntl(sockfd, F_GETFL, 0) | O_NONBLOCK);

 	              if (status == -1){
 	             	 perror("calling fcntl \n");
 	                // handle the error.  By the way, I've never seen fcntl fail in this way
 	              }
while(ros::ok())
{


	     newsockfd = accept(sockfd,
	                 (struct sockaddr *) &cli_addr,
	                 &clilen);
	     if (newsockfd < 0)
	     {
	    	 if(errno!=EWOULDBLOCK)perror("ERROR on accept");
	    	 else ROS_INFO_ONCE("Waiting Calibration Poses! \n");
	     }
	    else{ // get the poses


	     bzero(buffer,4095);
	     n = read(newsockfd,buffer,4095);
	     if (n < 0) perror("ERROR reading from socket");
	     buffer[n]='\0';
	   //  printf("%s\n",buffer);

	     memcpy( subbuff, &buffer[0], 2 );
	     subbuff[2] = '\0';
	     poseNumber=atoi( subbuff );
	     printf("Pose %d Received \n",poseNumber);
	     savePose=true;




	     }
ros::spinOnce();
}
 	    	     close(newsockfd);
 	    	     close(sockfd);

}

/******************************************************************/
//This function is called overtime a new image is published
void imageCallback(const sensor_msgs::ImageConstPtr& original_image)

{
    //Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        //Always copy, returning a mutable CvImage
        //OpenCV expects color images to use BGR channel order.
    	cv_ptr = cv_bridge::toCvCopy(original_image);
    	// cv_ptr = cv_bridge::toCvCopy(original_image, sensor_msgs::image_encodings::BGR8);
    	cv_ptr->image.convertTo(cv_ptr->image, CV_16UC1);
    	cv_ptr->image.copyTo(chess_image);
    }
    catch (cv_bridge::Exception& e)
    {
        //if there is an error during conversion, display it
        ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
        return;
    }





cv::imshow("KUKA Calibration", chess_image);
if(savePose){savePose=false;
char iamgeName[50];
char fileName[50];

std::sprintf(iamgeName, "Image%02d.png", poseNumber);
std::sprintf(fileName, "%s_%f.csv",calib_data_file.c_str(),fileID);

std::string path= ros::package::getPath("kuka_camera_calib");
path =path+ "/Calibration/Images/";
//printf("PATH: %s \n",path.c_str());
imwrite(path+iamgeName, chess_image); // A PNG FILE IS BEING SAVED

myfile.open ((path+fileName).c_str(), std::fstream::app);
myfile << buffer;

myfile.close();
//



Mat ok_image = Mat::zeros(chess_image.rows,chess_image.cols, CV_16UC1);

cv::circle(ok_image, Point(chess_image.cols/2,chess_image.rows/2),chess_image.rows/3,cv::Scalar(0xffff),11);
cv::putText(ok_image,iamgeName,Point(chess_image.cols/2-95,chess_image.rows/2), cv::FONT_HERSHEY_PLAIN,2,cv::Scalar(0xffff),2);

cv::imshow("KUKA Calibration", ok_image);
cv::waitKey(3); // wait
// Save the frame into a file

	sleep(2);
}

cv::waitKey(3);
}
