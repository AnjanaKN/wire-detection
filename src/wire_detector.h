// ===================== c++ Headers ====================
#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <string>
#include <iomanip>
#include <fstream>
#include <vector>
// ===================== ROS Headers ====================
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
// ===================== PCL Headers ====================
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/io/pcd_io.h>
#include <pcl_ros/filters/voxel_grid.h>
#include <pcl_ros/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/ModelCoefficients.h>

// ===================== OPENCV Headers ====================
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d.hpp>

#include "opencv2/imgcodecs.hpp"




using namespace std;
using namespace cv;
using namespace pcl;


class WireDetector{

public: 
	//Function to detect horizontal lines in an gray image, takes image and line orientation threshold as inputs and returns false colored image
	cv::Mat Detect2DLines(cv::Mat & out_image,float angle_thresh = 0.05){
		cv::Mat image,img_prewittx,kernelx;
		int filter_a=1;
		int filter_b=1;
		
		//pre-processing
		//blur(out_image,out_image,Size(5,5));
	    cvtColor(out_image, image, COLOR_BGR2GRAY);
        


	    //horizontal edge detetion
		kernelx = (Mat_<int>(3,5) <<filter_a, filter_a, filter_a,filter_a,filter_a,0,0, 0, 0, 0, -filter_b, -filter_b, -filter_b,-filter_b, -filter_b);
		

		filter2D(image,img_prewittx, -1, kernelx);
		threshold(img_prewittx, img_prewittx, 60, 255, 0);

        //Canny(image, img_prewittx, 2, 80, 3);

		//Hough line detection
	    vector<Vec4i> linesP;
	    HoughLinesP(img_prewittx, linesP, 1, CV_PI/180, 50, 200, 10);
	    //HoughLinesP(img_prewittx, linesP, 1, CV_PI/180, 15, 30, 20);
	    //cout<<linesP.size()<<endl;
	    for( size_t i = 0; i < linesP.size(); i++ )
	    {
	        Vec4i l = linesP[i];
	        float angle = atan2(l[1] - l[3], l[0] - l[2]);
	        if (abs(angle)<=CV_PI+angle_thresh &&abs(angle)>=CV_PI-angle_thresh){
				//Printing line on output image for display
	       	 	line(out_image, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,255,0), 5, LINE_AA);
	    	}
	    }



	    //Image display
	    namedWindow("Prewitt edge", WINDOW_NORMAL);
	    imshow("Prewitt edge", img_prewittx);
	    namedWindow("lINES", WINDOW_NORMAL);
	    imshow("lINES", out_image);
    	waitKey();
    	

    	return out_image;
	}




//RANSAC 3D line fitting function,input - point cloud, output- inlier indices
	vector<int> fit3Dlines(PointCloud<PointXYZ> wire_cloud,float thresh=0.01){

		 
	    SampleConsensusModelLinePtr model_l (new SampleConsensusModelLine<PointXYZ> (wire_cloud.makeShared ()));
	    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_l);
	    ransac.setDistanceThreshold (thresh);
	    ransac.computeModel();
	    ransac.getInliers(inliers);


		return inliers;
	}

private:
	typedef SampleConsensusModelLine<PointXYZ>::Ptr SampleConsensusModelLinePtr;
	std::vector<int> inliers;
};