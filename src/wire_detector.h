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
#include "pcl/common/impl/angles.hpp"
#include <pcl_ros/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl_ros/filters/voxel_grid.h>
#include <pcl_ros/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>
#include <pcl/common/eigen.h>
#include <pcl/common/common.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/sac_model_parallel_line.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/visualization/pcl_visualizer.h>
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
using namespace ros;



class fitted_lines{

public:
	fitted_lines(){};
	fitted_lines(vector<int> h, vector<int> v):horizontal_inliers(h),vertical_inliers(v){};
	vector<int> horizontal_inliers;
	vector<int> vertical_inliers;
};


class WireDetector{

public: 
	//Function to detect horizontal lines in an gray image, takes image and line orientation threshold as inputs and returns points belonging to line
	vector<Point> Detect2DLines(cv::Mat & out_image,float angle_thresh = 0.05){
		cv::Mat image,img_prewittx,kernelx,new_image;
		int filter_a=1;
		int filter_b=1;
		
		//pre-processing
		//blur(out_image,out_image,Size(5,5));
	    cvtColor(out_image, image, COLOR_BGR2GRAY);
        
		Mat1b mask(out_image.size(), uchar(0));

	    //horizontal edge detetion
		kernelx = (Mat_<int>(3,5) <<filter_a, filter_a, filter_a,filter_a,filter_a,0,0, 0, 0, 0, -filter_b, -filter_b, -filter_b,-filter_b, -filter_b);
		

		filter2D(image,img_prewittx, -1, kernelx);

		namedWindow("Input image with green lines", WINDOW_NORMAL);
		imshow("Input image with green lines", img_prewittx);

    	waitKey(0);
		threshold(img_prewittx, img_prewittx, 60, 255, 0);

		namedWindow("Input image with green lines", WINDOW_NORMAL);
		imshow("Input image with green lines",img_prewittx);
		
    	waitKey(0);
    	//DestroyWindow("Input image with green lines");

        //Canny(image, img_prewittx, 2, 80, 3);

		//Hough line detection
	    vector<Vec4i> linesP;
	    HoughLinesP(img_prewittx, linesP, 1, CV_PI/180, 50, 200, 10);
	    //HoughLinesP(img_prewittx, linesP, 1, CV_PI/180, 15, 30, 20);
	    //cout<<linesP.size()<<endl;

	    vector<Point> points_of_line;
	   
	    for( size_t i = 0; i < linesP.size(); i++ )
	    {
	        Vec4i l = linesP[i];
	        float angle = atan2(l[1] - l[3], l[0] - l[2]);
	        if (abs(angle)<=CV_PI+angle_thresh &&abs(angle)>=CV_PI-angle_thresh){
				


				LineIterator lit(image, Point(l[0], l[1]), Point(l[2], l[3]));
				
				    for (int j = 0; j < lit.count; ++j, ++lit)
				    {
				        points_of_line.push_back(lit.pos());
				    }
					//cout<<points_of_line.size()<<endl;
				//Creating mask in case the user wants to access masked image
	       	 	line(mask, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255), 5, LINE_AA);

	       	 	//Printing line on output image for display
	       	 	line(out_image, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,255,0), 5, LINE_AA);
	       	 	
	       	 	
	    	}
	    	
	    
	    	
		}
			out_image.copyTo(new_image, mask);
			//cout<<"copied"<<endl;
			//string filename ="Left_masked.png";
			//imwrite(filename,new_image);
	    //Image display	   
		//namedWindow("Input image with green lines", WINDOW_NORMAL);
		//imshow("Input image with green lines", out_image);
    	//waitKey();
    	

    	return points_of_line;
	}


pcl::visualization::PCLVisualizer::Ptr simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr input_cloud, pcl::PointCloud<pcl::PointXYZ>::ConstPtr output_cloud)
{
  
	pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
 
  
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(output_cloud, 0, 255, 0);
    viewer->addPointCloud<pcl::PointXYZ> (output_cloud, single_color, "cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
    viewer->initCameraParameters ();
    return (viewer);
}





//RANSAC 3D line fitting function,input - point cloud, output- inlier indices
fitted_lines fit3Dlines(PointCloud<PointXYZ> wire_cloud,float thresh=0.01){
		vector<int> v_inliers;
		vector<int> h_inliers;
		Eigen::VectorXf wire,cordon;
		pcl::ModelCoefficients line1,line2;
		Eigen::Vector3f ax,ax2;
		ax<<0.0,1.0,0.0;
		SampleConsensusModelParallelLine<pcl::PointXYZ>::Ptr model_l (new SampleConsensusModelParallelLine<PointXYZ> (wire_cloud.makeShared ()));
		model_l->setAxis(ax);
	    model_l->setEpsAngle(pcl::deg2rad (10.0));
		pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_l);
	   	ransac.setDistanceThreshold (thresh);
	    ransac.computeModel();
	    ransac.getInliers(h_inliers);
	    ransac.getModelCoefficients(wire);
	    //cout<<line1.values.size()<<endl;
	    for (int j=0;j<wire.size();j++)
	    {
	    	line1.values.push_back(wire[j]);
	    }
	    
		ax2<<1.0,0.0,0.0;
		//SampleConsensusModelParallelLine<pcl::PointXYZ>::Ptr model_l (new SampleConsensusModelParallelLine<PointXYZ> (wire_cloud.makeShared ()));
		model_l->setAxis(ax);
	    model_l->setEpsAngle(pcl::deg2rad (10.0));
		pcl::RandomSampleConsensus<pcl::PointXYZ> ransac2 (model_l);
	   	ransac2.setDistanceThreshold (thresh);
	    ransac2.computeModel();
	    ransac2.getInliers(v_inliers);
	    ransac2.getModelCoefficients(cordon);
	      for (int j=0;j<cordon.size();j++)
	    {
	    	line2.values.push_back(cordon[j]);
	    }
	    vtkSmartPointer<vtkDataSet> data1 = pcl::visualization::createLine (line1);
	    vtkSmartPointer<vtkDataSet> data2 = pcl::visualization::createLine (line2);
	    //pcl::visualization::PCLVisualizer::Ptr viewer;
	    //pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);
	    //pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ> (wire_cloud));
 		//pcl::copyPointCloud<pcl::PointXYZ>(wire_cloud, inliers, *final);
 		
 		//Store point cloud of wire
 		//pcl::PLYWriter wr;
		//wr.write("/home/anjana/catkin_ws/wire_detector_20_detected.ply", *final);
   		
		//Visualize point cloud of wire
   		/*
   		viewer = simpleVis(input_cloud,final);
	 	 while (!viewer->wasStopped ())
			  {
			    viewer->spinOnce (100);
			    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
			  }

        */
	    	fitted_lines f(h_inliers,v_inliers);
			return f;
		}



private:
	typedef SampleConsensusModelLine<PointXYZ>::Ptr SampleConsensusModelLinePtr;
	std::vector<int> inliers;
};
