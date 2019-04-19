#include "wire_detector.h"
#include <message_filters/subscriber.h>


#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <pcl/point_cloud.h>

using namespace std;
using namespace cv;


ros::Publisher wired_pub;

namespace patch
{
    template < typename T > std::string to_string( const T& n )
    {
        std::ostringstream stm ;
        stm << n ;
        return stm.str() ;
    }
}

/*int i=20;
class cloudHandler{
public:
    cloudHandler(){
       pcl_sub = nh.subscribe("/stereo/points2", 10, &cloudHandler::cloudCB, this);
    }

void cloudCB(const sensor_msgs::PointCloud2 &input){
    pcl::PLYWriter w;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(input, *cloud_in);
    std::vector<int> indices;
    string out_name = "pruner_cloud"+patch::to_string(i)+".ply";
    i++;
    cout<<"I: "<<i<<endl;
    pcl::removeNaNFromPointCloud (*cloud_in, *cloud_out,indices);
    
    w.write(out_name,*cloud_out);
    //WireDetector ww;
    //vector<int> wire_indices = ww.fit3Dlines(*cloud);

}


protected:
    ros::NodeHandle nh;
    ros::Subscriber pcl_sub;

  
};
*/


float z_val=-10.0;

void callback(const sensor_msgs::PointCloud2ConstPtr& mega)
{
    WireDetector w;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*mega ,*cloud);
    vector<int> inliers =w.fit3Dlines(*cloud);
    
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr wires( new pcl::PointCloud<pcl::PointXYZ>);
   

    //boost::shared_ptr< vector<int> > indicesPtr(new vector<int>(inliers));
    //pcl::ExtractIndices<pcl::PointXYZ> extract(true);
    //extract.setInputCloud (cloud);
    //extract.setIndices (indicesPtr);
    //extract.setNegative (true);
    //extract.applyFilterIndices(inliers);
    //extract.filter (*cloud_final);
    float x,y,z;
    pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *wires);
    for (int i=0;i<inliers.size();i++)

    {   x=wires->points[i].x;
        y=wires->points[i].y;
        z=wires->points[i].z+z_val;
        wires->points.push_back(pcl::PointXYZ(x,y,z));
    }
        
    sensor_msgs::PointCloud2 output;
        
    pcl::toROSMsg(*wires, output);
    output.header.frame_id = "camera_link";
    
    wired_pub.publish(output);
    
}

int main(int argc,char** argv){


    ros::init(argc, argv, "wire_detector");
    ros::NodeHandle nh;

    wired_pub = nh.advertise<sensor_msgs::PointCloud2> ("/wire", 1);
    //2d DETETCION
    //string img_filename="/media/anjana/bfd698a2-ee0e-403e-955d-bada0ce8da97/home/agvbotics/ros_ws/old src/pruner_image/src/a.png";
    //cv::Mat left = imread(img_filename);
    //vector<Point> points = w.Detect2DLines(left,0.2);

    //string img_filename="/home/anjana/catkin_ws/src/wire_detector/data/left/frame0000.jpg";
    //cv::Mat left = imread(img_filename);
    //vector<Point> points = w.Detect2DLines(left,0.1745);
    
    //ros::Subscriber<sensor_msgs::PointCloud2> wire_sub(nh, "/mega_cloud", 1);
    ros::Subscriber sub =  nh.subscribe("/mega_cloud",10, callback);
    //registerCallback(boost::bind(&callback, _1));


      ros::spin();


    //pcl::PLYReader Reader;
    //Reader.read("/home/anjana/catkin_ws/wire_detector_20.ply", *cloud);
    //vector<int> inliers =w.fit3Dlines(*cloud);



//ros::init(argc, argv, "pcl_matching");
//cloudHandler handler;
 //ros::spin();


return 0;

}



/*

    pcl::PointCloud<pcl::PointXYZ> wire_cloud;
    pcl::PointXYZ cloud_points;
    float base_line;
    int winSize = 12;
    int minDisp = 16;
    int maxDisp = 400;//128+16; // 208 224 240 256 272 304 400

    int count_call_back = 0;

    Mat sgbm_disp, sgbm_disp_color,disp_new;


    double f_norm;
    double Cx;
    double Cy;
//=========================Comment this part out to read image and disparity from ros topic===========================
    string img_filename="/home/anjana/catkin_ws/src/wire_detector/data/left/frame0004.jpg";

    cv::Mat left = imread(img_filename);
    cv::Mat right= imread("/home/anjana/catkin_ws/src/wire_detector/data/right/frame0004.jpg");

    cout<<"read images"<<left.size()<<right.size()<<endl;

    base_line = -0.108715;

    f_norm =  1507.6098/2;
    Cx = 1280.7518/2;
    Cy = 1022.667/2;



    Mat disp8(left.rows, left.cols,CV_32F);

    Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(minDisp,maxDisp,winSize);
    sgbm->setP1(0);
    sgbm->setP2(0);
    sgbm->setMinDisparity(minDisp);
    sgbm->setNumDisparities(maxDisp);
    sgbm->setUniquenessRatio(25);
    sgbm->setPreFilterCap(10);
    sgbm->setSpeckleWindowSize(15);
    sgbm->setSpeckleRange(7);   // lower the better
    sgbm->setDisp12MaxDiff(10);
    sgbm->setMode(StereoSGBM::MODE_SGBM_3WAY);

    cout<<"computing sgbm"<<endl;
    sgbm->compute(left,right,sgbm_disp);
    cout<<"Computed sgbm"<<endl;
    sgbm_disp.convertTo(disp8, CV_32F, 1.0/16.0);



    double min;
    double max;
    minMaxIdx(disp8, &min, &max);
    Mat cm_disp, scaledDisparityMap;
    cout << "disp min " << min << endl << "disp max " << max << endl;
    convertScaleAbs( disp8, scaledDisparityMap, 255 / ( max - min ) );
    applyColorMap( scaledDisparityMap, cm_disp, COLORMAP_JET );
    namedWindow("Disparity", WINDOW_NORMAL);
    imshow("Disparity",cm_disp);
    waitKey();

    float x_wire,y_wire,z_wire,depth_min=0.95,depth_max=2.0;
    WireDetector w;
    
    vector<Point> points = w.Detect2DLines(left);
    cout<<"Number of line points: "<<points.size()<<endl;


    for (int k=0;k<points.size();k++)
    {
        int i=points[k].x;
        int j=points[k].y;
        cloud_points.z = (float)((-1* f_norm * base_line) / (disp8.at<float>(i,j)));
        if(cloud_points.z<depth_max && cloud_points.z>depth_min){
            cloud_points.x = (float)(cloud_points.z/f_norm) * (j - Cx); // cols
            cloud_points.y= (float)(cloud_points.z/f_norm) * (i - Cy); // rows
            wire_cloud.push_back(cloud_points);
       }
    }
    pcl::PLYWriter wr;
    string out_name="wire_cloud1.ply";
    wr.write(out_name,wire_cloud);

    return 0;
}


*/
