#include "wire_detector.h"


using namespace std;
using namespace cv;




int main(){

//=========================Comment this part out to read image and disparity from ros topic===========================
string img_filename="/home/anjana/catkin_ws/src/wire_detector/data/left/frame0004.jpg";
string disparity_name="";
cv::Mat wire_img = imread(img_filename);
cv::Mat disparity_image=imread(disparity_name);
PointCloud<PointXYZ> cloud;
cloud.points.resize (10);
cloud.points[0].getVector3fMap () <<  1.0f,  2.00f,  3.00f;
cloud.points[1].getVector3fMap () <<  4.0f,  5.00f,  6.00f;
cloud.points[2].getVector3fMap () <<  7.0f,  8.00f,  9.00f;
cloud.points[3].getVector3fMap () << 10.0f, 11.00f, 12.00f;
cloud.points[4].getVector3fMap () << 13.0f, 14.00f, 15.00f;
cloud.points[5].getVector3fMap () << 16.0f, 17.00f, 18.00f;
cloud.points[6].getVector3fMap () << 19.0f, 20.00f, 21.00f;
cloud.points[7].getVector3fMap () << 22.0f, 23.00f, 24.00f;
cloud.points[8].getVector3fMap () << -5.0f,  1.57f,  0.75f;
cloud.points[9].getVector3fMap () << 4.0f, 2.00f, 3.00f;


//===================================================================================================================

float x_wire,y_wire,z_wire,depth_min=0.95,depth_max=2.0;
WireDetector w;
cv::Mat wire_colored_image;
wire_colored_image = w.Detect2DLines(wire_img);


//Populate the wire cloud
/*for (int i=0;i<wire_lines.size();i++)
{
    wire_line=wire_lines[i];

    i=wire_line[0];
    j=wire_line[1];
    z_wire = (float)((-1* f_norm * base_line) / (disparity_image.at<float>(i,j)));
    if(z_wire<depth_max && z_wire>depth_min){
        x_wire = (float)(z_wire/f_norm) * (j - Cx); // cols
        y_wire= (float)(z_wire/f_norm) * (i - Cy); // rows
        wire_cloud.push_back(Point3f(x_wire,y_wire,z_wire));
   }
}*/



vector<int> wire_indices = w.fit3Dlines(cloud, 0.01);

return 0;
}