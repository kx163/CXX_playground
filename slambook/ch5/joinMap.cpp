#include <iostream>
#include <fstream>
using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Geometry>
#include <boost/format.hpp>  // for formating string
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>


int main(int argc, char const *argv[])
{
    vector<cv::Mat> colorImgs, depthImgs;  // color and depth images
    vector<Eigen::Isometry3d> poses;  // camera poses

    string path = argv[1];

    ifstream fin(path + "/pose.txt");
    if (!fin) {
        cerr << "Please specify a correct path to pose.txt!" << endl;
        return 1;
    }

    for(int i = 0; i < 5; i++)
    {
        boost::format fmt(path + "/%s/%d.%s");  // image file format
        colorImgs.push_back(cv::imread( (fmt%"color"%(i+1)%"png").str() ));
        depthImgs.push_back(cv::imread( (fmt%"depth"%(i+1)%"pgm").str(), -1 ));  // using -1 to read the original image

        double data[7] = {0};
        
        for( auto& d:data )
            fin >> d;
        Eigen::Quaterniond q(data[6], data[3], data[4], data[5]);
        Eigen::Isometry3d T(q);
        T.pretranslate(Eigen::Vector3d(data[0], data[1], data[2]));
        poses.push_back(T);
    }
    
    // computing and fusing point cloud
    // camera intrinsic parameters
    double cx = 325.5, cy = 253.5, fx = 518.0, fy = 519.0, depthScale = 1000.0;

    cout << "Transforming the images to point clouds ..." << endl;
    // define the format used by point cloud, here is XYZRGB
    typedef pcl::PointXYZRGB PointT;
    typedef pcl::PointCloud<PointT> PointCloud;

    // create a new point cloud
    PointCloud::Ptr pointCloud(new PointCloud);
    for(int i = 0; i < 5; i++)
    {
        cout << "Transforming image: " << i+1 << endl;
        cv::Mat color = colorImgs[i];
        cv::Mat depth = depthImgs[i];
        Eigen::Isometry3d T = poses[i];
        
        for(int v = 0; v < depth.rows; v++)
        {
            for(int u = 0; u < depth.cols; u++)
            {
                unsigned int d = depth.ptr<unsigned short>(v)[u];  // depth value
                if ( d == 0) continue;  // 0 means no detection
                Eigen::Vector3d point;
                point[2] = double(d) / depthScale;
                point[0] = (u - cx) * point[2] / fx;
                point[1] = (v - cy) * point[2] / fy;
                Eigen::Vector3d pointWorld = T * point;

                PointT p;
                p.x = pointWorld[0];
                p.y = pointWorld[1];
                p.z = pointWorld[2];
                p.b = color.data[v*color.step + u*color.channels()];
                p.g = color.data[v*color.step + u*color.channels() + 1];
                p.r = color.data[v*color.step + u*color.channels() + 2];
                pointCloud->points.push_back(p);
            }
        }
    }

    pointCloud->is_dense = false;
    cout << "The point cloud has " << pointCloud->size() << " points." << endl;
    pcl::io::savePCDFileBinary(path + "/map.pcd", *pointCloud);
    
    return 0;
}
