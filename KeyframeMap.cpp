/*
 * KeyframeMap.cpp
 *
 *  Created on: 5 Jun 2014
 *      Author: thomas
 */

#include "KeyframeMap.h"
#include <pcl/io/pcd_io.h>

KeyframeMap::KeyframeMap(bool filter)
 : filter(filter)
{

}

KeyframeMap::~KeyframeMap()
{
}

void KeyframeMap::addKeyframe(unsigned char * rgbImage,
                              unsigned short * depthData,
                              Eigen::Matrix3f Rcurr,
                              Eigen::Vector3f tcurr,
                              uint64_t time)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr frameCloud = Projection::convertToXYZRGBPointCloud(rgbImage, depthData);

    Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();

    transformation.topLeftCorner(3, 3) = Rcurr;
    transformation.topRightCorner(3, 1) = tcurr;

    map.push_back(Keyframe(frameCloud, transformation, time));
}

void KeyframeMap::applyPoses(iSAMInterface & isam)
{
    for(size_t i = 0; i < map.size(); i++)
    {
        map.at(i).pose = isam.getCameraPose(map.at(i).timestamp);
        pcl::transformPointCloud(*map.at(i).points, *map.at(i).points, map.at(i).pose);
        cloud.insert(cloud.end(), map.at(i).points->begin(), map.at(i).points->end());
    }

    if(filter)
    {
        pcl::VoxelGrid<pcl::PointXYZRGB> sor;
        sor.setInputCloud(cloud.makeShared());
        sor.setLeafSize(0.05, 0.05, 0.05);
        sor.filter(cloud);
    }

   // pcl::io::savePCDFileASCII ("/home/lili/workspace/SLAMDec27/src/pointCloudMap/labPointCloud.pcd", cloud.makeShared());
    //cout << "Saved data points to labPointCloud.pcd." << std::endl;

    map.clear();
}

pcl::PointCloud<pcl::PointXYZRGB> * KeyframeMap::getMap()
{
    return &cloud;
}
