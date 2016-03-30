#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <cstdio>
#include <iostream>
#include <ctime>
#include <isam/Pose3d.h>
#include "KinectCamera.h"

#ifndef RGBDODOMETRY_H_
#define RGBDODOMETRY_H_

using namespace std;

class RGBDOdometry
{

public:
    RGBDOdometry(KinectCamera * kinectCamera):camera(kinectCamera) {}

    static void cvtDepth2Cloud( const cv::Mat& depth, cv::Mat& cloud, const cv::Mat& cameraMatrix )
    {
        const float inv_fx = 1.f/cameraMatrix.at<float>(0,0);
        const float inv_fy = 1.f/cameraMatrix.at<float>(1,1);
        const float ox = cameraMatrix.at<float>(0,2);
        const float oy = cameraMatrix.at<float>(1,2);
        cloud.create( depth.size(), CV_32FC3 );
        for( int y = 0; y < cloud.rows; y++ )
        {
            cv::Point3f* cloud_ptr = (cv::Point3f*)cloud.ptr(y);
            const float* depth_prt = (const float*) depth.ptr(y);
            for( int x = 0; x < cloud.cols; x++ )
            {
                float z = depth_prt[x];
                cloud_ptr[x].x = (x - ox) * z * inv_fx;
                cloud_ptr[x].y = (y - oy) * z * inv_fy;
                cloud_ptr[x].z = z;
            }
        }
    }

    template<class ImageElemType>
    static void warpImage( const cv::Mat& image, const cv::Mat& depth,
                            const cv::Mat& Rt, const cv::Mat& cameraMatrix, const cv::Mat& distCoeff,
                            cv::Mat& warpedImage )
    {
        const cv::Rect rect = cv::Rect(0, 0, image.cols, image.rows);

        vector<cv::Point2f> points2d;
        cv::Mat cloud, transformedCloud;

        cvtDepth2Cloud( depth, cloud, cameraMatrix );
        cv::perspectiveTransform( cloud, transformedCloud, Rt );
        cv::projectPoints( transformedCloud.reshape(3,1), cv::Mat::eye(3,3,CV_64FC1), cv::Mat::zeros(3,1,CV_64FC1), cameraMatrix, distCoeff, points2d );

        cv::Mat pointsPositions( points2d );
        pointsPositions = pointsPositions.reshape( 2, image.rows );

        warpedImage.create( image.size(), image.type() );
        warpedImage = cv::Scalar::all(0);

        cv::Mat zBuffer( image.size(), CV_32FC1, FLT_MAX );
        for( int y = 0; y < image.rows; y++ )
        {
            for( int x = 0; x < image.cols; x++ )
            {
                const cv::Point3f p3d = transformedCloud.at<cv::Point3f>(y,x);
                const cv::Point p2d = pointsPositions.at<cv::Point2f>(y,x);
                if( !cvIsNaN(cloud.at<cv::Point3f>(y,x).z) && cloud.at<cv::Point3f>(y,x).z > 0 &&
                    rect.contains(p2d) && zBuffer.at<float>(p2d) > p3d.z )
                {
                    warpedImage.at<ImageElemType>(p2d) = image.at<ImageElemType>(y,x);
                    zBuffer.at<float>(p2d) = p3d.z;
                }
            }
        }
    }

    Eigen::Matrix4d getRelativeTransformation(const cv::Mat &colorImage0,
                                              const cv::Mat &depth0,
                                              const cv::Mat &colorImage1,
                                              const cv::Mat &depth1)
    {
        const cv::Mat distCoeff(1,5,CV_32FC1,cv::Scalar(0));

        cv::Mat grayImage0, grayImage1, depthFlt0, depthFlt1/*in meters*/;
        cv::cvtColor( colorImage0, grayImage0, CV_BGR2GRAY );
        cv::cvtColor( colorImage1, grayImage1, CV_BGR2GRAY );
        depth0.convertTo( depthFlt0, CV_32FC1, 1./1000 );
        depth1.convertTo( depthFlt1, CV_32FC1, 1./1000 );

        cv::Mat Rt;

        vector<int> iterCounts(4);
        iterCounts[0] = 7;
        iterCounts[1] = 7;
        iterCounts[2] = 7;
        iterCounts[3] = 10;

        vector<float> minGradMagnitudes(4);
        minGradMagnitudes[0] = 12;
        minGradMagnitudes[1] = 5;
        minGradMagnitudes[2] = 3;
        minGradMagnitudes[3] = 1;

        const float minDepth = 0.01f; //in meters
        const float maxDepth = 9.f; //in meters
        const float maxDepthDiff = 0.07f; //in meters


        float vals[] = {528., 0., 320,
                    0., 528., 240,
                    0., 0., 1.};

        const cv::Mat cameraMatrix = cv::Mat(3,3,CV_32FC1,vals);

        bool isFound = cv::RGBDOdometry( Rt, cv::Mat(),
                                        grayImage0, depthFlt0, cv::Mat(),
                                        grayImage1, depthFlt1, cv::Mat(),
                                        cameraMatrix, minDepth, maxDepth, maxDepthDiff,
                                        iterCounts, minGradMagnitudes,  cv::RIGID_BODY_MOTION);

        Eigen::Matrix4d T;

        T(0,0) = Rt.at<double>(0,0);
        T(0,1) = Rt.at<double>(0,1);
        T(0,2) = Rt.at<double>(0,2);
        T(1,0) = Rt.at<double>(1,0);
        T(1,1) = Rt.at<double>(1,1);
        T(1,2) = Rt.at<double>(1,2);
        T(2,0) = Rt.at<double>(2,0);
        T(2,1) = Rt.at<double>(2,1);
        T(2,2) = Rt.at<double>(2,2);
        T(0,3) = Rt.at<double>(0,3);
        T(1,3) = Rt.at<double>(1,3);
        T(2,3) = Rt.at<double>(2,3);
        T(3,0) = 0;
        T(3,1) = 0;
        T(3,2) = 0;
        T(3,3) = 1;

        if(isFound)
        {
            return T;
        }
        else
        {
            cout<<"cannot find transfomration"<<endl;
        }

    }
private:
        KinectCamera * camera;
};
#endif /* RGBDODOMETRY_H_ */
