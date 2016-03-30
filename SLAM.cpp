#include "RawLogReader.h"
#include "KeyframeMap.h"
#include "PoseGraph/iSAMInterface.h"
#include "PlaceRecognition/PlaceRecognition.h"
#include "Odometry/FOVISOdometry.h"
#include "Odometry/DVOdometry.h"

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

//octomap
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>

#include <vector>

void drawPoses(std::vector<std::pair<uint64_t, Eigen::Matrix4f> > & poses,
               pcl::visualization::PCLVisualizer & cloudViewer, double r, double g, double b)
{
    static int count = 0;

    for(size_t i = 1; i < poses.size(); i++)
    {
        pcl::PointXYZ p1, p2;

        p1.x = poses.at(i - 1).second(0, 3);
        p1.y = poses.at(i - 1).second(1, 3);
        p1.z = poses.at(i - 1).second(2, 3);

        p2.x = poses.at(i).second(0, 3);
        p2.y = poses.at(i).second(1, 3);
        p2.z = poses.at(i).second(2, 3);

        std::stringstream strs;

        strs << "l" << count++;

        cloudViewer.addLine(p1, p2, r, g, b, strs.str());
    }
}

/*
struct RGBDFile {

    double rgb_timestamp;
    string rgb_frame;
    double depth_timestamp;
    string depth_frame;
};

vector<RGBDFile> RGBDFileInfo;

void readRGBDdata(string filename)
{

    string line, word;

    fstream fin;

    fin.open(filename);

    if(!fin)
    {
        cout<<"cannot open the file"<<endl;
    }
    else
    {
        cout<<"file is open"<<endl;
    }

    istringstream istr;

    string str;

    while(getline(fin, line))
    {
       RGBDFile fileInfo;
       istringstream record(line); //bind record to the line we just read
       record >> fileInfo.rgb_timestamp; //read the name
       record >> fileInfo.rgb_frame;
       record >> fileInfo.depth_timestamp;
       record >> fileInfo.depth_frame;
       RGBDFileInfo.push_back(fileInfo);

    }

    fin.close();

    int frame_index = 0;

    while(frame_index < RGBDFileInfo.size())
    {
        cv::Mat rgb_frame=imread(RGBDFileInfo[frame_index].rgb_frame, 1);
        cv::Mat depth_frame=imread(RGBDFileInfo[frame_index].depth_frame, -1);

        if(!rgb_frame.data )                              // Check for invalid input
        {
            cout <<  "Could not open or find the image" << std::endl ;

        }

        namedWindow( "RGB", WINDOW_AUTOSIZE );// Create a window for display.
        cv::imshow("RGB", rgb_frame);
        cv::imshow("Depth", depth_frame);

        char key = cv::waitKey(1);

        if(key == 'q')
        {
            break;
        }
        else if(key == ' ')
        {
            key = cv::waitKey(0);
        }

        frame_index++;
    }

}


cv::Mat readDepthImage(const char *fileName)
{
    cv::Mat depth = cv::imread(fileName, -1);
    assert(depth.type() == 2);
    return depth;
}
*/

void printCovariance(ofstream &printName,  const Eigen::MatrixXd &covariance)
{
    for(int i=0; i<covariance.rows(); i++)
    {
        for(int j=0; j<covariance.cols(); j++)
        {
             printName<<covariance(i,j)<<" ";
        }
    }
    printName<<endl;

}

int main(int argc, char * argv[])
{
    int width = 640;
    int height = 480;

    Resolution::getInstance(width, height);

    Intrinsics::getInstance(528, 528, 320, 240);

    cv::Mat intrinsicMatrix = cv::Mat(3,3,CV_64F);

    intrinsicMatrix.at<double>(0,0) = Intrinsics::getInstance().fx();
    intrinsicMatrix.at<double>(1,1) = Intrinsics::getInstance().fy();

    intrinsicMatrix.at<double>(0,2) = Intrinsics::getInstance().cx();
    intrinsicMatrix.at<double>(1,2) = Intrinsics::getInstance().cy();

    intrinsicMatrix.at<double>(0,1) =0;
    intrinsicMatrix.at<double>(1,0) =0;

    intrinsicMatrix.at<double>(2,0) =0;
    intrinsicMatrix.at<double>(2,1) =0;
    intrinsicMatrix.at<double>(2,2) =1;

    Bytef * decompressionBuffer = new Bytef[Resolution::getInstance().numPixels() * 2];
    IplImage * deCompImage = 0;

    std::string logFile="/home/lili/Kinect_Logs/2015-11-05.00.klg";
   // assert(pcl::console::parse_argument(argc, argv, "-l", logFile) > 0 && "Please provide a log file");

    RawLogReader logReader(decompressionBuffer,
                           deCompImage,
                           logFile,
                           true);


    cv::Mat1b tmp(height, width);
    cv::Mat3b depthImg(height, width);

    PlaceRecognition placeRecognition(&intrinsicMatrix);

    iSAMInterface iSAM;

    //Keyframes
    KeyframeMap map(true);
    Eigen::Vector3f lastPlaceRecognitionTrans = Eigen::Vector3f::Zero();
    Eigen::Matrix3f lastPlaceRecognitionRot = Eigen::Matrix3f::Identity();
    int64_t lastTime = 0;

    OdometryProvider * odom = 0;


    //int frame_index = 0;

   // uint64_t timestamp;

    /*if(true)
    {
        odom = new FOVISOdometry;
        if(logReader.hasMore())
        {
            logReader.getNext();

            Eigen::Matrix3f Rcurr = Eigen::Matrix3f::Identity();
            Eigen::Vector3f tcurr = Eigen::Vector3f::Zero();

            odom->getIncrementalTransformation(tcurr,
                                               Rcurr,
                                               logReader.timestamp,
                                               (unsigned char *)logReader.deCompImage->imageData,
                                               (unsigned short *)&decompressionBuffer[0]);
        }

    }*/
    //else
   // {
        odom = new DVOdometry;

        if(logReader.hasMore())
        {
            logReader.getNext();

            DVOdometry * dvo = static_cast<DVOdometry *>(odom);

            dvo->firstRun((unsigned char *)logReader.deCompImage->imageData,
                          (unsigned short *)&decompressionBuffer[0]);
        }
    //}

    ofstream fout1("camera_pose_DVOMarch28.txt");
    ofstream fout2("camera_pose_KeyframeMotionMetric0.1March28.txt");
    ofstream fout3("loop_closure_transformationMarch28.txt");
    ofstream fout4("camera_pose_after_optimizationMarch28.txt");
    ofstream fout5("camera_pose_after_optimizationMarch28DVOCov.txt");
    ofstream fout6("camera_pose_after_optimizationMarch28DVOLoopTransCov.txt");

    /*
    pcl::visualization::PCLVisualizer cloudViewer;

    cloudViewer.setBackgroundColor(1, 1, 1);
    cloudViewer.initCameraParameters();
    cloudViewer.addCoordinateSystem(0.1, 0, 0, 0);
    */
    //pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> color(cloud->makeShared());
    //cloudViewer.addPointCloud<pcl::PointXYZRGB>(cloud->makeShared(), color, "Cloud Viewer");


    int loopClosureCount=0;

    while(logReader.hasMore())
    {
        logReader.getNext();

        cv::Mat3b rgbImg(height, width, (cv::Vec<unsigned char, 3> *)logReader.deCompImage->imageData);

        cv::Mat1w depth(height, width, (unsigned short *)&decompressionBuffer[0]);

        cv::normalize(depth, tmp, 0, 255, cv::NORM_MINMAX, 0);

        cv::cvtColor(tmp, depthImg, CV_GRAY2RGB);

        cv::imshow("RGB", rgbImg);

        cv::imshow("Depth", depthImg);

        char key = cv::waitKey(1);

        if(key == 'q')
        {
            break;
        }
        else if(key == ' ')
        {
            key = cv::waitKey(0);
        }
        if(key == 'q')
        {
            break;
        }

        Eigen::Matrix3f Rcurr = Eigen::Matrix3f::Identity();
        Eigen::Vector3f tcurr = Eigen::Vector3f::Zero();



//        #1
        odom->getIncrementalTransformation(tcurr,
                                          Rcurr,
                                          logReader.timestamp,
                                          (unsigned char *)logReader.deCompImage->imageData,
                                          (unsigned short *)&decompressionBuffer[0]);


       fout1<<tcurr[0]<<" "<<tcurr[1]<<" "<<tcurr[2]<<" "<<Rcurr(0,0)<<" "<<Rcurr(0,1)<<" "<<Rcurr(0,2)<<" "<<Rcurr(1,0)<<" "<<Rcurr(1,1)<<" "<<Rcurr(1,2)<<" "<<Rcurr(2,0)<<" "<<Rcurr(2,1)<<" "<<Rcurr(2,2)<<endl;

        Eigen::Matrix3f Rdelta = Rcurr.inverse() * lastPlaceRecognitionRot;
        Eigen::Vector3f tdelta = tcurr - lastPlaceRecognitionTrans;

        //Eigen::MatrixXd covariance = odom->getCovariance();
         //Eigen::MatrixXd covariance=Eigen::Matrix<double, 6, 6>::Identity()* 1e-3;

        if((Projection::rodrigues2(Rdelta).norm() + tdelta.norm())  >= 0.1)
        {
            Eigen::MatrixXd covariance = odom->getCovariance();
            iSAM.addCameraCameraConstraint(lastTime,
                                           logReader.timestamp,
                                           lastPlaceRecognitionRot,
                                           lastPlaceRecognitionTrans,
                                           Rcurr,
                                           tcurr);
                                           //covariance);

            printCovariance(fout5,  covariance);

            lastTime = logReader.timestamp;

            lastPlaceRecognitionRot = Rcurr;
            lastPlaceRecognitionTrans = tcurr;

            cout<<"before add keyframe"<<endl;

//            #2
            map.addKeyframe((unsigned char *)logReader.deCompImage->imageData,
                            (unsigned short *)&decompressionBuffer[0],
                            Rcurr,
                            tcurr,
                            logReader.timestamp);

           fout2<<tcurr[0]<<" "<<tcurr[1]<<" "<<tcurr[2]<<" "<<Rcurr(0,0)<<" "<<Rcurr(0,1)<<" "<<Rcurr(0,2)<<" "<<Rcurr(1,0)<<" "<<Rcurr(1,1)<<" "<<Rcurr(1,2)<<" "<<Rcurr(2,0)<<" "<<Rcurr(2,1)<<" "<<Rcurr(2,2)<<endl;

           /*
            //Save keyframe
           {
            cv::Mat3b rgbImgKeyframe(height, width, (cv::Vec<unsigned char, 3> *)logReader.deCompImage->imageData);

            cv::Mat1w depthImgKeyframe(height, width, (unsigned short *)&decompressionBuffer[0]);

            //save keyframe depth
            char fileName[1024] = {NULL};
            sprintf(fileName, "keyframe_depth_%06d.png", frame_index);
            cv::imwrite(fileName, depthImgKeyframe);

            //save keyframe rgb

            sprintf(fileName, "keyframe_rgb_%06d.png", frame_index);
            cv::imwrite(fileName, rgbImgKeyframe);
            frame_index ++;

           }
        */

            int64_t matchTime;
            Eigen::Matrix4d transformation;
           // Eigen::MatrixXd cov(6,6);
            //isam::Covariance(0.001 * Eigen::Matrix<double, 6, 6>::Identity()))
            Eigen::MatrixXd cov=0.001 * Eigen::Matrix<double, 6, 6>::Identity();


            cout<<"map.addKeyframe is OK"<<endl;

//            #3
            if(placeRecognition.detectLoop((unsigned char *)logReader.deCompImage->imageData,
                                           (unsigned short *)&decompressionBuffer[0],
                                           logReader.timestamp,
                                           matchTime,
                                           transformation,
                                           cov,
                                           loopClosureCount))
            {

                //printCovariance(fout6,  cov);
               cout<<"logReader.timestamp "<<logReader.timestamp<<endl;
               cout<<"matchTime "<<matchTime<<endl;

               /*
               transformation << -0.2913457145219732, 0.228056050293173, -0.9290361201559172, 2.799184934345601,
                                0.6790194052589797, 0.7333821627861707, -0.03291277242681545, 1.310438143604587,
                                0.673832562222562, -0.6404225489719699, -0.3685222338703895, 6.988973505496276,
                                0, 0, 0, 0.999999999999998;
                */
                /*
              transformation << 0.9998996846969838, 0.003948215234314986, -0.01360265192291004, 0.05847011404293689,
                              -0.004032877285312574, 0.9999726343121815, -0.006202138950136233, 0.04528938486109094,
                                0.01357779229749574, 0.006256374606648019, 0.9998882444218992, 0.02203456132723125,
                                0, 0, 0, 1;
                */
              iSAM.addLoopConstraint(logReader.timestamp, matchTime, transformation);//, cov);
              fout3<<transformation(0,0)<<" "<<transformation(0,1)<<" "<<transformation(0,2)<<" "<<transformation(0,3)<<" "<<transformation(1,0)<<" "<<transformation(1,1)<<" "<<transformation(1,2)<<" "<<transformation(1,3)<<" "<<transformation(2,0)<<" "<<transformation(2,1)<<" "<<transformation(2,2)<<" "<<transformation(2,3)<<" "<<transformation(3,0)<<" "<<transformation(3,1)<<" "<<transformation(3,2)<<" "<<transformation(3,3)<<endl;
              loopClosureCount++;


            }

        }

        if(loopClosureCount>=1)
        {
            break;
        }
    }
    /*
    for(int i=0; i<loopClosureCount;i++)
    {

     iSAM.addLoopConstraint(placeRecognition.loopClosureConstraints.at(i)->time1,
                            placeRecognition.loopClosureConstraints.at(i)->time2,
                            placeRecognition.loopClosureConstraints.at(i)->constraint);

    }*/

    std::vector<std::pair<uint64_t, Eigen::Matrix4f> > posesBefore;
    iSAM.getCameraPoses(posesBefore);

    cout<<"It works good before optimization"<<endl;

//    #4
    double residual =iSAM.optimise();

    cout<<"It works good after optimize and before map.applyPoses"<<endl;

   // map.applyPoses(isam);
    //cout<<"It works good before *cloud=map.getMap and after map.applyPoses(isam)"<<endl;

    /*
    pcl::PointCloud<pcl::PointXYZRGB> *cloud = map.getMap();


     // Write it back to disk under a different name.
	// Another possibility would be "savePCDFileBinary()".
	cout<<"before storing the point cloud map"<<endl;
	pcl::io::savePCDFileASCII ("outputCloudMap03DVODensity005.pcd", *cloud);

    cout << "Saved data points to outputMap.pcd." << std::endl;


    cout<<"copy data into octomap..."<<endl;

    octomap::ColorOcTree tree( 0.05 );

    for (size_t i=0; i<(*cloud).points.size(); i++)
    {
        // 将点云里的点插入到octomap中
        tree.updateNode( octomap::point3d((*cloud).points[i].x, (*cloud).points[i].y, (*cloud).points[i].z), true );
    }

    for (size_t i=0; i<(*cloud).points.size(); i++)
    {
        tree.integrateNodeColor( (*cloud).points[i].x, (*cloud).points[i].y, (*cloud).points[i].z, (*cloud).points[i].r, (*cloud).points[i].g, (*cloud).points[i].b);
    }

    tree.updateInnerOccupancy();
    tree.write("OctomapColorLab03DVODensity005.ot");
    cout<<"please see the done."<<endl;
   */

    //pcl::visualization::PCLVisualizer cloudViewer;

   // cloudViewer.setBackgroundColor(1, 1, 1);
    //cloudViewer.initCameraParameters();
   // cloudViewer.addCoordinateSystem(0.1, 0, 0, 0);

    //pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> color(cloud->makeShared());
    //cloudViewer.addPointCloud<pcl::PointXYZRGB>(cloud->makeShared(), color, "Cloud Viewer");

    std::vector<std::pair<uint64_t, Eigen::Matrix4f> > newPoseGraph;

    iSAM.getCameraPoses(newPoseGraph);


    /*
    for(unsigned int i = 0; i < newPoseGraph.size(); i++)
    {
       // file << std::setprecision(6) << std::fixed << (double)newPoseGraph.at(i).first / 1000000.0 << " ";

        Eigen::Vector3f trans = newPoseGraph.at(i).second.topRightCorner(3, 1);
        Eigen::Matrix3f rot = newPoseGraph.at(i).second.topLeftCorner(3, 3);

        fout4 << trans(0) << " " << trans(1) << " " << trans(2) << " ";

        Eigen::Quaternionf currentCameraRotation(rot);

        //file << currentCameraRotation.x() << " " << currentCameraRotation.y() << " " << currentCameraRotation.z() << " " << currentCameraRotation.w() << "\n";
    }*/



    for(std::vector<std::pair<uint64_t, Eigen::Matrix4f> >::iterator ite=newPoseGraph.begin(); ite!=newPoseGraph.end(); ite++)
    {
        Eigen::Matrix3f Roptimized;
        Roptimized<<ite->second(0,0), ite->second(0,1), ite->second(0,2),
                    ite->second(1,0), ite->second(1,1), ite->second(1,2),
                    ite->second(2,0), ite->second(2,1), ite->second(2,2);

         Eigen::Quaternionf quatOptimized(Roptimized);

         fout4<<ite->second(0,3)<<" "<<ite->second(1,3)<<" "<<ite->second(2,3)<<" "<<quatOptimized.w()<<" "<<quatOptimized.x()<<" "<<quatOptimized.y()<<" "<<quatOptimized.z()<<endl;

    }

    cout<<"The number of optimized poses"<<newPoseGraph.size()<<endl;


   // drawPoses(poses, cloudViewer, 1.0, 0, 0);
    //drawPoses(posesBefore, cloudViewer, 0, 0, 1.0);







    //cloudViewer.spin();

    delete [] decompressionBuffer;

    return 0;
}

