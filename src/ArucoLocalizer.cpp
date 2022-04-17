#include "ArucoLocalizer.h"
#include <stdio.h>
#include <stdlib.h>
#include <algorithm>

ArucoLocalizer* ArucoLocalizer::instance = nullptr;
ArucoLocalizer::ArucoLocalizer(){
    arucoPublisher = nodeH.advertise<aruco_localization::aruco_msg>(ros::this_node::getName() + "/" + topicName, 10);
}
ArucoLocalizer *ArucoLocalizer::GetInstance(){
    if(instance == nullptr) instance = new ArucoLocalizer();
    return instance;
}

cv::Point3f ArucoLocalizer::PointToGlobal(cv::Mat &color, cv::Mat &depth, cv::Point2f &point){
    float z =  depth.at<uint16_t>(point.y    , point.x    ) + 
               depth.at<uint16_t>(point.y + 1, point.x    ) +
               depth.at<uint16_t>(point.y    , point.x + 1) +
               depth.at<uint16_t>(point.y - 1, point.x    ) +
               depth.at<uint16_t>(point.y    , point.x - 1) ;
    z/=5000.0;
    
    return cv::Point3f(z*(point.x - cx)/fx, 
                       z*(point.y - cy)/fy,
                          z);
}


void ArucoLocalizer::DetectMarkers(cv::Mat &image, std::vector<std::vector<cv::Point2f>> &resultBuffer, std::vector<int> &idsBuffer){
    resultBuffer.clear();
    idsBuffer.clear();
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

    cv::aruco::detectMarkers(image, dictionary, resultBuffer, idsBuffer);
    // if at least one marker detected
    
    
}

void ArucoLocalizer::LocalizeMarkers(cv::Mat &colorIm, cv::Mat &depthIm, std::vector<std::vector<cv::Point3f>> &buffer, std::vector<int> &idsBuffer){
    buffer.clear();
    idsBuffer.clear();
    std::vector<std::vector<cv::Point2f>> point2dBuffer;
    DetectMarkers(colorIm, point2dBuffer , idsBuffer);
    buffer.resize(point2dBuffer.size());
    for(int i =0; i< idsBuffer.size(); i++)
        for(int j = 0; j < point2dBuffer[i].size(); j++){
            cv::Point3f point = PointToGlobal(colorIm, depthIm, point2dBuffer[i][j]);
            buffer[i].push_back(point);
        }
}

void ArucoLocalizer::DrawMarkersWithCords(cv::Mat &colorIm, std::vector<std::vector<cv::Point3f>> cords){
    cv::Mat imgCopy;
    colorIm.copyTo(imgCopy);
    std::vector<std::vector<cv::Point2f>> buffer;
    std::vector<int> ids;
    DetectMarkers(colorIm, buffer, ids);
    if (ids.size() > 0){
        cv::aruco::drawDetectedMarkers(imgCopy, buffer, ids);
    }
    for(int i = 0; i < ids.size(); i++){
        char *buf = new char[std::snprintf(nullptr, 0, "%f, %f, %f", cords[i][0].x, cords[i][0].y, cords[i][0].z)];
        std::sprintf(buf, "%.4f, %.4f, %.4f", cords[i][0].x, cords[i][0].y, cords[i][0].z);
        std::string s_cord = buf;
        delete [] buf;
        cv::putText(imgCopy, 
                    s_cord, 
                    buffer[i][0], 
                    cv::FONT_HERSHEY_DUPLEX,
                    0.5,
                    cv::Scalar(0,0,0),
                    2,
                    false);
    }

    cv::imshow("out", imgCopy);
    cv::waitKey(3);
}


void ArucoLocalizer::PublishPoints(std::vector<int> &idsBuffer, std::vector<std::vector<cv::Point3f>> cords){
    auto rosTime = ros::Time::now();
    for (int i = 0; i < idsBuffer.size(); i++){
        aruco_localization::aruco_msg msg;
        msg.timeTag.data = rosTime;
        msg.arucoId = idsBuffer[i];
        msg.points.resize(cords[i].size());
        for(int j =0; j < cords[i].size(); j++){
            msg.points[j].x = cords[i][j].x;
            msg.points[j].y = cords[i][j].y;
            msg.points[j].z = cords[i][j].z;
        }
        arucoPublisher.publish(msg);
    }
}

