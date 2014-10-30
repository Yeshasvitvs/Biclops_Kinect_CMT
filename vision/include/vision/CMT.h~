#ifndef CMT_H
#define CMT_H

#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <iostream>


class CMT
{
public:
    std::string detectorType;
    std::string descriptorType;
    std::string matcherType;
    int descriptorLength;
    int thrOutlier;//Outlier Threshold
    float thrConf;
    float thrRatio;//Thresholding Ration

    bool estimateScale;
    bool estimateRotation;

    cv::Ptr<cv::FeatureDetector> detector;//pointer to detector
    cv::Ptr<cv::DescriptorExtractor> descriptorExtractor;//pointer to descriptors
    cv::Ptr<cv::DescriptorMatcher> descriptorMatcher;//pointer to descriptor matching

    cv::Mat selectedFeatures;
    std::vector<int> selectedClasses;
    cv::Mat featuresDatabase;
    std::vector<int> classesDatabase;

    std::vector<std::vector<float> > squareForm;
    std::vector<std::vector<float> > angles;

    //New global Variables
    cv::Point2f CENTER;
    float SCALE,ROTATION;

	
    //the points c1....c4 used for the initial user given box
    cv::Point2f topLeft;
    cv::Point2f topRight;
    cv::Point2f bottomRight;
    cv::Point2f bottomLeft;

    cv::Rect_<float> boundingbox;
    bool hasResult;

    //New boundingbox corners found  using the center given from voting schema
    cv::Point2f centerToTopLeft;
    cv::Point2f centerToTopRight;
    cv::Point2f centerToBottomRight;
    cv::Point2f centerToBottomLeft;

    std::vector<cv::Point2f> springs;

    cv::Mat im_prev;
    std::vector<std::pair<cv::KeyPoint,int> > activeKeypoints;
    std::vector<std::pair<cv::KeyPoint,int> > trackedKeypoints;

    unsigned int nbInitialKeypoints;

    std::vector<cv::Point2f> votes;

    std::vector<std::pair<cv::KeyPoint, int> > outliers;

    CMT();//Constructor
    void initialise(cv::Mat im_gray0, cv::Point2f topleft, cv::Point2f bottomright);//initialise using the initial bounding box values
    void estimate(const std::vector<std::pair<cv::KeyPoint, int> >& keypointsIN, cv::Point2f& center, float& scaleEstimate, float& medRot, std::vector<std::pair<cv::KeyPoint, int> >& keypoints);
    void processFrame(cv::Mat im_gray);
};

class Cluster
{
public:
    int first, second;//cluster id
    float dist;
    int num;
};

void inout_rect(const std::vector<cv::KeyPoint>& keypoints, cv::Point2f topleft, cv::Point2f bottomright, std::vector<cv::KeyPoint>& in, std::vector<cv::KeyPoint>& out);

void track(cv::Mat im_prev, cv::Mat im_gray, const std::vector<std::pair<cv::KeyPoint, int> >& keypointsIN, std::vector<std::pair<cv::KeyPoint, int> >& keypointsTracked, std::vector<unsigned char>& status, int THR_FB = 20);//Threshold value 20 giving best results
cv::Point2f rotate(cv::Point2f p, float rad);

#endif // CMT_H
