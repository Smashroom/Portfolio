#include <numeric>
#include "matching2D.hpp"

using namespace std;

// Find best matches for keypoints in two camera images based on several matching methods
void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef,
                      std::vector<cv::DMatch> &matches, std::string descriptorType, std::string matcherType, std::string selectorType, double *time)
{
    // configure matcher
    bool crossCheck = false;
    cv::Ptr<cv::DescriptorMatcher> matcher;
    
    // Common norm type selector if DES_BINARY or DES_HOG have been chosen
    int normType = descriptorType.compare("DES_BINARY") == 0 ? cv::NORM_HAMMING : cv::NORM_L2;
    if( descriptorType.compare("DES_HOG")){
        descSource.convertTo(descSource, CV_32F);
        descRef.convertTo(descRef, CV_32F);

    }
    if (matcherType.compare("MAT_BF") == 0)
    {
        matcher = cv::BFMatcher::create(normType, crossCheck);
    }
    else if (matcherType.compare("MAT_FLANN") == 0)
    {
        if (descSource.type() != CV_32F || descRef.type() != CV_32F)
        { // OpenCV bug workaround : convert binary descriptors to floating point due to a bug in current OpenCV implementation
            descSource.convertTo(descSource, CV_32F);
            descRef.convertTo(descRef, CV_32F);
        }
        matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
    }

    // perform matching task
    if (selectorType.compare("SEL_NN") == 0)
    { // nearest neighbor (best match)

        double t = (double)cv::getTickCount();
        matcher->match(descSource, descRef, matches); // Finds the best match for each descriptor in desc1
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        t = 1000 * t / 1.0;
        *time = t;

        std::cout << matcherType << " and (NN) with n=" << matches.size() << " matches in " << t << " ms" << endl;
    }
    else if (selectorType.compare("SEL_KNN") == 0)
    { // k nearest neighbors (k=2)
        std::vector< std::vector<cv::DMatch> > knn_matches;
        // Number of neighbour 
        int nn = 2;
        // Apply Knn match
        double t = (double)cv::getTickCount(); 

        matcher->knnMatch( descSource, descRef, knn_matches, nn);
        // Descriptor Distance Ratio for more robust pair matching 
        const float ratio_thresh = 0.6f;
        for (size_t i = 0; i < knn_matches.size(); i++)
        {
            if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
            {
                matches.push_back(knn_matches[i][0]);
            }
        }
        
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        t = 1000 * t / 1.0; // to convert to ms 
        *time = t;

        std::cout << matcherType << " and (KNN) with n=" << matches.size() << " matches in " << t << " ms" << endl;
    }
}

// Use one of several types of state-of-art descriptors to uniquely identify keypoints
void descKeypoints(vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, string descriptorType, double *time)
{
    // select appropriate descriptor
    cv::Ptr<cv::DescriptorExtractor> extractor;
    if (descriptorType.compare("BRISK") == 0)
    {

        int threshold = 30;        // FAST/AGAST detection threshold score.
        int octaves = 3;           // detection octaves (use 0 to do single scale)
        float patternScale = 1.0f; // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.

        extractor = cv::BRISK::create(threshold, octaves, patternScale);
    }
    else if (descriptorType.compare("SIFT") == 0){
        extractor = cv::SIFT::create();
    }
    else if (descriptorType.compare("FREAK") == 0){
        extractor = cv::xfeatures2d::FREAK::create();
    }
    else if (descriptorType.compare("ORB") == 0){
        extractor = cv::ORB::create(500,1.2f,3);
    }
    else if (descriptorType.compare("AKAZE") == 0){
       extractor = cv::AKAZE::create();
    }
    else if (descriptorType.compare("BRIEF") == 0){
       extractor = cv::xfeatures2d::BriefDescriptorExtractor::create();
    }

    // perform feature description
    double t = (double)cv::getTickCount();
    std::cout << "Image size: " << img.size() << "\n";
    extractor->compute(img, keypoints, descriptors);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    t = 1000 * t/ 1.0; // Convert to ms
    cout << descriptorType << " descriptor extraction in " << t<< " ms" << endl;
    *time = t;
}

// Detect keypoints in image using the traditional Shi-Thomasi detector
void detKeypointsShiTomasi(vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis, double *time)
{
    // compute detector parameters based on image size
    int blockSize = 4;       //  size of an average block for computing a derivative covariation matrix over each pixel neighborhood
    double maxOverlap = 0.0; // max. permissible overlap between two features in %
    double minDistance = (1.0 - maxOverlap) * blockSize;
    int maxCorners = img.rows * img.cols / max(1.0, minDistance); // max. num. of keypoints

    double qualityLevel = 0.01; // minimal accepted quality of image corners
    double k = 0.04;

    // Apply corner detection
    double t = (double)cv::getTickCount();
    vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(img, corners, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, false, k);

    // add corners to result vector
    for (auto it = corners.begin(); it != corners.end(); ++it)
    {

        cv::KeyPoint newKeyPoint;
        newKeyPoint.pt = cv::Point2f((*it).x, (*it).y);
        newKeyPoint.size = blockSize;
        keypoints.push_back(newKeyPoint);
    }
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    t = 1000 * t/ 1.0; // Convert to ms
    cout << "Shi-Tomasi detection with n=" << keypoints.size() << " keypoints in " << t << " ms" << endl;
    *time = t;
    
    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "Shi-Tomasi Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
}

void detKeypointsHarris(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis, double *time){
    // Detector parameters
    int blockSize = 2;     // for every pixel, a blockSize × blockSize neighborhood is considered
    int apertureSize = 3;  // aperture parameter for Sobel operator (must be odd)
    int minResponse = 100; // minimum value for a corner in the 8bit scaled response matrix
    double k = 0.04;       // Harris parameter (see equation for details)
    
    // Detect Harris corners and normalize output
    cv::Mat dst, dst_norm, dst_norm_scaled;
    dst = cv::Mat::zeros(img.size(), CV_32FC1);
    double t = (double)cv::getTickCount();

    cv::cornerHarris(img, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT);
    cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
    cv::convertScaleAbs(dst_norm, dst_norm_scaled);
    
    // Non-maxima supression enabled
    bool* wasOverlap = new bool;
    for(size_t i = 0 ; i < dst_norm.rows ; i++){
        for(size_t j = 0 ; j<dst_norm.cols ; j++){
            auto response = (int) dst_norm.at<float>(i,j);
            if (response > minResponse){ 
                // Check whether the pixel is a good candidate to become a keypoint or not
                cv::KeyPoint tempKp;
                tempKp.pt = cv::Point2f(j,i);
                // Taken from the solution 
                tempKp.size = 2*apertureSize;
                tempKp.response = response;
                
                // Check if any keypoint overlap with each other
                *wasOverlap = false;
                for(auto kPit = keypoints.begin(); kPit != keypoints.end(); kPit++){
                    double kpOverlapRatio = cv::KeyPoint::overlap(tempKp, *kPit);
                    // If there is overlap, compare their response value
                    if(kpOverlapRatio > 0.0){
                        *wasOverlap = true;
                        if(tempKp.response > (*kPit).response){
                            *kPit = tempKp;
                            break;
                        }
                    }
                }
                if(!(*wasOverlap))
                    keypoints.push_back(tempKp);
            }
        }
    }
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    t = 1000 * t/ 1.0; // Convert to ms    
    cout << "Harris detection with n=" << keypoints.size() << " keypoints in " << t << " ms" << endl;
    *time = t;
    
    // visualize results
    if(bVis) {
        // visualize results
        string windowName = "Harris Corner Detection Results";
        cv::namedWindow(windowName, 5);
        cv::Mat visImage = dst_norm_scaled.clone();
        cv::drawKeypoints(dst_norm_scaled, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        cv::imshow(windowName, visImage);
        cv::waitKey(0);
    }
}

void detKeypointsModern(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, std::string detectorType, bool bVis, double *time){
    //// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType
        //// -> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT
    if(detectorType.compare("FAST") == 0){
        cv::Ptr<cv::FastFeatureDetector> fast_detector = cv::FastFeatureDetector::create();
        double t = (double)cv::getTickCount();
        fast_detector->detect(img,keypoints,cv::Mat());
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        std::cout << "FAST detection with n= " << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << "\n";
        t = 1000 * (t) / 1.0; // Convert to ms
        *time = t;
    }
    else if(detectorType.compare("BRISK") == 0){
        cv::Ptr<cv::FeatureDetector> brisk_detector = cv::BRISK::create();
        double t = (double)cv::getTickCount();
        brisk_detector->detect( img,keypoints );
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        std::cout << "BRISK detection with n= " << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << "\n";
        t = 1000 * (t) / 1.0; // Convert to ms
        *time = t;
    } 
    else if(detectorType.compare("SIFT") == 0){
        // Number of maximum keypoint to extract
        int minHessian = 3000;
        cv::Ptr<cv::SIFT> siftDetector = cv::SIFT::create(minHessian);
        double t = (double)cv::getTickCount();
        siftDetector->detect( img, keypoints );
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        cout << "SIFT detector with n= " << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;
        t = 1000 * (t) / 1.0; // Convert to ms
        *time = t;
    }
    else if(detectorType.compare("ORB") == 0){
        cv::Ptr<cv::ORB> orbDetector = cv::ORB::create();
        double t = (double)cv::getTickCount();
        orbDetector->detect( img, keypoints );
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        cout << "ORB detector with n= " << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;
        t = 1000 * (t) / 1.0; // Convert to ms
        *time = t;
    }
    else if(detectorType.compare("AKAZE") == 0){
        cv::Ptr<cv::AKAZE> akazeDetector = cv::AKAZE::create();
        double t = (double)cv::getTickCount();
        akazeDetector->detect( img, keypoints );
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        cout << "AKAZE detector with n= " << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;
        t = 1000 * (t) / 1.0; // Convert to ms
        *time = t;
    }
    
    

    
    // visualize results
    if(bVis) {
        // visualize results
        string windowName = detectorType + " Detection Results";
        cv::namedWindow(windowName, 5);
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        cv::imshow(windowName, visImage);
        cv::waitKey(0);
    }
}
