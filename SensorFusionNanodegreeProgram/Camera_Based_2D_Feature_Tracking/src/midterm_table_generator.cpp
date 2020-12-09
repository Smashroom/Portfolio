/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <numeric>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"


using namespace std;

/* MAIN PROGRAM */
int main(int argc, const char *argv[])
{
    /* INIT VARIABLES AND DATA STRUCTURES */

    // data location
    string dataPath = "../";

    // camera
    string imgBasePath = dataPath + "images/";
    string imgPrefix = "KITTI/2011_09_26/image_00/data/000000"; // left camera, color
    string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 9;   // last file index to load
    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

    
    std::vector<string> detectorTypeList = {"SHITOMASI", "HARRIS", "BRISK", "FAST", "ORB", "AKAZE", "SIFT"};
    std::vector<string> decriptorTypeList = {"BRISK"}; //, "BRIEF" , "ORB", "FREAK" , "AKAZE" , "SIFT"};
    
    bool bVis = false;
    
    // misc
    int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time
    // Initialise a circular buffer from boost library (because it is efficient)
    circularBuffer<DataFrame> dataBuffer(dataBufferSize);
    
    
    for( auto detectorType: detectorTypeList){
            // Mean of neighborhood size
            std::vector<float> keypointNeighborhoodSize;
            // Mean of number of keypoints
            std::vector<float> keypointsSize;
            // Mean of spent time
            std::vector<double> keypointsElapsedTime;
            // Mean of matched points
            std::vector<float> matchesSize;
            // Mean of spent time
            std::vector<double> descriptorsElapsedTime;
               
            
            
        for (auto descriptorType: decriptorTypeList){
           
            // Special cases
            if((detectorType.compare("AKAZE") == 0)){
                if (!(descriptorType.compare("AKAZE") == 0)){
                    continue;
                }
            } else {
               if (descriptorType.compare("AKAZE") == 0) {
                   continue;
               }
            }
            
           
            if(((descriptorType.compare("ORB") == 0) && (detectorType.compare("SIFT") == 0))){
                // https://github.com/opencv/opencv/issues/17353 
                // Could be solved by editing orb implementation slightly 
                continue;   
            }
           
            // Clear the buffer before executing anything
            dataBuffer.clear();
            
            for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++)
            {
                   
                /* LOAD IMAGE INTO BUFFER */

                // assemble filenames for current index
                ostringstream imgNumber;
                imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
                string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

                // load image from file and convert to grayscale
                cv::Mat img, imgGray;
                img = cv::imread(imgFullFilename);
                cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

                //// STUDENT ASSIGNMENT
                //// TASK MP.1 -> replace the following code with ring buffer of size dataBufferSize

                // push image into data frame buffer
                DataFrame frame;
                frame.cameraImg = imgGray;
                dataBuffer.push_back(frame);
            
                //// EOF STUDENT ASSIGNMENT
         //       cout << "#1 : LOAD IMAGE INTO BUFFER done" << endl;

                /* DETECT IMAGE KEYPOINTS */

                // extract 2D keypoints from current image
                vector<cv::KeyPoint> keypoints; // create empty feature list for current image
                //string detectorType = "BRISK";

                //// STUDENT ASSIGNMENT
                //// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType
                //// -> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT
                double timeElapsedKeypoints=0;
                if (detectorType.compare("SHITOMASI") == 0)
                {
                    detKeypointsShiTomasi(keypoints, dataBuffer.element_indexed_from_last()->cameraImg, false, &timeElapsedKeypoints);
                }
                else if(detectorType.compare("HARRIS") == 0)
                {
                    detKeypointsHarris(keypoints, dataBuffer.element_indexed_from_last()->cameraImg, false, &timeElapsedKeypoints);
                }
                else
                {
                    detKeypointsModern(keypoints, dataBuffer.element_indexed_from_last()->cameraImg, detectorType, false, &timeElapsedKeypoints);
                }
                //// EOF STUDENT ASSIGNMENT

                //// STUDENT ASSIGNMENT
                //// TASK MP.3 -> only keep keypoints on the preceding vehicle
                
                // only keep keypoints on the preceding vehicle
                bool bFocusOnVehicle = true;
                cv::Rect vehicleRect(535, 180, 180, 150);

                if (bFocusOnVehicle)
                {
                    //Apply keypoint mask
                    auto kPit = keypoints.begin();
                    while(kPit != keypoints.end()){
                        // Check whether the point is in the predefined rectangle or not
                        if(vehicleRect.contains(kPit->pt)){
                            kPit++;
                            continue;   
                        }
                        kPit = keypoints.erase(kPit);
                    }
                }
                
                //// EOF STUDENT ASSIGNMENT
                // To extract information from the keypoints after ROI masking
                // Mean of neighborhood size
                float kpNeighborhoodSize=0;
                for(auto kPit = keypoints.begin(); kPit != keypoints.end(); kPit++){
                    kpNeighborhoodSize += kPit->size;
                }
                kpNeighborhoodSize = kpNeighborhoodSize/keypoints.size();
                keypointNeighborhoodSize.push_back(kpNeighborhoodSize);
                keypointsSize.push_back(keypoints.size());
                keypointsElapsedTime.push_back(timeElapsedKeypoints);
                
                // optional : limit number of keypoints (helpful for debugging and learning)
                bool bLimitKpts = false;
                if (bLimitKpts)
                {
                    int maxKeypoints = 50;

                    if (detectorType.compare("SHITOMASI") == 0)
                    { // there is no response info, so keep the first 50 as they are sorted in descending quality order
                        keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
                    }
                    cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
                    cout << " NOTE: Keypoints have been limited!" << endl;
                }
        //        std::cout << "Number of keypoints used: " << keypoints.size() << "\n";
                dataBuffer.element_indexed_from_last()->keypoints = keypoints;
                // push keypoints and descriptor for current frame to end of data buffer
         //       cout << "#2 : DETECT KEYPOINTS done" << endl;

                /* EXTRACT KEYPOINT DESCRIPTORS */

                //// STUDENT ASSIGNMENT
                //// TASK MP.4 -> add the following descriptors in file matching2D.cpp and enable string-based selection based on descriptorType
                //// -> BRIEF, ORB, FREAK, AKAZE, SIFT
                
                cv::Mat descriptors;
                //string descriptorType = "BRISK"; // BRIEF, ORB, FREAK, AKAZE, SIFT
                double timeElapsedDescriptor;
                
 
                descKeypoints(dataBuffer.element_indexed_from_last()->keypoints, 
                      dataBuffer.element_indexed_from_last()->cameraImg, 
                      descriptors, descriptorType, &timeElapsedDescriptor);
               
                descriptorsElapsedTime.push_back(timeElapsedDescriptor);
                //// EOF STUDENT ASSIGNMENT

                // push descriptors for current frame to end of data buffer
                dataBuffer.element_indexed_from_last()->descriptors = descriptors;

              //  cout << "#3 : EXTRACT DESCRIPTORS done" << endl;

                if (dataBuffer.size() > 1) // wait until at least two images have been processed
                {
                    /* MATCH KEYPOINT DESCRIPTORS */
                    vector<cv::DMatch> matches;
                    string matcherType = "MAT_BF";        // MAT_BF, MAT_FLANN
                    if((descriptorType.compare("SIFT") == 0) || (descriptorType.compare("FREAK") == 0)) {
                        std::cout << "L2 has been chosen instead of hamming distance \n";   
                        string descriptorType = "DES_HOG"; // DES_BINARY, DES_HOG
                    }
                    else{
                        string descriptorType = "DES_BINARY"; // DES_BINARY, DES_HOG    
                    }
                   
                    string selectorType = "SEL_KNN";       // SEL_NN, SEL_KNN

                    //// STUDENT ASSIGNMENT
                    //// TASK MP.5 -> add FLANN matching in file matching2D.cpp
                    //// TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp
                    double timeElapsedMatch;

                    matchDescriptors(dataBuffer.element_indexed_from_last(1)->keypoints, dataBuffer.element_indexed_from_last()->keypoints,
                             dataBuffer.element_indexed_from_last(1)->descriptors, dataBuffer.element_indexed_from_last()->descriptors,
                             matches, descriptorType, matcherType, selectorType, &timeElapsedMatch);
                    
                    matchesSize.push_back(matches.size());
                    //// EOF STUDENT ASSIGNMENT

                    // store matches in current data frame
                    dataBuffer.element_indexed_from_last()->kptMatches = matches;
                    
         //           cout << "#4 : MATCH KEYPOINT DESCRIPTORS done" << endl;

                    // visualize matches between current and previous image
                    bVis = false;
                    if (bVis)
                    {
                        cv::Mat matchImg = (dataBuffer.element_indexed_from_last()->cameraImg).clone();
                        cv::drawMatches(dataBuffer.element_indexed_from_last(1)->cameraImg, dataBuffer.element_indexed_from_last(1)->keypoints,
                                        dataBuffer.element_indexed_from_last()->cameraImg, dataBuffer.element_indexed_from_last()->keypoints,
                                        matches, matchImg,
                                        cv::Scalar::all(-1), cv::Scalar::all(-1),
                                        vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

                        string windowName = "Matching keypoints between two camera images";
                        cv::namedWindow(windowName, 7);
                        cv::imshow(windowName, matchImg);
                        cout << "Press key to continue to next image" << endl;
                        std::cout << " " <<  endl;
                        cv::waitKey(0); // wait for key to be pressed
                    }
                    bVis = false;
                }
                
            } // eof loop over all images
            std::cout << "############################### -- RESULTS -- ############################### \n";
            std::cout << detectorType << " Results \n";
            std::cout << descriptorType << " Results\n";
            std::cout << "Mean of Neighborhood size: " << std::accumulate( keypointNeighborhoodSize.begin(), keypointNeighborhoodSize.end(), 0.0)/keypointNeighborhoodSize.size() << "\n";
            std::cout << "Mean of Keypoint size: " << std::accumulate( keypointsSize.begin(), keypointsSize.end(), 0.0)/keypointsSize.size() << "\n";
            std::cout << "Mean of Elapsed Time: " << std::accumulate( keypointsElapsedTime.begin(), keypointsElapsedTime.end(), 0.0)/keypointsElapsedTime.size() << "\n";
            std::cout << "Mean of Elapsed Time for descriptor: " << std::accumulate( descriptorsElapsedTime.begin(), descriptorsElapsedTime.end(), 0.0)/descriptorsElapsedTime.size() << "\n";
            std::cout << "Mean of matched size: " << std::accumulate( matchesSize.begin(), matchesSize.end(), 0.0)/matchesSize.size() << "\n";
            
        } // eof descriptor type
    } // eof keypoint detector type
}