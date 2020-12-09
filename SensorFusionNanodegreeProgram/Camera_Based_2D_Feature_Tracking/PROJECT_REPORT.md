# 2D Feature Tracking Midterm for Sensor Fusion Nanodegree Program

Base of the repo has been taken from: https://github.com/udacity/SFND_2D_Feature_Tracking.git


In this project the purpose is:
- Setting a datastructure to create a stable/robust framework to successfully track features without stressing memory bandwith of the system (MP.1)
- Setting a feature tracking framework to successfully track features from consecutive images
- Using different keypoint and descriptor detectors to compare their performance against each other for urban driving environment (MP.2, MP.3, MP.4 )
- Using different matching algorithm with different configuration to utilize 2d feature tracking app for urban driving environment (MP.5, MP.6)
- Quantative Analysis of different descriptors, keypoint detectors and matcher (MP.7, MP.8, MP.9)

## MP.1 - Data Buffer
In this task a circular buffer has been implemented to handle data infrastracture in the app:
```cpp
// MP.1 Circular buffer 
template <typename DataType> 
struct circularBuffer{
    std::vector<DataType> data_buffer_;
    size_t max_buffer_size;
    size_t head;
    size_t tail;
    size_t contents_size;
    typedef typename std::vector<DataType>::iterator iterator;
    circularBuffer(size_t size_ = 2) /* Default value */
    : max_buffer_size(size_), 
      head(0), tail(0), 
      contents_size(0)
    {} 
      
    void clear(){
        head = tail = contents_size = 0;
    }
        
    
    iterator element_indexed_from_last(size_t index=0){
        if(index > max_buffer_size)
            index = index % max_buffer_size;
        if(index == 0)
            return (data_buffer_.begin() + tail);
        else{
            if(tail < index)
                return (data_buffer_.begin() + tail + max_buffer_size - index);
            return (data_buffer_.begin() + tail -index);
        }
    }
    iterator start()  {
        return (data_buffer_.begin()+head);
    }
       
    bool is_empty(){
        // Check if buffer is empty or not 
        if(contents_size == 0)
            return true;
        return false;
    }
    // Check condition of tail and head
    // Change related other parameter to the increment
    void increment_tail(){
        ++tail;
        ++contents_size;
        if(tail == max_buffer_size) tail = 0; /* Return to the beginning if tail reached to max number of element*/
    }
    
    void increment_head(){
        if(is_empty()){
            std::cout << "Buffer is empty head is not incremented\n";
        } else{
            ++head;   
            --contents_size;
            if (head == max_buffer_size) head = 0;
        }
    }
    
    void push_back(const DataType &item){
        if(is_empty()){
            data_buffer_.push_back(item);
            tail = head;
            ++contents_size;    
            
        } 
        else if(contents_size != max_buffer_size){ // If we did not reach end of buffer yet
            increment_tail();
            data_buffer_.push_back(item);
        }
        else { // Lose front and add new element 
            increment_head();
            increment_tail();
            data_buffer_.at(tail) = item;
        }
    }
    // Just in case if it will be needed
    void pop_front(){
        increment_head();
    }
    
    size_t size(){
        return contents_size;
    }
};
```

To access any element in the circular buffer **element_indexed_from_last()** can be used. That function will return a iterator.
```cpp
    circularBuffer<DataFrame> cbf;
    cbf.element_indexed_from_last()->keypoints;
    cbf.element_indexed_from_last()->cameraImg;
    cbf.element_indexed_from_last()->descriptors;
    cbf.element_indexed_from_last()->kptMatches;
```

## MP.2 - Keypoint Detection
In this task, different keypoint detectors have been implemented to be able to compare them in the upcoming tasks. The detector type has been decided by the string called **detectorType**. List of implemented detectors are:

- HARRIS
- FAST
```cpp
cv::Ptr<cv::FastFeatureDetector> fast_detector = cv::FastFeatureDetector::create();
double t = (double)cv::getTickCount();
fast_detector->detect(img,keypoints,cv::Mat());
t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
std::cout << "FAST detection with n= " << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << "\n";
```
- BRISK
```cpp
cv::Ptr<cv::FeatureDetector> brisk_detector = cv::BRISK::create();
double t = (double)cv::getTickCount();
brisk_detector->detect( img,keypoints );
t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
std::cout << "BRISK detection with n= " << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << "\n";
```
- ORB
```cpp
cv::Ptr<cv::ORB> orbDetector = cv::ORB::create();
double t = (double)cv::getTickCount();
orbDetector->detect( img, keypoints );
t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
cout << "ORB detector with n= " << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;
```
- AKAZE
```cpp
cv::Ptr<cv::AKAZE> akazeDetector = cv::AKAZE::create();
double t = (double)cv::getTickCount();
akazeDetector->detect( img, keypoints );
t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
cout << "AKAZE detector with n= " << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;
```
- SIFT
```cpp
// Number of maximum keypoint to extract
int minHessian = 3000;
cv::Ptr<cv::SIFT> siftDetector = cv::SIFT::create(minHessian);
double t = (double)cv::getTickCount();
siftDetector->detect( img, keypoints );
t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
cout << "SIFT detector with n= " << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;
```

Almost all of them(except **HARRIS**) has been implemented by initialising corresponding **FeatureDetector** pointer, and corresponding built-in function.

As we discussed in the course before, different detectors extract different amount of keypoint in different amount of time. 

The keypoint and elapsed time will be compared later. 

## MP.3 - Keypoint Removal (Defining Region Of Interest)

When keypoints are generated no masking has been used with the detector(Look at any documentation related to detector, one can pass **mask** as an input to the detector and detector will take care of ROI). So, as a next step the keypoints that are out of ROI should be removed. Following code snippet take care of that little task:

```cpp
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
```

## MP.4 (Keypoint Descriptor Extractor)
In this task, different descriptor detectors have been implemented to be able to compare them in the upcoming tasks. The detector type has been decided by the string called **descriptorType**. List of implemented detectors are:
- BRIEF
```cpp
    extractor = cv::xfeatures2d::BriefDescriptorExtractor::create();
```
- SIFT
```cpp
    extractor = cv::SIFT::create();
```
- AKAZE
```cpp
    extractor = cv::AKAZE::create();
```
- ORB
```cpp
    extractor = cv::ORB::create();
```
- FREAK
```cpp
   extractor = cv::xfeatures2d::FREAK::create();
```

All all of them has been implemented by initialising corresponding **DescriptorExtractor** pointer, and corresponding built-in function.

The keypoint and elapsed time will be compared later. 

## MP.5 (Descriptor Matching)
In this task, one more(FLANN) descriptor matching has been implemented and pair matching has been extended with additional method which is KNN selection. 

Additional info from Udacity notes: 

> As FLANN-based matching entails a whole new body of knowledge with several concepts that have limited relevance for this course, there is no detailed description of the method given here. The FLANN-based matching is available in the OpenCV and you will see it again in the code example below. At the time of writing (May 2019), there is a potential bug in the current implementation of the OpenCV, which requires a conversion of the binary descriptors into floating point vectors, which is inefficient. Yet still there is an improvement in speed, albeit not as large as it potentially could be.
Both BFMatching and FLANN accept a descriptor distance threshold T which is used to limit the number of matches to the ‘good’ ones and discard matches where the respective pairs are no correspondences. 
> 

- FLANN
```cpp
if (descSource.type() != CV_32F || descRef.type() != CV_32F)
{ // OpenCV bug workaround : convert binary descriptors to floating point due to a bug in current OpenCV implementation
    descSource.convertTo(descSource, CV_32F);
    descRef.convertTo(descRef, CV_32F);
}
matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
```
- KNN
```cpp
std::vector< std::vector<cv::DMatch> > knn_matches;
// Number of neighbour 
int nn = 2;
// Apply Knn match
double t = (double)cv::getTickCount(); 
matcher->knnMatch( descSource, descRef, knn_matches, nn);
```

## MP.6 (Descriptor Distance Ratio)
Use the K-Nearest-Neighbor matching to implement the descriptor distance ratio test, which looks at the ratio of best vs. second-best match to decide whether to keep an associated pair of keypoints.

In this task, the KNN implementation has been extended with descriptor distance ratio test to make the matching more resilient against false positives. Briefly, in this task distance ratio of best and second-best match have been compared to decide whether to keep an associated pair of keypoints.

```cpp
const float ratio_thresh = 0.8f;
for (size_t i = 0; i < knn_matches.size(); i++)
{
    if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
    {
        matches.push_back(knn_matches[i][0]);
    }
}
```

## MP.7 (Performance Evaluation 1)

Table 1: Mean of Number of keypoint, elapsed time and size of neighborhood for sequential ten frames for given keypoint detector

|           | Number of Keypoint | Elapsed Time(ms) | Size of Neighborhood Keypoint |
|-----------|--------------------|------------------|-------------------------------|
| HARRIS    |         25         |       10.59      |               6.0               |
| SHITOMASI |         118        |       7.46       |            4                  |
| BRISK     |        276.2       |       31.82      |             21.94             |
| FAST      |         409        |      1.5612      |               7               |
| ORB       |         116        |       8.28       |             56.06             |
| AKAZE     |         167        |       47.25      |              7.69             |
| SIFT      |         139        |       63.52      |              5.03             |



## MP.8 (Performance Evaluation 2 )

For the rest of the tasks different keypoint/descriptor detectors have been tested with different combintation to find which detector pair work best for our use case. Same matching algorithm (**BRUTE_FORCE**) with same configuration (**KNN with 0.8 descriptor distance ratio**) have been used to have no bias from the matching algorithm. 

**NOTE:** Some of the modalities could not be tested because of the bugs on opencv or data format conflict between keypoint/descriptor detectors. 
- AKAZE keypoint detector works only with AKAZE descriptor detector
- SIFT keypoint detector does not work with ORB descriptor detector


```cpp
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
```

The results can be regenerated once the application has been built. The command to generate the values:

```bash
$> ./2D_feature_tracking_table_generator
```

Table 2: Mean on Number of matches for ten sequential frames for given keypoint/descriptor detector 

|  #Matches |  BRIEF |   ORB  |  FREAK |  AKAZE |  SIFT  | BRISK |
|:---------:|:------:|:------:|:------:|:------:|:------:|:------:
| HARRIS    |  15.67 |  15.89 | 15.037 |    X   |  15.80 |    9   | 
| SHITOMASI |  90.67 |  87.83 |  79.85 |    X   |  85.64 |  46.11 |
| BRISK     | 149.33 | 125.39 | 123.96 |    X   | 138.69 |  85.67 |
| FAST      |   242  | 234.83 | 214.70 |    X   | 238.31 | 117.78 |
| ORB       |   50   |  53.61 |  48.67 |    X   |  57.69 | 49.56  | 
| AKAZE     |    X   |    X   |    X   | 130.22 |    X   |   X    |
| SIFT      |  66.33 |    X   |  60.94 |    X   |  70.26 |   50   |


## MP.9 (Performance Evaluation 3)

Elapsed time for descriptor detector for given keypoint detector method. The tables generated in the last 3 tasks will help us to come up TOP 3 keypoint/descriptor detector candidate for our use case which is 2D Feature Tracking and later Collision Avoidance system.
 
Table 3: Mean of Elapsed time for ten sequential frames for given keypoint/descriptor matches

|  time(ms) | BRIEF |  ORB | FREAK | AKAZE |  SIFT | BRISK |
|:---------:|:-----:|:----:|:-----:|:-----:|:-----:|:------:
| HARRIS    |  0.41 | 7.82 |  6.91 |   X   |  7.59 |  0.74 |
| SHITOMASI |  0.86 | 1.78 |  7.88 |   X   |  8.45 |  1.86 |
| BRISK     |  0.71 | 3.55 |  9.18 |   X   | 12.56 |  2.58 |
| FAST      |  1.03 | 1.78 |  7.29 |   X   |  7.80 |  3.05 |
| ORB       |  0.48 | 3.38 |  7.32 |   X   | 10.27 |  1.56 |
| AKAZE     |   X   |   X  |   X   | 37.86 |   X   |   X   |
| SIFT      |  1.28 |   X  | 11.14 |   X   | 21.27 |  1.71 |

As one can see from Table 1-3 different metrics/information about different keypoint/descriptor detectors have been shared with the reader. Based on the generated metrics following pairs have been chosen as top 3 candidate for given use case:

(Fast & High amount of matching pairs)

- FAST & BRIEF ( **#Matches**: 242 / **Elapsed Time**: 1.03 / **Neighborhood size**: 7)
- BRISK & BRIEF ( **#Matches**: 149.33 / **Elapsed Time**: 0.71 / **Neighborhood size**: 7)
- FAST & ORB  ( **#Matches**: 234.83 / **Elapsed Time**: 1.78 / **Neighborhood size**: 7)

(According to the literatur)
- SIFT & SIFT 
- AKAZE & AKAZE (Alternative to the SIFT because their patent restriction)
- ORB & FREAK 

In general **FAST** keypoint detector was faster and generating way more keypoints than the other detectors (Feature Detection with Automatic Scale Selection - the name acronym makes perfect sense). However, one has to remember that FAST is fast but also a greedy detector which means most of the keypoints could be false positive. That's why I applied more conservative descriptor distance ratio test to compare the results. Eventually, number of matches was still more than rest of the detector pairs for **FAST & BRIEF**.
- FAST & BRIEF ( **#Matches**: 242 / **Elapsed Time**: 1.03 / **Neighborhood size**: 7 / **Distance Threshold**: 0.6)


## Build Instructions

```bash
$> nano CMakeLists.txt (if Opencv built from source, specifcy the PATHS explicitly)
$> mkdir build && cd build
$> cmake ..
$> make
$> ./2D_feature_tracking
```

## BUILDING NOTE 

If **opencv**>4.3.0 add change namespace for **SIFT** detector:

- **opencv** < 4.3.0
```cpp
cv::Ptr<cv::xfeatures2d::SIFT> siftDetector = cv::xfeatures2d::SIFT::create(minHessian);
```

- **opencv** >= 4.3.0
```cpp
cv::Ptr<cv::SIFT> siftDetector = cv::SIFT::create(minHessian);
```
