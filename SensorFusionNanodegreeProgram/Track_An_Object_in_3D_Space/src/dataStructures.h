
#ifndef dataStructures_h
#define dataStructures_h

#include <vector>
#include <map>
#include <opencv2/core.hpp>

struct LidarPoint { // single lidar point in space
    double x,y,z,r; // x,y,z in [m], r is point reflectivity
};

struct BoundingBox { // bounding box around a classified object (contains both 2D and 3D data)
    
    int boxID; // unique identifier for this bounding box
    int trackID; // unique identifier for the track to which this bounding box belongs
    
    cv::Rect roi; // 2D region-of-interest in image coordinates
    int classID; // ID based on class file provided to YOLO framework
    double confidence; // classification trust

    std::vector<LidarPoint> lidarPoints; // Lidar 3D points which project into 2D image roi
    std::vector<cv::KeyPoint> keypoints; // keypoints enclosed by 2D roi
    std::vector<cv::DMatch> kptMatches; // keypoint matches enclosed by 2D roi
};

struct DataFrame { // represents the available sensor information at the same time instance
    
    cv::Mat cameraImg; // camera image
    
    std::vector<cv::KeyPoint> keypoints; // 2D keypoints within camera image
    cv::Mat descriptors; // keypoint descriptors
    std::vector<cv::DMatch> kptMatches; // keypoint matches between previous and current frame
    std::vector<LidarPoint> lidarPoints;

    std::vector<BoundingBox> boundingBoxes; // ROI around detected objects in 2D image coordinates
    std::map<int,int> bbMatches; // bounding box matches between previous and current frame
};

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
    
    //~circularBuffer {clear();}
    
    void clear(){
        head = 0;
        tail = 0;
        contents_size = 0;
        data_buffer_.clear();
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

#endif /* dataStructures_h */