#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string COLOR_OPENCV_WINDOW = "Color Image";
static const std::string DEPTH_OPENCV_WINDOW = "Depth Image";

int offset = 50;
int window_size_x = 20;
int window_size_y = 100;

void Color_Image_Callback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_color_ptr;
    cv::namedWindow(COLOR_OPENCV_WINDOW);
    try
    {
        cv_color_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Image processing
    int rows = cv_color_ptr->image.rows;
    int cols = cv_color_ptr->image.cols;
    
    cv::Point pt1(offset, rows/2 - 50);
    cv::Point pt2(offset + window_size_x, rows/2 - 50 + window_size_y);
    cv::rectangle(cv_color_ptr->image, pt1, pt2, cv::Scalar(255, 0, 0), 3);

    cv::Point pt3((cols-window_size_x)/2, rows/2 - 50);
    cv::Point pt4((cols-window_size_x)/2 + window_size_x, rows/2 - 50 + window_size_y);
    cv::rectangle(cv_color_ptr->image, pt3, pt4, cv::Scalar(255, 0, 0), 3);

    cv::Point pt5(cols-window_size_x-offset, rows/2 - 50);
    cv::Point pt6(cols-window_size_x-offset + window_size_x, rows/2 - 50 + window_size_y);
    cv::rectangle(cv_color_ptr->image, pt5, pt6, cv::Scalar(255, 0, 0), 3);
    // Update GUI Window
    cv::imshow(COLOR_OPENCV_WINDOW, cv_color_ptr->image);
    cv::waitKey(3);
}

unsigned short Depth_Calculation(cv_bridge::CvImagePtr cv_depth_ptr, int left_upper_x, int left_upper_y, int window_size_x, int window_size_y)
{
    std::vector<unsigned short> depth_vector;
    int count = 0;
    for(int i = left_upper_y;i < left_upper_y + window_size_y; i++)
    {
        for(int j = left_upper_x;j < left_upper_x + window_size_x; j++)
        {
            unsigned short depth = cv_depth_ptr->image.at<unsigned short>(i, j);//you can change 240,320 to your interested pixel
            if(depth != 0)
            {
                count++;
                depth_vector.push_back(depth);
            }
        }
    }

    if(depth_vector.size() != 0){
        std::sort(depth_vector.begin(), depth_vector.end());
        return depth_vector[count/2];
    }
    else{
        return 0;
    }
}

void Depth_Image_Callback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_depth_ptr;
    
    cv::namedWindow(DEPTH_OPENCV_WINDOW);
    try
    {
        cv_depth_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);//now cv_ptr is the matrix, do not forget "TYPE_" before "16UC1"
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    int rows = cv_depth_ptr->image.rows;
    int cols = cv_depth_ptr->image.cols;
    
    // Print out the depth information
    unsigned short left_depth = Depth_Calculation(cv_depth_ptr, offset, rows/2 - 50, window_size_x, window_size_y);
    unsigned short center_depth = Depth_Calculation(cv_depth_ptr, (cols-window_size_x)/2, rows/2 - 50, window_size_x, window_size_y);
    unsigned short right_depth = Depth_Calculation(cv_depth_ptr, cols-window_size_x-offset, rows/2 - 50, window_size_x, window_size_y);
    ROS_INFO("Left: %u, Center: %u, Right: %u", left_depth, center_depth, right_depth);

    // Update GUI Window
    cv::imshow(DEPTH_OPENCV_WINDOW, cv_depth_ptr->image);
    cv::waitKey(3);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_converter");
    // ImageConverter ic;
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);

    // Create a color image subscriber
    image_transport::Subscriber color_image_sub;

    // Subscribe to color input video feed and publish output video feed
    color_image_sub = it.subscribe("/camera/color/image_raw", 1, Color_Image_Callback);

    
    // Create a depth image subscriber
    image_transport::Subscriber depth_image_sub;

    // Subscribe to depth input video feed
    depth_image_sub = it.subscribe("/camera/depth/image_rect_raw", 1, Depth_Image_Callback);



    ros::spin();
    return 0;
}
