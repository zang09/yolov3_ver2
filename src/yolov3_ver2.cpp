/*******************************************************************************
* Copyright 2018 ROBIT CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/**********************  Maintainer: Hae-Bum Jung  ****************************/

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include "../include/yolov3_ver2/darknetdetector.hpp"

using namespace cv;
using namespace std;

//static const string OPENCV_WINDOW = "Tldeakbot";

int yolo_state;
string sign_value;
int percentage;
Scalar color;

void detect_mat(Mat frame_detect, float* detections_output, int* num_output_class, float thresh, float hier_thresh, vector<cv::Rect>& objBoxes)
{
    double time = what_is_the_time_now();

    // do detect in an unsigned char* data buffer getted from Mat::data or IplImage.imageData
    unsigned char* data = (unsigned char*)frame_detect.data;
    int w = frame_detect.cols;
    int h = frame_detect.rows;
    int c = frame_detect.channels();
    float* detections = test_detector_uchar(data, w, h, c, thresh, hier_thresh, num_output_class);

    objBoxes.clear();

    for(int i = 0; i < *num_output_class; i++)
    {
        yolo_state = (int)detections[i*6+0];
        percentage = (int)(round(detections[i*6+1]*100));
        detections_output[i*6+0] = detections[i*6+0];// ith detection's category
        detections_output[i*6+1] = detections[i*6+1];// ith detection's confidence score
        detections_output[i*6+2] = detections[i*6+2];// ith detection's top-left x-coordinate of bbox
        detections_output[i*6+3] = detections[i*6+3];// ith detection's top-left y-coordinate of bbox
        detections_output[i*6+4] = detections[i*6+4];// ith detection's width of bbox
        detections_output[i*6+5] = detections[i*6+5];// ith detection's height of bbox

        objBoxes.push_back(cv::Rect(detections_output[i*6+2], detections_output[i*6+3], detections_output[i*6+4], detections_output[i*6+5]));

        switch(yolo_state){
        case 0:
            sign_value = "left";
            color = Scalar(0, 255, 0); break;
        case 1:
            sign_value = "right";
            color = Scalar(255, 255, 0); break;
        case 2:
            sign_value = "parking";
            color = Scalar(255, 0, 0); break;
        default:
            break;
        }

        cout << sign_value << ": " << percentage << "%" << endl;
    }
}

void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cam_image;

    try
    {
        cam_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat frame = cam_image->image.clone();

    float* detections = new float(255*6);
    int num_of_outputs = 0;
    double thresh = 0.5;
    double hier_thresh = 0.0;
    std::vector<cv::Rect> results;

    detect_mat(frame, detections, &num_of_outputs, thresh, hier_thresh, results);

    for(int i = 0; i < results.size(); i++)
    {
        cv::rectangle(frame, results[i]	, cv::Scalar(255,0,0), 4);
    }

    delete[]  detections;
    // Update GUI Window
    //cv::imshow(OPENCV_WINDOW, frame);
    cv::waitKey(3);
}

int darknet_simulator(const char* filename)
{
    cv::VideoCapture cap(filename);

    if(!cap.isOpened())
    {
        cout << "\nERROR: Capture could not be opened succesfully" << endl;
        return -1;
    }
    else
        cout << "\nvideo file: " << filename << endl;

    Mat frame;
    cv::namedWindow("darknet_simulator");

    float* detections = (float*)calloc(255*6, sizeof(float));
    int num_of_outputs = 0;
    double thresh = 0.9;
    double hier_thresh = 0.0;
    vector<Rect> results;

    while(1)
    {
        cap >> frame;
        if(frame.empty())
        {
            cout << "\nVideo Over!" << endl;
            break;
        }

        //Detector
        Rect rect(0, 0, 320, 240);
        Mat roi_frame = frame(rect).clone();
        cvtColor(roi_frame,roi_frame, CV_BGR2RGB);

        //Text location
        cv::Point myPoint1;
        myPoint1.x = 10;
        myPoint1.y = 40;

        //cv::putText(frame, sign_value, myPoint1, 4, 0.8, Scalar(255, 255, 255) );

        detect_mat(roi_frame, detections, &num_of_outputs, thresh, hier_thresh, results);


        if(results.empty())
        {
            //cout << "sign : None" << endl;
        }
        else
            //cout << "sign : " << sign_value << endl;

            for(int i = 0; i < results.size(); i++)
            {
                rectangle(frame, results[i], color, 4);
            }

        //File simulator
        imshow("darknet_simulator", frame);
        char c = waitKey(10);
        if(c == 27)
        { cout << "\nExit" << endl; break; }
        if(c == 32)
        {
            cout << "\nPause" << endl;
            while((c = waitKey(10)) != 32 && c != 27);
            if(c == 27)
            { cout << "Exit" << endl; break; }
            else
            { cout << "Restart\n" << endl; }
        }
    }

    free(detections);
    destroyAllWindows();
    return 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "yolov3_darknet_ver2");
    ros::NodeHandle nh_;

    image_transport::ImageTransport it_(nh_);
    image_transport::Subscriber image_sub_;

    detector_init("/home/robit/catkin_ws/src/yolov3_ver2/darkent/yolov3-tiny_obj.cfg",
                  "/home/robit/catkin_ws/src/yolov3_ver2/darkent/yolov3-tiny_obj_10000.weights");

    //image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, &imageCb); //Receive Webcam image

    darknet_simulator("/home/robit/catkin_ws/src/yolov3_ver2/darkent/test_video/drive.avi");  //Receive Video image

    ros::spin();

    detector_uninit();
    return 0;
}
