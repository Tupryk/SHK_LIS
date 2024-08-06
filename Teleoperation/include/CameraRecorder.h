#include <opencv2/opencv.hpp>
#include <opencv2/opencv_modules.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <vector>
#include <filesystem>
#include <chrono>
#include <thread>

class CameraRecorder {
public:
    CameraRecorder(int cam1, int cam2, int fps, std::string folderName, bool saveVideo);
    ~CameraRecorder();

    void init();
    void recordFrame(int index, bool saveImg);
    void finalize();

private:
    int camIndex1;
    int camIndex2;
    int fps;
    bool saveVideo;
    std::string folderName;
    double timeForEachFrame;
    cv::VideoCapture cap1;
    cv::VideoCapture cap2;
    std::vector<cv::Mat> frames1;
    std::vector<cv::Mat> frames2;
    cv::VideoWriter video1;
    cv::VideoWriter video2;
    cv::VideoWriter outFrame;
    std::string videoFolder;
    std::string imageFolder;
};
