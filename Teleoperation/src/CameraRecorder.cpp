#include "include/CameraRecorder.h"

CameraRecorder::CameraRecorder(int cam1, int cam2 = -1, int fps = 10, std::string folderName = "test", bool saveVideo = false) 
/**
 * @brief Construct a new Camera Recorder:: Camera Recorder object
 * 
 * @param cam1 camera port of the first camera
 * @param cam2 camera port of the second camera (disabled by default = -1)
 * @param fps frames per second
 * @param folderName folder name to save the images and videos
 * @param saveVideo flag to save a video of all the frames once recording is done
 */
    : camIndex1(cam1), camIndex2(cam2), fps(fps), timeForEachFrame(1.0 / fps) {
    videoFolder = "videos";
    imageFolder = "images";
    fps = fps;
    this->folderName = folderName;
    this->saveVideo = saveVideo;
}

CameraRecorder::~CameraRecorder() {
    cap1.release();
    if (camIndex2 != -1){
        cap2.release();
    }
    cv::destroyAllWindows();
}

void CameraRecorder::init() {
    cap1.open(camIndex1);
    if (camIndex2 != -1){
        cap2.open(camIndex2);
    }
    if (!cap1.isOpened() || (camIndex2 != -1 && !cap2.isOpened())) {
        throw std::runtime_error("Error: Unable to open the cameras.");
    }

    // create video and image subfolder in folderName
    if (this->saveVideo) {
        videoFolder = folderName + "/" + videoFolder;
        std::filesystem::create_directory(videoFolder);

        auto fourcc = cv::VideoWriter::fourcc('X', 'V', 'I', 'D');
        int frameWidth = static_cast<int>(cap1.get(cv::CAP_PROP_FRAME_WIDTH));
        int frameHeight = static_cast<int>(cap1.get(cv::CAP_PROP_FRAME_HEIGHT));
        video1 = cv::VideoWriter(videoFolder + "/output1.avi",fourcc, fps, cv::Size(frameWidth, frameHeight));

        if (camIndex2 != -1) {
            frameWidth = static_cast<int>(cap2.get(cv::CAP_PROP_FRAME_WIDTH));
            frameHeight = static_cast<int>(cap2.get(cv::CAP_PROP_FRAME_HEIGHT));
            video2 = cv::VideoWriter(videoFolder + "/output2.avi",fourcc, fps, cv::Size(frameWidth, frameHeight));
        }

    }
    imageFolder = folderName + "/" + imageFolder;
    std::filesystem::create_directory(imageFolder);
}

void CameraRecorder::recordFrame(int index, bool saveImg = true) {
    cv::Mat frame1, frame2;
    cap1 >> frame1;
    if (camIndex2 != -1){
        cap2 >> frame2;
    }

    if (frame1.empty() || (camIndex2 != -1 && frame2.empty())) {
        throw std::runtime_error("Error: Empty frame captured.");
    }

    cv::imshow("frame1", frame1);
    frames1.push_back(frame1);

    if (saveImg) {
        cv::imwrite(imageFolder + "/cam1_" + std::to_string(index) + ".jpg", frame1);
        if (camIndex2 != -1) {
            cv::imshow("frame2", frame2);
            frames2.push_back(frame2);

            cv::imwrite(imageFolder + "/cam2_" + std::to_string(index) + ".jpg", frame2);
        }
    }
}

void CameraRecorder::finalize() {
    // write the frames to video
    if (this->saveVideo) {
        for (size_t i = 0; i < frames1.size(); ++i) {
            video1.write(frames1[i]);
        }
        video1.release();

        if (camIndex2 != -1) {
            for (size_t i = 0; i < frames2.size(); ++i) {
                video2.write(frames2[i]);
            }
            video2.release();
        }
    }

    // delete cap and destroy windows
    cap1.release();
    if (camIndex2 != -1){
        cap2.release();
    }
    cv::destroyAllWindows();
}
