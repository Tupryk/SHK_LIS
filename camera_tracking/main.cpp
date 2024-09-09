#include <iostream>
#include <OptiTrack/optitrack.h>
#include <Core/graph.h>
#include <BotOp/bot.h>
#include <include/CameraRecorder.h>


int main(int argc,char **argv)
{
    // Paul
    auto CAMERA_PORT_1 = 4; 
    auto CAMERA_PORT_2 = 10; //disabled: -1
    bool SAVE_VIDEO = true;
    const int FPS = 20; // Desired frequency in frames per second
    //const double tau = 1./FPS; // Desired time between frames
    // end Paul

    //init camera
    CameraRecorder recorder(CAMERA_PORT_1, CAMERA_PORT_2, FPS, folder_name, SAVE_VIDEO); // camera port, second camera port, FPS
    recorder.init();
    auto frameNumber = 1;

    // Config
    rai::Configuration C;
    C.view(false);

    // Bot
    BotOp bot(C, true);

    // Mocap
    rai::OptiTrack OT;
    OT.pull(C);
    const char* to_follow = "l_gamepad";
    arr l_controller_origin;
    arr l_target_origin;
    rai::Quaternion l_rotation_offset;
    auto camera_tracker = C.getFrame("camera_tracker");
    auto camera_tracker_prev = C.addFrame("camera_tracker_prev")->setShape(rai::ST_marker, {.01}).setPosition({0, 0, 0});
    arr previous_pos = camera_tracker->getPosition();
    arr current_pos = camera_tracker->getPosition();

    // Folder creation
    auto timestamp = std::chrono::system_clock::now();
    std::string folder_name = "camera_" + std::to_string(std::chrono::duration_cast<std::chrono::seconds>(timestamp.time_since_epoch()).count());
    std::filesystem::create_directory(folder_name);
    std::ofstream poses_file;
    poses_file.open(folder_name + "/poses.txt");

    while(1)
    {
        OT.pull(C);
        if(C.view(false)=='q' || G.quitSignal.get()) break;
        
        recorder.recordFrame(0, false);
        bool should_record = -C.eval(FS_negDistance, {"camera_tracker", "camera_tracker_prev"}).elem(0) > .01;
        if (should_record) {
            arr camera_tracker_prev.setPosition(camera_tracker->getPosition());
            // Store state info
            recorder.recordFrame(frameNumber++, true);
            poses_file << camera_tracker->getPosition(); << " " << camera_tracker->getQuaternion(); << std::endl;
        }
    }
    return 0;
}
