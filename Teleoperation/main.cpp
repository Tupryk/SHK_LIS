#include <chrono>
#include <iostream>
#include <OptiTrack/optitrack.h>
#include <Core/graph.h>
#include <KOMO/komo.h>
#include <BotOp/bot.h>
#include <Optim/NLP_Solver.h>
#include <Gamepad/gamepad.h>
#include <include/CameraRecorder.h>

#define USE_BOTH_ARMS 0 // USING BOTH ARMS IS STILL NOT WORKING PERFECTLY, USE AT OWN RISK!!!
#define TRANSLATION_SCALE 1


struct TrackingData {
    bool gripper_closed = false;
    std::string controller_frame;
    arr controller_origin;
    arr target_origin;
    rai::Quaternion rotation_offset;
    std::string target_frame;
};

void reload_target(rai::Configuration* C, TrackingData arm, bool keep_pos=false) {
    // Update position
    arr controller_pos;
    if (keep_pos) {
        controller_pos = arr{0., 0., 0.};
    } else {
        controller_pos = C->getFrame(arm.controller_frame.c_str())->getPosition() - arm.controller_origin;
    }
    arr target_pos = arm.target_origin + controller_pos*TRANSLATION_SCALE;
    C->getFrame(arm.target_frame.c_str())->setPosition(target_pos);

    // Update rotation
    rai::Quaternion controller_quat(C->getFrame(arm.controller_frame.c_str())->getQuaternion());
    controller_quat.append(arm.rotation_offset);
    C->getFrame(arm.target_frame.c_str())->setQuaternion(controller_quat.getArr4d());
}

void update_gripper(bool* gripper_closed, bool button_pressed, rai::ArgWord which_gripper, BotOp* bot)
{
    if (button_pressed) {
        if (*gripper_closed && bot->gripperDone(which_gripper)) {
            bot->gripperMove(which_gripper, .079);
            *gripper_closed = false;
        }
        else if (!*gripper_closed && bot->gripperDone(which_gripper)) {
            bot->gripperClose(which_gripper);
            *gripper_closed = true;
        }
    }
}

void reload_target_based_on_input(rai::Configuration* C, int controller_id, TrackingData* arm, const char* endeffector, GamepadInterface* G)
{
    if (G->getButtonPressed(controller_id)==BTN_B)
    {
        // Keep current translation
        reload_target(C, *arm, true);
        return;
    }
    else if (G->getButtonPressed(controller_id)==BTN_X)
    {
        // Keep current rotation
        rai::Quaternion controller_quat(C->getFrame(arm->controller_frame.c_str())->getQuaternion());
        arm->rotation_offset = controller_quat.invert();
        rai::Quaternion initial_gripper_rot(C->getFrame(endeffector)->getQuaternion());
        arm->rotation_offset.append(initial_gripper_rot);
    }
    reload_target(C, *arm);
}

bool is_move_button_pressed(GamepadInterface* G, int controller_id) {
    return (G->getButtonPressed(controller_id)==BTN_A ||
            G->getButtonPressed(controller_id)==BTN_B ||
            G->getButtonPressed(controller_id)==BTN_X);
}

void reset_data(TrackingData* arm, const char* endeffector, rai::Configuration* C)
{
    arm->controller_origin = C->getFrame(arm->controller_frame.c_str())->getPosition();
    arm->target_origin = C->getFrame(endeffector)->getPosition(); 
    rai::Quaternion controller_quat(C->getFrame(arm->controller_frame.c_str())->getQuaternion());
    arm->rotation_offset = controller_quat.invert();
    rai::Quaternion initial_gripper_rot(C->getFrame(endeffector)->getQuaternion());
    arm->rotation_offset.append(initial_gripper_rot);
}

bool joint_target_dangerous(arr target, arr current, float limit)
{
    #if USE_BOTH_ARMS
        int joint_count = 14;
    #else
        int joint_count = 7;
    #endif
    for (int i = 0; i < joint_count; i++) {
        if (abs(target.elem(i)-current.elem(i)) >= limit) {
            return true;
        }
    }
    return false;
}

int main(int argc,char **argv)
{
    // Paul
    auto CAMERA_PORT_1 = 4; 
    auto CAMERA_PORT_2 = 10; //disabled: -1
    bool SAVE_VIDEO = true;
    const int FPS = 20; // Desired frequency in frames per second
    //const double tau = 1./FPS; // Desired time between frames
    // end Paul


    // Config
    rai::Configuration C;
    #if USE_BOTH_ARMS
        C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandasTable.g"));
    #else
        C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandaSingle.g"));
    #endif
    C.view(false);

    // Bot
    BotOp bot(C, true);
    bot.home(C);
    bot.gripperMove(rai::_left, .079);
    #if USE_BOTH_ARMS
        bot.gripperMove(rai::_right, .079);
    #endif
    arr qHome = C.getJointState();
    arr last_komo = bot.get_q();
    arr q_dot_ref = bot.get_qDot();

    // PAUL
    // INITIALIZE RECORDING STUFF
    // create folder with timestamp name
    auto timestamp = std::chrono::system_clock::now();
    std::string folder_name = "recordings_" + std::to_string(std::chrono::duration_cast<std::chrono::seconds>(timestamp.time_since_epoch()).count());
    std::filesystem::create_directory(folder_name);

    //init camera
    CameraRecorder recorder(CAMERA_PORT_1, CAMERA_PORT_2, FPS, folder_name, SAVE_VIDEO); // camera port, second camera port, FPS
    recorder.init();
    auto frameNumber = 1;

    // file to write poses to
    std::ofstream poses_file;
    poses_file.open(folder_name + "/poses.txt");

    // file to write proprioceptives to
    std::ofstream proprioceptive_file;
    proprioceptive_file.open(folder_name + "/proprioceptive.txt");
    
    bot.gripperClose(rai::_left);
    // END PAUL

    // Mocap
    std::cout << "Initializing mocap...";
    rai::OptiTrack OT;
    OT.pull(C);
    std::cout << "Done.\n";

    // Tracking data
    TrackingData l_arm;
    l_arm.controller_frame = "l_gamepad";
    l_arm.target_frame = "l_gripper_target";
    C.addFrame("l_gripper_target")->setShape(rai::ST_marker, {.1})
        .setPosition(C.getFrame("l_gripper")->getPosition());
    #if USE_BOTH_ARMS
        TrackingData r_arm;
        r_arm.controller_frame = "r_gamepad";
        r_arm.target_frame = "r_gripper_target";
        C.addFrame("r_gripper_target")->setShape(rai::ST_marker, {.1})
            .setPosition(C.getFrame("r_gripper")->getPosition());
    #endif
    
    // Gamepad
    GamepadInterface G;
    if (G.count == 0) {
        std::cout << "No controller found! Shutting down..." << std::endl;
        return -1;
    }
    std::cout << "Waiting for Gamepad thread...";
    G.gamepadState[0].waitForNextRevision();
    #if USE_BOTH_ARMS
        if (G.count < 2) {
            std::cout << "A controller is missing for usage of both arms!" << std::endl;
            return -1;
        }
        G.gamepadState[1].waitForNextRevision();
    #endif
    std::cout << "Done.\n";

    // Timing
    const double delta = .2; // Timestep size for spline generation
    const double tau = .05; // Desired frequency in Hz
    const std::chrono::milliseconds interval(static_cast<int>(1000.0 * tau));

    // Safety
    bool move_button_on = false;
    const std::chrono::milliseconds time_until_move(100);
    auto move_last_pressed = std::chrono::steady_clock::now();

    while(1)
    {
        OT.pull(C);
        if(C.view(false)=='q' || G.quitSignal.get()) break;

        auto loop_start = std::chrono::steady_clock::now();
        
        // immediatly after recording the frame, we log the EE pose
        recorder.recordFrame(0, false);
        if (move_button_on){
            recorder.recordFrame(frameNumber++, true);
            arr l_gripper_pos = C.getFrame("l_gripper")->getPosition();
            arr l_gripper_quat = C.getFrame("l_gripper")->getQuaternion();
            arr l_joint_state = C.getJointState();
            arr l_tau_external = bot.get_tauExternal();
            bool l_is_gripper_open = (bot.gripperPos(rai::_left) > .005 ? true : false);

            poses_file << l_gripper_pos << " " << l_gripper_quat << std::endl;
            proprioceptive_file << l_joint_state << " " << l_tau_external <<  " " << l_is_gripper_open  << std::endl;
        }

        if (is_move_button_pressed(&G, 0)
                #if USE_BOTH_ARMS
                    || is_move_button_pressed(&G, 1)
                #endif
        ) {
            if (!move_button_on) {
                move_button_on = true;
                move_last_pressed = std::chrono::steady_clock::now();
            }
            
            auto now = std::chrono::steady_clock::now();
            std::chrono::duration<double, std::milli> elapsed_time = now - move_last_pressed;
            auto time_til_move = time_until_move - std::chrono::duration_cast<std::chrono::milliseconds>(elapsed_time);
            if (time_til_move.count() > 0) {
                reset_data(&l_arm, "l_gripper", &C);
                #if USE_BOTH_ARMS
                    reset_data(&r_arm, "r_gripper", &C);
                #endif
            }

            KOMO komo(C, 1., 1, 2, true);
            
            komo.setConfig(C, true);
            komo.setTiming(1, 1, 1, 0);
            komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1e1});
            komo.addObjective({}, FS_jointLimits, {}, OT_ineq, {1e1});

            komo.addObjective({}, FS_qItself, {}, OT_sos, {1e-1}, qHome);            
            komo.addObjective({}, FS_qItself, {}, OT_sos, {1e0}, last_komo);            

            komo.addObjective({1.}, FS_positionDiff, {"l_gripper", "l_gripper_target"}, OT_eq, {1e1});
            komo.addObjective({1.}, FS_quaternionDiff, {"l_gripper", "l_gripper_target"}, OT_eq, {1e1});

            #if USE_BOTH_ARMS
                komo.addObjective({1.}, FS_positionDiff, {"r_gripper", "r_gripper_target"}, OT_eq, {1e1});
                komo.addObjective({1.}, FS_quaternionDiff, {"r_gripper", "r_gripper_target"}, OT_eq, {1e1});
            #endif

            auto ret = NLP_Solver(komo.nlp(), 0).solve();
            arr q = komo.getPath_qOrg();

            if (joint_target_dangerous(q[0], bot.get_q(), .4)) {
                // Safety measure: Don't move if the target possition is too far away
                bot.stop(C);
                reset_data(&l_arm, "l_gripper", &C);
                #if USE_BOTH_ARMS
                    reset_data(&r_arm, "r_gripper", &C);
                #endif
                std::cout << "Safety measure activated! Robot did not follow target.\n";
            }
            else {
                arr q_dot_ref = (q[0]-last_komo)/tau;

                arr q_atdelta = q[0] + delta*q_dot_ref;
                arr q_at2delta = q[0] + 2.*delta*q_dot_ref;  
                arr q_at10delta = q[0] + 5.*delta*q_dot_ref;  
                
                double overwrite_time = bot.get_t();
                bot.move((q_atdelta, q_at2delta, q_at10delta).reshape(-1, q[0].N), {delta, 2.*delta, 10.*delta}, true, overwrite_time);
                bot.move((q_atdelta, q_at2delta).reshape(-1, q[0].N), {1.5*delta, 4.*delta}, true, overwrite_time);
                bot.move(q_at2delta.reshape(-1, q[0].N), {4.*delta}, true, overwrite_time);
                last_komo = q[0];
                bot.sync(C, 0);
            }
        }
        else if (G.getButtonPressed(0)==BTN_Y
                    #if USE_BOTH_ARMS
                        || G.getButtonPressed(1)==BTN_Y
                    #endif
        ) {
            bot.home(C);
            last_komo = bot.get_q();
            move_button_on = false;
        }
        else
        {
            bot.stop(C);
            reset_data(&l_arm, "l_gripper", &C);
            #if USE_BOTH_ARMS
                reset_data(&r_arm, "r_gripper", &C);
            #endif
            move_button_on = false;
        }

         // Reset translation and rotation offsets for the target frame if no movement command

        reload_target_based_on_input(&C, 0, &l_arm, "l_gripper", &G);
        #if USE_BOTH_ARMS
            reload_target_based_on_input(&C, 1, &r_arm, "r_gripper", &G);
        #endif

        update_gripper(&l_arm.gripper_closed, G.getButtonPressed(0)==BTN_R, rai::_left, &bot);
        #if USE_BOTH_ARMS
            update_gripper(&r_arm.gripper_closed, G.getButtonPressed(1)==BTN_R, rai::_right, &bot);
        #endif

        auto loop_end = std::chrono::steady_clock::now();
        std::chrono::duration<double, std::milli> elapsed_time = loop_end - loop_start;
        auto sleep_duration = interval - std::chrono::duration_cast<std::chrono::milliseconds>(elapsed_time);
        if (sleep_duration.count() > 0) {
            std::this_thread::sleep_for(sleep_duration);
        }
    }


    // finalize recording
    recorder.finalize();
    poses_file.close();

    return 0;
}
