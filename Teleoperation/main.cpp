#include <chrono>
#include <iostream>
#include <OptiTrack/optitrack.h>
#include <Core/graph.h>
#include <KOMO/komo.h>
#include <BotOp/bot.h>
#include <Optim/NLP_Solver.h>
#include <Gamepad/gamepad.h>

#define USE_BOTH_ARMS 1


void reload_target(rai::Configuration* C, arr target_origin, arr controller_origin, rai::Quaternion rotation_offset, const char* controller, const char* gripper_target) {
    arr controller_pos = C->getFrame(controller)->getPosition() - controller_origin;
    arr target_pos = target_origin + controller_pos*1.5;
    C->getFrame(gripper_target)->setPosition(target_pos);

    rai::Quaternion controller_quat(C->getFrame(controller)->getQuaternion());
    controller_quat.append(rotation_offset);
    C->getFrame(gripper_target)->setQuaternion(controller_quat.getArr4d());
}

void update_gripper(bool* helper_bool, bool button_pressed, rai::ArgWord which_gripper, BotOp* bot) {
    if (button_pressed) {
        if (helper_bool && button_pressed && bot->gripperDone(which_gripper)) {
            bot->gripperMove(which_gripper, .075);
            *helper_bool = false;
            std::cout << "OPEN" << std::endl;
        }
        else if (!helper_bool && button_pressed && bot->gripperDone(which_gripper)) {
            bot->gripperClose(which_gripper);
            *helper_bool = true;
            std::cout << "CLOSE" << std::endl;
        }
        std::cout << "gripper" << bot->gripperDone(which_gripper) << std::endl;
    }
}

void update_robot_pose() {
    
}

int main(int argc,char **argv)
{
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
    bool l_gripper_closed = false;
    #if USE_BOTH_ARMS
        bot.gripperMove(rai::_right, .079);
        bool r_gripper_closed = false;
    #endif
    arr qHome = C.getJointState();
    arr last_komo = bot.get_q();
    arr q_dot_ref = bot.get_qDot();

    // Mocap
    std::cout << "Initializing mocap...";
    rai::OptiTrack OT;
    OT.pull(C);
    std::cout << "Done.\n";
    const char* l_to_follow = "l_gamepad";
    arr l_controller_origin;
    arr l_target_origin;
    rai::Quaternion l_rotation_offset;
    C.addFrame("l_gripper_target")->setShape(rai::ST_marker, {.2});
    #if USE_BOTH_ARMS
        const char* r_to_follow = "r_gamepad";
        arr r_controller_origin;
        arr r_target_origin;
        rai::Quaternion r_rotation_offset;
        C.addFrame("r_gripper_target")->setShape(rai::ST_marker, {.2});
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

    while(1)
    {
        OT.pull(C);
        if(C.view(false)=='q' || G.quitSignal.get()) break;
        
        auto loop_start = std::chrono::steady_clock::now();

        if (G.getButtonPressed(0)==BTN_A ||
            G.getButtonPressed(0)==BTN_B ||
            G.getButtonPressed(0)==BTN_X) {
            KOMO komo(C, 1., 1, 2, true);
            
            komo.setConfig(C, true);
            komo.setTiming(1, 1, 1, 0);
            komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1e1});
            komo.addObjective({}, FS_jointLimits, {}, OT_ineq, {1e1});

            komo.addObjective({}, FS_qItself, {}, OT_sos, {1e-1}, qHome);            
            komo.addObjective({}, FS_qItself, {}, OT_sos, {1e0}, last_komo);            

            komo.addObjective({1.}, FS_positionDiff, {"l_gripper", "l_gripper_target"}, OT_eq, {1e1});
            komo.addObjective({1.}, FS_quaternionDiff, {"l_gripper", "l_gripper_target"}, OT_eq, {1e1});

            auto ret = NLP_Solver()
                .setProblem(komo.nlp())
                .solve();

            arr q = komo.getPath_qOrg();
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
        else if (G.getButtonPressed(0)==BTN_Y || G.getButtonPressed(1)==BTN_Y)
        {
            bot.home(C);
            last_komo = bot.get_q();
        }
        else
        {
            bot.stop(C);
            // Reset translation and rotation offsets for the target frame
            l_controller_origin = C.getFrame(l_to_follow)->getPosition();
            l_target_origin = C.getFrame("l_gripper")->getPosition(); 
            rai::Quaternion l_controller_quat(C.getFrame(l_to_follow)->getQuaternion());
            l_rotation_offset = l_controller_quat.invert();
            rai::Quaternion l_initial_gripper_rot(C.getFrame("l_gripper")->getQuaternion());
            l_rotation_offset.append(l_initial_gripper_rot);
            #if USE_BOTH_ARMS
                r_controller_origin = C.getFrame(r_to_follow)->getPosition();
                r_target_origin = C.getFrame("r_gripper")->getPosition(); 
                rai::Quaternion r_controller_quat(C.getFrame(r_to_follow)->getQuaternion());
                r_rotation_offset = r_controller_quat.invert();
                rai::Quaternion r_initial_gripper_rot(C.getFrame("r_gripper")->getQuaternion());
                r_rotation_offset.append(r_initial_gripper_rot);
            #endif
        }

        if (G.getButtonPressed(0)==BTN_B)
        {
            reload_target(&C, l_target_origin, C.getFrame(l_to_follow)->getPosition(), l_rotation_offset, r_to_follow, "l_gripper_target");
        }
        else if (G.getButtonPressed(0)==BTN_X)
        {
            rai::Quaternion l_controller_quat(C.getFrame(l_to_follow)->getQuaternion());
            l_rotation_offset = l_controller_quat.invert();
            rai::Quaternion l_initial_gripper_rot(C.getFrame("l_gripper")->getQuaternion());
            l_rotation_offset.append(l_initial_gripper_rot);
            reload_target(&C, l_target_origin, l_controller_origin, l_rotation_offset, l_to_follow, "l_gripper_target");
        }
        else
        {
            reload_target(&C, l_target_origin, l_controller_origin, l_rotation_offset, l_to_follow, "l_gripper_target");
        }

        #if USE_BOTH_ARMS
            if (G.getButtonPressed(1)==BTN_B)
            {
                reload_target(&C, r_target_origin, C.getFrame(r_to_follow)->getPosition(), r_rotation_offset, r_to_follow, "r_gripper_target");
            }
            else if (G.getButtonPressed(1)==BTN_X)
            {
                rai::Quaternion r_controller_quat(C.getFrame(r_to_follow)->getQuaternion());
                r_rotation_offset = r_controller_quat.invert();
                rai::Quaternion r_initial_gripper_rot(C.getFrame("r_gripper")->getQuaternion());
                r_rotation_offset.append(r_initial_gripper_rot);
                reload_target(&C, r_target_origin, r_controller_origin, r_rotation_offset, r_to_follow, "r_gripper_target");
            }
            else
            {
                reload_target(&C, r_target_origin, r_controller_origin, r_rotation_offset, r_to_follow, "r_gripper_target");
            }
        #endif

        update_gripper(&l_gripper_closed, G.getButtonPressed(0)==BTN_R, rai::_left, &bot);
        #if USE_BOTH_ARMS
            update_gripper(&r_gripper_closed, G.getButtonPressed(1)==BTN_R, rai::_right, &bot);
        #endif

        auto loop_end = std::chrono::steady_clock::now();
        std::chrono::duration<double, std::milli> elapsed_time = loop_end - loop_start;
        auto sleep_duration = interval - std::chrono::duration_cast<std::chrono::milliseconds>(elapsed_time);
        if (sleep_duration.count() > 0) {
            std::this_thread::sleep_for(sleep_duration);
        }
    }
    return 0;
}
