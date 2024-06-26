#include <chrono>
#include <iostream>
#include <OptiTrack/optitrack.h>
#include <Core/graph.h>
#include <KOMO/komo.h>
#include <BotOp/bot.h>
#include <Optim/NLP_Solver.h>
#include <Gamepad/gamepad.h>

#define USE_BOTH_ARMS 1
#define TRANSLATION_SCALE 2


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
    #if USE_BOTH_ARMS
        bot.gripperMove(rai::_right, .079);
    #endif
    arr qHome = C.getJointState();
    arr last_komo = bot.get_q();
    arr q_dot_ref = bot.get_qDot();

    // Mocap
    std::cout << "Initializing mocap...";
    rai::OptiTrack OT;
    OT.pull(C);
    std::cout << "Done.\n";

    // Tracking data
    TrackingData l_arm;
    l_arm.controller_frame = "l_gamepad";
    l_arm.target_frame = "l_gripper_target";
    C.addFrame("l_gripper_target")->setShape(rai::ST_marker, {.2});
    #if USE_BOTH_ARMS
        TrackingData r_arm;
        r_arm.controller_frame = "r_gamepad";
        r_arm.target_frame = "r_gripper_target";
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

        if (is_move_button_pressed(&G, 0)
                #if USE_BOTH_ARMS
                    || is_move_button_pressed(&G, 1)
                #endif
        ) {
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
        else if (G.getButtonPressed(0)==BTN_Y
                    #if USE_BOTH_ARMS
                        || G.getButtonPressed(1)==BTN_Y
                    #endif
        ) {
            bot.home(C);
            last_komo = bot.get_q();
        }
        else
        {
            bot.stop(C);
            reset_data(&l_arm, "l_gripper", &C);
            #if USE_BOTH_ARMS
                reset_data(&r_arm, "r_gripper", &C);
            #endif
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
    return 0;
}
