#include <iostream>
#include <OptiTrack/optitrack.h>
#include <Core/graph.h>
#include <KOMO/komo.h>
#include <BotOp/bot.h>
#include <Gamepad/gamepad.h>
#include <Optim/NLP_Solver.h>
#include <chrono>

#define BOTH_ARMS 0

arr normalize(arr q1) {
    q1 /= sqrt( q1.elem(0)*q1.elem(0) +
                q1.elem(1)*q1.elem(1) +
                q1.elem(2)*q1.elem(2) +
                q1.elem(3)*q1.elem(3));
    return q1;
}

arr conjugate(arr q1) {
    q1.elem(1) *= -1;
    q1.elem(2) *= -1;
    q1.elem(3) *= -1;
    return q1;
}

arr quaternion_mult(arr q1, arr q2) {
    arr result = arr{q1.elem(0) * q2.elem(0) - q1.elem(1) * q2.elem(1) - q1.elem(2) * q2.elem(2) - q1.elem(3) * q2.elem(3),
                     q1.elem(0) * q2.elem(1) + q1.elem(1) * q2.elem(0) + q1.elem(2) * q2.elem(3) - q1.elem(3) * q2.elem(2),
                     q1.elem(0) * q2.elem(2) - q1.elem(1) * q2.elem(3) + q1.elem(2) * q2.elem(0) + q1.elem(3) * q2.elem(1),
                     q1.elem(0) * q2.elem(3) + q1.elem(1) * q2.elem(2) - q1.elem(2) * q2.elem(1) + q1.elem(3) * q2.elem(0)};
    return result;
}

arr quaternion_add(arr q1, arr q2, bool inv=false) {
    q1 /= sqrt( q1.elem(0)*q1.elem(0) +
                q1.elem(1)*q1.elem(1) +
                q1.elem(2)*q1.elem(2) +
                q1.elem(3)*q1.elem(3));
    q2 /= sqrt( q2.elem(0)*q2.elem(0) +
                q2.elem(1)*q2.elem(1) +
                q2.elem(2)*q2.elem(2) +
                q2.elem(3)*q2.elem(3));

    arr q1_conj = q1;
    q1_conj.elem(1) *= -1;
    q1_conj.elem(2) *= -1;
    q1_conj.elem(3) *= -1;

    arr result = q1;
    if (inv) {
        result = quaternion_mult(q1_conj, q2);
        result = quaternion_mult(result, q1);
    } else {
        //   q1**-1 (q2 q1)
        result = quaternion_mult(q1, q2);
        result = quaternion_mult(result, q1_conj);
    }
    
    return result;
}

void reload_target(rai::Configuration* C, arr target_origin, arr controller_origin, arr rotation_offset, const char* controller, const char* gripper_target) {
    // POSITION UPDATE
    arr controller_pos = C->getFrame(controller)->getPosition() - controller_origin;
    auto tmp_pos = controller_pos.elem(0);
    controller_pos.elem(0) = -controller_pos.elem(1);
    controller_pos.elem(1) = tmp_pos;
    arr target_pos = target_origin + controller_pos*2.;
    C->getFrame(gripper_target)->setPosition(target_pos);
    
    // ROTATION UPDATE
    arr controller_quat = C->getFrame(controller)->getQuaternion();

    controller_quat = quaternion_add(controller_quat, rotation_offset);
    auto tmp = controller_quat.elem(1);
    controller_quat.elem(1) = -controller_quat.elem(2);
    controller_quat.elem(2) = tmp;
    controller_quat = normalize(controller_quat);
    C->getFrame(gripper_target)->setQuaternion(controller_quat);
}

int main(int argc,char **argv) {
    rai::initCmdLine(argc, argv);

    rai::Configuration C;
    C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandaSingle.g"));
    C.view(false);

    BotOp bot(C, true);
    bot.home(C);
    bot.gripperMove(rai::_left, .079);
    #if BOTH_ARMS
    bot.gripperMove(rai::_right, .079);
    #endif

    double delta = .2;
    const char* to_follow = "l_hand";

    rai::OptiTrack OT;
    OT.pull(C);

    GamepadInterface G;
    std::cout << "Waiting for Gamepad thread...";
    G.gamepadState.waitForNextRevision();
    std::cout << "Done.\n";

    double last_timestamp = 0.;
    bool l_gripper_closed = false;
    // bool r_gripper_closed = false;

    #if BOTH_ARMS
    arr r_controller_origin = arr{0., 0., 0.};
    arr r_target_origin = arr{0., 0., 0.};
    arr r_rotation_offset = arr{0., 0., 0., 1.};
    #endif

    arr l_controller_origin = arr{0., 0., 0.};
    arr l_target_origin = arr{0., 0., 0.};
    arr l_rotation_offset = arr{0., 0., 0., 1.};
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    // Desired frequency in Hz
    const double frequency = 10.;
    // Calculate the interval duration in milliseconds
    const std::chrono::milliseconds interval(static_cast<int>(1000.0 / frequency));
    arr last_komo = bot.get_q();
    arr q_dot_ref = bot.get_qDot();

    while(!(C.view(false)=='k' || G.gamepadState.get()().elem(0)==BTN_B)) { OT.pull(C); }

    #if BOTH_ARMS
    r_controller_origin = C.getFrame("r_controller")->getPosition();
    r_target_origin = C.getFrame("r_gripper")->getPosition();
    r_rotation_offset = C.getFrame("r_controller")->getQuaternion();
    C.addFrame("r_gripper_target")->setPosition(r_target_origin).setShape(rai::ST_marker, {.2});
    #endif

    l_controller_origin = C.getFrame(to_follow)->getPosition();
    l_target_origin = C.getFrame("l_gripper")->getPosition();

    auto controller_quat = C.getFrame(to_follow)->getQuaternion();
    // auto tmp = controller_quat.elem(1);
    // controller_quat.elem(1) = -controller_quat.elem(2);
    // controller_quat.elem(2) = tmp;
    l_rotation_offset = quaternion_add(conjugate(controller_quat), C.getFrame("l_gripper")->getQuaternion());
    l_rotation_offset = normalize(l_rotation_offset);
    
    std::cout << "Gripper Quat " <<  C.getFrame("l_gripper")->getQuaternion() << std::endl;
    std::cout << "OFFSET " << l_rotation_offset << std::endl;
    std::cout << "Controller + OFFSET " << quaternion_add(controller_quat, l_rotation_offset) << std::endl;

    C.addFrame("l_gripper_target")->setPosition(l_target_origin).setShape(rai::ST_marker, {.2});
    last_timestamp = bot.get_t();

    while(1)
    {
        OT.pull(C);

        if(C.view(false)=='q' || G.quitSignal.get()) break;

        auto loop_start = std::chrono::steady_clock::now();

        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

        #if BOTH_ARMS
        reload_target(&C, r_target_origin, r_controller_origin, r_rotation_offset, "r_controller", "r_gripper_target");
        #endif
        reload_target(&C, l_target_origin, l_controller_origin, l_rotation_offset, to_follow, "l_gripper_target");

        KOMO komo(C, 1., 1, 2, true);
        komo.addControlObjective({}, 0, 1e1);
        komo.addControlObjective({}, 1, .1);
        komo.addControlObjective({}, 2, .1);
        komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1e10});
        komo.addObjective({}, FS_jointLimits, {}, OT_ineq, {1e0});

        bool l_should_translate = -C.eval(FS_negDistance, {"l_gripper", "l_gripper_target"}).elem(0) > .01;
        if (l_should_translate) {
            komo.addObjective({1.}, FS_positionDiff, {"l_gripper", "l_gripper_target"}, OT_eq, {1e1});
        } else {
            komo.addObjective({}, FS_position, {"l_gripper"}, OT_eq, {1e1}, C.getFrame("l_gripper")->getPosition());
        }

        #if BOTH_ARMS
        bool r_should_translate = -C.eval(FS_negDistance, {"r_gripper", "r_gripper_target"}).elem(0) > .01;
        if (r_should_translate) {
            komo.addObjective({1.}, FS_positionDiff, {"r_gripper", "r_gripper_target"}, OT_eq, {1e1});
        } else {
            komo.addObjective({}, FS_position, {"r_gripper"}, OT_eq, {1e1}, C.getFrame("r_gripper")->getPosition());
        }
        #else
        bool r_should_translate = false;
        #endif

        #if BOTH_ARMS
        komo.addObjective({1.}, FS_quaternionDiff, {"r_gripper", "r_gripper_target"}, OT_eq, {1e1});
        #endif
        komo.addObjective({1.}, FS_quaternionDiff, {"l_gripper", "l_gripper_target"}, OT_eq, {1e1});

        if (r_should_translate || l_should_translate) {
            auto ret = NLP_Solver()
                .setProblem(komo.nlp())
                .solve();
            cout << *ret <<endl;
            arr q = komo.getPath_qOrg();
            //bot.moveTo(q[0], {5.}, true);
            bot.sync(C, 0);
        }

        if (l_gripper_closed && !(G.gamepadState.get()().elem(0)==BTN_R)) {
            bot.gripperMove(rai::_left, .079);
            l_gripper_closed = false;
        }
        else if (!l_gripper_closed && G.gamepadState.get()().elem(0)==BTN_R) {
            bot.gripperClose(rai::_left);
            l_gripper_closed = true;
        }

        auto loop_end = std::chrono::steady_clock::now();
        std::chrono::duration<double, std::milli> elapsed_time = loop_end - loop_start;
        auto sleep_duration = interval - std::chrono::duration_cast<std::chrono::milliseconds>(elapsed_time);
        if (sleep_duration.count() > 0) {
            std::cout<<sleep_duration.count()<<std::endl;
            std::this_thread::sleep_for(sleep_duration);
        }
    }

    return 0;
}
