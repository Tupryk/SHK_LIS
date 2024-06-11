#include <iostream>
#include <OptiTrack/optitrack.h>
#include <Core/graph.h>
#include <KOMO/komo.h>
#include <BotOp/bot.h>
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
    arr controller_pos = C->getFrame(controller)->getPosition() - controller_origin;
    arr target_pos = target_origin + controller_pos*1;
    C->getFrame(gripper_target)->setPosition(target_pos);

    arr controller_quat = C->getFrame(controller)->getQuaternion();
    controller_quat = quaternion_add(controller_quat, rotation_offset);
    controller_quat = normalize(controller_quat);
    C->getFrame(gripper_target)->setQuaternion(controller_quat);
}

int main(int argc,char **argv) {
    rai::initCmdLine(argc, argv);

    rai::Configuration C;
    C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandaSingle.g"));
    C.view(false);

    rai::Configuration C2;
    C2 = C;

    BotOp bot(C, false);
    bot.home(C);
    arr qHome = C.getJointState();
    bot.gripperMove(rai::_left, .079);
    #if BOTH_ARMS
    bot.gripperMove(rai::_right, .079);
    #endif

    double delta = .2;

    const char* to_follow = "l_hand";

    rai::OptiTrack OT;
    OT.pull(C);

    double last_timestamp = 0.;
    bool activated = false;
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
    const double tau = .05;
    // Calculate the interval duration in milliseconds
    const std::chrono::milliseconds interval(static_cast<int>(1000.0 * tau));
    arr last_komo = bot.get_q();
    arr q_dot_ref = bot.get_qDot();

    auto controller_quat = C.getFrame(to_follow)->getQuaternion();
    l_rotation_offset = quaternion_add(conjugate(controller_quat), C.getFrame("l_gripper")->getQuaternion());
    l_rotation_offset = normalize(l_rotation_offset);

    while(1) {
        
        OT.pull(C);

        if(C.view(false)=='q') break;
        if (C.view(false)=='k' && !activated) {

            activated = true;

            #if BOTH_ARMS
            r_controller_origin = C.getFrame("r_controller")->getPosition();
            r_target_origin = C.getFrame("r_gripper")->getPosition();
            r_rotation_offset = C.getFrame("r_controller")->getQuaternion();
            C.addFrame("r_gripper_target")->setPosition(r_target_origin).setShape(rai::ST_marker, {.2});
            #endif

            l_controller_origin = C.getFrame(to_follow)->getPosition();
            l_target_origin = C.getFrame("l_gripper")->getPosition();
            l_rotation_offset = C.getFrame(to_follow)->getQuaternion();
            C.addFrame("l_gripper_target")->setPosition(l_target_origin).setShape(rai::ST_marker, {.2});
            last_timestamp = bot.get_t();
        }

        // Define KOMO problem towards the target waypoint
        if (activated) {
            
            auto loop_start = std::chrono::steady_clock::now();

            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

            #if BOTH_ARMS
            reload_target(&C, r_target_origin, r_controller_origin, r_rotation_offset, "r_controller", "r_gripper_target");
            #endif
            reload_target(&C, l_target_origin, l_controller_origin, l_rotation_offset, to_follow, "l_gripper_target");

            KOMO komo(C, 1., 1, 2, true);
            
            komo.setConfig(C, true);
            komo.setTiming(1, 1, 1, 0);
            //komo.addControlObjective({}, 0, 1e-1);
            // komo.addControlObjective({}, 1, 1e1);
            // komo.addControlObjective({}, 2, frequency*frequency);
            
            double eps = 1e1;
            komo.addObjective({}, FS_qItself, {}, OT_sos, {1e-1}, qHome);            
            komo.addObjective({}, FS_qItself, {}, OT_sos, {1e0}, last_komo);            
            // komo.addObjective({}, FS_qItself, {}, OT_sos, {frequency*frequency}, (last_komo+q_dot_ref/(frequency)));

            // komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1e10});
            // komo.addObjective({}, FS_jointLimits, {}, OT_ineq, {1e1});
            bool l_should_translate = -C.eval(FS_negDistance, {"l_gripper", "l_gripper_target"}).elem(0) > .0001; // only translate if controllerposition changed one cm (to prevent jitter)
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
                // C2.setJointState(q);
                // C2.view(false,"config copy");
                
                //q̇ ref = (q ref − q(t − θ))/θ
                std::cout << "q_dot reeefefee" << bot.get_qDot() << std::endl;

                std::cout << "bot.get_t: " << bot.get_t() << std::endl;
                std::cout << "last t: " << last_timestamp << std::endl;
                arr q_dot_ref = (q[0]-last_komo)/tau;

                // std::cout << "\nqref" << q[0] << std::endl;
                // std::cout << "q(t-theta)" << last_komo << std::endl;  
                // std::cout << "q diff" << q[0]-last_komo << std::endl;
                // std::cout << "q(t-theta)" << last_komo << std::endl; 
                // std::cout << "Time diff" << bot.get_t()-last_timestamp << "\n" << std::endl;

                arr q_now = q[0] + delta*q_dot_ref;
                arr q_atdelta = q[0] + delta*q_dot_ref;
                arr q_at2delta = q[0] + 2.*delta*q_dot_ref;  
                arr q_at10delta = q[0] + 5.*delta*q_dot_ref;  
                
                double overwrite_time = bot.get_t();
                std::cout << "Input to move: " << (q_atdelta, q_at2delta, q_at10delta).reshape(-1, q[0].N) << std::endl;
                

                //bot.move((q_atdelta).reshape(-1, q[0].N), {delta, 2*delta, 10*delta}, true, overwrite_time);

                // bot.move((q_atdelta, q_at2delta, q_at10delta).reshape(-1, q[0].N), {delta, 2.*delta, 10.*delta}, true, overwrite_time);
                // bot.move((q_atdelta, q_at2delta).reshape(-1, q[0].N), {1.5*delta, 4.*delta}, true, overwrite_time);
                bot.move(q_at2delta.reshape(-1, q[0].N), {4.*delta}, true, overwrite_time);
                // bot.moveTo(q_at2delta, 2., true);
                last_komo = q[0];
                last_timestamp = bot.get_t();

                std::cout << "QDOT REF:" << q_dot_ref<< std::endl;
                
                bot.sync(C, 0);
            }

            float l_gripper_pos = .079 - (C.eval(FS_negDistance, {"l_controller", "l_thumb"}).elem(0)+.014) / -.07 * .079;
            if (l_gripper_pos > .079) l_gripper_pos = .079;
            if (l_gripper_pos < .0) l_gripper_pos = .0;
            // std::cout << "Left gripper pos: " << l_gripper_pos << std::endl;

            if (l_gripper_closed && l_gripper_pos >= .079*.66) {
                bot.gripperMove(rai::_left, .079);
                l_gripper_closed = false;
            }
            else if (!l_gripper_closed && l_gripper_pos <= .079*.33) {
                bot.gripperClose(rai::_left);
                l_gripper_closed = true;
            }

            #if BOTH_ARMS
            float r_gripper_pos = .079 - (C.eval(FS_negDistance, {"r_controller", "r_thumb"}).elem(0)+.014) / -.07 * .079;
            if (r_gripper_pos > .079) r_gripper_pos = .079;
            if (r_gripper_pos < .0) r_gripper_pos = .0;
            std::cout << "Right gripper pos: " << r_gripper_pos << std::endl;

            if (r_gripper_closed && r_gripper_pos >= .079*.66) {
                bot.gripperMove(rai::_right, .079);
                l_gripper_closed = false;
            }
            else if (!r_gripper_closed && r_gripper_pos <= .079*.33) {
                bot.gripperClose(rai::_right);
                r_gripper_closed = true;
            }
            #endif

            auto loop_end = std::chrono::steady_clock::now();
            std::chrono::duration<double, std::milli> elapsed_time = loop_end - loop_start;
            auto sleep_duration = interval - std::chrono::duration_cast<std::chrono::milliseconds>(elapsed_time);
            if (sleep_duration.count() > 0) {
                std::cout<<sleep_duration.count()<<std::endl;
                std::this_thread::sleep_for(sleep_duration);
            }
        }
    }

    return 0;
}
