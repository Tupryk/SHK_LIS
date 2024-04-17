#include <iostream>
#include <OptiTrack/optitrack.h>
#include <Core/graph.h>
#include <KOMO/komo.h>
#include <BotOp/bot.h>
#include <Optim/NLP_Solver.h>

#define BOTH_ARMS 1


void reload_target(rai::Configuration* C, arr target_origin, arr controller_origin, const char* controller, const char* gripper_target) {
    arr controller_quat = C->getFrame(controller)->getQuaternion();
    controller_quat.elem(1) *= -1;
    controller_quat.elem(2) *= -1;
    C->getFrame(gripper_target)->setQuaternion(controller_quat);
    arr controller_pos = C->getFrame(controller)->getPosition() - controller_origin;
    controller_pos.elem(0) *= -1;
    controller_pos.elem(1) *= -1;
    arr target_pos = target_origin + controller_pos*2;
    C->getFrame(gripper_target)->setPosition(target_pos);
}

int main(int argc,char **argv) {
    rai::initCmdLine(argc, argv);

    rai::Configuration C;
    C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandasTable.g"));

    rai::OptiTrack OT;

    OT.pull(C);
    C.view(false);

    bool activated = false;

    BotOp bot(C, true);
    bot.home(C);

    #if BOTH_ARMS
    arr r_controller_origin = arr{0., 0., 0.};
    arr r_target_origin = arr{0., 0., 0.};
    arr r_rotation_offset = arr{0., 0., 0., 1.};
    #endif

    arr l_controller_origin = arr{0., 0., 0.};
    arr l_target_origin = arr{0., 0., 0.};
    arr l_rotation_offset = arr{0., 0., 0., 1.};

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

            l_controller_origin = C.getFrame("l_controller")->getPosition();
            l_target_origin = C.getFrame("l_gripper")->getPosition();
            l_rotation_offset = C.getFrame("l_controller")->getQuaternion();
            C.addFrame("l_gripper_target")->setPosition(l_target_origin).setShape(rai::ST_marker, {.2});
        }

        // Define KOMO problem towards the target waypoint
        if (activated) {

            #if BOTH_ARMS
            reload_target(&C, r_target_origin, r_controller_origin, "r_controller", "r_gripper_target");
            #endif
            reload_target(&C, l_target_origin, l_controller_origin, "l_controller", "l_gripper_target");

            KOMO komo(C, 1., 1, 2, true);
            komo.addControlObjective({}, 0, 1.);
            komo.addControlObjective({}, 1, .1);
            komo.addControlObjective({}, 2, .1);
            komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1e10});

            bool should_translate = -C.eval(FS_negDistance, {"l_gripper", "l_gripper_target"}).elem(0) > .01;
            if (should_translate) {
                komo.addObjective({1.}, FS_positionDiff, {"l_gripper", "l_gripper_target"}, OT_eq, {1e1});
            } else {
                komo.addObjective({}, FS_position, {"l_gripper"}, OT_eq, {1e1}, C.getFrame("l_gripper")->getPosition());
            }

            #if BOTH_ARMS
            should_translate = -C.eval(FS_negDistance, {"r_gripper", "r_gripper_target"}).elem(0) > .01;
            if (should_translate) {
                komo.addObjective({1.}, FS_positionDiff, {"r_gripper", "r_gripper_target"}, OT_eq, {1e1});
            } else {
                komo.addObjective({}, FS_position, {"r_gripper"}, OT_eq, {1e1}, C.getFrame("r_gripper")->getPosition());
            }
            #endif

            #if BOTH_ARMS
            komo.addObjective({1.}, FS_quaternionDiff, {"r_gripper", "r_gripper_target"}, OT_eq, {1e1});
            #endif
            komo.addObjective({1.}, FS_quaternionDiff, {"l_gripper", "l_gripper_target"}, OT_eq, {1e1});

            if (should_translate) {
                auto ret = NLP_Solver()
                    .setProblem(komo.nlp())
                    .solve();
                cout << *ret <<endl;
                arr q = komo.getPath_qOrg();
                bot.moveTo(q[0], {5.}, true);
                bot.sync(C, 0);
            }
        }
    }

    return 0;
}
