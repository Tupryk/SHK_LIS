#include <iostream>
#include <OptiTrack/optitrack.h>
#include <Core/graph.h>
#include <KOMO/komo.h>
#include <BotOp/bot.h>


int main(int argc,char **argv) {
    rai::initCmdLine(argc, argv);

    rai::Configuration C;
    C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandaSingle.g"));

    rai::OptiTrack OT;

    OT.pull(C);
    C.view(false);

    bool activated = false;

    BotOp bot(C, true);
    bot.home(C);

    arr controller_origin = arr{0., 0., 0.};
    arr target_origin = arr{0., 0., 0.};

    while(1) {
        OT.pull(C);

        if(C.view(false)=='q') break;
        if (C.view(false)=='k' && !activated) {
            activated = true;
            controller_origin = C.getFrame("controller")->getPosition();
            target_origin = C.getFrame("l_gripper")->getPosition();
            C.addFrame("gripper_target")->setPosition(target_origin).setShape(rai::ST_marker, {.2});
        }

        // Define KOMO problem towards the target waypoint
        if (activated) {
            C.getFrame("gripper_target")->setQuaternion(C.getFrame("controller")->getQuaternion());
            arr controller_pos = C.getFrame("controller")->getPosition() - controller_origin;
            controller_pos.elem(0) *= -1;
            controller_pos.elem(1) *= -1;
            arr target_pos = target_origin + controller_pos;
            C.getFrame("gripper_target")->setPosition(target_pos);

            KOMO komo(C, 1., 1, 2, false);
            komo.addControlObjective({}, 1, .1);
            komo.addControlObjective({}, 2, .1);
            //komo.addObjective({1.}, FS_qItself, {}, OT_eq, {10.}, {}, 1);
            bool should_translate = -C.eval(FS_negDistance, {"l_gripper", "gripper_target"}).elem(0) > .01;
            bool should_rotate = true;
            if (should_translate) {
                komo.addObjective({1.}, FS_positionDiff, {"l_gripper", "gripper_target"}, OT_eq, {1e1});
            } else {
                komo.addObjective({}, FS_position, {"l_gripper"}, OT_eq, {1e1}, C.getFrame("l_gripper")->getPosition());
            }
            // if (should_rotate) {
            //     komo.addObjective({1.}, FS_quaternionDiff, {"l_gripper", "gripper_target"}, OT_eq, {1e1});
            // } else {
            //     komo.addObjective({}, FS_quaternion, {"l_gripper"}, OT_eq, {1e1}, C.getFrame("l_gripper")->getQuaternion());
            // }

            if (should_translate && should_rotate) {
                komo.optimize();
                arr q = komo.getPath_qOrg();
                bot.move(q, {1.});
                bot.wait(C);
            }
        }
    }

    return 0;
}
