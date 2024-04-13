#include <KOMO/komo.h>
#include <BotOp/bot.h>

//===========================================================================

arr getPath(const rai::Configuration& C, int verbose=0){
  KOMO komo(C, 1., 20, 2, false);
  komo.addControlObjective({}, 2, 1.);

  // komo.addObjective({1.}, FS_positionDiff, {"omnibase", "way1"}, OT_eq, {1e2});
  // komo.addObjective({2.}, FS_positionDiff, {"omnibase", "way2"}, OT_eq, {1e2});
  // komo.addObjective({1.,2.}, FS_vectorX, {"omnibase"}, OT_eq, {1e2}, {0,1,0});
  // komo.addObjective({3.}, FS_positionDiff, {"omnibase", "way0"}, OT_eq, {1e2});
  // komo.addObjective({3.}, FS_vectorX, {"omnibase"}, OT_eq, {1e2}, {1,0,0});
  // komo.addObjective({3.}, FS_qItself, {}, OT_eq, {1e2}, {}, 1);

  komo.addObjective({1.}, FS_positionDiff, {"r_gripper", "way3"}, OT_eq, {1e2});

  // komo.addObjective({1.}, FS_position, {"r_gripper"}, OT_eq, {1e2}, C.getFrame("r_gripper")->getPosition() + arr{0, 0, -.2});
  // komo.addObjective({2.}, FS_position, {"r_gripper"}, OT_eq, {1e2}, C.getFrame("r_gripper")->getPosition() + arr{0, 0, .2});
  // komo.addObjective({3.}, FS_position, {"r_gripper"}, OT_eq, {1e2}, C.getFrame("r_gripper")->getPosition());

  // komo.addObjective({}, FS_position, {"omnibase"}, OT_eq, {1e2}, C.getFrame("omnibase")->getPosition());
  // komo.addObjective({}, FS_quaternion, {"omnibase"}, OT_eq, {1e2}, C.getFrame("omnibase")->getQuaternion());


  komo.opt.verbose=verbose;
  komo.optimize();

  if(verbose>0){
    //  cout <<"TIME OPTIM: total=" <<sum(komo.getPath_times()) <<komo.getPath_times() <<endl;
    komo.plotTrajectory();
    //  komo.reportProxies();
    //  komo.checkGradients();
    komo.view(true, "result");
    while(komo.view_play(true));
  }

  return komo.getPath_qOrg();
}

//===========================================================================

void moveOmnibase(){
  rai::Configuration C;
  C.addFile("../g_files/scenarios/pandaOmnibase.g");
  //C.addFile("../g_files/omnibase/omnibase.g");

  C.addFrame("way0")->setPosition({0., 0., .1});
  C.addFrame("way1")->setPosition({.5, 0., .1});
  C.addFrame("way2")->setPosition({0., 1., .1});
  C.addFrame("way3")->setPosition({0., -2., .2});
  C.view(false);

  BotOp bot(C, true);
  bot.home(C);

  arr q = getPath(C);

  rai::wait(.1);

  bot.move(q, {5.});
  bot.wait(C);

  rai::wait();
}

//===========================================================================

int main(int argc,char** argv){
  rai::initCmdLine(argc,argv);

  moveOmnibase();

  return 0;
}

