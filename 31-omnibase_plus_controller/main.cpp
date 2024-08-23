#include <KOMO/komo.h>
#include <BotOp/bot.h>
#include <Gamepad/gamepad.h>

/*
ls /dev/hidraw*
sudo chmod a+rw /dev/hidraw*
*/

void moveOmnibase(){
  rai::Configuration C;
  C.addFile(rai::raiPath("../rai-robotModels/omnibase/omnibase.g"));

  BotOp bot(C, true);
  GamepadInterface G;

  while(true) {
    G.gamepadState[0].waitForNextRevision();
    
    int button_pressed = G.getButtonPressed();
    arr target;
    float target_dist = .5;
    float target_rot = -.1;
    switch(button_pressed) {
      case BTN_A:
        target = arr{-target_dist, 0., 0.};
        break;
      case BTN_Y:
        target = arr{target_dist, 0., 0.};
        break;
      case BTN_X:
        target = arr{0., -target_dist, 0.};
        break;
      case BTN_B:
        target = arr{0., target_dist, 0.};
        break;
      case BTN_L:
        target = arr{0., 0., target_rot};
        break;
      case BTN_R:
        target = arr{0., 0., -target_rot};
        break;
      default:
        target = arr{0., 0., 0.};
        break;
    }
    arr omnibase_joint_state = bot.get_q();
    target += omnibase_joint_state;
    // arr omnibase_pos = C.getFrame("omnibase")->getPosition();
    // target.elem(0) += omnibase_pos.elem(0);
    // target.elem(1) += omnibase_pos.elem(1);
    bot.moveTo(target, {3.}, true);
    bot.sync(C);
    if(G.quitSignal.get()) break;
  }
}

//===========================================================================

int main(int argc,char** argv){
  rai::initCmdLine(argc,argv);

  moveOmnibase();

  return 0;
}

