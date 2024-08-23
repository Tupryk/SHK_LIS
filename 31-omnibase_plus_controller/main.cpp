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
    float target_dist = .2;
    float target_rot = .2;
    float x = sin(omnibase_joint_state.elem(0));
    float y = cos(omnibase_joint_state.elem(0));
    arr omnibase_joint_state = bot.get_q();
    switch(button_pressed) {
      case BTN_A: // Right
        target = target_dist * arr{y, -x, 0.};
        break;
      case BTN_Y: // Left
        target = target_dist * arr{-y, x, 0.};
        break;
      case BTN_X: // Forwards
        target = target_dist * arr{-x, -y, 0.};
        break;
      case BTN_B: // Backwards
        target = target_dist * arr{x, y, 0.};
        break;
      case BTN_L: // Turn counter-clockwise
        target = arr{0., 0., target_rot};
        break;
      case BTN_R: // Turn clockwise
        target = arr{0., 0., -target_rot};
        break;
      default:
        target = arr{0., 0., 0.};
        break;
    }
    target += omnibase_joint_state;

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

