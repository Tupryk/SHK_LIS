from robotic import ry

from config import setup_config, startup_robot
from RandomPushing.utils.bird_search import flyToPoint, getBirdView

if __name__ == "__main__":
    ry.params_print()
    C = setup_config()

    bot = startup_robot(C)
    obj_pos = C.getFrame("obj").getPosition()
    
    flyToPoint([obj_pos[0]+.2, obj_pos[1]+.1], C, bot)
    cx, cy, dist = getBirdView(bot, C)
    while dist > 20:
        flyToPoint([cx, cy], C, bot)
        cx, cy, dist = getBirdView(bot, C)
    getBirdView(bot, C)
