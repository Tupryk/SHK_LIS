import numpy as np
import robotic as ry

def valueInSegments(segments: [[float]], value: float) -> bool:
    for seg in segments:
        if value >= seg[0] and value <= seg[1]:
            return True
    return False

def visualizeViewAreas(C: ry.Config,
                       objPos: np.ndarray,
                       segments: [[float]],
                       startPushDist: float=.15,
                       resolution: int=16):
    
    step_size = 2*np.pi/resolution
    for i in range(resolution):
        angle = step_size*i
        offsetAngle = -angle + np.pi*.5
        dir_vec = np.array([np.cos(offsetAngle), np.sin(offsetAngle), 0])

        pos = objPos + startPushDist*dir_vec

        color = [1, 1, 0] if valueInSegments(segments, angle) else [1, 0, 1]
        
        C.addFrame(f'viewAreaParticle_{i}') \
            .setPosition(pos) \
            .setShape(ry.ST.sphere, size=[.01]) \
            .setColor(color)
