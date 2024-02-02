import numpy as np
import robotic as ry
from raiUtils import plotLine


class Arena:
    def __init__(self, position=np.array([0, 0]), name="arena"):
        self.position = position
        self.name = name

    def isPointInside(self, point: np.ndarray) -> bool:
        return False
    
    def randomPointInside(z_value: float=0.) -> np.ndarray:
        return np.array([0., 0., z_value])

class RectangularArena(Arena):
    
    def __init__(self, position: np.ndarray, width: float, height: float, name: str="arena"):
        super().__init__(position, name)
        self.width = width
        self.height = height


    def area(self) -> float:
        return self.width * self.height
    
    
    def display(self, C: ry.Config, color: [float]=[1., 0., 0.], resolution: int=10):
        """
        Visualises the area in the rai-Config.
        """
        offset = np.array([self.width, self.height, .0]) * .5
        # Horizontal lines
        a = self.position - offset
        offset[0] *= -1
        b = self.position - offset
        plotLine(C, a, b, f"{self.name}_lineA", resolution, color)
        offset[1] *= -1
        a = self.position - offset
        offset[0] *= -1
        b = self.position - offset
        plotLine(C, a, b, f"{self.name}_lineB", resolution, color)

        # Vertical lines
        a = self.position - offset
        offset[1] *= -1
        b = self.position - offset
        plotLine(C, a, b, f"{self.name}_lineC", resolution, color)
        offset[0] *= -1
        a = self.position - offset
        offset[1] *= -1
        b = self.position - offset
        plotLine(C, a, b, f"{self.name}_lineD", resolution, color)
    

    def isPointInside(self, point: np.ndarray) -> bool:
        return not (point[0] > self.position[0]+.5*self.width or point[0] < self.position[0]-.5*self.width or point[1] > self.position[1]+.5*self.height or point[1] < self.position[1]-.5*self.height)
    
    
    def randomPointInside(self, z_value: float=0.) -> np.ndarray:
        point = ( np.random.rand(3) - .5 ) * np.array([self.width, self.height, 0.]) + self.position
        if z_value:
            point[2] = z_value
        return point
