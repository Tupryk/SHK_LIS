import numpy as np
import robotic as ry
from typing import Optional, Tuple
from utils.geometry import *
from RobotEnviroment.visual import plotLine


class Arena:
    def __init__(self, middleP=np.array([0, 0]), name="arena"):
        self.middleP = middleP
        self.name = name

    def area(self):
        return self.width * self.height
    
    def point_in_inner_circ(self, point, circlecent, rad=0):
        if rad==0:
            return False
        if point[0] <= circlecent[0]+rad and point[0] >= circlecent[0]-rad and point[1] <= circlecent[1]+rad and point[1] >= circlecent[1]-rad:
            return True
        return False
    

class CircularArena(Arena):

    def __init__(self, middleP, outerR, innerR=None):
        super().__init__(middleP)
        self.outerR = outerR
        self.innerR = innerR


    def area(self):
        outer_area = np.pi * self.outerR ** 2
        if self.innerR:
            inner_area = np.pi * self.innerR ** 2
        return outer_area - inner_area
    
    
    def plotArena(self, ry_config, resolution=48):
        '''
        Visualises the pushing area.
        Returns:
            None: Adds visualization frames representing red spheres for the inner and outer area.
        '''
        step_size = 2*np.pi/resolution
        for i in range(resolution):
            angle = step_size*i
            dir_vec = np.array([np.cos(angle), np.sin(angle), 0])
            outer_point = self.middleP + self.outerR*dir_vec

            if self.innerR:
                inner_point = self.middleP + self.innerR*dir_vec
                ry_config.addFrame(f'inner_arena_{i}') \
                    .setPosition(inner_point) \
                    .setShape(ry.ST.sphere, size=[.02]) \
                    .setColor([1, 0, 0])
            
            ry_config.addFrame(f'outer_arena_{i}') \
                .setPosition(outer_point) \
                .setShape(ry.ST.sphere, size=[.02]) \
                .setColor([1, 0, 0])
            
            
    def generate_waypoints(self, obj_pos, obj_width=0, start_distance=.07, ry_config=None):
        '''
        Args:
            obj_pos: Position of object in arena 
            obj_width: Width of the object
            start_distance: The distance of the starting waypoint to the object (default: 7cm)
            ry_config: Configuration of the rai simulation (only to plot the path/waypoints generated)
        
        Returns:
            Starting waypoint for push,
            Ending waypoint for push,
            Intersection with arena,
            Intersection with arena,
            Path generation success status
        '''

        # Check if object is inside the arena
        if not self.point_in_arena(obj_pos):
            print("Object is outside of arena!")
            return None, None, None, None, False
        
        # Generate a random direction vector for the push
        angle = np.random.random() * np.pi*2
        move_vec = np.array([np.cos(angle), np.sin(angle)])

        # Calculate intersections with inner/outer circle
        inner_points = line_circle_intersection(obj_pos, move_vec, self.middleP, self.innerR) if self.innerR else []
        outer_points = line_circle_intersection(obj_pos, move_vec, self.middleP, self.outerR)

        # The intersection fuctions remove the z value of vector, so we nee to put it back
        try: z = obj_pos[2]
        except: z = None
        if z != None:
            inner_points = [np.array([i[0], i[1], z]) for i in inner_points]
            outer_points = [np.array([i[0], i[1], z]) for i in outer_points]

        # Select relevant intersection points
        if inner_points:
            if len(inner_points) <= 1 or (len(inner_points) > 1 and np.linalg.norm(inner_points[1]-obj_pos) > np.linalg.norm(inner_points[0]-obj_pos)):
                point0 = inner_points[0]
            else:
                point0 = inner_points[1]

            if np.linalg.norm(outer_points[1]-obj_pos) > np.linalg.norm(outer_points[0]-obj_pos):
                point1 = outer_points[0]
            else:
                point1 = outer_points[1]
        else:
            point0 = outer_points[0]
            point1 = outer_points[1]
    
        # Pick a random orientation for the vector
        if np.random.choice([0, 1]):
            start_vec = point0-obj_pos
            end_vec = point1-obj_pos

        else:
            start_vec = point1-obj_pos
            end_vec = point0-obj_pos

        start_vec /= np.linalg.norm(start_vec)
        start_vec *= start_distance + obj_width
        start_point = obj_pos + start_vec

        end_vec *= np.random.random()
        end_point = obj_pos + end_vec

        # Display the path in the rai simulation
        if ry_config:
            plotLine(ry_config, np.array([point0[0], point0[1], .651]), np.array([point1[0], point1[1], .651]), name="path")
            
            points = segment_line(start_point, end_point, 6)
            for i, p in enumerate(points):
                way = ry_config.getFrame(f'way{i}')
                way.setPosition(p)

        return start_point, end_point, point0, point1, True
    
    
    def point_in_arena(self, point):
        rob2obj_len = np.linalg.norm(point[:2]-self.middleP[:2])
        return not (rob2obj_len >= self.outerR or (self.innerR and rob2obj_len < self.innerR))


class RectangularArena(Arena):
    
    def __init__(self, middleP, width, height, innerR=None, middlePCirc=[], name="arena"):
        super().__init__(middleP, name)
        self.width = width
        self.height = height
        self.innerR = innerR
        self.middlePCirc = middlePCirc if len(middlePCirc) else middleP


    def area(self):
        return self.width * self.height
    

    #TODO Implement plotArena for rectArena
    def plotArena(self, C, color=[1., 0., 0.], resolution=48):
        '''
        Visualises the pushing area.
        Returns:
            None: Adds visualization frames representing red spheres for the inner and outer area.
        '''

        # You can keep the step size consistent for both circular and rectangular arenas
        step_size = (2*self.height+2*self.width)/resolution

        """following: spaghetti code of doom"""
        step_size2 = 2*np.pi/resolution
        
        for i in range(resolution):
            angle = step_size * i
            angle2 = step_size2*i
            dir_vec_circ = np.array([np.cos(angle2), np.sin(angle2), 0])
            
            if(angle <= self.height):
                outer_point = self.middleP   + np.array([-1/2*self.width, step_size*i-1/2*self.height,0])
            elif(angle > self.height and angle <= self.height+self.width):
                outer_point = self.middleP  + np.array([step_size*i-self.height  - 1/2*self.width, 1/2*self.height,0])
            elif(angle > self.height+self.width and angle <= self.height*2+self.width):
                outer_point = self.middleP  + np.array([1/2*self.width, -(step_size*i-self.height-self.width)+1/2*self.height,0])
            else:
               outer_point = self.middleP + np.array([step_size*i-2*self.height-self.width-1/2*self.width,-1/2*self.height,0])

            if self.innerR:
                inner_point = self.middlePCirc + self.innerR * dir_vec_circ
                C.addFrame(f'{self.name}-inner_arena_{i}') \
                    .setPosition(inner_point) \
                    .setShape(ry.ST.sphere, size=[.02]) \
                    .setColor(color)
                
            C.addFrame(f'{self.name}-outer_arena_{i}') \
                .setPosition(outer_point) \
                .setShape(ry.ST.sphere, size=[.02]) \
                .setColor(color)
    

    def point_in_rect(self, point):
        return point[0] > self.middleP[0]+.5*self.width or point[0] < self.middleP[0]-.5*self.width or point[1] > self.middleP[1]+.5*self.height or point[1] < self.middleP[1]-.5*self.height
    
    
    def randomPointInArena(self) -> np.ndarray:
        return ( np.random.rand(3) - .5 ) * np.array([self.width, self.height, 0.]) + self.middleP


    def generate_waypoints(self,
                           point_pos: np.ndarray,
                           C: ry.Config,
                           start_distance: float=.15,
                           displayPath: bool=False) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        '''
        Args:
            point_pos: Point through which we want to push
            start_distance: The distance of the starting waypoint to the object (default: 15cm)
            C: Configuration of the rai simulation to create the waypoint frames
        
        Returns:
            Starting waypoint for push,
            Ending waypoint for push,
            Direction vector
        '''

        # Sample random point in rect arena
        end_point = self.randomPointInArena()
        while self.point_in_inner_circ(end_point, self.middlePCirc, self.innerR): # This repetition could be made better
            end_point = self.randomPointInArena()

        # Set z coord of point to point_pos z coord
        end_point[2] = point_pos[2]

        start_point = point_pos
        direction = end_point - start_point
        direction /= np.linalg.norm(direction)

        start_point = start_point - start_distance*direction

        # Generate waypoint frames
        self.generateWaypointFrames(C, start_point, end_point, point_pos)
 
        # Display the path in the simulation
        if displayPath:
            plotLine(C, start_point, end_point, name="PushPath_")

        return start_point, end_point, direction
    

    def generateWaypointFrames(self, C: ry.Config, start: np.ndarray, end: np.ndarray, point: np.ndarray):

        frame = C.getFrame("startWayPoint")
        if not frame:
            frame = C.addFrame("startWayPoint") \
                .setShape(ry.ST.sphere, size=[.02]) \
                .setColor([1, 0, 0])
            
        frame.setPosition(start)

        frame = C.getFrame("pushWayPoint")
        if not frame:
            frame = C.addFrame("pushWayPoint") \
                .setShape(ry.ST.sphere, size=[.02]) \
                .setColor([0, 1, 0])
            
        frame.setPosition(point)

        frame = C.getFrame("endWayPoint")
        if not frame:
            frame = C.addFrame("endWayPoint") \
                .setShape(ry.ST.sphere, size=[.02]) \
                .setColor([0, 0, 1])
            
        frame.setPosition(end)
    
    
    def point_in_arena(self, point):
        return not (self.point_in_rect(point) or (self.innerR and np.linalg.norm(point[:2]-self.middlePCirc[:2]) < self.innerR))
