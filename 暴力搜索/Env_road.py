import numpy as np
import Vehicle
import matplotlib.pyplot as plt

class EnvRoads:
    def __init__(self,size=25,lanewidth=4):
        self.size = size
        self.lanewidth = lanewidth
        self.center = np.array([0,0])
        self.rect=[
            [[-size,-size,-2*lanewidth,-lanewidth,-lanewidth,-size],[-size,-lanewidth,-lanewidth,-2*lanewidth,-size,-size]],
            [[size,size,2*lanewidth,lanewidth,lanewidth,size],[-size,-lanewidth,-lanewidth,-2*lanewidth,-size,-size]],
            [[size,size,2*lanewidth,lanewidth,lanewidth,size],[size,lanewidth,lanewidth,2*lanewidth,size,size]],
            [[-size,-size,-2*lanewidth,-lanewidth,-lanewidth,-size],[size,lanewidth,lanewidth,2*lanewidth,size,size]],
        ]
        self.laneline=[
            [[0,0],[-size,-2*lanewidth]],
            [[0,0],[size,2*lanewidth]],
            [[-size,-2*lanewidth],[0,0]],
            [[size,2*lanewidth],[0,0]],
        ]
        
    def draw_env(self,ax,vehicles=None,show_safe=False):
        ax.set_xlim(-self.size, self.size)
        ax.set_ylim(-self.size, self.size)
        ax.set_aspect('equal', adjustable='box')
        # Draw the road rectangles
        for rect in self.rect:
            ax.fill(rect[0], rect[1], color='gray', alpha=0.5)
        for rect in self.rect:
            ax.plot(rect[0], rect[1], color='black', linewidth=2)
        
        # Draw the lane lines
        for line in self.laneline:
            ax.plot(line[0], line[1],linestyle='--' ,color='orange', linewidth=2)
        
        #arrows for directions
        arrows=[(2, -18, 0, 4),(-2, -13, 0, -4),(-2, 18, 0, -4),(2, 13, 0, 4),(18, 2, -4, 0),(13, -2, 4, 0),(-18, -2, 4, 0),(-13, 2, -4, 0)]
        for arrow in arrows:
            ax.arrow(arrow[0], arrow[1], arrow[2], arrow[3], width=0.8, head_width=1.5, head_length=1, fc='blue', ec='blue')
        
        # Draw vehicles if provided
        if vehicles is not None:
            vehicles[0].draw_vehicle(ax, color='green',safe_color='red',show_safe=show_safe)
            vehicles[1].draw_vehicle(ax, color='blue',safe_color='red',show_safe=show_safe)
        
        ax.set_title('Road Environment')
        

    
  