import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon


class vehicle:
    def __init__(self,x,y,v,theta,target_state,dt):
        self.x = x
        self.y = y
        self.v = v
        self.theta = theta
        self.dt = dt
        self.target_state=target_state
        self.length=5.0
        self.width=2.0
        self.safe_length=8.0
        self.safe_width=2.4
        self.a_max=2.5
        self.a_min=-5.0
        self.omega_max=np.pi/2
        self.omega_min=-np.pi/2
        self.action_map={
            "Maintain": [0,0], # no action
            "TurnLeft": [0,np.pi/4], # accelerate
            "TurnRight": [0,-np.pi/4], # decelerate
            "Accelerate": [2.5,0], # turn left
            "Decelerate": [-2.5,0], # turn right
            "Brake": [-5,0], # brake
        }

    def Update(self,action):
        # Update the vehicle's state based on the action taken
        a, omega = self.action_map[action]
        a= np.clip(a, self.a_min, self.a_max)
        omega = np.clip(omega, self.omega_min, self.omega_max)
        self.v += a * self.dt
        self.theta += omega * self.dt
        self.theta = np.mod(self.theta, 2 * np.pi)  # 限制在 [0, 2pi)
        # Update velocity and angle
        self.x += self.v * np.cos(self.theta) * self.dt
        self.y += self.v * np.sin(self.theta) * self.dt

    def is_get_target(self):
        # Check if the vehicle has reached its target state
        target_x, target_y, target_v, target_theta = self.target_state
        delta_x = np.abs(self.x - target_x)
        delta_y = np.abs(self.y - target_y)
        delta_v = np.abs(self.v - target_v)
        delta_theta = np.abs(self.theta - target_theta)

        # Check if the vehicle is within a certain threshold of the target state
        return delta_x < 0.5 and delta_y < 0.5 and delta_v < 0.5 and delta_theta < 0.1    
    def get_state(self):
        # Return the current state of the vehicle
        return np.array([self.x, self.y, self.theta, self.v])
    def get_box2d(self):
        # Calculate the corners of the vehicle's bounding box
        dx=self.length/2
        dy=self.width/2

        corners=np.array([
            [dx,dy],
            [-dx,dy],
            [-dx,-dy],
            [dx,-dy],
        ])
        #rotation matrix
        rotation_matrix=np.array([
            [np.cos(self.theta), -np.sin(self.theta)],
            [np.sin(self.theta), np.cos(self.theta)]
        ])

        #rotate the corners
        rotated_corners=np.dot(corners,rotation_matrix.T)
        #translate the corners
        rotated_corners+=np.array([self.x,self.y])

        return rotated_corners
    
    def get_safe_box2d(self):
        # Calculate the corners of the vehicle's bounding box
        dx=self.safe_length/2
        dy=self.safe_width/2

        corners=np.array([
            [dx,dy],
            [-dx,dy],
            [-dx,-dy],
            [dx,-dy],
        ])
        #rotation matrix
        rotation_matrix=np.array([
            [np.cos(self.theta), -np.sin(self.theta)],
            [np.sin(self.theta), np.cos(self.theta)]
        ])

        #rotate the corners
        rotated_corners=np.dot(corners,rotation_matrix.T)
        #translate the corners
        rotated_corners+=np.array([self.x,self.y])

        return rotated_corners
    
    def copy(self):
        # Create a copy of the vehicle object
        return vehicle(self.x, self.y, self.v, self.theta, self.target_state, self.dt)
    
    def draw_vehicle(self,ax, color='blue',safe_color='red',show_safe=True):
        """
        Draw a vehicle on the given axes.
        
        Parameters:
        ax (matplotlib.axes.Axes): The axes to draw on.
        vehicle (Vehicle): The vehicle object to draw.
        color (str): Color of the vehicle.
        """
        box=self.get_box2d()
        safe_box=self.get_safe_box2d()
        # Create a polygon from the vehicle's corners
        vehicle_polygon = Polygon(box, closed=True, edgecolor='black', facecolor=color, alpha=0.7)

        # Add the polygon to the axes
        ax.add_patch(vehicle_polygon)

        quarter_length = self.length / 4
        mid_front_x= self.x + quarter_length * np.cos(self.theta)
        mid_front_y= self.y + quarter_length * np.sin(self.theta)
        dx= self.width/2 * np.sin(self.theta)
        dy= -self.width/2 * np.cos(self.theta)

        # Draw the front of the vehicle
        x1,y1= mid_front_x-dx, mid_front_y-dy
        x2,y2= mid_front_x+dx, mid_front_y+dy
        ax.plot([x1, x2], [y1, y2], color='black', linewidth=1.0)

        if show_safe:
            # Create a polygon from the vehicle's corners
            safe_polygon = Polygon(safe_box, closed=True, edgecolor='black', facecolor=safe_color, alpha=0.3)

            # Add the polygon to the axes
            ax.add_patch(safe_polygon)
        # Set the aspect of the plot to be equal
        ax.set_aspect('equal', adjustable='box')
