import numpy as np
from Vehicle import vehicle
from Env_road import EnvRoads

def is_overlapping(box1, box2):
    """
    Check if two bounding boxes overlap.
    
    Parameters:
    box1 (np.array): The first bounding box [[x1, y1], [x2, y2], ...].
    box2 (np.array): The second bounding box [[x1, y1], [x2, y2], ...].
    
    Returns:
    bool: True if the boxes overlap, False otherwise.
    """
    def get_axes(corners):
        if np.array_equal(corners[0],corners[-1]):
            corners=corners[:-1]
        # Calculate the axes of the box
        axes=[]
        for i in range(len(corners)):
            p1 = corners[i]
            p2 = corners[(i + 1) % len(corners)]
            edge=np.array([p2[0]-p1[0], p2[1]-p1[1]])
            # Calculate the normal vector to the edge
            normal = np.array([-edge[1], edge[0]])
            axes.append(normal / np.linalg.norm(normal))
        return axes
    def project_onto_axis(corners, axis):
        projections = [np.dot(corner, axis) for corner in corners]
        return min(projections), max(projections)
    def is_separating_axis(box1, box2, axis):
        min1, max1 = project_onto_axis(box1, axis)
        min2, max2 = project_onto_axis(box2, axis)
        return max1 < min2 or max2 < min1
    # Get the corners of the boxes
    axes1= get_axes(box1)
    axes2= get_axes(box2)

    # Check for separating axes
    for axis in axes1 + axes2:
        if is_separating_axis(box1, box2, axis):
            return False
    return True  # No separating axis found, boxes overlap

def compute_reward(ego_vehicle,other_vehicle,road,weights):
    """
    Compute the reward for the ego vehicle based on its state, other vehicles' states, and the target point.
    
    Parameters:
    ego_vehicle (Vehicle): The ego vehicle object.
    other_vehicles (list): List of other vehicle objects.
    road (EnvRoad): The road environment object.
    weights (list): List of weights for each component of the reward.
    
    Returns:
    float: The computed reward.
            reward = w1*c+w2*s+w3*o+w4*l+w5*d
    """
    # Unpack ego vehicle state
    ego_x, ego_y, ego_v, ego_theta = ego_vehicle.get_state()
    
    # Unpack target point
    target_x, target_y,target_v,target_theta = ego_vehicle.target_state
    # Compute distance to target point
    delta_theta=np.abs(ego_theta-target_theta)
    delta_theta=min(delta_theta,2*np.pi-delta_theta)
    dist_to_target = -np.abs(ego_x - target_x) - np.abs(ego_y - target_y)-0.5*delta_theta  # Manhattan distance
    
    # Compute collision avoidance component
    collision_avoidance = 0.0

    other_corners=  other_vehicle.get_box2d()
    ego_corners = ego_vehicle.get_box2d()
    # Check for collision between ego vehicle and other vehicles
    if is_overlapping(ego_corners, other_corners):
        collision_avoidance =-1.0  # Collision detected
    else:
        collision_avoidance = 0.0
    
    # Compute safe component
    safe_component = 0.0

    other_safe_corners=  other_vehicle.get_safe_box2d()
    ego_safe_corners = ego_vehicle.get_safe_box2d()
    # Check for collision between ego vehicle and other vehicles
    if is_overlapping(ego_safe_corners, other_safe_corners):
        safe_component =-1.0  # Collision detected
    else:
        safe_component = 0.0
    
    # Compute off_road component
    off_road_component = 0.0

    for rect in road.rect:
        # Check if the ego vehicle is off the road
        rect_x= rect[0]
        rect_y= rect[1]
        rect_points=[[x,y]for x,y in zip(rect_x,rect_y)]
        if is_overlapping(ego_corners, rect_points):
            off_road_component =-1.0
            break
    
    # Compute lane component
    lane_component = 0.0
    for lane in road.laneline:
        # Check if the ego vehicle is within the lane
        lane_x= lane[0]
        lane_y= lane[1]
        lane_points=[[x,y]for x,y in zip(lane_x,lane_y)]
        if is_overlapping(ego_corners, lane_points):
            lane_component = -1.0
            break
    # down lane
    if ego_x > -road.lanewidth and ego_x < 0 and (ego_y < -road.lanewidth or ego_y > road.lanewidth):
        if ego_theta > 0 and ego_theta < np.pi:
            lane_component = -1.0
    # up lane
    elif ego_x > 0 and ego_x < road.lanewidth and (ego_y < -road.lanewidth or ego_y > road.lanewidth):
        if not (ego_theta > 0 and ego_theta < np.pi):
            lane_component = -1.0
    # right lane
    elif ego_y > -road.lanewidth and ego_y < 0 and (ego_x < -road.lanewidth or ego_x > road.lanewidth):
        if ego_theta > 0.5 * np.pi and ego_theta < 1.5 * np.pi:
            lane_component = -1.0
    # left lane
    elif ego_y > 0 and ego_y < road.lanewidth and (ego_x < -road.lanewidth or ego_x > road.lanewidth):
        if not (ego_theta > 0.5 * np.pi and ego_theta < 1.5 * np.pi):
            lane_component = -1.0

    #computer total reward
    total_reward=weights[0]* collision_avoidance + weights[1]*safe_component + weights[2]*off_road_component +weights[3]*lane_component + weights[4]*dist_to_target 
    return total_reward
    