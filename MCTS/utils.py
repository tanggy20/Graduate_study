import math
import random
import numpy as np
from enum import Enum
from typing import List, Optional, Tuple



class Action(Enum):
    """Enum of action sets for vehicle."""
    MAINTAIN = [0, 0]              # maintain
    TURNLEFT = [0, math.pi / 4]    # turn left
    TURNRIGHT = [0, -math.pi / 4]  # turn right
    ACCELERATE = [2.5, 0]          # accelerate
    DECELERATE = [-2.5, 0]         # decelerate
    BRAKE = [-5, 0]                # brake

ActionList = [Action.MAINTAIN, Action.TURNLEFT, Action.TURNRIGHT,
              Action.ACCELERATE, Action.DECELERATE, Action.BRAKE]


class State:
    def __init__(self, x=0, y=0, yaw=0, v=0) -> None:
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

    def to_list(self) -> List:
        return [self.x, self.y, self.yaw, self.v]


class StateList:
    def __init__(self, state_list = None) -> None:
        self.state_list: List[State] = state_list if state_list is not None else []

    def append(self, state: State) -> None:
        self.state_list.append(state)

    def reverse(self) -> 'StateList':
        self.state_list = self.state_list[::-1]

        return self

    def expand(self, excepted_len: int, expand_state: Optional[State] = None) -> None:
        cur_size = len(self.state_list)
        if cur_size >= excepted_len:
            return
        else:
            if expand_state is None:
                expand_state = self.state_list[-1]
            for _ in range(excepted_len - cur_size):
                self.state_list.append(expand_state)

    def to_list(self, is_vertical: bool = True) -> List:
        if is_vertical is True:
            states = [[],[],[],[]]
            for state in self.state_list:
                states[0].append(state.x)
                states[1].append(state.y)
                states[2].append(state.yaw)
                states[3].append(state.v)
        else:
            states = []
            for state in self.state_list:
                states.append([state.x, state.y, state.yaw, state.v])

        return states

    def __len__(self) -> int:
        return len(self.state_list)

    def __getitem__(self, key: int) -> State:
        return self.state_list[key]

    def __setitem__(self, key: int, value: State) -> None:
        self.state_list[key] = value

class Node:
    MAX_LEVEL: int = 6
    cal_value_callback= None

    def __init__(self,state=State(),level=0,parent: Optional["Node"] = None,action: Optional[Action] = None,others:StateList=StateList(),goal:State=State()) -> None:
        self.state = state
        self.value:float = 0.0
        self.reward:float = 0.0
        self.visits:int = 0
        self.action:Action = action
        self.parent:Node = parent
        self.current_level:int = level
        self.goal_pos:State = goal
        self.actions=[]

        self.children:List["Node"] = []
        self.others:StateList = others

    
    @property
    def is_terminal(self) -> bool:
        return self.current_level >= Node.MAX_LEVEL 
    
    @property
    def is_fully_expanded(self) -> bool:
        return len(self.children) == len(ActionList)
    
    @staticmethod
    def initialize(max_level:int,callback) -> None:
        Node.MAX_LEVEL = max_level
        Node.cal_value_callback = callback
    
    def add_child(self,next_action,dt,others:StateList) -> "Node":
        next_state = kinematic_propagate(self.state, next_action.value, dt)
        child = Node(state=next_state,level=self.current_level+1,parent=self,action=next_action,others=others,goal=self.goal_pos)
        child.actions=self.actions+[next_action]
        Node.cal_value_callback(child,self.value)
        self.children.append(child)
        return child
    
    def next_node(self,dt,others):
        next_action=random.choice(ActionList)   
        next_state = kinematic_propagate(self.state, next_action.value, dt)
        child = Node(state=next_state,level=self.current_level+1,parent=self,action=next_action,others=others,goal=self.goal_pos)
        Node.cal_value_callback(child,self.value)

        return child
    
    def __repr__(self):
        return (f"children: {len(self.children)}, visits: {self.visits}, "
                f"reward: {self.reward}, actions: {self.actions}")


def has_overlap(box2d_0, box2d_1) -> bool:
    total_sides = []
    for i in range(1, len(box2d_0[0])):
        vec_x = box2d_0[0][i] - box2d_0[0][i - 1]
        vec_y = box2d_0[1][i] - box2d_0[1][i - 1]
        total_sides.append([vec_x, vec_y])
    for i in range(1, len(box2d_1[0])):
        vec_x = box2d_1[0][i] - box2d_1[0][i - 1]
        vec_y = box2d_1[1][i] - box2d_1[1][i - 1]
        total_sides.append([vec_x, vec_y])

    for i in range(len(total_sides)):
        separating_axis = [-total_sides[i][1], total_sides[i][0]]

        vehicle_min = np.inf
        vehicle_max = -np.inf
        for j in range(0, len(box2d_0[0])):
            project = separating_axis[0] * box2d_0[0][j] + separating_axis[1] * box2d_0[1][j]
            vehicle_min = min(vehicle_min, project)
            vehicle_max = max(vehicle_max, project)

        box2d_min = np.inf
        box2d_max = -np.inf
        for j in range(0, len(box2d_1[0])):
            project = separating_axis[0] * box2d_1[0][j] + separating_axis[1] * box2d_1[1][j]
            box2d_min = min(box2d_min, project)
            box2d_max = max(box2d_max, project)

        if vehicle_min > box2d_max or box2d_min > vehicle_max:
            return False

    return True




def kinematic_propagate(state: State, act: List[float], dt: float) -> State:
    next_state = State()
    acc, omega = act[0], act[1]

    
    next_state.v = state.v + acc * dt
    next_state.yaw = state.yaw + omega * dt
    next_state.yaw = np.mod(next_state.yaw, 2 * np.pi)  # Limit to [0, 2pi)
    next_state.x = state.x + state.v * np.cos(state.yaw) * dt
    next_state.y = state.y + state.v * np.sin(state.yaw) * dt


    

    return next_state