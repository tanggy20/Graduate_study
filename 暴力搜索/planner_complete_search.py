import numpy as np
import itertools
from reward import compute_reward
import Vehicle
from Env_road import EnvRoads
import math
import random


class Kplanner:
    def __init__(self, dt, time,gamma=0.9):
        self.dt = dt  # Time step for the planner
        self.time = time  # Number of steps to plan ahead
        self.actions = []  # List to store actions
        self.state = None  # Current state of the planner
        self.gamma = gamma  # Discount factor for future rewards    
    
    def generate_action_sequences(self, action_set):
        """
        Generate action sequences for the ego vehicle.
        
        Parameters:
        action_set (list): List of possible actions.
        
        Returns:
        list: List of action sequences.
        """
        action_sequences = []
        for action in itertools.product(action_set, repeat=int(self.time/self.dt)):
            action_sequences.append(action)
        return action_sequences
    
    # def generate_action_sequences(self, action_set, ego_vehicle, other_vehicle, road, weights):
    #     return [self.mcts_search(ego_vehicle, other_vehicle, action_set, road, weights)]

    

    def level0_decision(self,ego_vehicle,other_vehicle,action_set,road,weights):
        """
        Level 0 decision-making function.
        This function is a placeholder and should be implemented based on the specific requirements of the planner.
        """
        # Placeholder for decision-making logic
        # This should return the best action based on the current state and action set
        best_reward=-np.inf
        best_sequence=None
        action_sequences = self.generate_action_sequences(action_set)
        # Iterate through all possible action sequences
        for action_sequence in action_sequences:
            ego_vehicle_copy = ego_vehicle.copy()
            other_vehicle_copy = other_vehicle.copy()
            # Initialize cumulative reward and gamma
            cum_reward=0
            gamma_t=1.0
            # Reset ego vehicle state
            for i in range(len(action_sequence)):
                # Update the ego vehicle state based on the action taken    
                ego_vehicle_copy.Update(action_sequence[i])
                # Update the other vehicle state based on the predicted action
                # print(f"ego_vehicle_copy state: {ego_vehicle_copy.get_state()}")
                # print(f"other_vehicle_copy state: {other_vehicle_copy.get_state()}")
                # Compute reward for the action sequence
                temp_reward=compute_reward(ego_vehicle_copy, other_vehicle_copy, road, weights)
                cum_reward+=temp_reward*gamma_t
                gamma_t*=self.gamma
            # Check if the current action sequence has the best reward
            if(cum_reward>best_reward):
                best_reward=cum_reward
                best_sequence=action_sequence
        # Return the best action sequence and its corresponding reward
        return best_sequence, best_reward
    

    def levelk_decision(self,ego_vehicle,other_vehicle,level,action_set,road,weights):
        # Placeholder for decision-making logic
        # This should return the best action based on the current state and action set
        best_reward=-np.inf
        best_sequence=None
        ego_action_sequences = self.generate_action_sequences(action_set)
        if level == 0:
            return self.level0_decision(ego_vehicle, other_vehicle, action_set, road, weights)
        else:
            pred_f,_=self.levelk_decision(other_vehicle, ego_vehicle, level - 1, action_set, road, weights)
            for ego_seq in ego_action_sequences:
                ego_vehicle_copy = ego_vehicle.copy()
                other_vehicle_copy = other_vehicle.copy()
                cum_reward=0
                gamma_t=1.0
                for i in range(len(ego_seq)):
                    # Update the ego vehicle state based on the action taken    
                    ego_vehicle_copy.Update(ego_seq[i])
                    # print(f"ego_vehicle_copy state: {ego_vehicle_copy.get_state()}")
                    # Update the other vehicle state based on the predicted action
                    other_vehicle_copy.Update(pred_f[i])
                    # print(f"other_vehicle_copy state: {other_vehicle_copy.get_state()}")
                    # Compute reward for the action sequence
                    temp_reward=compute_reward(ego_vehicle_copy, other_vehicle_copy, road, weights)
                    cum_reward+=temp_reward*gamma_t
                    gamma_t*=self.gamma
                # Check if the current action sequence has the best reward
                if(cum_reward>best_reward):
                    best_reward=cum_reward
                    best_sequence=ego_seq
            # Return the best action sequence and its corresponding reward
            return best_sequence,best_reward

