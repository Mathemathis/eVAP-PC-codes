#!/usr/bin/env python
# coding=utf-8
import numpy as np

class EB:
    def __init__(self, id, type, rate):
        self.id = id
        self.type = type
        self.rate = rate
        self.SOC_init = 100
        self.parked_at_lane_space = None
        self.assigned_to_block = None
        self.is_charged =None
        self.energy = None
    
    def initialize_planning(self):
        self.parked_at_lane_space = dict()
        self.assigned_to_block = dict()
        self.is_charged =dict()
        self.energy=dict()
    
        
    def set_energy_at_t(self,t, e):
        self.energy[t]=e
        
    def set_situation_EB_at_t(self, t, parked_at_lane_space_at_t=None, assigned_to_block_at_t=None, is_charged_at_t=None):
        self.assigned_to_block[t]=assigned_to_block_at_t
        self.parked_at_lane_space[t]=parked_at_lane_space_at_t
        self.is_charged[t]=is_charged_at_t
    
    def is_parked_at_t(self,t):
        if t not in self.parked_at_lane_space:
            print('Error: there is no time t in list of time for parking')
        if self.parked_at_lane_space[t] != None :
            return True
        else:
            return False
        
    def is_charged_at_t(self,t):
        if t not in self.is_charged:
            print('Error: there is no time t in list of time for charging')
        return self.is_charged[t]
    
    def is_assigned_to_a_block_at_t(self,t):
        if t not in self.assigned_to_block:
            print('Error: there is no time t in list of time for planning')
        if self.assigned_to_block[t] != None :
            return True
        else:
            return False
    
    def get_situation_EB_at_t(self,t):
        if self.is_parked_at_t(t):
            if self.is_charged_at_t(t):
                return [2, self.parked_at_lane_space[t]]
            else :
                return [1, self.parked_at_lane_space[t]]
        if self.is_assigned_to_a_block_at_t(t):
            return [0 , self.assigned_to_block[t]]
        print('Error the EB '+ str(self.id) + ' is not assigned to a single operation at t '+ str(t))
    

