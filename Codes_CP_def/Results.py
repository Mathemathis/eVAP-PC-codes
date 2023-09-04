#!/usr/bin/env python
# coding=utf-8
import numpy as np
import Parameters
import matplotlib.pyplot as plt
import Vehicle
import Check_solution

class Results_planning:
    def __init__(self, NUM_BUS, NUM_TRIPS, NUM_LANES, NUM_SPOTS, C, RATE, param,M, EBs, Blocks, link_trip_to_EB, charge_events, parking_events_for_trip, parking_events_for_bus):
        self.NUM_BUS= NUM_BUS
        self.NUM_TRIPS = NUM_TRIPS
        self.NUM_LANES =NUM_LANES
        self.NUM_SPOTS=NUM_SPOTS
        self.C =C
        self.RATE=RATE
        self.param=param
        self.EBs=EBs
        self.Blocks=Blocks

        self.M=M
        self.link_trips_to_EB = link_trip_to_EB
        self.charge=charge_events
        self.parking_for_trip=parking_events_for_trip
        self.parking_for_bus=parking_events_for_bus

        self.lane=dict()
        self.set_link_trips_to_lane()

        self.planning=dict()
        self.set_planning()
        self.time = np.arange(0, M)

        self.set_total_all_planning_all_EBs()

    def get_time(self):
        return self.time
    
    def set_link_trip_to_EB(self, links):
        self.link_trips_to_EB=links
    
    def set_charge(self, charge_events):
        self.charge=charge_events
    
    def set_parking(self, parking_events):
        self.parking=parking_events
    
    def set_planning(self):
        for b in range(self.NUM_BUS):
            self.planning[b]=[]
            for i in range(self.NUM_TRIPS):
                if self.link_trips_to_EB[b][i]!= ():
                    self.planning[b].append([i, self.link_trips_to_EB[b][i].start])
        
        for bus in self.planning:
            if self.planning[bus]!=[]:
                self.planning[bus]=[self.planning[bus][i][0] for i in np.argsort(np.array(self.planning[bus]), axis=0)[:,1]]
   
    def set_link_trips_to_lane(self):
        for v in range(self.NUM_LANES):
            for i in range(self.NUM_TRIPS):
                if self.parking_for_trip[v][i]!= ():
                    self.lane[i]=v

    def set_total_planning_one_EB(self, EB,b):
        '''
        Put in the object EB for each time where the EB is and what it does. Il manque le cas de jointure sur la fin d'un block normalement
        '''
        EB.initialize_planning()
        i = 0
        p=0
        e=EB.SOC_init
        while  p< len(self.time) and self.time[p] <= self.parking_for_bus[b].end :
            EB.set_situation_EB_at_t(self.time[p], parked_at_lane_space_at_t=b%self.NUM_LANES, is_charged_at_t=False, parked_for_trip_at_t=[b])
            EB.set_energy_at_t(self.time[p], e)
            p+=1
        while i< len(self.planning[b]):
            trip= self.planning[b][i]
            block=self.link_trips_to_EB[b][trip]
            ei = self.Blocks[trip].ei/self.param
            # print(ei, e)
            while self.time[p] <= block.end :
                e-=ei*(self.time[p]-self.time[p-1])/(block.end-block.start)
                EB.set_situation_EB_at_t(self.time[p], assigned_to_block_at_t=trip)
                EB.set_energy_at_t(self.time[p], e)
                p+=1
            lane=self.lane[trip]
            while  p< len(self.time) and self.time[p] <= self.parking_for_trip[lane][trip].end :
                if self.charge[b][trip].start< self.time[p]<= self.charge[b][trip].end:
                    e+=(self.RATE/self.param)*(self.time[p]-self.time[p-1])
                    EB.set_situation_EB_at_t(self.time[p], parked_at_lane_space_at_t=lane, is_charged_at_t=True, parked_for_trip_at_t=[b,trip])
                else:
                    EB.set_situation_EB_at_t(self.time[p], parked_at_lane_space_at_t=lane, is_charged_at_t=False, parked_for_trip_at_t=[b,trip])
                EB.set_energy_at_t(self.time[p], e)
                p+=1
            i+=1
    
    def set_total_all_planning_all_EBs(self):
        for b in range(len(self.EBs)):
            self.set_total_planning_one_EB(self.EBs[b], b)
    
    def plot_energies(self, number_of_EBs=500):
        '''
        Plot the evolution of EBs' energy
        '''
        fig = plt.figure(figsize=(15, 8))
        for EB in self.EBs[:number_of_EBs]:
            plt.plot(self.time, [EB.energy[t] for t in self.time])

    def get_bus(self,id):
        '''
        Return the EB assigned to block id and check if all blocks are performed by a unique EB
        '''
        nb=0
        EB=None
        # print(self.planning)
        for b in self.planning:
            if id in self.planning[b]:
                nb+=1
                EB=self.EBs[b].id
        if nb!=1:
            print('Error : the block' + str(id)  +  ' has performed by '+ str(nb) + ' any EB')
        return EB

    def get_parking_at_t(self,t):
        '''
        Return all the EB parked in the parking lanes at t in the right order.
        '''
        res=dict()
        for k in range(self.NUM_LANES):
            res[k]=list()
        for EB in self.EBs:
            if EB.is_parked_at_t(t):
                res[EB.parked_at_lane_space[t]].append(EB.id)
        return res
    
    def get_charging_at_t(self,t):
        '''
        Return all the EB charged in the parking lanes at t in the right order.
        '''
        res=[]
        for EB in self.EBs:
            if EB.is_charged_at_t(t):
                res.append(EB.id)
        return res
    
    def get_EBs_parked_at_t(self,t, v):
        res=[]
        index=[]
        beginning_horizon=False
        finishing_horizon=False

        doublon_start=False
        doublon_end=False

        for EB_index, EB in enumerate(self.EBs):
            if EB.is_parked_at_t(t) and EB.parked_at_lane_space[t]==v:
                if len(EB.parked_for_trip[t])==1:
                    beginning_horizon=True
                    res.append([EB, 0, self.parking_for_bus[EB_index].end])
                    index.append([EB.id, 0, self.parking_for_bus[EB_index].end])
                else:
                    b,i = EB.parked_for_trip[t]
                    res.append([EB, self.parking_for_trip[v][i].start, self.parking_for_trip[v][i].end])
                    index.append([EB.id, self.parking_for_trip[v][i].start, self.parking_for_trip[v][i].end])
                    if self.parking_for_trip[v][i].end==self.M:
                        finishing_horizon=True
        for i in range(len(res)):
            if [res[k][1] for k in range(len(res))].count(res[i][1])!=1:
                doublon_start=True
            if [res[k][2] for k in range(len(res))].count(res[i][2])!=1:
                doublon_end=True

        if res!=[]:
                res1=[res[i][0] for i in np.argsort(np.array(index), axis=0)[:,1]]
                res2=[res[i][0] for i in np.argsort(np.array(index), axis=0)[:,2]]

                index1=[index[i][0] for i in np.argsort(np.array(index), axis=0)[:,1]]
                index2=[index[i][0] for i in np.argsort(np.array(index), axis=0)[:,2]]
                
                if beginning_horizon:
                    return res2
                if finishing_horizon:
                    return res1
                if doublon_end:
                    return res1
                if doublon_start:
                    return res2
                if index1!=index2:
                    print('Error'+ str(t), index1, index2)
                return res1
        else: 
            return res
    
    def get_parking_at_spaces_at_t(self, parking_lane, parking_space, t):
        full = False
        Parked_lane = self.get_EBs_parked_at_t(t, parking_lane)
        if parking_space>= len(Parked_lane):
            return None
        else: 
            return Parked_lane[parking_space]