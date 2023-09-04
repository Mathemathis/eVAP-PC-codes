#!/usr/bin/env python
# coding=utf-8
import numpy as np
import Parameters
import matplotlib.pyplot as plt
import Vehicle
import Check_solution

class Results_planning:
    def __init__(self, model, num_chg=None, step=None):
        self.V= model.V.value
        self.L = model.L
        self.C = num_chg
        self.puit =model.B_puit
        self.x=Parameters.mask_value(model.x, model.Arcs_bus)
        # self.w=Parameters.mask_value(model.w, model.Arcs_park)
        if step=='3':
            self.u=Parameters.mask_value(model.u, model.Arcs_chg)
        self.y=Parameters.convert(model.y, model.Arcs_bus)
        self._q = Parameters.convert(model._q, model.B_bus_trips_id)
        self.q_ = Parameters.convert(model.q_, model.B_bus_trips_id)
        self.s=dict()
        for i in model.B_bus_trips_id:
            b= model.get_block[i]
            self.s[i] = b.end
        self.delta=Parameters.convert(model.delta, [(i,l) for i in model.B_bus_trips_id for l in model.L])
        self.a=Parameters.convert(model.a, model.B_bus_trips_id)
        self.t=Parameters.convert(model.t, model.B_bus_trips_id)
        if step=='2':
            self.eps=Parameters.convert(model.eps, [(i,l) for i in model.B_bus_trips_id for l in model.L])

        self.EBs_id = model.B_bus_id
        self.EBs = model.EBs
        self.B_bus_trips = model.B_bus_trips
        self.B_bus_trips_puit = model.B_bus_trips_puit
        self.B_trips = model.B_trips

        self.SOCmin=model.SOCmin
        self.SOCmax=model.SOCmax

        self.get_block = model.get_block

        self.link_block_parking_lane=dict()

        self.paths_trips=self.all_the_paths_bus()
        self.paths_park=self.all_the_paths_park()

        self.time = np.arange(0, self.puit.start)

        self.set_total_all_planning_all_EBs()

    def get_time(self):
        return self.time
    
    def set_total_planning_one_EB(self, EB):
        '''
        Put in the object EB for each time where the EB is and what it does. Il manque le cas de jointure sur la fin d'un block normalement
        '''
        EB.initialize_planning()
        i = 0
        p=0
        e=EB.SOC_init
        # EB.set_energy_at_t(0, EB.SOC_init)
        b=self.get_block[EB.id]
        for l in self.L:
            while  p< len(self.time) and self.time[p] < self.s[b.id] + self.delta[b.id, l] :
                if p!=0 and self.a[b.id]<= self.time[p]< self.a[b.id]+self.t[b.id]:
                    if self.a[b.id]<= self.time[p-1] :
                        e+=EB.rate*(self.time[p]-self.time[p-1])
                    else :
                        e+=EB.rate*(self.time[p]-self.a[b.id])
                    EB.set_situation_EB_at_t(self.time[p], parked_at_lane_space_at_t=[self.link_block_parking_lane[b.id], l], is_charged_at_t=True)
                else:
                    if p!=0 and self.a[b.id]< self.time[p-1]< self.a[b.id]+self.t[b.id]:
                        e+=EB.rate*(self.a[b.id]+self.t[b.id]-self.time[p-1])
                    EB.set_situation_EB_at_t(self.time[p], parked_at_lane_space_at_t=[self.link_block_parking_lane[b.id], l], is_charged_at_t=False)
                EB.set_energy_at_t(self.time[p], e)
                p+=1
        while i< len(self.paths_trips[EB.id])-1: #On enlève le puit
            if self.time[p-1]< self.a[b.id]+self.t[b.id]:
                    e+=EB.rate*(self.a[b.id]+self.t[b.id]-self.time[p-1])
            b=self.get_block[self.paths_trips[EB.id][i]]
            while self.time[p] < b.end :
                if self.time[p-1]>=b.start :
                    e-=b.ei*(self.time[p]-self.time[p-1])/(b.end-b.start)
                else:
                    e-=b.ei*(self.time[p]-b.start)/(b.end-b.start)
                EB.set_situation_EB_at_t(self.time[p], assigned_to_block_at_t=b.id)
                EB.set_energy_at_t(self.time[p], e)
                p+=1
            if p!=0 and self.time[p-1]<b.end:
                e-=b.ei*(b.end-self.time[p-1])/(b.end-b.start)
            for l in self.L:
                while  p< len(self.time) and self.time[p] < self.s[b.id] + self.delta[b.id, l] :
                    if self.a[b.id]<= self.time[p]< self.a[b.id]+self.t[b.id]:
                        if self.a[b.id]<= self.time[p-1] :
                            e+=EB.rate*(self.time[p]-self.time[p-1])
                        else :
                            e+=EB.rate*(self.time[p]-self.a[b.id])
                        EB.set_situation_EB_at_t(self.time[p], parked_at_lane_space_at_t=[self.link_block_parking_lane[b.id], l], is_charged_at_t=True)
                    else:
                        if p!=0 and self.a[b.id]<= self.time[p-1]< self.a[b.id]+self.t[b.id]:
                            e+=EB.rate*(self.a[b.id]+self.t[b.id]-self.time[p-1])
                        EB.set_situation_EB_at_t(self.time[p], parked_at_lane_space_at_t=[self.link_block_parking_lane[b.id], l], is_charged_at_t=False)
                    EB.set_energy_at_t(self.time[p], e)
                    p+=1
            i+=1
    
    def set_total_all_planning_all_EBs(self):
        for EB in self.EBs:
            self.set_total_planning_one_EB(EB)

    def one_path(self,id, X):
        '''
        Return all the blocks performed by bus id in the right order
        '''
        path = []
        while X[id][0]!=66666:
            path.append(X[id][0])
            id= X[id][0]
        path.append(X[id][0])
        return path

    def all_the_paths_bus(self):
        '''
        return all the path in the graph, i.e the planning assignment
        '''
        paths = dict()
        for id in self.EBs_id:
            paths[id] = self.one_path(id, self.x)
        return paths

    def all_the_paths_park(self):
        '''
        Return all the paths in the parking graph
        '''
        paths = dict()
        for k,id in enumerate(self.w[0]):
            paths[k] = [id] + self.one_path(id, self.w)
            for bid in paths[k]:
                if bid != 66666:
                    self.link_block_parking_lane[bid]=k
        return paths
    
    def plot_energies(self, number_of_EBs):
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
        for b in self.paths_trips:
            if b == id:
                nb+=1
                EB=b
            elif id in self.paths_trips[b]:
                nb+=1
                EB=b
        if nb!=1:
            print('Error : the block' + str(id)  +  ' has performed by '+ str(nb) + ' any EB')
        return EB

    def get_parking_at_t(self,t):
        '''
        Return all the EB parked in the parking lanes at t in the right order.
        '''
        res=dict()
        for k in self.paths_park:
            res[k]=list()
        for EB in self.EBs:
            if EB.is_parked_at_t(t):
                res[EB.parked_at_lane_space[t][0]].append(EB.id)
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
    
    def get_parking_at_spaces_at_t(self, parking_lane, parking_space, t):
        full = False
        for EB in self.EBs:
            if EB.is_parked_at_t(t):
                if EB.parked_at_lane_space[t]== [parking_lane, parking_space] :
                    full = True
                    return EB
        return None
