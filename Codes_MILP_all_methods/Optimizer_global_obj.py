#!/usr/bin/env python
# coding=utf-8
import numpy as np
import Parameters
import Vehicle
import pyomo.environ as pyo
from pyomo.opt import SolverStatus, TerminationCondition


class Planning_model:
    def __init__(self, Blocks, EBs, type, rates, SOCmax, SOCmin, C, nb_parking_lanes, nb_spaces):
        Trips =[Blocks[i] for i in range(len(Blocks)) if Blocks[i].type == type]
        Puit=Parameters.create_puit(Trips)
        Trips_id = Parameters.get_all_id(Trips)
        EBs =[EBs[i] for i in range(len(EBs)) if EBs[i].type == type]
        EBs_id =[v.id for v in EBs]
        init_block = Parameters.create_init_block(EBs)


        self.model=pyo.ConcreteModel()
        self.model.V = pyo.Param(initialize=nb_parking_lanes)
        self.model.cardL = pyo.Param(initialize=nb_spaces)
        self.model.L = pyo.Set(initialize=[l+1 for l in range(0,nb_spaces)])
        self.model.rate= pyo.Param(initialize=rates[type])
        self.model.SOCmax=pyo.Param(initialize=SOCmax)
        self.model.SOCmin=pyo.Param(initialize=SOCmin)
        self.model.C=pyo.Set(initialize=C)

        self.model.EBs = EBs
        self.model.B_puit= Puit
        self.model.B_puit_id= Puit.id
        self.model.B_bus = pyo.Set(initialize=init_block)
        self.model.B_bus_id = pyo.Set(initialize=EBs_id)
        self.model.B_trips = pyo.Set(initialize=Trips)
        self.model.B_trips_id = pyo.Set(initialize=Trips_id)
        self.model.B_bus_trips_id = pyo.Set(initialize=Trips_id+EBs_id)
        self.model.B_bus_trips = pyo.Set(initialize=Trips+init_block)
        self.model.B_bus_trips_puit_id = pyo.Set(initialize=Trips_id+EBs_id+[Puit.id])
        self.model.B_bus_trips_puit = pyo.Set(initialize=Trips+init_block+[Puit])

        self.model.get_block=dict()
        for b in self.model.B_bus_trips_puit:
            self.model.get_block[b.id]=b

        self.model.Arcs_bus = pyo.Set(initialize=Parameters.define_all_arc_bus(init_block+Trips+[Puit], int(1.5*len(EBs_id))))
        Arcs_init, park_init = Parameters.define_arc_park_init_optimal(init_block, nb_parking_lanes, nb_spaces)
        self.model.Arcs_park = pyo.Set(initialize=Parameters.define_all_arc_park(init_block+Trips+[Puit], 4*self.model.V)+ Arcs_init)
        Head_parking_lanes = [a[1] for a in Arcs_init if a[0]==0]
        self.model.Break_symetry_init=pyo.Set(initialize=[(Head_parking_lanes[i], Head_parking_lanes[i+1]) for i in range(len(Head_parking_lanes)-1) if len(park_init[i])==len(park_init[i+1])])
        # self.model.M = Parameters.get_optimal_M_parking(self.model.B_bus_trips_puit, self.model.Arcs_bus, self.model.Arcs_park)
        self.model.M = dict()
        for (i,j) in self.model.Arcs_park:
            if j != 66666 and i != 0:
                self.model.M[i,j]=Parameters.get_optimal_M_parking_one_block(i, j, self.model)

        if len(self.model.Break_symetry_init)>self.model.V:
            return 'Error in the initialization of the parking'
        
        self.model.Cost_x=dict()
        for (i,j) in self.model.Arcs_bus:
            bi= self.model.get_block[i]
            bj= self.model.get_block[j]
            self.model.Cost_x[i,j]=np.square(bj.start-bi.end - 4*60)
        
        self.model.Cost_w=dict()
        for (i,j) in self.model.Arcs_park:
            if i !=0:
                bi= self.model.get_block[i]
                bj= self.model.get_block[j]
                self.model.Cost_w[i,j]=np.square(bj.end-bi.end)
        
        self.model.B_chg_id= pyo.Set(initialize=self.model.B_bus_trips_id)
        self.model.Arcs_chg = pyo.Set(initialize=Parameters.create_charging_graph(self.model))
        # print(self.model.Arcs_chg.data())
        self.model.M_chg =dict()
        for (i,j) in self.model.Arcs_chg:
            if j != 66666 and i != 0:
                # self.model.M_chg[i,j]= Parameters.get_block(self.model.x[i][0],self.model.B_bus_trips_puit).start - Parameters.get_block(j,self.model.B_bus_trips_puit).end
                self.model.M_chg[i,j]=10000
                # self.model.M_chg[i,j]= Parameters.get_optimal_M_parking_one_block(i, j, self.model)
        
        # Arcs_driver=[]
        # for (i,j) in self.model.Arcs_chg:
        #     if i!=0 and j!=66666:
        #         for l in self.model.L:
        #             for k in self.model.L:
        #                 Arcs_driver.append([(i,l), (j, k)])

        # for i in self.model.B_bus_trips_id:
        #     for l in self.model.L:
        #             Arcs_driver.append([(i,l), (66666,66666)])
        #             Arcs_driver.append([(0,0),(i,l)])
        #             for k in self.model.L:
        #                     if k>l:
        #                         Arcs_driver.append([(i,l), (i,k)])
        
        # self.model.Arcs_driver=pyo.Set(initialize=Arcs_driver)
        # print(self.model.Arcs_driver.data())
        
        self.model.Cost_u=dict()
        for (i,j) in self.model.Arcs_chg:
            if i !=0:
                bi= self.model.get_block[i]
                bj= self.model.get_block[j]
                self.model.Cost_u[i,j]=np.square(bj.end-bi.end-150)
        # print('Nodes and Edges are created')

    def create_variables(self):
        self.model.x = pyo.Var(self.model.Arcs_bus, within=pyo.Binary)
        self.model.y = pyo.Var(self.model.Arcs_bus, within = pyo.NonNegativeReals)
        self.model._q = pyo.Var(self.model.B_bus_trips_id, within=pyo.NonNegativeReals)
        self.model.q_ = pyo.Var(self.model.B_bus_trips_id, within=pyo.NonNegativeReals)
        # self.model.s = pyo.Var(self.model.B_bus_trips_id, within = pyo.NonNegativeReals)
        self.model.delta = pyo.Var(self.model.B_bus_trips_id, self.model.L, within=pyo.NonNegativeReals)
        self.model.a = pyo.Var(self.model.B_bus_trips_id, within = pyo.NonNegativeReals)
        self.model.t = pyo.Var(self.model.B_bus_trips_id, within=pyo.NonNegativeReals)
        self.model.w = pyo.Var(self.model.Arcs_park, within=pyo.Binary)
        self.model.u = pyo.Var(self.model.Arcs_chg, within=pyo.Binary)
        # print(len(self.model.u), len(self.model.x), len(self.model.w))
        # self.model.eps = pyo.Var(self.model.B_bus_trips_id, self.model.L, within=pyo.Binary)
        # self.model.alpha = pyo.Var(self.model.B_bus_trips_id, self.model.L, within=pyo.Binary)
        # self.model.beta = pyo.Var(self.model.Arcs_driver,  within=pyo.Binary)

        # print('Variables are created')
    
    def create_obj_and_constraints(self):
        obj=sum([self.model.x[i,j]* self.model.Cost_x[i,j] for (i,j) in self.model.Arcs_bus])
        obj+=sum([self.model.w[i,j]* self.model.Cost_w[i,j] for (i,j) in self.model.Arcs_park if i!=0])
        # obj+=sum([self.model.eps[i,l] for i in self.model.B_bus_trips_id for l in self.model.L])
        # obj+=sum([self.model.u[i,j]* self.model.Cost_u[i,j] for (i,j) in self.model.Arcs_chg if i!=0])
        self.model.obj = pyo.Objective(expr= obj , sense=pyo.minimize)

        self.define_constraint_flow_bus_out()
        self.define_constraint_flow_bus_in()
        self.define_constraint_energy()
        self.define_constraint_energy_post()
        self.define_constraint_energy_before()
        self.define_constraint_energy_init()
        self.define_constraint_energy_min()
        self.define_constraint_charge()
        # self.define_constraint_start_parking()
        self.define_constraint_end_parking()
        self.define_constraint_start_charging()
        self.define_constraint_end_charging()
        self.define_constraint_flow_park_out()
        self.define_constraint_flow_park_in()
        self.define_constraint_parking_order()
        self.define_constraint_parking_order_init()
        # self.define_constraint_number_parking_lanes()
        self.define_constraint_increase_parking()
        self.define_constraint_break_symetry()
        self.define_constraint_flow_chg_out()
        self.define_constraint_flow_chg_in()
        self.define_constraint_number_chargers()
        self.define_constraint_rule_chargers_order()
        # self.define_constraint_flow_driver_out()
        # self.define_constraint_flow_driver_in()
        # self.define_constraint_number_drivers()
        # self.define_constraint_rule_drivers_order()
        # self.define_constraint_rule_enter_parking()
        # self.define_constraint_order_shift_1()
        # self.define_constraint_order_shift_2()

        # print('Constraints and Objective are created')
    
    def define_constraint_flow_bus_out(self):

        def rule_flow_bus_out(_b,i):
            sum = 0
            nb=0
            for (k,j) in self.model.Arcs_bus:
                if k==i:
                    sum+=self.model.x[i,j]
                    nb+=1
            if nb!=0:
                return sum==1
            else :
                return pyo.Constraint.Skip
            
        self.model.C1 = pyo.Constraint(self.model.B_bus_trips_id, rule = rule_flow_bus_out)

    def define_constraint_flow_bus_in(self):

        def rule_flow_bus_in(_b,i):
            sum = 0
            nb=0
            for (j,k) in self.model.Arcs_bus:
                if k==i:
                    sum+=self.model.x[j,i]
                    nb+=1
            if nb!=0:
                return sum==1
            else :
                return pyo.Constraint.Skip
        
        self.model.C2 = pyo.Constraint(self.model.B_trips_id, rule = rule_flow_bus_in)

    def define_constraint_energy(self):
            
        def rule_energy(_b,i,j):
            return self.model.y[i,j] <= self.model.SOCmax*self.model.x[i,j]
        
        self.model.C3 = pyo.Constraint(self.model.Arcs_bus, rule = rule_energy) 
    
    def define_constraint_energy_post(self):

        def rule_energy_post(_b, i):
            sum =0
            nb=0
            for (k,j) in self.model.Arcs_bus:
                if k==i:
                    sum+=self.model.y[i,j]
                    nb+=1
            if nb!=0:
                return sum == self.model._q[i]
            else :
                return pyo.Constraint.Skip
            
        self.model.C4 = pyo.Constraint(self.model.B_bus_trips_id, rule = rule_energy_post) 

    def define_constraint_energy_before(self):

        def rule_energy_before(_b, i):
            sum = 0
            nb=0
            b= self.model.get_block[i]
            for (j,k) in self.model.Arcs_bus:
                if k==i:
                    sum+=self.model.y[j,i]
                    nb+=1
            if nb!=0:
                return sum == self.model.q_[i]+b.ei
            else:
                return pyo.Constraint.Skip
            
        self.model.C5 = pyo.Constraint(self.model.B_trips_id, rule = rule_energy_before)

    def define_constraint_energy_init(self):

        def rule_energy_bus_init(_b, i):
            return self.model._q[i]==self.model.SOCmax
        
        self.model.C5bis = pyo.Constraint(self.model.B_bus_id, rule = rule_energy_bus_init)

    def define_constraint_energy_min(self):

        def rule_energy_min(_b, i):
            return self.model.q_[i] >= self.model.SOCmin
        
        self.model.C6 = pyo.Constraint(self.model.B_bus_trips_id, rule = rule_energy_min)

    def define_constraint_charge(self):

        def rule_charge(_b,i):
            return self.model._q[i] - self.model.q_[i]==self.model.rate*self.model.t[i]
        
        self.model.C7 = pyo.Constraint(self.model.B_bus_trips_id, rule = rule_charge)

    # def define_constraint_start_parking(self):

    #     def rule_start_parking(_b,i):
    #         b= self.model.get_block[i]
    #         return self.model.s[i] == b.end
        
    #     self.model.C8 = pyo.Constraint(self.model.B_bus_trips_id, rule = rule_start_parking)

    def define_constraint_end_parking(self):

        def rule_end_parking(_b, i):
            sum = 0
            nb=0
            for (k,j) in self.model.Arcs_bus:
                if k==i:
                    bj= self.model.get_block[j]
                    sum+=self.model.x[i,j]*bj.start
                    nb+=1
            if nb!=0:
                b= self.model.get_block[i]
                return sum == b.end + self.model.delta[i, len(self.model.L)]
            else :
                return pyo.Constraint.Skip
            
        self.model.C9 = pyo.Constraint(self.model.B_bus_trips_id, rule = rule_end_parking)

    def define_constraint_start_charging(self):

        def rule_start_charging(_b, i):
            b= self.model.get_block[i]
            return b.end <= self.model.a[i]
        
        self.model.C10 = pyo.Constraint(self.model.B_bus_trips_id, rule = rule_start_charging)

    def define_constraint_end_charging(self):

        def rule_end_charging(_b, i):
            b= self.model.get_block[i]
            return b.end + self.model.delta[i, len(self.model.L)] >= self.model.a[i]+self.model.t[i]

        self.model.C11 = pyo.Constraint(self.model.B_bus_trips_id, rule = rule_end_charging)

    def define_constraint_flow_park_out(self):

        def rule_flow_park_out(_b,i):
            sum = 0
            for (k,j) in self.model.Arcs_park:
                if k==i:
                    sum+=self.model.w[i,j]
            return sum==1
        
        self.model.C14 = pyo.Constraint(self.model.B_bus_trips_id, rule = rule_flow_park_out) 

    def define_constraint_flow_park_in(self):

        def rule_flow_park_in(_b,i):
            sum = 0
            nb=0
            for (j,k) in self.model.Arcs_park:
                if k==i:
                    nb+=1
                    sum+=self.model.w[j,i]
            if nb!=0:
                return sum==1
            else:
                return pyo.Constraint.Skip
            
        self.model.C13 = pyo.Constraint(self.model.B_bus_trips_id, rule = rule_flow_park_in) 

    def define_constraint_parking_order(self):

        def rule_park_order(_b,i,j, l):
            if j == 66666 or i == 0:
                return pyo.Constraint.Skip
            if l!= len(self.model.L):
                b= self.model.get_block[i]
                bj= self.model.get_block[j]
                return b.end+ self.model.delta[i, l+1]<= bj.end+ self.model.delta[j, l]+self.model.M[i,j]*(1-self.model.w[i,j])
            else:
                return pyo.Constraint.Skip
        self.model.C15 = pyo.Constraint(self.model.Arcs_park, self.model.L, rule = rule_park_order)

    def define_constraint_parking_order_init(self):

        def rule_park_order_init(_b,i,j):
            if j == 66666 or i == 0:
                return pyo.Constraint.Skip
            b= self.model.get_block[i]
            bj= self.model.get_block[j]
            return b.end+ self.model.delta[i, 1]<= bj.end+self.model.M[i,j]*(1-self.model.w[i,j])
        
        self.model.C15bis = pyo.Constraint(self.model.Arcs_park, rule = rule_park_order_init) 

    def define_constraint_number_parking_lanes(self):

        def rule_number_parking_lanes(_b):
            sum=0
            for vid in self.model.B_bus_id:
                sum+=self.model.w[0, vid]
            return sum <= self.model.V
        
        self.model.C16 = pyo.Constraint(rule = rule_number_parking_lanes)

    def define_constraint_increase_parking(self):

        def rule_increase(_b, i, l):
            if l==1:
                return 0<= self.model.delta[i,l]
            elif l <= len(self.model.L):
                return self.model.delta[i, l-1] <= self.model.delta[i, l]
            else:
                return pyo.Constraint.Skip
        
        self.model.C17 = pyo.Constraint(self.model.B_bus_trips_id, self.model.L, rule = rule_increase)

    def define_constraint_break_symetry(self):
    
        def rule_break_symetry(_b, i, j):
            b= self.model.get_block[i]
            bj= self.model.get_block[j]
            return b.end+self.model.delta[i, len(self.model.L)] <= bj.end+self.model.delta[j, len(self.model.L)]
        
        self.model.C18 = pyo.Constraint(self.model.Break_symetry_init, rule = rule_break_symetry)
    
    def define_constraint_flow_chg_out(self):
        
        def rule_flow_chg_out(_b,i):
            sum = 0
            for (k,j) in self.model.Arcs_chg:
                if k==i:
                    sum+=self.model.u[i,j]
            return sum==1
        
        self.model.C19 = pyo.Constraint(self.model.B_chg_id, rule = rule_flow_chg_out) 

    def define_constraint_flow_chg_in(self):

        def rule_flow_chg_in(_b,i):
            sum = 0
            nb=0
            for (j,k) in self.model.Arcs_chg:
                if k==i:
                    nb+=1
                    sum+=self.model.u[j,i]
            if nb!=0:
                return sum==1
            else:
                return pyo.Constraint.Skip
            
        self.model.C20 = pyo.Constraint(self.model.B_chg_id, rule = rule_flow_chg_in) 

    # def define_constraint_number_chargers(self):
    
    #     def rule_number_chargers(_b):
    #         sum = 0
    #         for (k,j) in self.model.Arcs_chg:
    #             if k==0:
    #                 sum+=self.model.u[0,j]
    #         return sum <= self.model.C
        
    #     self.model.C21 = pyo.Constraint(rule = rule_number_chargers)

    def define_constraint_number_chargers(self):

        sum = 0
        for (k,j) in self.model.Arcs_chg:
            if k==0:
                sum+=self.model.u[0,j]
        
        self.model.C21list = pyo.ConstraintList()

        for c in self.model.C:
            self.model.C21list.add(sum <= c)
    
    def define_constraint_rule_chargers_order(self):

        def rule_chargers_order(_b, i, j):
            if i ==0 or j == 66666:
                return pyo.Constraint.Skip
            else:
                return self.model.a[i]+self.model.t[i] <= self.model.a[j] + self.model.M_chg[i,j] * (1-self.model.u[i,j])
        
        self.model.C22 = pyo.Constraint(self.model.Arcs_chg, rule = rule_chargers_order)
    
    def define_constraint_flow_driver_out(self):
        
        def rule_flow_driver_out(_b,i, l):
            sum = 0
            for (i1, l1, j, k) in self.model.Arcs_driver:
                if i1==i and l1==l:
                    sum+=self.model.beta[i,l, j, k]
            return sum<=1
        
        self.model.C25 = pyo.Constraint(self.model.B_bus_trips_id, self.model.L, rule = rule_flow_driver_out) 

    def define_constraint_flow_driver_in(self):

        def rule_flow_driver_in(_b,i, l):
            sum_out = 0
            sum_in=0
            for (i1, l1, j1, k1) in self.model.Arcs_driver:
                if i1==i and l1==l:
                    sum_out+=self.model.beta[i,l, j1, k1]
                if j1==i and k1==l:
                    sum_in+=self.model.beta[i1,l1, i, l]
            return sum_in==sum_out
            
        self.model.C26 = pyo.Constraint(self.model.B_bus_trips_id, self.model.L, rule = rule_flow_driver_in) 

    def define_constraint_number_drivers(self):
    
        def rule_number_chargers(_b):
            sum = 0
            for (i1, l1, j, k) in self.model.Arcs_driver:
                if i1==0:
                    sum+=self.model.beta[0, 0, j, k]
            return sum<=4
        
        self.model.C27 = pyo.Constraint(rule = rule_number_chargers)
    
    def define_constraint_rule_drivers_order(self):

        def rule_drivers_order(_b, i, l, j, k):
            if i ==0 or j == 66666:
                return pyo.Constraint.Skip
            else:
                b= self.model.get_block[i]
                bj= self.model.get_block[j]
                return b.end+self.model.delta[i,l]+5 <= bj.end+self.model.delta[j,k] + 10000 * (1-self.model.beta[i,l, j, k])
        
        self.model.C28 = pyo.Constraint(self.model.Arcs_driver, rule = rule_drivers_order)

    
    def define_constraint_rule_enter_parking(self):
    
        def rule_enter_parking(_b, i,):
            sum=0
            for l in self.model.L:
                sum+=self.model.alpha[i,l]
            return sum==1
        self.model.C29 = pyo.Constraint(self.model.B_bus_trips_id, rule = rule_enter_parking)
    
    
def deactivate_charging_problem(model):
        model.u.fixed=True
        model.C19.deactivate()
        model.C20.deactivate()
        for c in range(len(model.C)):
            model.C21list[c+1].deactivate()
        model.C22.deactivate()

def activate_charging_problem(model, index_chg):
        model.u.fixed=False
        model.C19.activate()
        model.C20.activate()
        for c in range(len(model.C)):
            model.C21list[c+1].deactivate()
        model.C21list[index_chg].activate()
        model.C22.activate()

def deactivate_assignment_problem(model):
        for (i,j) in model.Arcs_bus:
            model.x[i,j].fixed=True
        model.C1.deactivate()
        model.C2.deactivate()

def activate_assignment_problem(model):
        for (i,j) in model.Arcs_bus:
            model.x[i,j].fixed=False
        model.C1.activate()
        model.C2.activate()

def deactivate_parking_problem_bis(model):
        for (i,j) in model.Arcs_park:
            model.w[i,j].fixed=True
        # for i in model.B_bus_trips_id:
        #     model.s[i].fixed=True
        #     for l in model.L:
        #         model.delta[i,l].fixed=True
        # model.C8.deactivate()
        # model.C9.deactivate()
        model.C13.deactivate()
        model.C14.deactivate()
        model.C15.deactivate()
        model.C15bis.deactivate()
        model.C17.deactivate()
        model.C18.deactivate()

def deactivate_parking_problem(model):
        for (i,j) in model.Arcs_park:
            model.w[i,j].fixed=True
        for i in model.B_bus_trips_id:
            # model.s[i].fixed=True
            for l in model.L:
                model.delta[i,l].fixed=True
        # model.C8.deactivate()
        model.C9.deactivate()
        model.C13.deactivate()
        model.C14.deactivate()
        model.C15.deactivate()
        model.C15bis.deactivate()
        model.C17.deactivate()
        model.C18.deactivate()

def activate_parking_problem(model):
        for (i,j) in model.Arcs_park:
            model.w[i,j].fixed=False
        for i in model.B_bus_trips_id:
            # model.s[i].fixed=False
            for l in model.L:
                model.delta[i,l].fixed=False
        # model.C8.activate()
        model.C9.activate()
        model.C13.activate()
        model.C14.activate()
        model.C15.activate()
        model.C15bis.activate()
        model.C17.activate()
        model.C18.activate()
