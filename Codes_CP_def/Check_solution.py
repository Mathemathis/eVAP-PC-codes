#!/usr/bin/env python
# coding=utf-8
import numpy as np

eps=1e-5

def check_solution(plan):
    solution_is_ok = True

    def check_assignment_and_capacity_depot_at_t(plan,t):
        Park_at_t = plan.get_parking_at_t(t)
        count_park_at_t = dict()
        count_block_at_t =dict()
        for EB in plan.EBs:
            count_park_at_t[EB.id]=0
            count_block_at_t[EB.id]=0
        if len(Park_at_t) > plan.NUM_LANES:
            print('Error: There are '+ str(len(Park_at_t)) + ' parking lanes used, but the maximum is ' + str(plan.NUM_LANES))
            return False
        for k in Park_at_t:
            if k>= plan.NUM_LANES:
                print('Error : the depot has not the parking lane n°'+str(k))
                return False
            if len(Park_at_t[k]) > plan.NUM_SPOTS:
                print('Error : There are ' + str(len(Park_at_t[k])) +' in the parking lane ' + str(k) + ' but the maximum is ' + str(len(plan.NUM_SPOTS)) + 'at t=' +str(t))
                return False
            for EB in Park_at_t[k]:
                count_park_at_t[EB]+=1
        for i in range(plan.NUM_TRIPS):
            b= plan.Blocks[i]
            # print(b.id, b.start, plan.get_bus(i))
            if t>b.start and t <= b.end:
                EB = plan.get_bus(i)
                # print(b.id, EB)
                # print(EB, count_block_at_t)
                count_block_at_t[EB]+=1

        for EB in plan.EBs:
            if count_park_at_t[EB.id] + count_block_at_t[EB.id]!=1:
                print(count_park_at_t[EB.id],count_block_at_t[EB.id])
                print('Error: The EB ' + str(EB.id) + ' is not assigned to a single task at '+ str(t))
                return False

        return True

    def check_assignment_and_capacity_depot_all_times(plan):
        for t in np.arange(2, plan.M):
            if not check_assignment_and_capacity_depot_at_t(plan, t):
                print('Error during check assignment or capacity depot')
                return False
        return True
    
    if not check_assignment_and_capacity_depot_all_times(plan):
        solution_is_ok =False
    
    def check_number_max_chargers(plan):
        for t in np.arange(0, plan.M):
            if len(plan.get_charging_at_t(t))>plan.C:
                print('Error during check number max chargers')
                return False
        return True
    
    if not check_number_max_chargers(plan):
        solution_is_ok =False

    if solution_is_ok:
        return True
        print('The planning solution is ok')
    else:
        return False
        print('The solution is not ok')