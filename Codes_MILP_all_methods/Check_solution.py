#!/usr/bin/env python
# coding=utf-8
import numpy as np
import Parameters
import Results

eps=1e-5

def check_solution(plan):
    solution_is_ok = True

    def check_increasing_time_one_bus(plan, id):
        trips = plan.paths_trips[id]
        for l in plan.L:
            if l==1:
                if plan.s[id]>eps+plan.s[id]+plan.delta[id,l]:
                    print('Error : the times link to EB ' + str(id) + ' during the parking of block ' + str(id) + ' space n°' + str(l))
                    return False
            elif l <= len(plan.L) :
                if plan.s[id]+plan.delta[id,l-1]>eps+plan.s[id]+plan.delta[id,l]:
                    print('Error : the times link to EB ' + str(id) + ' during the parking of block ' + str(id) + ' space n°' + str(l))
                    return False
            else :
                if plan.s[id]+plan.delta[id,l]>eps+plan.get_block[trips[0]].start:
                    print('Error : the times link to EB ' + str(id) + ' during the parking of block ' + str(id) + ' space n°' + str(l))
                    return False
        for i in trips :
            if i!=66666:
                for l in plan.L:
                    if l==1:
                        if plan.s[i]>eps+plan.s[i]+plan.delta[i,l]:
                            print('Error : the times link to EB ' + str(i) + ' during the parking of block ' + str(i) + ' space n°' + str(l))
                            return False
                    elif l <= len(plan.L) :
                        if plan.s[i]+plan.delta[i,l-1]>eps+plan.s[i]+plan.delta[i,l]:
                            print('Error : the times link to EB ' + str(i) + ' during the parking of block ' + str(i) + ' space n°' + str(l))
                            return False
                    if l == len(plan.L) :
                        if plan.s[i]+plan.delta[i,l]>eps+plan.get_block[plan.x[i][0]].start:
                            print('Error : the times link to EB ' + str(i) + ' during the parking of block ' + str(i) + ' space n°' + str(l))
                            return False
        return True

    def check_increasing_time_all_bus(plan):
        for id in plan.EBs_id:
            if not check_increasing_time_one_bus(plan, id):
                print('Error during check increasing times')
                return False
        return True
    
    if not check_increasing_time_all_bus(plan):
        solution_is_ok =False

    def check_assignment_and_capacity_depot_at_t(plan,t):
        Park_at_t = plan.get_parking_at_t(t)
        count_park_at_t = dict()
        count_block_at_t =dict()
        for EB in plan.EBs_id:
            count_park_at_t[EB]=0
            count_block_at_t[EB]=0
        if len(Park_at_t) > plan.V:
            print('Error: There are '+ str(len(Park_at_t)) + ' parking lanes used, but the maximum is ' + str(plan.V))
            return False
        for k in Park_at_t:
            if k>= plan.V:
                print('Error : the depot has not the parking lane n°'+str(k))
                return False
            if len(Park_at_t[k]) > len(plan.L):
                print('Error : There are ' + str(len(Park_at_t[k])) +' in the parking lane ' + str(k) + ' but the maximum is ' + str(len(plan.L)) + 'at t=' +str(t))
                return False
            for EB in Park_at_t[k]:
                count_park_at_t[EB]+=1
        for b in Parameters.get_trips_at_t(plan.B_bus_trips, t):
            EB = plan.get_bus(b.id)
            count_block_at_t[EB]+=1

        for EB in plan.EBs_id:
            if count_park_at_t[EB] + count_block_at_t[EB]!=1:
                print('Error: The EB ' + str(EB) + ' is not assigned to a single task at '+ str(t))
                return False

        return True

    def check_assignment_and_capacity_depot_all_times(plan):
        for t in np.arange(0, plan.puit.start):
            if not check_assignment_and_capacity_depot_at_t(plan, t):
                print('Error during check assignment or capacity depot')
                return False
        return True
    
    if not check_assignment_and_capacity_depot_all_times(plan):
        solution_is_ok =False
    
    def check_number_max_chargers(plan):
        if plan.C != None:
            for t in np.arange(0, plan.puit.start):
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