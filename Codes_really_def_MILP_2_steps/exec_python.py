#!/usr/bin/env python3
import numpy as np
import pandas
import Parameters
import Optimizer
import Results
import Vehicle
import Check_solution
import os
# import sys
import pyomo.environ as pyo
import logging
from Write_Output import *

logging.getLogger('pyomo.core').setLevel(logging.ERROR)

def launch_planning(args, time_limit):

    # NUM_BUS = int(sys.argv[1])
    NUM_BUS = int(args[0])
    EB_excel = pandas.read_csv(
        'data/Vehicles.txt', sep="\t")

    rates=dict()
    rate_e1=42/(3*60)
    rate_e2=50*100/(380*60)
    rates['e1']= rate_e2
    rates['e2']= rate_e2

    EBs =[]

    for row in EB_excel.iterrows():
        id, typ = row[1].values
        EBs.append(Vehicle.EB(id, 'e2', rates[typ]))


    # NUM_TRIPS_PER_EB = float(sys.argv[2])
    # if NUM_TRIPS_PER_EB%1==0:
    #     NUM_TRIPS_PER_EB =int(NUM_TRIPS_PER_EB)
    # NUM_DAYS = int(sys.argv[3])
    # num_sample = int(sys.argv[4])

    NUM_TRIPS_PER_EB = float(args[1])
    if NUM_TRIPS_PER_EB%1==0:
        NUM_TRIPS_PER_EB =int(NUM_TRIPS_PER_EB)
    NUM_DAYS = int(args[2])
    num_sample = int(args[3])

    block_excel= pandas.read_excel("data/" + str(NUM_BUS)+ "_bus/"+str(NUM_TRIPS_PER_EB)+ "_trips_per_bus\instance_"+str(num_sample)+".xlsx", sheet_name=0)

    Block = []


    for h in range(NUM_DAYS):
        i=0
        for row in block_excel.iterrows():
            id, typ, start, end, ei = row[1].values
            b = Parameters.Blocks(id+h*100000,start+h*24*60, end+h*24*60, ei, typ)
            Block.append(b)
            i+=1
    Block=[Block[i] for i in range(len(Block)) if Block[i].type=="e2"]


    hist=[Parameters.get_number_of_trips_at_t(Block, p) for p in range(NUM_DAYS*24*60)]

    instances=dict()

    if np.max(hist)>NUM_BUS:
        print(np.max(hist), NUM_BUS)
        print('infeasible')

    SOCmin=15
    SOCmax=100
    # NUM_BUS_PER_LANE=int(sys.argv[5])
    NUM_BUS_PER_LANE=int(args[4])
    nb_parking_lanes=ceil(NUM_BUS/NUM_BUS_PER_LANE)
    nb_spaces=NUM_BUS_PER_LANE
    # PROP_CHG= float(sys.argv[6])
    PROP_CHG= [float(c) for c in args[5]]
    NUM_CHG = [int(NUM_BUS*prop) for prop in PROP_CHG]

    Planning=Optimizer.Planning_model(Block, EBs[:NUM_BUS], 'e2', rates,SOCmax, SOCmin, NUM_CHG, nb_parking_lanes, nb_spaces)
    Planning.create_variables()
    Planning.create_obj_and_constraints()

    ##Résolution du problème
    from datetime import datetime
    DATE_TIME_FORMAT = "%Y-%b-%d_%H-%M"
    directory= str(NUM_DAYS)+'_'+str(NUM_BUS_PER_LANE)+'_'+str(num_sample)
    parent_dir = "data\\"+str(NUM_BUS) + "_bus\\" + str(NUM_TRIPS_PER_EB) + "_trips_per_bus"

    path = os.path.join(parent_dir, directory)

    if not os.path.exists(path):
        os.mkdir(path)

    logpath = os.path.join(path, 'step1.log')

    excel_path=os.path.join(path, 'results.xlsx')
    write(excel_path, Planning.model, 'Param')

    opt = pyo.SolverFactory('cplex')
    opt.options['mip limits solutions'] = 1
    opt.options['timelimit'] = time_limit
    opt.options['threads'] = 1
    Optimizer.deactivate_shift_problem(Planning.model)
    # Optimizer.deactivate_charging_problem(Planning.model)
    Optimizer.deactivate_parking_problem(Planning.model)
    res1 = opt.solve(Planning.model, logfile= f'{logpath}')

    exit1 = res1.solver.termination_condition
    time1 = res1.solver.time
    if exit1 == 'unknown':
        if time1>=time_limit-1:
            exit1 = 'Time limit'
        else:
            exit1 = 'Feasible'

    if exit1 not in ['Time limit', 'Feasible', 'infeasible', 'optimal']:
        print('Error exit1' + str(args))

    write(excel_path, Planning.model, '1', exit1, time1)

    # Planning_exit = Results.Results_planning(Planning.model, '1')

    # import Optimizer_nb_chg

    # Planning_step_2 = Optimizer_nb_chg.Planning_model(Planning_exit)

    # Planning_step_2.solve(120, True)


    # opt.options['mip limits solutions'] = 200
    # logpath3 = os.path.join(path, 'logfile3.log')
    # Optimizer.deactivate_assignment_problem(Planning.model)
    # Optimizer.deactivate_parking_problem(Planning.model)
    # Optimizer.activate_shift_problem(Planning.model)
    # res3 = opt.solve(Planning.model, warmstart=True, logfile= f'{logpath3}')

    # opt.options['mip limits solutions'] = 10
    # logpath3 = os.path.join(path, 'logfile4.log')
    # Optimizer.activate_assignment_problem(Planning.model)
    # Optimizer.activate_parking_problem(Planning.model)
    # Optimizer.activate_shift_problem_bis(Planning.model)
    # res3 = opt.solve(Planning.model, warmstart=True, logfile= f'{logpath3}')
    # do_step_2= exit1 not in ['infeasible', 'Time limit']

    # if do_step_2:

    #     opt.options['mip limits solutions'] = 1
    #     i=0
    #     flag=True
    #     while i <len(NUM_CHG) and flag:
    #         logpath3 = os.path.join(path, 'step3_'+str(NUM_CHG[i])+'.log')
    #         Optimizer.deactivate_assignment_problem(Planning.model)
    #         Optimizer.deactivate_parking_problem(Planning.model)
    #         Optimizer.activate_charging_problem(Planning.model, i+1)
    #         res3 = opt.solve(Planning.model, logfile= f'{logpath3}')

    #         exit3 = res3.solver.termination_condition
    #         time3 = res3.solver.time
    #         if exit3 == 'unknown':
    #             if time3>=time_limit-1:
    #                 exit3 = 'Time limit'
    #             else:
    #                 exit3 = 'Feasible'

    #         if exit3 not in ['Time limit', 'Feasible', 'infeasible', 'optimal']:
    #             print('Error exit2' +str(NUM_CHG[i])+'_'+ str(args))

    #         write(excel_path, Planning.model, '3', NUM_CHG[i], exit3, time3)

    # ##Check solution
    #         if exit3 not in ['Time limit', 'infeasible']:
    #             Planning_exit = Results.Results_planning(Planning.model, NUM_CHG[i], step='3')
    #         else:
    #             flag = False
    #             Planning_exit = Results.Results_planning(Planning.model, step='1')

    #         if not Check_solution.check_solution(Planning_exit):
    #             print('ERROOOOOR' + str(args))
            
    #         i+=1

    #     opt.options['mip limits solutions'] = 20000

    #     logpath2 = os.path.join(path, 'step2.log')
    #     Optimizer.deactivate_charging_problem(Planning.model)
    #     Optimizer.activate_shift_problem(Planning.model)
    #     res2 = opt.solve(Planning.model, logfile= f'{logpath2}')

    #     exit2 = res2.solver.termination_condition
    #     time2 = res2.solver.time
    #     if exit2 == 'unknown':
    #         if time2>=time_limit-1:
    #             exit2 = 'Time limit'
    #         else:
    #             exit2 = 'Feasible'

    #     if exit2 not in ['Time limit', 'Feasible', 'infeasible', 'optimal']:
    #         print('Error exit3' + str(args))

    #     write(excel_path, Planning.model, '2', exit2, time2)

    #     Planning_exit = Results.Results_planning(Planning.model, step='2')