import sys
import numpy as np
from importlib import reload
import pandas
import Parameters
import Optimizer
import Results
import Vehicle
import Check_solution
from Write_Output import *
# from matplotlib import animation, rc
# # try:
# import docplex.cp
# # except:
# #     if hasattr(sys, 'real_prefix'):
# #         #we are in a virtual env.
# #         !pip install docplex
# #     else:
# #         !pip install --user docplex
# from docplex.cp.model import *

# blocks_excel = pandas.read_excel("data/Blocks.xlsx", sheet_name=0)

param=100

## Parameters

def launch_planning(args, time_limit):
    NUM_BUS=int(args[0])
    NUM_TRIPS_PER_EB = float(args[1])
    if NUM_TRIPS_PER_EB%1==0:
        NUM_TRIPS_PER_EB =int(NUM_TRIPS_PER_EB)
    NUM_DAYS = int(args[2])
    NUM_TRIPS = int(NUM_TRIPS_PER_EB*NUM_BUS*NUM_DAYS)
    NUM_NODES=NUM_TRIPS+2*NUM_BUS
    num_sample = int(args[3])
    NUM_BUS_PER_LANE=int(args[4])
    PROP_CHG = float(args[5])
    NUM_LANES=ceil(NUM_BUS/NUM_BUS_PER_LANE)
    NUM_SPOTS=NUM_BUS_PER_LANE
    C = int(PROP_CHG*NUM_BUS)

    Params=dict()
    Params['NUM_BUS']=NUM_BUS
    Params['NUM_TRIPS_PER_EB']=NUM_TRIPS_PER_EB
    Params['NUM_DAYS']=NUM_DAYS
    Params['sample']=num_sample
    Params['NUM_BUS_PER_LANE']=NUM_BUS_PER_LANE
    Params['PROP_CHG']=PROP_CHG
    Params['param']=param

    directory= str(NUM_DAYS)+'_'+str(NUM_BUS_PER_LANE)+'_'+str(num_sample)
    parent_dir = "data/"+str(NUM_BUS) + "_bus/" + str(NUM_TRIPS_PER_EB) + "_trips_per_bus"

    path = os.path.join(parent_dir, directory)

    if not os.path.exists(path):
        os.mkdir(path)

    excel_path=os.path.join(path, 'results.xlsx')
    write(excel_path=excel_path, Param=Params, model=None, sol=None, step='Param', exit=None, time=None)

    typ = 'e2'
    if typ == 'e2':
        RATE=22
    else:
        RATE=24
        
    ## EBS 
    EB_excel = pandas.read_csv(
        'data/Vehicles.txt', sep="\t")

    rates=dict()
    rate_e1=42/(3*60)
    rate_e2=50*100/(380*60)
    rates['e1']= rate_e1
    rates['e2']= rate_e2

    EBs =[]

    for row in EB_excel.iterrows():
        id, typ = row[1].values
        EBs.append(Vehicle.EB(id, typ, rates[typ]))

    EBs =[EBs[i] for i in range(len(EBs))]
    EBs= EBs[:NUM_BUS]


    block_excel= pandas.read_excel("data/" + str(NUM_BUS)+ "_bus/"+str(NUM_TRIPS_PER_EB)+ "_trips_per_bus/instance_"+str(num_sample)+".xlsx", sheet_name=0)

    Block = []


    for h in range(NUM_DAYS):
        i=0
        for row in block_excel.iterrows():
            id, typ, start, end, ei = row[1].values
            b = Parameters.Blocks(id+h*100000,start+h*24*60, end+h*24*60, int(ei*param), typ)
            Block.append(b)
            i+=1
    Block=[Block[i] for i in range(len(Block)) if Block[i].type=="e2"]


    hist=[Parameters.get_number_of_trips_at_t(Block, p) for p in range(NUM_DAYS*24*60)]

    instances=dict()

    if np.max(hist)>NUM_BUS:
        print(np.max(hist), NUM_BUS)
        print('infeasible')


    M= Parameters.get_last_block(Block)+180

    # build the model

    Planning= Optimizer.Planning_model(Block, EBs, 'e2', 100, 15, C, NUM_LANES, NUM_SPOTS)
    solution, link_trip_to_EB, charge_events, parking_events_for_trip, parking_events_for_bus =Planning.solve(time_limit, False, excel_path=excel_path)

    logpath = os.path.join(path, 'log_output.txt')
    f = open(logpath, "w") 
    f.write(solution.get_solver_log()) 
    f.close() 

    if solution.get_solve_status() == 'Feasible':
        Planning_exit=Results.Results_planning(NUM_BUS, NUM_TRIPS, NUM_LANES, NUM_SPOTS, C, RATE, param, M, EBs, Block, link_trip_to_EB, charge_events, parking_events_for_trip, parking_events_for_bus)

        if not Check_solution.check_solution(Planning_exit):
                        print('ERROOOOOR' + str(args))