#!/usr/bin/env python
# coding=utf-8
import Parameters
import pandas as pd
from docplex.cp.model import *
from Write_Output import *


class Planning_model:
    def __init__(self, Block, EBs, type, SOCmax, SOCmin, C, nb_parking_lanes, nb_spaces):
        

        param=100
        Starts = [Block[i].start for i in range(len(Block))]
        Ends = [Block[i].end for i in range(len(Block))]

        soc_consumed = [Block[i].ei for i in range(len(Block))]

        M= Parameters.get_last_block(Block)+180

        Starts.extend([0 for i in EBs])
        Starts.extend([M for i in range(len(EBs))])

        Ends.extend([0 for i in EBs])
        Ends.extend([M for i in range(len(EBs))])

        soc_consumed.extend([0 for i in EBs])
        soc_consumed.extend([0 for i in EBs])

        NUM_TRIPS = len(Block)
        NUM_BUS=len(EBs)
        NUM_LANES=nb_parking_lanes
        NUM_SPOTS=nb_spaces
        if type=='e2':
            RATE=22
        else:
            RATE=24
        
        TRIPS_NODES = [i for i in range(NUM_TRIPS)]


        mdl = CpoModel()

        bus=[interval_var(start=0, length=1, name='start:'+str(b)) for b in range(NUM_BUS)]
        #charge_bus=[interval_var() for b in range(NUM_BUS)]
        bus_end=[interval_var(start=M, length=1, name='end:'+str(b)) for b in range(NUM_BUS)]
        trips =[interval_var(start=Starts[i], length=Ends[i]-Starts[i], name='trips:'+str(i)) for i in TRIPS_NODES]
        trips_duplicated =[[interval_var(start=Starts[i], length=Ends[i]-Starts[i], optional=True, name= 'copy_trips:'+str(b)+'_'+str(i)) for b in range(NUM_BUS) ] for i in TRIPS_NODES]
        trips_assigned_to_EB =[[trips_duplicated[i][b] for i in TRIPS_NODES] for b in range(NUM_BUS)]
        charge = [[interval_var(optional=True, name= 'charge:'+str(b)+'_'+str(i)) for i in TRIPS_NODES] for b in range(NUM_BUS)]
        parking_after_EB =[interval_var(start=1, name='start_parking:'+str(b)) for b in range(NUM_BUS)]
        parking_after_trips_to_EB =[[interval_var(start=Ends[i], optional=True, name='copy_bus_parking:'+str(b)+'_'+str(i)) for i in TRIPS_NODES] for b in range(NUM_BUS)]
        parking_after_trips =[interval_var(start=Ends[i], optional=False, name='parking:'+str(i)) for i in TRIPS_NODES]
        parking_after_trips_to_LANE =[[interval_var(start=Ends[i], optional=True, name='copy_lane_parking:'+str(v)+'_'+str(i)) for i in TRIPS_NODES] for v in range(NUM_LANES)]


        seq_charge= [sequence_var(trips_assigned_to_EB[b]+charge[b]+[bus[b]]+[bus_end[b]], name='seq_charge:'+str(b)) for b in range(NUM_BUS)]
        seq_park= [sequence_var(trips_assigned_to_EB[b]+[parking_after_EB[b]]+parking_after_trips_to_EB[b]+[bus_end[b]], name='seq_park:'+str(b)) for b in range(NUM_BUS)]


        for i in TRIPS_NODES:
            mdl.add(alternative(trips[i], trips_duplicated[i]))
            mdl.add(alternative(parking_after_trips[i], [parking_after_trips_to_EB[b][i] for b in range(NUM_BUS)]))
            mdl.add(alternative(parking_after_trips[i], [parking_after_trips_to_LANE[v][i] for v in range(NUM_LANES)]))

        for b in range(NUM_BUS):
            mdl.add(no_overlap(seq_charge[b]))
            mdl.add(no_overlap(seq_park[b]))
            for i in TRIPS_NODES:
                # mdl.add(start_before_start(parking_after_trips_to_EB[b][i], charge[b][i]))
                # mdl.add(end_before_end(charge[b][i], parking_after_trips_to_EB[b][i]))
                mdl.add(previous(seq_charge[b], trips_assigned_to_EB[b][i], charge[b][i]))
                mdl.add(presence_of(charge[b][i])== presence_of(trips_assigned_to_EB[b][i]))
                # mdl.add(presence_of(charge[b][i]) <= presence_of(trips_assigned_to_EB[b][i]))
                mdl.add(presence_of(trips_assigned_to_EB[b][i]) == presence_of(parking_after_trips_to_EB[b][i]))
                mdl.add(start_of_next(seq_park[b], parking_after_trips_to_EB[b][i], lastValue=M)== end_of(parking_after_trips_to_EB[b][i]))
            mdl.add(start_of_next(seq_park[b], parking_after_EB[b])== end_of(parking_after_EB[b]))

        Q= [step_at_start(bus[b], param*100) - sum([step_at_start(trips_assigned_to_EB[b][i], soc_consumed[i]) for i in TRIPS_NODES]) + sum([step_at_start(charge[b][i], (0, 100*param)) for i in TRIPS_NODES]) for b in range(NUM_BUS)]
        for b in range(NUM_BUS):
            for i in TRIPS_NODES:
                mdl.add(height_at_start(charge[b][i], Q[b])==RATE*size_of(charge[b][i]))

        for b in range(NUM_BUS):
            mdl.add(always_in(Q[b], [0, M], SOCmin*param, SOCmax*param))
            mdl.add(first(seq_park[b], parking_after_EB[b]))
            mdl.add(first(seq_charge[b], bus[b]))
            mdl.add(last(seq_park[b], bus_end[b]))
            mdl.add(last(seq_charge[b], bus_end[b]))

        for b in range(NUM_BUS-NUM_LANES):
            mdl.add(length_of(parking_after_EB[b])<=length_of(parking_after_EB[b+NUM_LANES]))

        for b in range(NUM_LANES-1):
            mdl.add(length_of(parking_after_EB[b])<=length_of(parking_after_EB[b+1]))

        Nb_chargers = sum([pulse(charge[b][i], 1) for b in range(NUM_BUS) for i in TRIPS_NODES])
        mdl.add(always_in(Nb_chargers, [0, M], 0, C))

        for v in range(NUM_LANES):
            Nb_EB_in_LANE = sum([pulse(parking_after_trips_to_LANE[v][i], 1) for i in TRIPS_NODES]) + sum([pulse(parking_after_EB[b], 1) for b in range(NUM_BUS) if b%NUM_LANES==v])
            mdl.add(always_in(Nb_EB_in_LANE, [0, M], 0, NUM_SPOTS))
            for i in TRIPS_NODES:
                for j in TRIPS_NODES:
                    if Ends[i]<=Ends[j] and i!=j:
                        mdl.add(end_of(parking_after_trips_to_LANE[v][i], absentValue=0) <= end_of(parking_after_trips_to_LANE[v][j], absentValue=M))
                for b in range(NUM_BUS):
                    if b%NUM_LANES==v:
                        mdl.add(end_of(parking_after_EB[b]) <= end_of(parking_after_trips_to_LANE[v][i], absentValue=M))

        mdl.NoOverlapInferenceLevel='Extented'
        mdl.CumulFunctionInferenceLevel='Extented'
        # mdl.StateFunctionInferenceLevel='Extented'

        self.model = mdl
        self.NUM_BUS =NUM_BUS
        self.NUM_TRIPS =NUM_TRIPS
        self.NUM_LANES = NUM_LANES
        self.trips_assigned_to_EB = trips_assigned_to_EB
        self.charge = charge
        self.parking_after_trips_to_LANE = parking_after_trips_to_LANE
        self.parking_after_EB = parking_after_EB

    def solve(self, time_limit, trace_log, excel_path):
        msol = self.model.solve(TimeLimit=time_limit, trace_log=trace_log, Workers=1, LogVerbosity= 'Normal')
        link_trip_to_EB=[[msol[self.trips_assigned_to_EB[b][i]] for i in range(self.NUM_TRIPS) ]for b in range(self.NUM_BUS)]
        charge_events=[[msol[self.charge[b][i]] for i in range(self.NUM_TRIPS) ] for b in range(self.NUM_BUS) ]
        parking_events_for_trip=[[msol[self.parking_after_trips_to_LANE[v][i]] for i in range(self.NUM_TRIPS) ] for v in range(self.NUM_LANES)]
        parking_events_for_bus=[msol[self.parking_after_EB[b]] for b in range(self.NUM_BUS) ]

        exit1= msol.get_solve_status()
        time1 = msol.get_solve_time()
        write(excel_path=excel_path, Param=None, model=self.model, sol=msol, step='Global', exit=exit1, time=time1)

        return msol, link_trip_to_EB, charge_events, parking_events_for_trip, parking_events_for_bus