#!/usr/bin/env python
# coding=utf-8
import numpy as np
import Vehicle

class Blocks:
    def __init__(self, id, start, end, ei, type):
        self.id = id
        self.start = start
        self.end = end
        self.ei= ei
        self.type = type
    
    def converter_start(self, day):
        hour, min = int(self.start.split(sep=':')[0]), int(self.start.split(sep=':')[1])
        self.start = min+ hour*60+day*24*60
    
    def converter_end(self, day):
        hour, min = int(self.end.split(sep=':')[0]), int(self.end.split(sep=':')[1])
        self.end = min+ hour*60+day*24*60
    
    def get_length(self):
        return self.end-self.start

def get_length(b):
        return b.end-b.start

def get_block(id, Blocks):
    '''
    Return block corresponding to the id
    '''
    for b in Blocks:
        if b.id==id:
            return b

def get_last_block(Blocks):
    '''
    Return the end time of the last block of the horizon
    '''
    bmax = Blocks[0]
    max= bmax.end
    for i in range(len(Blocks)):
        if Blocks[i].end>max:
            bmax = Blocks[i]
            max = Blocks[i].end
    return max

def create_puit(Block):
    '''
    Create the block puit
    '''
    M= get_last_block(Block)+180
    return Blocks(66666, M, M, 0, None)

def get_all_id(Blocks):
    '''
    Return all the ids of the blocks
    '''
    return [b.id for b in Blocks]
    
def arc_bus_sortant(b, Blocks, limit):
    arc_bus_s = []
    for bn in Blocks:
        if bn.start > b.end and bn.start< b.end +24*60:
            arc_bus_s.append([bn.id, bn.start])
    if b.id ==66666:
        return []
    return [arc_bus_s[id][0] for id in np.argsort(np.array(arc_bus_s), axis=0)[:,1]][:limit]

def arc_park_sortant(b, Blocks, limit):
    arc_park_s = []
    for bn in Blocks:
        if bn.end > b.end and bn.end< b.end +24*60:
            arc_park_s.append([bn.id, bn.end])
    if b.id ==66666:
        return []
    # limit= int(0.5*len(arc_park_s))+1

    return [arc_park_s[id][0] for id in np.argsort(np.array(arc_park_s), axis=0)[:,1]][:limit]

def define_all_arc_bus(Blocks, limit):
    all_arc_bus=[]
    for b in Blocks:
        for bnid in arc_bus_sortant(b, Blocks, limit):
            all_arc_bus.append((b.id,bnid))
    return all_arc_bus

def define_all_arc_park(Blocks, limit):
    all_arc_park=[]
    for b in Blocks:
        for bnid in arc_park_sortant(b, Blocks, limit):
            all_arc_park.append((b.id,bnid))
    return all_arc_park

def get_arcs_park_sortant(i, Arcs_parks):
    res=[]
    for (k,j) in Arcs_parks:
        if k == i:
            res.append(j)
    return res

def get_arcs_bus_sortant(i, Arcs_bus):
    res=[]
    for (k,j) in Arcs_bus:
        if k == i:
            res.append(j)
    return res


def define_arc_park_init(EB_blocks):
    return [(0, v.id) for v in EB_blocks] + [(v.id, vp.id) for v in EB_blocks for vp in EB_blocks if v.id != vp.id]

def define_arc_park_init_optimal(EB_blocks, V, L):
    park_init=[[EB_blocks[i].id] for i in range(V) if i< len(EB_blocks)]
    i=V
    while i< len(EB_blocks):
        park_init[i%V].append(EB_blocks[i].id)
        i+=1
    if len(park_init[0])> L:
        print('It is impossible to park ' + str(len(EB_blocks)) + ' EBs because there are only ' + str(L*V) + ' parking spaces in the depot')
        return None
    arc_park_init_opt=[]
    for parking_lane in park_init:
        arc_park_init_opt.append((0, parking_lane[0]))
        for i in range(len(parking_lane)-1):
            arc_park_init_opt.append((parking_lane[i], parking_lane[i+1]))
    return arc_park_init_opt, park_init

def define_arc_end(Blocks):
    return [(b.id,66666) for b in Blocks]
    
def vehicule_to_block(v):
    return Blocks(v.id, 0, 0, 0, v.type)

def create_init_block(v_list):
    return [vehicule_to_block(v) for v in v_list]

def get_number_of_trips_at_t(Blocks, t):
    sum=0
    for b in Blocks:
        if b.start <= t and b.end >t:
            sum+=1
    return sum

def get_trips_at_t(Blocks, t):
    trips=[]
    for b in Blocks:
        if b.start <= t and b.end >t:
            trips.append(b)
    return trips

def get_max_arc_sortant(i, model):
    Bbus=[model.get_block[k].start for k in get_arcs_bus_sortant(i, model.Arcs_bus)]
    return np.max(Bbus)

def get_optimal_M_parking_one_block(i, j, model):
    return get_max_arc_sortant(i, model) - model.get_block[j].end

def get_optimal_M_parking(Blocks, model):
    Mi=[get_optimal_M_parking_one_block(b.id, model) for b in model.B_bus_trips]
    return np.max(Mi)

def mask_value(L, support):
    '''
    Delete all the 0 in L
    '''
    res =dict()
    for s in support:
        if (len(s)==2):
            if L[s].value>0.1:
                res[s[0]] = list()
        else:
            print('error')
    
    for s in support:
        if (len(s)==2):
            if L[s].value>0.1:
                res[s[0]].append(s[1])
        else:
            print('error')
    return res

def convert(L, support):
    '''
    Convert model object into real values
    '''
    res= dict()
    for i in support:
        res[i] = np.round(L[i].value,5)
    return res

def create_graphe_bidirected(source, nodes, puits):
    Arcs=[]
    for b in nodes:
        Arcs.append((source, b))
        for bp in nodes:
            if bp !=b:
                Arcs.append((b, bp))
        Arcs.append((b, puits))  
    return Arcs

def create_nights(model):
    Time_plan = np.arange(0, model.B_puit.start)
    A=[get_number_of_trips_at_t(model.B_trips, p) for p in Time_plan]
    B=[p for p in Time_plan if A[p]<=len(model.B_bus_id)-model.C1]
    night=[]
    i=0
    j=1
    while j< len(B):
        st=B[i]
        while j< len(B) and B[j]-B[i] == 1:
            i+=1
            j+=1
        en = B[i]
        # if st%(24*60)>en%(24*60) and not en+1==model.B_puit.start:
        print(st,en)
        night.append((st,en))
        i+=1
        j+=1
    return night

def select_block_for_night(night, model):
    B_charge=dict()
    for n in night:
        B_charge[n]=list()
        for b in model.B_bus_trips:
            if b.end <= n[1] and get_block(model.x[b.id][0], model.B_bus_trips_puit).start >= n[0]:
                B_charge[n].append(b.id)
    return B_charge

def create_graphs_for_overnight_charge(model):
    night=create_nights(model)
    B_chg = select_block_for_night(night, model)
    Graphs_chg = dict()
    nb_night=0
    for n in night:
        Graphs_chg[n]=create_graphe_bidirected('SC'+str(nb_night), B_chg[n], 'PC'+str(nb_night))
        nb_night+=1
    return Graphs_chg

def sum_elements_dict(D):
    res=[]
    for d in D:
        res.extend(D[d])
    return res

def is_parked_together(model,i,j):
    bi=model.get_block[i]
    # binext = get_block(park_model.next_x[i][0], park_model.B_bus_trips_puit)
    bj = model.get_block[j]
    # bjnext = get_block(park_model.next_x[j][0], park_model.B_bus_trips_puit)
    # print(i,j, bi.id, bj.id, bi.end, bj.end, binext.start)
    # if bi.end < bj.end and binext.start > bj.end:
    #     return True
    # if bj.end < bi.end and bjnext.start > bi.end:
    #     return True
    if -24*60<bj.end-bi.end <24*60:
        return True
    return False
    

def create_charging_graph(park_model):
    Arcs=[]
    for EB in park_model.B_bus_id:
        Arcs.append((0, EB))
    for i in park_model.B_bus_trips_id:
        Arcs.append((i, 66666))
        for j in park_model.B_bus_trips_id:
            if is_parked_together(park_model,i,j) and i!=j:
                Arcs.append((i,j))
                # Arcs.append((j,i))
        # for j in park_model.B_bus_trips_id:
        #     if i<j:
        #         Arcs.append((i,j))
        #         Arcs.append((j,i)) 
    return Arcs

def create_charging_graph_bis(park_model):
    Arcs=[]
    for i in park_model.B_trips_id:
        Arcs.append((i, 66666))
        if park_model.get_block[i].start <= 24*60:
                Arcs.append((0,i))
        for j in park_model.B_trips_id:
            if is_parked_together(park_model,i,j) and i!=j:
                Arcs.append((i,j))
    return Arcs
