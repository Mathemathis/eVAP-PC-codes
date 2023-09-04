SAMPLE=['3']
NUM_BUS=['30', '40', '50']
NUM_TRIPS_PER_EB=['2.5', '3', '3.5']
NUM_DAYS=['3']
NUM_BUS_PER_LANE=['5', '6', '7']
PROP_CHG =['0.5']

from CP import *
from multiprocessing import Process
from multiprocessing import Semaphore
from datetime import datetime
import time
import sys

nb_tests= len(NUM_BUS)*len(NUM_TRIPS_PER_EB)*len(NUM_DAYS) *len(SAMPLE) * len(NUM_BUS_PER_LANE)
# print(nb_tests)

Args_list=[]
nb_tests=0
for nb_bus in NUM_BUS:
    for nb_trips_per_eb in NUM_TRIPS_PER_EB: 
        for day in NUM_DAYS:
            for sample in SAMPLE :
                for nb_bus_per_lane in NUM_BUS_PER_LANE: 
                    for c in PROP_CHG:
                        nb_tests+=1
                        Args_list.append([nb_bus, nb_trips_per_eb, day, sample, nb_bus_per_lane, c])


def launch_test(args, sema, start, count):
    elapsed_time = time.time() - start
    time_left = elapsed_time * (nb_tests/count - 1) if count != 0 else 0
    percent = np.round(100*count/nb_tests)
    sys.stdout.write(f"\r  {count:>9}   {percent:>7.2f}%   {time_left:>13.0f}s")
    sys.stdout.flush()
    launch_planning(args, time_limit = 3600)
    sema.release()

if __name__ == '__main__':
    start = time.time()
    print(start)
    concurrency = 5
    sema = Semaphore(concurrency)
    all_processes = []
    for count, args in enumerate(Args_list):
        sema.acquire()
        p = Process(target=launch_test, args=(args, sema, start, count))
        all_processes.append(p)
        p.start()

    # inside main process, wait for all processes to finish
    for p in all_processes:
        p.join()
    
    end= time.time()

    print(end-start)