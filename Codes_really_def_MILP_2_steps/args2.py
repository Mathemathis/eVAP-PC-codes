SAMPLE=['2']
NUM_BUS=['30', '40', '50']
NUM_TRIPS_PER_EB=['3', '3.5', '2.5']
NUM_DAYS=['3', '5']
NUM_BUS_PER_LANE=['7','6', '5']
PROP_CHG =['0.6', '0.5', '0.4']

from exec_python import *
from multiprocessing import Process
from multiprocessing import Semaphore
from datetime import datetime
import time
import sys

nb_tests= len(NUM_BUS)*len(NUM_TRIPS_PER_EB)*len(NUM_DAYS) *len(SAMPLE) * len(NUM_BUS_PER_LANE)
# print(nb_tests)

Args_list=[]

for nb_bus in NUM_BUS:
    for nb_trips_per_eb in NUM_TRIPS_PER_EB: 
        for day in NUM_DAYS:
            for sample in SAMPLE :
                for nb_bus_per_lane in NUM_BUS_PER_LANE: 
                    Args_list.append([nb_bus, nb_trips_per_eb, day, sample, nb_bus_per_lane, PROP_CHG])


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

    print(end-start, ceil(concurrency/nb_tests)*(end-start))