SAMPLE=['1']
NUM_BUS=['30']
NUM_TRIPS_PER_EB=['3']
NUM_DAYS=['2']
NUM_BUS_PER_LANE=['5']
PROP_CHG =['0.5']
# Ways=['global', 'global_obj', '2_steps', '2_steps_obj', '2_steps_obj_non_simplify']
Ways=['global_obj']

from exec_python_global import launch_planning as launch_global
from exec_python_global_obj import launch_planning as launch_global_obj
from exec_python_2_step import launch_planning as launch_2_step
from exec_python_2_step_obj import launch_planning as launch_2_step_obj
from exec_python_2_step_obj_non_simplify import launch_planning as launch_2_step_obj_non_simplify
from multiprocessing import Process
from multiprocessing import Semaphore
from datetime import datetime
import time
import sys
from exec_python_2_step_obj import *


nb_tests= 0
# print(nb_tests)

Args_list=[]

for nb_bus in NUM_BUS:
    for nb_trips_per_eb in NUM_TRIPS_PER_EB: 
        for day in NUM_DAYS:
            for sample in SAMPLE :
                for nb_bus_per_lane in NUM_BUS_PER_LANE:
                    for way in Ways:
                        nb_tests+=1 
                        Args_list.append([way, [nb_bus, nb_trips_per_eb, day, sample, nb_bus_per_lane, PROP_CHG]])


def launch_test(args, sema, start, count):
    elapsed_time = time.time() - start
    time_left = elapsed_time * (nb_tests/count - 1) if count != 0 else 0
    percent = np.round(100*count/nb_tests)
    sys.stdout.write(f"\r  {count:>9}   {percent:>7.2f}%   {time_left:>13.0f}s")
    sys.stdout.flush()
    if args[0] == '2_steps_obj':
        launch_2_step_obj(args[1], time_limit = 3600)
    elif args[0] == '2_steps_obj_non_simplify':
        launch_2_step_obj_non_simplify(args[1], time_limit = 3600)
    elif args[0] == '2_steps':
        launch_2_step(args[1], time_limit = 3600)
    elif args[0] == 'global':
        launch_global(args[1], time_limit = 3600)
    elif args[0] == 'global_obj':
        launch_global_obj(args[1], time_limit = 3600)
    else:
        print('ERROR')
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