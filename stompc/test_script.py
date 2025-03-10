#!/usr/bin/env python3
import os
import signal
from subprocess import Popen, PIPE
import time
import psutil
import csv
import datetime

NUMBER_OF_RUNS = 20
MAX_TIME_PER_RUN = 600
START_TIMESTAMP = datetime.datetime.now()

def kill_proc_tree(pid, sig=signal.SIGKILL, include_parent=True,
                   timeout=None, on_terminate=None):
    """Kill a process tree (including grandchildren) with signal
    "sig" and return a (gone, still_alive) tuple.
    "on_terminate", if specified, is a callback function which is
    called as soon as a child terminates.
    """
    assert pid != os.getpid(), "won't kill myself"
    parent = psutil.Process(pid)
    children = parent.children(recursive=True)
    if include_parent:
        children.append(parent)
    for p in children:
        try:
            p.send_signal(sig)
        except psutil.NoSuchProcess:
            pass
    gone, alive = psutil.wait_procs(children, timeout=timeout,
                                    callback=on_terminate)
    
    gz_p = None
    try:
        for p in psutil.process_iter(['cmdline']):
            if p.info['cmdline'] and 'gz sim' in ' '.join(p.info['cmdline']):
                gz_p = p
        if gz_p != None:
            gz_p.send_signal(sig)
    except (psutil.AccessDenied, psutil.NoSuchProcess):
        print('No process with gz :thinking:')
        pass


    return (gone, alive)

file = 'experiments/nobug_Experiment_open=1_turningcost=20_movingcost=20_discoveryreward=10_pumpreward=1000_safetyrange=40cm_maxiter=3_rnb=default_gr=50_tr=300_rps=default_h=20.csv'

def get_number_of_lines_csv (filename):
    number_of_lines = 0
    with open(filename) as testfile:
        reader = csv.reader(testfile, delimiter=',')
        number_of_lines = len([row for row in reader])
    return number_of_lines - 1

i = 0
number_of_lines = get_number_of_lines_csv(file)
while number_of_lines < NUMBER_OF_RUNS:
    print("Starting run")
    start_time = time.time()
    stompc_proc = psutil.Popen("python3 stompc.py",
                    shell=True,
                    #stderr=PIPE,
                    )
    curr_time = 0
    while curr_time < MAX_TIME_PER_RUN + 120:
        time.sleep(30)
        check_len = get_number_of_lines_csv(file)
        curr_time = time.time() - start_time
        print("Time spent so far for run {}: {} seconds".format(i+1, curr_time))
        if check_len > number_of_lines:
            number_of_lines = check_len
            print('Run seems to be finished. Proceeding to kill processes')
            print(f'total number of good runs so far: {number_of_lines}')
            break
    i += 1

    print("Run finished, killing processes".format(i+1))
    kill_proc_tree(stompc_proc.pid)
    time.sleep(5)
    print("Processes killed\n")

print("Experiment started at : ", START_TIMESTAMP)
print("Experiment finished at: ", datetime.datetime.now())