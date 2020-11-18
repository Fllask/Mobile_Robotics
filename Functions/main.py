#!/usr/bin/python3

import os
from multiprocessing import Process, Pipe
# I use the multiprocessing library for parallelism ==> https://docs.python.org/3.8/library/multiprocessing.html
import time

def printMiliseconds(time):
    print( str(round(time*1000,2)) )

## example local function
def processSend(t0,conn):
    """process function"""
    #wait some time to simulate compute
    time.sleep(2)
    #print out process parameters
    print('module name:', __name__)
    print('parent process:', os.getppid())
    print('process id:', os.getpid())
    #print time
    tf = time.time()
    printMiliseconds(tf-t0)
    #send some data to some other process
    conn.send([42, None, 'hello'])
    #closing the process
    return

## example local function
def processRecv(t0,conn):
    """process function"""
    #print out process parameters
    print('parent process:', os.getppid())
    print('process id:', os.getpid())
    #print time
    #send some data to some other process
    print(conn.recv())
    #closing the process
    return

if __name__ == '__main__':

    #using a list to store the processes
    jobs = []

    #printing out the process id of main:
    print('module name:', __name__)
    print('parent process:', os.getppid())
    print('process id:', os.getpid())
    
    #creating a pipe for communication
    sendConn, recvConn = Pipe()
    t0 = time.time()
    #starting sending process
    p = Process(target=processSend,args=(t0,sendConn,))
    jobs.append(p)
    p.start()

    #starting reciveving process
    p = Process(target=processRecv,args=(t0,recvConn,))
    jobs.append(p)
    p.start()