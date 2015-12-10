#!/usr/bin/env python

"""
Script for running omniscient and agent nodes at once.
Takes in the number of agents.
Example: python run_nodes.py 4
"""

import subprocess
import sys
import termios
import helper_funcs as hp


def run_omniscient(num):
    """
    Use subprocess to call 'rosrun project_polygon omniscient.py'
    with the given parameter, return process so it can be killed later
    """
    cmd = ['rosrun', 'project_polygon', 'omniscient.py', str(num)]

    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    return proc


def run_agent(i):
    """
    Use subprocess to call 'rosrun project_polygon agent.py'
    with the given parameter, return process so it can be killed later
    """
    cmd = ['rosrun', 'project_polygon', 'agent.py', str(i)]

    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    return proc


if __name__ == '__main__':
    num = int(sys.argv[1])

    processes = []
    for i in range(num):
        p = run_agent(i)
        processes.append(p)

    p = run_omniscient(num)
    processes.append(p)

    print 'Ctrl-C to quit and close nodes'

    # Wait for Ctrl-C
    settings = termios.tcgetattr(sys.stdin)
    key = None
    while key != '\x03':
        key = hp.get_key(settings)

    # Terminate the connections
    for proc in processes:
        proc.terminate()
