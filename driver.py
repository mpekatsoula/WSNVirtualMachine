#! /usr/bin/python
import sys
import time

import TOSSIM
from TOSSIM import *
import random

if ( len(sys.argv) < 3 ):
  print "Usage: " + sys.argv[0] + " <topology.txt> <simulation time>" 
  exit(0)
  
t = Tossim([])
m = t.mac()
r = t.radio()

t.addChannel("TIMER", sys.stdout)
t.addChannel("BIN", sys.stdout)
t.addChannel("TASK", sys.stdout)
t.addChannel("ERR", sys.stdout)
t.addChannel("SENSOR", sys.stdout)

# Open topology file
f = open(sys.argv[1], "r")
lines = f.readlines()
number_of_nodes = 0

for line in lines:
  s = line.split()
  if ( len(s) == 1 ):
    number_of_nodes = int(s[0])
  elif ( len(s) > 0 ):
    print " ", s[0], " ", s[1], " ", s[2];
    r.add(int(s[0]), int(s[1]), float(s[2]))

noise = open("./noise_small.txt", "r")

lines = noise.readlines()
for line in lines:
  str1 = line.strip()
  if (str1 != ""):
    val = int(str1)
    for i in range(0, number_of_nodes):
      t.getNode(i).addNoiseTraceReading(val)

for i in range (0, number_of_nodes):
  t.getNode(i).createNoiseModel()
  t.getNode(i).bootAtTime(i*10)



for i in range(int(sys.argv[2])):
        t.runNextEvent()

  
