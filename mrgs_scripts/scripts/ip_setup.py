#!/usr/bin/env python
import subprocess               # For shell calls
import sys                      # For command line arguments
import time                     # So that we can wait
import re                       # regex
from os.path import expanduser  # So that we can get to the home directory


# We start by checking if an interface was passed via command line
if len(sys.argv) < 2:
  print 'Usage: ' + sys.argv[0] +' <interface>'
  exit()

if sys.argv[1] != 'clean':
  # Next we get the interface's address from 'ip addr'
  interesting_lines = False
  cmd = subprocess.Popen('ip addr', shell=True, stdout=subprocess.PIPE)
  ip = []  
  for line in cmd.stdout:
    if sys.argv[1] in line:
      if interesting_lines == False:
        interesting_lines = True
      else:
        # We've found our address!
        #ip = line[9:23]
	ip = re.findall( r'[0-9]+(?:\.[0-9]+){3}', line )
        ip = ip[0]
  # We show the address to the user, and give them the choice of exiting right away if something's wrong
  if len(ip) == 0:
      print 'The interface you gave me doesn\'t exist!'
      exit()
  print 'I think I\'ve found an ip: ' + ip + '. Next I\'ll check and correct .bashrc.\nIf the address looks like garbled mumbo jumbo, you have 3 seconds to CTRL+C.'
  #print ip
  time.sleep(3)

# We're going through the file twice, we only needed to go once...
# Now we check if these lines already exist in .bashrc
cmd2 = subprocess.Popen('cat ~/.bashrc', shell=True, stdout=subprocess.PIPE)
already_set = False
i = 0
linelist = []
for line in cmd2.stdout:
  if ('export ROS_MASTER_URI=' in line) or ('export ROS_IP' in line) or ('export MRGS_INTERFACE' in line):
    already_set = True
    linelist.append(i)
  i = i+1

# If the lines don't exist, we append them
home = expanduser("~")
if linelist:
  if sys.argv[1] == 'clean':
    print 'Lines exist. Cleaning...'
  else:
    print 'Lines already existed. Substituting...'
  i = 0
  bashrc = open(home+'/.bashrc', "r")
  rc_lines = bashrc.readlines()
  bashrc.close()
  bashrc = open(home+'/.bashrc', "w")
  for line in rc_lines:
    if i not in linelist:
      bashrc.write(line)
    i = i+1
else:
  print 'Lines didn\'t exist.'

if sys.argv[1] != 'clean':
  lines = ['export ROS_MASTER_URI=http://'+ip+':11311\n', 'export ROS_IP='+ip+'\n', 'export MRGS_INTERFACE='+sys.argv[1]+'\n']
  bashrc = open(home+'/.bashrc', "a")
  bashrc.write(lines[0])
  bashrc.write(lines[1])
  bashrc.write(lines[2])
  bashrc.close()
  print 'Lines appended.'
