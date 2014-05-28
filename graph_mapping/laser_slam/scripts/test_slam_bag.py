#!/usr/bin/env python

# Run laser slam in stage with exploration and take a bag of the results
#
# Author: Bhaskara Marthi

import subprocess as sp
import os.path as op
import os
import optparse
import time

def make_dirname(i):
    return 'laser_slam.{0}'.format(i)

def setup_dir(path):
    """
    Set up new location for storing output files
    """
    path = op.expanduser(op.expandvars(path))
    if not op.exists(path):
        os.makedirs(path)
    i = 0
    loc = op.join(path, make_dirname(i))
    while op.exists(loc):
        i += 1
        loc = op.join(path, make_dirname(i))
    loc = op.join(path, make_dirname(i))
    os.mkdir(loc)
    latest = op.join(path, 'latest')
    if op.exists(latest):
        os.remove(latest)
    os.symlink(loc, latest)
    return loc
    

def main(path, max_time, bag):
    loc = setup_dir(path)
    print('Using output directory {0}'.format(loc))

    env = os.environ.copy()
    env['PYTHONUNBUFFERED'] = 'True'
    
    # Start roslaunch
    roslaunch_outfile = open(op.join(loc, 'roslaunch.out'), 'w')
    args = ['roslaunch', 'laser_slam', 'laser_slam_bag.launch',
            'graph_loc:={0}'.format(op.join(loc, 'graph')),
            'bag:={0}'.format(op.expanduser(bag))]
    slam_launch = sp.Popen(args, shell=False, stdout=roslaunch_outfile,
                           stderr=sp.STDOUT, bufsize=1, env=env)
    print('Invoked {0}'.format(' '.join(args)))


    i = 0
    while True:
        time.sleep(1)
        i += 1
        if i % 100 == 0:
            print('t = {0}'.format(i))
        if max_time > 0 and i > max_time:
            print('Time limit reached')
            slam_launch.terminate()
            break
        if (slam_launch.poll()):
            print('roslaunch exited.')
            break

    
    

if __name__ == "__main__":
    parser = optparse.OptionParser()
    parser.add_option('-d', '--output_dir', dest='path',
                      action='store', type='string', default='~/.ros/laser_slam')
    parser.add_option('-t', '--time', dest='time', action='store', type='int',
                      default=0)
    parser.add_option('-b', '--bag', dest='bag',
                      action='store', type='string')
    (options, args) = parser.parse_args()
    main(options.path, options.time, options.bag)
