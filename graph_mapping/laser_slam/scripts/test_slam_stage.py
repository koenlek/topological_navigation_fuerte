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
    

def main(path, max_time):
    loc = setup_dir(path)
    print('Using output directory {0}'.format(loc))

    # Start rosbag
    rosbag_outfile = open(op.join(loc, 'rosbag.out'), 'w')
    args = ['rosbag', 'record', '-a', '-O{0}'.format(op.join(loc, 'slam.bag'))]
    rosbag = sp.Popen(args, shell=False, stdout=rosbag_outfile, stderr=sp.STDOUT)
    print('Invoked {0}'.format(' '.join(args)))

    # Start roslaunch
    roslaunch_outfile = open(op.join(loc, 'roslaunch.out'), 'w')
    args = ['roslaunch', 'laser_slam', 'move_base_slam_5cm.launch',  'graph_loc:={0}'.\
            format(op.join(loc, 'graph'))]
    slam_launch = sp.Popen(args, shell=False, stdout=roslaunch_outfile,
                           stderr=sp.STDOUT, bufsize=1)
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
            rosbag.terminate()
            break
        if (rosbag.poll()):
            print('Rosbag exited.')
            slam_launch.terminate()
            break
        if (slam_launch.poll()):
            print('roslaunch exited.')
            rosbag.terminate()
            break

    
    

if __name__ == "__main__":
    parser = optparse.OptionParser()
    parser.add_option('-d', '--output_dir', dest='path',
                      action='store', type='string', default='~/.ros/laser_slam')
    parser.add_option('-t', '--time', dest='time', action='store', type='int',
                      default=0)
    (options, args) = parser.parse_args()
    main(options.path, options.time)
