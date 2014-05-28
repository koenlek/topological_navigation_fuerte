#!/usr/bin/env python

from pylab import *

C = load('centers.txt')
level0 = C[:5]
level1 = C[5:30]
level2 = C[30:155]

scatter(level2[:,:1], level2[:,1:2], c='g')
scatter(level1[:,:1], level1[:,1:2], c='r')
scatter(level0[:,:1], level0[:,1:2])
show()
