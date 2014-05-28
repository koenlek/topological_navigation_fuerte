#!/usr/bin/env python

import random

if __name__ == "__main__":
  samples = 1000
  centers = [(0,0,0), (10,10,10), (10,5,0)]
  for i in range(samples):
    for c in centers:
      for coord in c:
        print random.gauss(coord, 1.0),
      print
