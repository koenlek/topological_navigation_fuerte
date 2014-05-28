#!/usr/bin/env python

import random
import math
import pylab

def generate_leafs(k, depth, centers = [(0.0, 0.0)]):
  if depth == 0:
    return centers
  radius = 10 ** depth
  angle = 2*math.pi/k
  new_centers = []
  for c in centers:
    new_centers.extend( [(c[0] + radius*math.cos(i*angle), c[1] + radius*math.sin(i*angle)) for i in range(k)] )
  return generate_leafs(k, depth - 1, new_centers)

if __name__ == "__main__":
  k = 5          # branching factor
  depth = 3      # depth
  samples = 100  # number of random samples at each leaf
  centers = generate_leafs(k, depth)
  for i in range(samples):
    for c in centers:
      for coord in c:
        print random.gauss(coord, 1.0),
      print

  pylab.scatter([c[0] for c in centers], [c[1] for c in centers])
  pylab.title("Centers (exact)")
  pylab.show()
