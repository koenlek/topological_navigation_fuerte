#!/bin/bash

#
# runs the Lourakis/Argyros system over a set of files in a directory
#

for F in *cams.txt; 
do 
  echo "$F"
  ~/devel64/sba-1.6/demo/eucsbademo ${F} ${F/cams/pts} ${F/cams/calib};
done
