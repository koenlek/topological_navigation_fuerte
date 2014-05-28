#!/bin/bash          
#
# Script to cleanup rosmake ws
#
# Koen Lekkerkerker
# 25 May 2014 
#

mv .gitignore gitignore_tmp
mv .rosinstall rosinstall_tmp

find . -name '*~' | xargs rm -v
find . -name 'cmake_install.cmake' | xargs rm -v

find . -name '.*' | xargs rm -r -v #would also remove .gitignore...
find . -name 'srv_gen' | xargs rm -r -v
find . -name 'msg_gen' | xargs rm -r -v
find . -name 'build' | xargs rm -r -v
find . -name 'bin' | xargs rm -r -v
find . -name 'lib' | xargs rm -r -v
find . -name '_gtest_from_src' | xargs rm -r -v

rm -r graph_mapping/laser_slam/data

mv gitignore_tmp .gitignore
mv rosinstall_tmp .rosinstall
