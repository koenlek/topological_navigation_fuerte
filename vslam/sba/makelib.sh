#!/bin/bash
# shell script for making a source distribution of the sSBA library

DISTDIR=ssba-0.1

mkdir -p ${DISTDIR} ${DISTDIR}/src ${DISTDIR}/include/ ${DISTDIR}/include/sba \
         ${DISTDIR}/include/bpcg ${DISTDIR}/obj ${DISTDIR}/lib \
         ${DISTDIR}/examples ${DISTDIR}/data ${DISTDIR}/bin

cp src/spa2d.cpp src/csparse.cpp src/proj.cpp src/sba.cpp src/spa.cpp src/node.cpp src/sba_file_io.cpp ${DISTDIR}/src
cp ../bpcg/include/bpcg/bpcg.h ${DISTDIR}/include/bpcg
cp include/sba/sba.h include/sba/csparse.h include/sba/proj.h include/sba/sba_file_io.h \
   include/sba/node.h include/sba/spa2d.h include/sba/sba_setup.h include/sba/read_spa.h ${DISTDIR}/include/sba
cp test/run_sba_graph_file.cpp ${DISTDIR}/examples/run_sba.cpp
cp test/run_spa_graph_file.cpp ${DISTDIR}/examples/run_spa.cpp
cp data/*.graph ${DISTDIR}/data
cp Makefile-lib ${DISTDIR}/Makefile
#tar -cvzf ${DISTDIR}.tgz ${DISTDIR}

# Making a standalone sSBA library

# Dependencies
# ============

# External libs
#    Eigen3  (LGPL3 license)
#    CSparse (LGPL license)
#    CHOLMOD/SuiteSparse (optional, uses LGPL license for CHOLMOD)



# Include file setup
# ==================

# sba/include/sba                 spa2d.h, csparse.h
# csparse/include/CSparse         csparse.h


# Compile flags
# =============
# DSIF (include delayed state filter)
# CHOLMOD (include CHOLMOD libraries)


# API
# ===

# // make an SPA object
# SysSPA2d SysSPA2d()  

# // add nodes and constraints
# int SysSPA2d::addNode2d(Vector3d &pos)
# bool SysSPA2d::addConstraint2d(int nd0, int nd1, Vector3d &mean, Matrix3d &prec)

# // run the optimizer
# int SysSPA2d::doSPA(int iter, double lambda=1.0e-4)

# // return cost and nodes
# double SysSPA2d::getCost()
# std::vector<Node2d,Eigen::aligned_allocator<Node2d> > SysSPA2d::getNodes()


