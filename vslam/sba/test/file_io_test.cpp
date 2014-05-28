/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

// test fixture

// Bring in SBA and file IO
#include <sba/sba.h>
#include <sba/sba_file_io.h>

// Bring in gtest
#include <gtest/gtest.h>

// For random seed.
#include <time.h>

using namespace sba;
using namespace std;

class SBAFileIOTest : public :: testing::Test
{
  public:
    // actual SBA object
    SysSBA sys;
    
    // Vector containing the true point positions.
    vector<Point, Eigen::aligned_allocator<Point> > points;
    
    void SetUp();
    
};

void SBAFileIOTest::SetUp()
{
    // Create camera parameters.
    frame_common::CamParams cam_params;
    cam_params.fx = 430; // Focal length in x
    cam_params.fy = 430; // Focal length in y
    cam_params.cx = 320; // X position of principal point
    cam_params.cy = 240; // Y position of principal point
    cam_params.tx = 0;   // Baseline (no baseline since this is monocular)

    // Define dimensions of the image.
    int maxx = 640;
    int maxy = 480;

    // Create a plane containing a wall of points.
    int npts_x = 10; // Number of points on the plane in x
    int npts_y = 5;  // Number of points on the plane in y
    
    double plane_width = 5;     // Width of the plane on which points are positioned (x)
    double plane_height = 2.5;    // Height of the plane on which points are positioned (y)
    double plane_distance = 5; // Distance of the plane from the cameras (z)

    for (int ix = 0; ix < npts_x ; ix++)
    {
      for (int iy = 0; iy < npts_y ; iy++)
      {
        // Create a point on the plane in a grid.
        points.push_back(Point(plane_width/npts_x*(ix+.5), -plane_height/npts_y*(iy+.5), plane_distance, 1.0));
      }
    }
    
    // Create nodes and add them to the system.
    unsigned int nnodes = 5; // Number of nodes.
    double path_length = 3; // Length of the path the nodes traverse.
    
    unsigned int i = 0, j = 0;
    
    for (i = 0; i < nnodes; i++)
    { 
      // Translate in the x direction over the node path.
      Vector4d trans(i/(nnodes-1.0)*path_length, 0, 0, 1);
            
      // Don't rotate.
      Quaterniond rot(1, 0, 0, 0);
      rot.normalize();
      
      // Add a new node to the system.
      sys.addNode(trans, rot, cam_params, false);
    }
        
    // Set the random seed.
    unsigned short seed = (unsigned short)time(NULL);
    seed48(&seed);
        
    // Add points into the system, and add noise.
    for (i = 0; i < points.size(); i++)
    {
      // Add up to .5 points of noise.
      Vector4d temppoint = points[i];
      temppoint.x() += drand48() - 0.5;
      temppoint.y() += drand48() - 0.5;
      temppoint.z() += drand48() - 0.5;
      sys.addPoint(temppoint);
    }
    
    Vector2d proj;
    
    // Project points into nodes.
    for (i = 0; i < points.size(); i++)
    {
      for (j = 0; j < sys.nodes.size(); j++)
      {
        // Project the point into the node's image coordinate system.
        sys.nodes[j].setProjection();
        sys.nodes[j].project2im(proj, points[i]);
        
        // If valid (within the range of the image size), add the monocular 
        // projection to SBA.
        if (proj.x() > 0 && proj.x() < maxx && proj.y() > 0 && proj.y() < maxy)
        {
          sys.addMonoProj(j, i, proj);
        }
      }
    }
    
    // Add noise to node position.
    
    double transscale = 1.0;
    double rotscale = 0.2;
    
    // Don't actually add noise to the first node, since it's fixed.
    for (i = 1; i < sys.nodes.size(); i++)
    {
      Vector4d temptrans = sys.nodes[i].trans;
      Quaterniond tempqrot = sys.nodes[i].qrot;
      
      // Add error to both translation and rotation.
      temptrans.x() += transscale*(drand48() - 0.5);
      temptrans.y() += transscale*(drand48() - 0.5);
      temptrans.z() += transscale*(drand48() - 0.5);
      tempqrot.x() += rotscale*(drand48() - 0.5);
      tempqrot.y() += rotscale*(drand48() - 0.5);
      tempqrot.z() += rotscale*(drand48() - 0.5);
      tempqrot.normalize();
      
      sys.nodes[i].trans = temptrans;
      sys.nodes[i].qrot = tempqrot;
      
      // These methods should be called to update the node.
      sys.nodes[i].normRot();
      sys.nodes[i].setTransform();
      sys.nodes[i].setProjection();
      sys.nodes[i].setDr(true);
    }
        
}

TEST_F(SBAFileIOTest, WriteRead)
{
    // Create an empty SBA system.
    // SysSBA temp_sys;
    // SysSBA sys;
    SysSBA testsys;
    
    const char *filename = "file_io_test.out";
    
    writeBundlerFile(filename, sys);
    readBundlerFile(filename, testsys);
    
    // Perform SBA with 10 iterations, an initial lambda step-size of 1e-3, 
    // and using CSPARSE.
    testsys.doSBA(10, 1e-3, SBA_SPARSE_CHOLESKY);
    testsys.doSBA(10, 1e-4, SBA_SPARSE_CHOLESKY);
    
    for (unsigned int i = 0; i < points.size(); i++)
    {
      EXPECT_NEAR(points[i].x(), testsys.tracks[i].point.x(), 0.2);
      EXPECT_NEAR(points[i].y(), testsys.tracks[i].point.y(), 0.2);
      EXPECT_NEAR(points[i].z(), testsys.tracks[i].point.z(), 0.2);
    }
    
    // Don't check nodes yet because we don't store node information. To do!    
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

