#include <ros/ros.h>
#include <sba/sba.h>
#include <sba/visualization.h>

// Bring in gtest
#include <gtest/gtest.h>

// For random seed.
#include <time.h>

using namespace sba;
using namespace std;

const double PI = 3.141592;

class Plane
{
  public:
    vector<Point, Eigen::aligned_allocator<Point> > points;
    Eigen::Vector3d normal;
    
    void rotate(const Eigen::Quaterniond& qrot)
    {
      Eigen::Matrix3d rotmat = qrot.toRotationMatrix();
      
      for (unsigned int i = 0; i < points.size(); i++)
      {
        points[i].head<3>() = rotmat*points[i].head<3>();
      }
      
      normal = rotmat*normal;
    }
    
    void rotate(double angle, double x, double y, double z)
    {
      Eigen::AngleAxis<double> angleaxis(angle, Vector3d(x, y, z));
      rotate(Eigen::Quaterniond(angleaxis));
    }
         
    void translate(const Eigen::Vector3d& trans)
    {
      for (unsigned int i = 0; i < points.size(); i++)
      {
        points[i].head<3>() += trans;
      }
    }
    
    void translate(double x, double y, double z)
    {
      Vector3d trans(x, y, z);
      translate(trans);
    }
    
    // Creates a plane with origin at (0,0,0) and opposite corner at (width, height, 0).
    void resize(double width, double height, int nptsx, int nptsy)
    {
      for (int ix = 0; ix < nptsx ; ix++)
      {
        for (int iy = 0; iy < nptsy ; iy++)
        {
          // Create a point on the plane in a grid.
          points.push_back(Point(width/nptsx*(ix+.5), -height/nptsy*(iy+.5), 0.0, 1.0));
        }
      }
      
      normal << 0, 0, -1;
    }
};

class SBAPointPlaneTest : public :: testing::Test
{
  public:
    // actual SBA object
    SysSBA sys;
    
    // Vector containing the true point positions.
    vector<Point, Eigen::aligned_allocator<Point> > points;
    
    void SetUp();
    
};

void SBAPointPlaneTest::SetUp()
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
    Plane middleplane;
    middleplane.resize(3, 2, 10, 5);
    middleplane.translate(0.0, 0.0, 5.0);
    
    Plane leftplane;
    leftplane.resize(1, 2, 6, 12);
    leftplane.rotate(-PI/4.0, 0, 1, 0);
    leftplane.translate(0, 0, 5.0);
    
    Plane rightplane;
    rightplane.resize(1, 2, 6, 12);
    rightplane.rotate(PI/4.0, 0, 1, 0);
    rightplane.translate(2, 0, 5.0);
    
    Plane topplane;
    topplane.resize(1, 1.5, 6, 12);
    topplane.rotate(PI/4.0, 1, 0, 0);
    topplane.translate(2, 0, 5.0);

    // Vector containing the true point positions.
    rightplane.normal = rightplane.normal; 
  
    vector<Point, Eigen::aligned_allocator<Point> > points;
    vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > normals;
    
    points.insert(points.end(), middleplane.points.begin(), middleplane.points.end());
    normals.insert(normals.end(), middleplane.points.size(), middleplane.normal);
    
    points.insert(points.end(), leftplane.points.begin(), leftplane.points.end());
    normals.insert(normals.end(), leftplane.points.size(), leftplane.normal);
    
    points.insert(points.end(), rightplane.points.begin(), rightplane.points.end());
    normals.insert(normals.end(), rightplane.points.size(), rightplane.normal);
    
    points.insert(points.end(), topplane.points.begin(), topplane.points.end());
    normals.insert(normals.end(), topplane.points.size(), topplane.normal);
    
    // Create nodes and add them to the system.
    unsigned int nnodes = 2; // Number of nodes.
    double path_length = 2; // Length of the path the nodes traverse.
    
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
    
    double pointnoise = 1.0;
    
    // Add points into the system, and add noise.
    for (i = 0; i < points.size(); i++)
    {
      // Add up to .5 points of noise.
      Vector4d temppoint = points[i];
      temppoint.x() += pointnoise*(drand48() - 0.5);
      temppoint.y() += pointnoise*(drand48() - 0.5);
      temppoint.z() += pointnoise*(drand48() - 0.5);
      sys.addPoint(temppoint);
    }
    
    Vector2d proj2d;
    Vector3d proj, pc, baseline;
    
    Vector3d imagenormal(0, 0, 1);
    
    Matrix3d covar0;
    covar0 << sqrt(imagenormal(0)), 0, 0, 
              0, sqrt(imagenormal(1)), 0, 
              0, 0, sqrt(imagenormal(2));
    Matrix3d covar;
    
    Quaterniond rotation;
    Matrix3d rotmat;
    
    printf("Normal for Middle Plane: [%f %f %f]\n", 
      middleplane.normal.x(), middleplane.normal.y(), middleplane.normal.z());
    printf("Normal for Left Plane:   [%f %f %f]\n", 
      leftplane.normal.x(), leftplane.normal.y(), leftplane.normal.z());
    printf("Normal for Right Plane:  [%f %f %f]\n", 
      rightplane.normal.x(), rightplane.normal.y(), rightplane.normal.z());
    
    // Project points into nodes.
    for (i = 0; i < points.size(); i++)
    {
      for (j = 0; j < sys.nodes.size(); j++)
      {
        // Project the point into the node's image coordinate system.
        sys.nodes[j].setProjection();
        sys.nodes[j].project2im(proj2d, points[i]);
        
        
        // Camera coords for right camera
        baseline << sys.nodes[j].baseline, 0, 0;
        pc = sys.nodes[j].Kcam * (sys.nodes[j].w2n*points[i] - baseline); 
        proj.head<2>() = proj2d;
        proj(2) = pc(0)/pc(2);
        
        // If valid (within the range of the image size), add the stereo 
        // projection to SBA.
        if (proj.x() > 0 && proj.x() < maxx && proj.y() > 0 && proj.y() < maxy)
        {
          sys.addStereoProj(j, i, proj);
          
          // Create the covariance matrix: 
          // image plane normal = [0 0 1]
          // wall normal = [0 0 -1]
          // covar = (R)T*[0 0 0;0 0 0;0 0 1]*R
          
          rotation.setFromTwoVectors(imagenormal, normals[i]);
          rotmat = rotation.toRotationMatrix();
          covar = rotmat.transpose()*covar0*rotmat;
          
          if (!(i % sys.nodes.size() == j))
            sys.setProjCovariance(j, i, covar);
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

TEST_F(SBAPointPlaneTest, WriteRead)
{
    // Perform SBA with 10 iterations, an initial lambda step-size of 1e-3, 
    // and using CSPARSE.
    sys.doSBA(10, 1e-3, SBA_SPARSE_CHOLESKY);
    //testsys.doSBA(10, 1e-4, SBA_SPARSE_CHOLESKY);
    
    for (unsigned int i = 0; i < points.size(); i++)
    {
      EXPECT_NEAR(points[i].x(), sys.tracks[i].point.x(), 0.01);
      EXPECT_NEAR(points[i].y(), sys.tracks[i].point.y(), 0.01);
      EXPECT_NEAR(points[i].z(), sys.tracks[i].point.z(), 0.01);
    }
    
    // Don't check nodes yet because we don't store node information. To do!    
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

