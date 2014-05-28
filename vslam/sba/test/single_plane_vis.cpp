#include <ros/ros.h>
#include <sba/sba.h>
#include <sba/visualization.h>


// For random seed.
#include <time.h>

#include <visualization_msgs/Marker.h>

using namespace sba;
using namespace std;

const double PI = 3.141592;

int addPointAndProjection(SysSBA& sba, vector<Point, Eigen::aligned_allocator<Point> >& points, int ndi);
void calculateProj(SysSBA& sba, Point& point, int ndi, Vector3d& proj);

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


void setupSBA(SysSBA &sba)
{
    // Create camera parameters.
    frame_common::CamParams cam_params;
    cam_params.fx = 430; // Focal length in x
    cam_params.fy = 430; // Focal length in y
    cam_params.cx = 320; // X position of principal point
    cam_params.cy = 240; // Y position of principal point
    cam_params.tx = -30; // Baseline (no baseline since this is monocular)

    // Create a plane containing a wall of points.
    Plane plane0, plane1;
    plane0.resize(3, 2, 10, 5);
    
    plane1 = plane0;
    plane1.translate(0.1, 0.05, 0.0);
    
    plane1.rotate(PI/4.0, 1, 0, 0);
    plane1.translate(0.0, 0.0, 7.0);
    
    
    plane0.rotate(PI/4.0, 1, 0, 0);
    plane0.translate(0.0, 0.0, 7.0);
    
    //plane1.translate(0.05, 0.0, 0.05);
    
    // Create nodes and add them to the system.
    unsigned int nnodes = 2; // Number of nodes.
    double path_length = 2; // Length of the path the nodes traverse.

    // Set the random seed.
    unsigned short seed = (unsigned short)time(NULL);
    seed48(&seed);
    
    for (int i = 0; i < nnodes; i++)
    { 
      // Translate in the x direction over the node path.
      Vector4d trans(i/(nnodes-1.0)*path_length, 0, 0, 1);
            
#if 0
      if (i >= 0)
	    {
	      // perturb a little
	      double tnoise = 0.5;	// meters
	      trans.x() += tnoise*(drand48()-0.5);
	      trans.y() += tnoise*(drand48()-0.5);
	      trans.z() += tnoise*(drand48()-0.5);
	    }
#endif

      // Don't rotate.
      Quaterniond rot(1, 0, 0, 0);
#if 0
      if (i > 0)
	    {
	      // perturb a little
	      double qnoise = 0.1;	// meters
	      rot.x() += qnoise*(drand48()-0.5);
	      rot.y() += qnoise*(drand48()-0.5);
	      rot.z() += qnoise*(drand48()-0.5);
	    }
#endif
      rot.normalize();
      
      // Add a new node to the system.
      sba.addNode(trans, rot, cam_params, false);
    }
    
    Vector3d imagenormal(0, 0, 1);
    
    Matrix3d covar0;
    covar0 << sqrt(imagenormal(0)), 0, 0, 0, sqrt(imagenormal(1)), 0, 0, 0, sqrt(imagenormal(2));
    Matrix3d covar;
    
    Quaterniond rotation;
    Matrix3d rotmat;
    
    // Project points into nodes.
    addPointAndProjection(sba, plane0.points, 0);
    addPointAndProjection(sba, plane1.points, 1);
    
    int offset = plane0.points.size();
    
    Vector3d normal0 = sba.nodes[0].qrot.toRotationMatrix().transpose()*plane0.normal; 
    Vector3d normal1 = sba.nodes[1].qrot.toRotationMatrix().transpose()*plane1.normal; 
    
    printf("Normal: %f %f %f -> %f %f %f\n", plane0.normal.x(), plane0.normal.y(), plane0.normal.z(), normal0.x(), normal0.y(), normal0.z());
    printf("Normal: %f %f %f -> %f %f %f\n", plane1.normal.x(), plane1.normal.y(), plane1.normal.z(), normal1.x(), normal1.y(), normal1.z());
    
    for (int i = 0; i < plane0.points.size(); i++)
    {
      sba.addPointPlaneMatch(0, i, normal0, 1, i+offset, normal1);

      Matrix3d covar;
      covar << 0.1, 0, 0,
                0, 0.1, 0, 
          	0, 0, 0.1;
      sba.setProjCovariance(0, i+offset, covar);
      sba.setProjCovariance(1, i, covar);
    }
    
    // Add noise to node position.
    
    double transscale = 1.0;
    double rotscale = 0.1;
    
    // Don't actually add noise to the first node, since it's fixed.
    for (int i = 1; i < sba.nodes.size(); i++)
    {
      Vector4d temptrans = sba.nodes[i].trans;
      Quaterniond tempqrot = sba.nodes[i].qrot;
      
      // Add error to both translation and rotation.
      temptrans.x() += transscale*(drand48() - 0.5);
      temptrans.y() += transscale*(drand48() - 0.5);
      temptrans.z() += transscale*(drand48() - 0.5);
      tempqrot.x() += rotscale*(drand48() - 0.5);
      tempqrot.y() += rotscale*(drand48() - 0.5);
      tempqrot.z() += rotscale*(drand48() - 0.5);
      tempqrot.normalize();
      
      sba.nodes[i].trans = temptrans;
      sba.nodes[i].qrot = tempqrot;
      
      // These methods should be called to update the node.
      sba.nodes[i].normRot();
      sba.nodes[i].setTransform();
      sba.nodes[i].setProjection();
      sba.nodes[i].setDr(true);
    }
}

int addPointAndProjection(SysSBA& sba, vector<Point, Eigen::aligned_allocator<Point> >& points, int ndi)
{
    // Define dimensions of the image.
    int maxx = 640;
    int maxy = 480;

    // Project points into nodes.
    for (int i = 0; i < points.size(); i++)
    {
      double pointnoise = 0.1;
  
      // Add points into the system, and add noise.
      // Add up to .5 pixels of noise.
      Vector4d temppoint = points[i];
      temppoint.x() += pointnoise*(drand48() - 0.5);
      temppoint.y() += pointnoise*(drand48() - 0.5);
      temppoint.z() += pointnoise*(drand48() - 0.5);
      int index = sba.addPoint(temppoint);
    
      Vector3d proj;
      calculateProj(sba, points[i], ndi, proj);
      
      // If valid (within the range of the image size), add the stereo 
      // projection to SBA.
      //if (proj.x() > 0 && proj.x() < maxx && proj.y() > 0 && proj.y() < maxy)
      //{
        sba.addStereoProj(ndi, index, proj);
      //}
    }
    
    
    return sba.tracks.size() - points.size();
}

void calculateProj(SysSBA& sba, Point& point, int ndi, Vector3d& proj)
{
    Vector2d proj2d;
    Vector3d pc, baseline;
    // Project the point into the node's image coordinate system.
    sba.nodes[ndi].setProjection();
    sba.nodes[ndi].project2im(proj2d, point);
    
    // Camera coords for right camera
    baseline << sba.nodes[ndi].baseline, 0, 0;
    pc = sba.nodes[ndi].Kcam * (sba.nodes[ndi].w2n*point - baseline); 
    proj.head<2>() = proj2d;
    proj(2) = pc(0)/pc(2);
}

void processSBA(ros::NodeHandle node)
{
    // Create publisher topics.
    ros::Publisher cam_marker_pub = node.advertise<visualization_msgs::Marker>("/sba/cameras", 1);
    ros::Publisher point_marker_pub = node.advertise<visualization_msgs::Marker>("/sba/points", 1);
    ros::spinOnce();
    
    //ROS_INFO("Sleeping for 2 seconds to publish topics...");
    ros::Duration(0.2).sleep();
    
    // Create an empty SBA system.
    SysSBA sba;
    
    setupSBA(sba);
    
    // Provide some information about the data read in.
    unsigned int projs = 0;
    // For debugging.
    for (int i = 0; i < (int)sba.tracks.size(); i++)
    {
      projs += sba.tracks[i].projections.size();
    }
    ROS_INFO("SBA Nodes: %d, Points: %d, Projections: %d", (int)sba.nodes.size(),
      (int)sba.tracks.size(), projs);
        
    //ROS_INFO("Sleeping for 5 seconds to publish pre-SBA markers.");
    //ros::Duration(5.0).sleep();
        
    // Perform SBA with 10 iterations, an initial lambda step-size of 1e-3, 
    // and using CSPARSE.
    /* sba.doSBA(20, 1e-4, SBA_SPARSE_CHOLESKY);
    
    int npts = sba.tracks.size();

    ROS_INFO("Bad projs (> 10 pix): %d, Cost without: %f", 
        (int)sba.countBad(10.0), sqrt(sba.calcCost(10.0)/npts));
    ROS_INFO("Bad projs (> 5 pix): %d, Cost without: %f", 
        (int)sba.countBad(5.0), sqrt(sba.calcCost(5.0)/npts));
    ROS_INFO("Bad projs (> 2 pix): %d, Cost without: %f", 
        (int)sba.countBad(2.0), sqrt(sba.calcCost(2.0)/npts));
    
    ROS_INFO("Cameras (nodes): %d, Points: %d",
        (int)sba.nodes.size(), (int)sba.tracks.size());
        
    // Publish markers
    drawGraph(sba, cam_marker_pub, point_marker_pub);
    ros::spinOnce();
    //ROS_INFO("Sleeping for 2 seconds to publish post-SBA markers.");
    ros::Duration(0.2).sleep(); */
    
    // Perform SBA with 1 iteration, an initial lambda step-size of 1e-3, 
    // and using CSPARSE.
    /* sba.doSBA(1, 1e-4, SBA_SPARSE_CHOLESKY);
    
    int npts = sba.tracks.size();

    ROS_INFO("Bad projs (> 10 pix): %d, Cost without: %f", 
        (int)sba.countBad(10.0), sqrt(sba.calcCost(10.0)/npts));
    ROS_INFO("Bad projs (> 5 pix): %d, Cost without: %f", 
        (int)sba.countBad(5.0), sqrt(sba.calcCost(5.0)/npts));
    ROS_INFO("Bad projs (> 2 pix): %d, Cost without: %f", 
        (int)sba.countBad(2.0), sqrt(sba.calcCost(2.0)/npts));
    
    ROS_INFO("Cameras (nodes): %d, Points: %d",
        (int)sba.nodes.size(), (int)sba.tracks.size()); */
        
    // Publish markers
    drawGraph(sba, cam_marker_pub, point_marker_pub, 1, sba.tracks.size()/2);
    ros::spinOnce();
    ros::Duration(0.5).sleep();

    for (int j=0; j<20; j++)
      {
        if (!ros::ok())
	        break;
	      sba.doSBA(1, 0, SBA_SPARSE_CHOLESKY);
	      drawGraph(sba, cam_marker_pub, point_marker_pub, 1, sba.tracks.size()/2);
	      ros::spinOnce();
	      ros::Duration(0.2).sleep();
      }
    
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sba_system_setup");
    
    ros::NodeHandle node;
    
    processSBA(node);
    ros::spinOnce();

    return 0;
}
