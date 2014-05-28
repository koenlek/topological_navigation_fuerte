#include <ros/ros.h>
#include <sba/sba.h>
#include <sba/visualization.h>


// For random seed.
#include <time.h>

#include <visualization_msgs/Marker.h>

using namespace sba;
using namespace std;

const double PI = 3.141592;

#define USE_PPL
#define USE_PP

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
      double w2 = width/2.0;
      double h2 = height/2.0;
      for (int ix = 0; ix < nptsx ; ix++)
      {
        for (int iy = 0; iy < nptsy ; iy++)
        {
          // Create a point on the plane in a grid.
          points.push_back(Point(width/nptsx*(ix+.5)-w2, -height/nptsy*(iy+.5)+h2, 0.0, 1.0));
        }
      }
      
      normal << 0, 0, -1;
    }
};


// this needs to be global
vector<Point, Eigen::aligned_allocator<Point> > points;

void setupSBA(SysSBA &sys)
{
    // Create camera parameters.
    frame_common::CamParams cam_params;
    cam_params.fx = 430; // Focal length in x
    cam_params.fy = 430; // Focal length in y
    cam_params.cx = 320; // X position of principal point
    cam_params.cy = 240; // Y position of principal point
    cam_params.tx = 0.3; // Baseline 

    // Define dimensions of the image.
    int maxx = 640;
    int maxy = 480;

    // Create a plane containing a wall of points.
    Plane middleplane;
    middleplane.resize(3, 2, 10, 5);
    //middleplane.rotate(PI/4.0, PI/6.0, 1, 0);
    middleplane.rotate(PI/4.0, 1, 0, 1);
    middleplane.translate(0.0, 0.0, 5.0);
    
    // Create another plane containing a wall of points.
    Plane mp2;
    mp2.resize(3, 2, 10, 5);
    mp2.rotate(0, 0, 0, 1);
    mp2.translate(0.0, 0.0, 4.0);


    // Create another plane containing a wall of points.
    Plane mp3;
    mp3.resize(3, 2, 10, 5);
    mp3.rotate(-PI/4.0, 1, 0, 1);
    mp3.translate(0.0, 0.0, 4.5);



    // Vector containing the true point positions.
    vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > normals;
    
    points.insert(points.end(), middleplane.points.begin(), middleplane.points.end());
    normals.insert(normals.end(), middleplane.points.size(), middleplane.normal);
    points.insert(points.end(), mp2.points.begin(), mp2.points.end());
    normals.insert(normals.end(), mp2.points.size(), mp2.normal);
    points.insert(points.end(), mp3.points.begin(), mp3.points.end());
    normals.insert(normals.end(), mp3.points.size(), mp3.normal);
    
    // Create nodes and add them to the system.
    unsigned int nnodes = 2; // Number of nodes.
    double path_length = 1.0; // Length of the path the nodes traverse.

    // Set the random seed.
    unsigned short seed = (unsigned short)time(NULL);
    seed48(&seed);
    
    unsigned int i = 0;
    
    Vector3d inormal0 = middleplane.normal;
    Vector3d inormal1 = middleplane.normal;
    Vector3d inormal20 = mp2.normal;
    Vector3d inormal21 = mp2.normal;
    Vector3d inormal30 = mp3.normal;
    Vector3d inormal31 = mp3.normal;
    
    for (i = 0; i < nnodes; i++)
    { 
      // Translate in the x direction over the node path.
      Vector4d trans(i/(nnodes-1.0)*path_length, 0, 0, 1);
            
#if 1
      if (i >= 2)
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
#if 1
      if (i >= 2)
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
      sys.addNode(trans, rot, cam_params, false);
      
      // set normal
      if (i == 0)
	{
	  inormal0 = rot.toRotationMatrix().transpose() * inormal0;
	  inormal20 = rot.toRotationMatrix().transpose() * inormal20;
	  inormal30 = rot.toRotationMatrix().transpose() * inormal30;
	}
      else
	{
	  inormal1 = rot.toRotationMatrix().transpose() * inormal1;
	  inormal21 = rot.toRotationMatrix().transpose() * inormal21;
	  inormal31 = rot.toRotationMatrix().transpose() * inormal31;
	}
    }
        
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
    
    // Each point gets added twice
    for (i = 0; i < points.size(); i++)
    {
      // Add up to .5 points of noise.
      Vector4d temppoint = points[i];
      temppoint.x() += pointnoise*(drand48() - 0.5);
      temppoint.y() += pointnoise*(drand48() - 0.5);
      temppoint.z() += pointnoise*(drand48() - 0.5);
      sys.addPoint(temppoint);
    }

    Vector2d proj2d, proj2dp;
    Vector3d proj, projp, pc, pcp, baseline, baselinep;
    
    Vector3d imagenormal(0, 0, 1);
    
    Matrix3d covar0;
    covar0 << sqrt(imagenormal(0)), 0, 0, 0, sqrt(imagenormal(1)), 0, 0, 0, sqrt(imagenormal(2));
    Matrix3d covar;
    
    Quaterniond rotation;
    Matrix3d rotmat;
    
    unsigned int midindex = middleplane.points.size();
    printf("Normal for Middle Plane: [%f %f %f], index %d -> %d\n", middleplane.normal.x(), middleplane.normal.y(), middleplane.normal.z(), 0, midindex-1);
    
    int nn = points.size();

    // Project points into nodes.
    for (i = 0; i < points.size(); i++)
    {
      int k=i;
      // Project the point into the node's image coordinate system.
      sys.nodes[0].setProjection();
      sys.nodes[0].project2im(proj2d, points[k]);
      sys.nodes[1].setProjection();
      sys.nodes[1].project2im(proj2dp, points[k]);
        
      // Camera coords for right camera
      baseline << sys.nodes[0].baseline, 0, 0;
      pc = sys.nodes[0].Kcam * (sys.nodes[0].w2n*points[k] - baseline); 
      proj.head<2>() = proj2d;
      proj(2) = pc(0)/pc(2);
        
      baseline << sys.nodes[1].baseline, 0, 0;
      pcp = sys.nodes[1].Kcam * (sys.nodes[1].w2n*points[k] - baseline); 
      projp.head<2>() = proj2dp;
      projp(2) = pcp(0)/pcp(2);

      // add noise to projections
      double prnoise = 0.5;	// 0.5 pixels
      proj.head<2>() += Vector2d(prnoise*(drand48() - 0.5),prnoise*(drand48() - 0.5));
      projp.head<2>() += Vector2d(prnoise*(drand48() - 0.5),prnoise*(drand48() - 0.5));


      // If valid (within the range of the image size), add the stereo 
      // projection to SBA.
      if (proj.x() > 0 && proj.x() < maxx && proj.y() > 0 && proj.y() < maxy)
        {
          // add point cloud shape-holding projections to each node
          sys.addStereoProj(0, k, proj);
          sys.addStereoProj(1, k+nn, projp);

#ifdef USE_PP
          // add exact matches
          if (i == 20 || i == 65 || i == 142)
            sys.addStereoProj(1, k, projp);
#endif

#ifdef USE_PPL
          // add point-plane matches
	  if (i < midindex)
            sys.addPointPlaneMatch(0, k, inormal0, 1, k+nn, inormal1);
	  else if (i < 2*midindex)
	    sys.addPointPlaneMatch(0, k, inormal20, 1, k+nn, inormal21);
	  else
	    sys.addPointPlaneMatch(0, k, inormal30, 1, k+nn, inormal31);
          //          sys.addStereoProj(0, k+nn, projp);
          //          sys.addStereoProj(1, k, proj);

          Matrix3d covar;
          double cv = 0.01;
          covar << cv, 0, 0,
            0, cv, 0, 
            0, 0, cv;
          sys.setProjCovariance(0, k+nn, covar);
          sys.setProjCovariance(1, k, covar);
#endif

        }
      else
        {
          cout << "ERROR! point not in view of nodes" << endl;
          //return;
        }
    }

    // Add noise to node position.
    
    double transscale = 2.0;
    double rotscale = 0.5;
    
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

void processSBA(ros::NodeHandle node)
{
    // Create publisher topics.
    ros::Publisher cam_marker_pub = node.advertise<visualization_msgs::Marker>("/sba/cameras", 1);
    ros::Publisher point_marker_pub = node.advertise<visualization_msgs::Marker>("/sba/points", 1);
    ros::spinOnce();
    
    //ROS_INFO("Sleeping for 2 seconds to publish topics...");
    ros::Duration(0.2).sleep();
    
    // Create an empty SBA system.
    SysSBA sys;
    
    setupSBA(sys);
    
    // Provide some information about the data read in.
    unsigned int projs = 0;
    // For debugging.
    for (int i = 0; i < (int)sys.tracks.size(); i++)
    {
      projs += sys.tracks[i].projections.size();
    }
    ROS_INFO("SBA Nodes: %d, Points: %d, Projections: %d", (int)sys.nodes.size(),
      (int)sys.tracks.size(), projs);
        
    //ROS_INFO("Sleeping for 5 seconds to publish pre-SBA markers.");
    //ros::Duration(5.0).sleep();
        
    // Perform SBA with 10 iterations, an initial lambda step-size of 1e-3, 
    // and using CSPARSE.
    sys.doSBA(1, 1e-3, SBA_SPARSE_CHOLESKY);
    
    int npts = sys.tracks.size();

    ROS_INFO("Bad projs (> 10 pix): %d, Cost without: %f", 
        (int)sys.countBad(10.0), sqrt(sys.calcCost(10.0)/npts));
    ROS_INFO("Bad projs (> 5 pix): %d, Cost without: %f", 
        (int)sys.countBad(5.0), sqrt(sys.calcCost(5.0)/npts));
    ROS_INFO("Bad projs (> 2 pix): %d, Cost without: %f", 
        (int)sys.countBad(0.5), sqrt(sys.calcCost(2.0)/npts));
    
    ROS_INFO("Cameras (nodes): %d, Points: %d",
        (int)sys.nodes.size(), (int)sys.tracks.size());
        
    // Publish markers
    drawGraph(sys, cam_marker_pub, point_marker_pub, 1, sys.tracks.size()/2);
    ros::spinOnce();


    //ROS_INFO("Sleeping for 2 seconds to publish post-SBA markers.");
    ros::Duration(0.5).sleep();

    for (int j=0; j<30; j+=3)
      {
        if (!ros::ok())
	        break;
	      sys.doSBA(1, 0, SBA_SPARSE_CHOLESKY);
	      drawGraph(sys, cam_marker_pub, point_marker_pub, 1, sys.tracks.size()/2);
	      ros::spinOnce();
	      ros::Duration(0.5).sleep();
      }


#ifdef USE_PPL
    // reset covariances and continue
    for (int i = 0; i < points.size(); i++)
    {
      int nn = points.size();
      Matrix3d covar;
      double cv = 0.3;
      covar << cv, 0, 0,
	0, cv, 0, 
	0, 0, cv;
      sys.setProjCovariance(0, i+nn, covar);
      sys.setProjCovariance(1, i, covar);
    }
#endif

    for (int j=0; j<30; j+=3)
      {
        if (!ros::ok())
	        break;
	      sys.doSBA(1, 0, SBA_SPARSE_CHOLESKY);
	      drawGraph(sys, cam_marker_pub, point_marker_pub, 1, sys.tracks.size()/2);
	      ros::spinOnce();
	      ros::Duration(0.5).sleep();
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

