#include <ros/ros.h>
#include <sba/sba.h>
#include <sba/visualization.h>


// For random seed.
#include <time.h>

#include <visualization_msgs/Marker.h>
#include <sba/Frame.h>
#include <sba/sba_file_io.h>

using namespace sba;
using namespace std;

void setupSBA(SysSBA &sys)
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

    // Vector containing the true point positions.
    vector<Point, Eigen::aligned_allocator<Point> > points;

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
    double ptscale = 1.0;
        
    // Add points into the system, and add noise.
    for (i = 0; i < points.size(); i++)
    {
      // Add up to .5 points of noise.
      Vector4d temppoint = points[i];
      temppoint.x() += ptscale*(drand48() - 0.5);
      temppoint.y() += ptscale*(drand48() - 0.5);
      temppoint.z() += ptscale*(drand48() - 0.5);
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
        if (proj.x() > 0 && proj.x() < maxx-1 && proj.y() > 0 && proj.y() < maxy-1)
        {
          sys.addMonoProj(j, i, proj);
          //printf("Adding projection: Node: %d Point: %d Proj: %f %f\n", j, i, proj.x(), proj.y());
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

void publishNode(unsigned int index, Eigen::Matrix<double,4,1> trans, 
                Eigen::Quaternion<double> fq,
                const frame_common::CamParams &cp,
                bool isFixed, sba::CameraNode &msg)
{ 
    msg.index = index;
    
    msg.transform.translation.x = trans.x();
    msg.transform.translation.y = trans.y();
    msg.transform.translation.z = trans.z();
    msg.transform.rotation.x = fq.x();
    msg.transform.rotation.y = fq.y();
    msg.transform.rotation.z = fq.z();
    msg.transform.rotation.w = fq.w();
    msg.fx = cp.fx;
    msg.fy = cp.fy;
    msg.cx = cp.cx;
    msg.cy = cp.cy;
    msg.baseline = cp.tx;
    msg.fixed = isFixed;
}

void publishPoint(unsigned int index, sba::Point pt, sba::WorldPoint &msg)
{
    msg.index = index;
    
    msg.x = pt.x();
    msg.y = pt.y();
    msg.z = pt.z();
    msg.w = pt.w();
}

void publishProjection(int ci, int pi, Eigen::Vector3d &q, 
                       bool stereo, sba::Projection &msg)
{
    msg.camindex = ci;
    msg.pointindex = pi;
    msg.u = q.x();
    msg.v = q.y();
    msg.d = q.z();
    msg.stereo = stereo;
}

void processSBA(ros::NodeHandle nh)
{
    // Publications for SBA Node
    ros::Publisher sba_frames_pub = nh.advertise<sba::Frame>("/sba/frames", 5000);
    ros::spinOnce();
    
    ROS_INFO("Sleeping for 2 seconds to publish topics...");
    ros::Duration(2.0).sleep();
    
    // Create an empty SBA system.
    SysSBA sys;
    
    setupSBA(sys);
    
    // Provide some information about the data read in.
    // For debugging.
    unsigned int projs = 0;
    for (int i = 0; i < (int)sys.tracks.size(); i++)
    {
      projs += sys.tracks[i].projections.size();
    }
    ROS_INFO("SBA Nodes: %d, Points: %d, Projections: %d", (int)sys.nodes.size(),
      (int)sys.tracks.size(), projs);
    
    // Create camera parameters.
    frame_common::CamParams cp;
    cp.fx = 430; // Focal length in x
    cp.fy = 430; // Focal length in y
    cp.cx = 320; // X position of principal point
    cp.cy = 240; // Y position of principal point
    cp.tx = 0;   // Baseline (no baseline since this is monocular)
    
    // Add frames to SBA node.
    for (unsigned int i = 0; i < sys.nodes.size(); i++)
    {
      sba::Frame framemsg;
      sba::CameraNode nodemsg;
      
      publishNode(i, sys.nodes[i].trans, sys.nodes[i].qrot, cp, false, nodemsg);
      
      framemsg.nodes.push_back(nodemsg);
      
      for (unsigned int j = 0; j < sys.tracks.size(); j++)
      {
        if (i == 0)
        {
          sba::WorldPoint ptmsg;
          publishPoint(j, sys.tracks[j].point, ptmsg);
          framemsg.points.push_back(ptmsg);
        }
        
        sba::Projection prjmsg;
        
        ProjMap::iterator iter = sys.tracks[j].projections.find(i);
	      if (iter != sys.tracks[j].projections.end()) 
        {
          sba::Proj &prj = iter->second;
          publishProjection(i, j, prj.kp, false, prjmsg);
        
          framemsg.projections.push_back(prjmsg);
        }
      }
      
      sba_frames_pub.publish(framemsg);
      ros::spinOnce();
      //ros::Duration(5.0).sleep();
      
      ROS_INFO("Publishing node #%d", i);
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

