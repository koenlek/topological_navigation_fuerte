#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/CvBridge.h>
#include <opencv/highgui.h>
#include <fstream>
#include <dirent.h>
#include <fnmatch.h>
#include <cstdio>

// Names of left and right files in directory (with wildcards)
char *lreg, *rreg, *dreg;

// Filters for scandir
int getleft(struct dirent const *entry)
{
  if (!fnmatch(lreg,entry->d_name,0))
    return 1;
  return 0;
}

int getright(struct dirent const *entry)
{
  if (!fnmatch(rreg,entry->d_name,0))
    return 1;
  return 0;
}


int getidir(struct dirent const *entry)
{
  if (!fnmatch(dreg,entry->d_name,0))
    return 1;
  return 0;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fake_stereo_cam");

  // Set up publishers
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::CameraPublisher l_pub = it.advertiseCamera("left/image_rect", 1);
  image_transport::CameraPublisher r_pub = it.advertiseCamera("right/image_rect", 1);

  // Read in camera info
  std::fstream cam_params;
  cam_params.open(argv[1], std::fstream::in);
  if (!cam_params.is_open()) {
    ROS_FATAL("Can't open camera file %s\n", argv[1]);
    return -1;
  }
  double fx, fy, cx, cy, Tx;
  cam_params >> fx >> fy >> cx >> cy >> Tx;
  sensor_msgs::CameraInfo l_info, r_info;
  l_info.header.frame_id = "stereo_optical_frame";
  l_info.K[0] = l_info.P[0] = fx;
  l_info.K[4] = l_info.P[5] = fy;
  l_info.K[2] = l_info.P[2] = cx;
  l_info.K[5] = l_info.P[6] = cy;
  l_info.R[0] = l_info.R[4] = l_info.R[8] = 1.0;
  r_info = l_info;
  r_info.P[3] = -Tx * fx;

  // set up directories
  struct dirent **lims, **rims, **dirs;
  int nlim, nrim, ndirs;
  std::string dname = argv[2];
  if (!dname.compare(dname.size()-1,1,"/")) // see if slash at end
    dname.erase(dname.size()-1);

  std::string dirfn = dname.substr(dname.rfind("/")+1);
  std::string tdir = dname.substr(0,dname.rfind("/")+1);
  std::cout << "Top directory is " << tdir << std::endl;
  std::cout << "Search directory name is " << dirfn << std::endl;
  dreg = (char *)dirfn.c_str();

  ndirs = scandir(tdir.c_str(), &dirs, getidir, alphasort);
  printf("Found %d directories\n", ndirs);
  printf("%s\n",dirs[0]->d_name);

  lreg = argv[3];
  rreg = argv[4];

  // loop over directories
  ros::Rate rate(2);
  for (int dd=0; dd<ndirs && ros::ok(); dd++) {
    char dir[2048];
    sprintf(dir,"%s%s",tdir.c_str(),dirs[dd]->d_name);
    printf("Current directory: %s\n", dir);

    // get left/right image file names, sorted
    nlim = scandir(dir,&lims,getleft,alphasort);
    printf("Found %d left images\n", nlim);
    printf("%s\n",lims[0]->d_name);

    nrim = scandir(dir,&rims,getright,alphasort);
    printf("Found %d right images\n", nrim);
    printf("%s\n",rims[0]->d_name);

    if (nlim != nrim) {
      ROS_FATAL("Number of left/right images does not match: %d vs. %d\n", nlim, nrim);
      return -1;
    }

    // loop over each stereo pair and publish them
    for (int ii=0; ii<nlim && ros::ok(); ii++) {
      // Load images
      char fn[2048];
      sprintf(fn,"%s/%s",dir,lims[ii]->d_name);
      cv::Mat left = cv::imread(fn,0);
      sprintf(fn,"%s/%s",dir,rims[ii]->d_name);
      cv::Mat right = cv::imread(fn,0);

      IplImage left_ipl = left, right_ipl = right;
      sensor_msgs::ImagePtr left_msg  = sensor_msgs::CvBridge::cvToImgMsg(&left_ipl);
      sensor_msgs::ImagePtr right_msg = sensor_msgs::CvBridge::cvToImgMsg(&right_ipl);

      ros::Time stamp = ros::Time::now();
      l_pub.publish(*left_msg, l_info, stamp);
      r_pub.publish(*right_msg, r_info, stamp);

      ros::spinOnce();
      rate.sleep();
    }
  }
}
