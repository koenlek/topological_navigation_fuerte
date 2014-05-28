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


//
// A class for visual odometry
//

#include <vslam_system/vo.h>
#include <cstdio>

using namespace std;
using namespace frame_common;
using namespace Eigen;
using namespace sba;

namespace vslam
{

  // initialize the VO structures
  voSt::voSt(boost::shared_ptr<pe::PoseEstimator> pose_estimator, int ws, int wf, int mini, double mind, double mina)
  {
    // estimator
    pose_estimator_ = pose_estimator;

    // params
    wsize = ws;                 // total window size
    wfixed = wf;                // size of fixed portion
    mindist = mind;             // meters
    minang  = mina;             // radians
    mininls = mini;             // inliers
    doPointPlane = true;        // true if point-plane matches are included

    // set up structures
    sba.useCholmod(true);

    // for RANSAC
    srand(time(NULL));
  }

  // TODO <fnew> is not changed, can be declared const
  bool voSt::addFrame(const Frame &fnew)
  {
    int nframes = frames.size();
    bool init = nframes == 0;

    // check if we need to remove oldest frame
    if (nframes >= wsize)
      removeFrame();

    // get RW position given relative position
    Quaterniond fq;
    Vector4d trans;
    int inl = 0;
    if (init)                     // use zero origin
      {
        trans = Vector4d(0,0,0,1);
        fq.coeffs() = Vector4d(0,0,0,1);
      }
    else                        // else find pose relative to previous frame
      {
        // check for initial pose estimate from last matched frame
        Frame &refFrame = frames.back();

        inl = pose_estimator_->estimate(refFrame,fnew);
        fq = Quaterniond(pose_estimator_->rot);
        trans.head(3) = pose_estimator_->trans;
        trans(3) = 1.0;

        // check for keyframe
        double dist = pose_estimator_->trans.norm();
        double angledist = fq.angularDistance(fq.Identity());
        if(pose_estimator_->getMethod() == pe::PoseEstimator::Stereo)
        {
          //if (dist < maxdist && inl > mininls)
          // Even if it's closer than the max distance, let it in if there's a 
          // small number of inliers because we're losing inliers.
          if (((dist < mindist && angledist < minang) && (inl > mininls))) // check for angle as well
          {
            // not a keyframe, set up translated keypoints in ref frame
            cout << "[Stereo VO] Skipping frame " << (mindist) << " " << (minang) << " " << (inl) << "/" << (mininls) << endl;
            refFrame.setTKpts(trans,fq);
            return false;
          }
        }
        else
        {
          if(inl < mininls) return false;
          if((pose_estimator_->getMethod() == pe::PoseEstimator::PnP && dist < mindist) && inl > mininls)
          {
            cout << "dist = " << dist << " maxdist = " << mindist << " inl = " << inl
                << " mininls = " << mininls << endl;
            return false;       // no keyframe
          }
        }
      }

    Matrix<double,3,4> f2w, f2w_frame0, f2w_frame1;
    if (!init)
      {
        // rotation
        Node &nd0 = sba.nodes.back(); // last node
        Quaterniond fq0;
        fq0 = nd0.qrot;
        fq = fq*fq0;                  // RW rotation
  
        // translation
        if (isnan(fq.x()) || isnan(fq.y()) || isnan(fq.z()) || isnan(fq.w()))
          return false; // Not a keyframe, not a valid node.
          
        transformF2W(f2w,nd0.trans,fq0);
        trans.head(3) = f2w*trans;
        //      cout << endl << f2w << endl << endl;
        //      cout << trans.head(3).transpose() << endl << endl;
        
        transformF2W(f2w_frame0,nd0.trans,nd0.qrot);
      }

    // Add node for the frame, setting it to fixed if we're iniitializing,
    // floating otherwise.
    sba.addNode(trans, fq, fnew.cam, init);

    int ndi = sba.nodes.size()-1;   // index of this new node
    
    Node &nd1 = sba.nodes.back();
    transformF2W(f2w_frame1,nd1.trans,nd1.qrot);
    //f2w_frame0 = f2w;

    // copy and save this frame
    frames.push_back(fnew);
    Frame &f1 = frames.back();
    // set up point indices to sentinel value -1 indicating unassigned
    f1.ipts.assign(f1.kpts.size(), -1);

    // for first frame, just return
    if (init) 
      {
        init = false;
        return true;
      }

    // get previous frame
    Frame &f0 = *(frames.end()-2);
    nframes = frames.size();

    // add connections to previous frame
    addProjections(f0, f1, frames, sba, pose_estimator_->inliers, f2w_frame0, ndi-1, ndi, &ipts);
    
    // do SBA, setting up fixed frames
    int nfree = wsize-wfixed;
    if (nframes <= nfree)
      sba.nFixed = 1;
    else
      sba.nFixed = wfixed - (wsize - nframes);

    cout << "[Stereo VO] Inliers: " << inl << "  Nodes: " << sba.nodes.size() << "   Points: " << sba.tracks.size() << endl;
    sba.verbose = 0;
    sba.doSBA(4,1.0e-5,0);          // dense version

    // Do pointcloud matching and add the projections to the system.
    // Rot,trans is wrong, should be from updated SBA values
    if (pointcloud_proc_ && doPointPlane)
    {
      pointcloud_proc_->match(f0, f1, pose_estimator_->trans, Quaterniond(pose_estimator_->rot), pointcloud_matches_);
      addPointCloudProjections(f0, f1, sba, pointcloud_matches_, f2w_frame0, f2w_frame1, ndi-1, ndi, &ipts);
    }
    
    return true;
  } // end addFrame


  // removes the oldest node from the sba system
  // this is a pain because we're using indices rather than pointers
  void voSt::removeFrame()
  {
    vector<int> pidx(sba.tracks.size()); // point index for re-indexing

    // run through tracks, resetting node indices by -1 and removing
    //   references to node 0
    int tn = 0;
    for(size_t i=0; i<sba.tracks.size(); i++)
      {
        ProjMap &prjs = sba.tracks[i].projections;
        if (prjs.size() == 0) continue;
        int n = 0;
        for(ProjMap::iterator itr = prjs.begin(); itr != prjs.end(); itr++)
          {
            Proj &prj = itr->second;
            prj.ndi -= 1;
            if (prj.ndi >= 0) n++;
            else prj.isValid = false;
          }
        if (n < 2)              // remove this guy
          pidx[i] = -1;
        else                    // keep this guy
          pidx[i] = tn++;
      }

    //    cout << "[RemoveFrame] Removing " << sba.tracksSt.size() - tn << " tracks" << endl;

    // now remake the tracks and points, removing useless ones
    for (int i=0; i<(int)pidx.size(); i++)
      {
        if (pidx[i] < 0 || pidx[i] == i) continue;
        sba.tracks[pidx[i]] = sba.tracks[i];
        ipts[pidx[i]] = ipts[i];
      }
    sba.tracks.resize(tn);
    ipts.resize(tn);

    // finally, go through frames and adjust point indices
    sba.nodes.erase(sba.nodes.begin()); // erase oldest node
    frames.erase(frames.begin()); // erase oldest frame
    for (int i=0; i<(int)frames.size(); i++)
      {
        Frame &f = frames[i];
        for (int j=0; j<(int)f.ipts.size(); j++)
          if (f.ipts[j] >= 0)
            f.ipts[j] = pidx[f.ipts[j]];
        for (int j=0; j<(int)f.pl_ipts.size(); j++)
          if (f.pl_ipts[j] >= 0)
            f.pl_ipts[j] = pidx[f.pl_ipts[j]];
            
      }
      
    // Redo point indeces of point-plane projections
    for(size_t i=0; i<sba.tracks.size(); i++)
      {
        ProjMap &prjs = sba.tracks[i].projections;
        if (prjs.size() == 0) continue;
        for(ProjMap::iterator itr = prjs.begin(); itr != prjs.end(); itr++)
          {
            Proj &prj = itr->second;
            if (prj.pointPlane && prj.plane_point_index >= 0)
            {
              prj.plane_point_index = pidx[prj.plane_point_index];
              prj.plane_node_index -= 1;
              if (prj.plane_node_index < 0 || prj.plane_point_index < 0)
                prj.isValid = false;
            }
          }
      }
  }

  
  // transfer most recent frame to an external SBA system
  void voSt::transferLatestFrame(std::vector<fc::Frame, Eigen::aligned_allocator<fc::Frame> > &eframes,
                                 SysSBA &esba)
  {
    bool init = esba.nodes.size() == 0;

    // latest frame
    Frame &f1 = eframes.back();

    // set up point indices to NULL
    f1.ipts.assign(f1.kpts.size(), -1);
    f1.pl_ipts.assign(f1.pl_pts.size(), -1);

    // ADD THIS LINE SO THAT PL_PTS ARE CREATED CORRECTLY IN LARGE SBA:
    f1.pl_ipts.assign(f1.pl_pts.size(), -1);

    // add a frame ??? already passed in correct frame...
    //    f1 = frames.back();         // most recent frame in VO

    // get RW position given relative position
    Quaterniond fq;
    Vector4d trans;
    if (init)                   // use zero origin
      {
        /// NOTE! set for NewCollege data
        //trans = Vector4d(0,-1.0,0,1);
        //fq.coeffs() = Vector4d(-0.16,0.0,0,1);
        //fq.normalize();
        trans = Vector4d(0,0,0,1);
        fq.coeffs() = Vector4d(0,0,0,1);
      }
    else
      {
        /// should use relative pose between last two VO frames
        
        transformN2N(trans, fq, *(sba.nodes.end()-2), *(sba.nodes.end()-1));
        
        //fq = Quaterniond(pose_estimator_->rot);
        //trans.head(3) = pose_estimator_->trans;
        //trans(3) = 1.0;
      }

    Matrix<double,3,4> f2w, f2w_frame0, f2w_frame1;
    if (!init)
      {
        // rotation
        Node &nd0 = esba.nodes.back(); 
        Quaterniond fq0;
        fq0 = nd0.qrot;
        fq = fq0*fq;            // RW rotation
  
        // translation
        transformF2W(f2w,nd0.trans,fq0);
        trans.head(3) = f2w*trans;
        
        transformF2W(f2w_frame0,nd0.trans,nd0.qrot);
        //      cout << endl << f2w << endl << endl;
        //      cout << trans.head(3).transpose() << endl << endl;
      }

    // Add node for the frame, setting it to fixed if we're iniitializing,
    // floating otherwise.
    esba.addNode(trans, fq, f1.cam, init);

    int ndi = esba.nodes.size()-1;   // index of this new node

    Node &nd1 = esba.nodes.back();
    transformF2W(f2w_frame1,nd1.trans,nd1.qrot); 
    
    if (init) 
      return;

    /// TODO this also assume most recent VO operation was a keyframe
    /// should reconstruct inliers from most recent two frames
    Frame &f0 = *(eframes.end()-2);
    addProjections(f0, f1, eframes, esba, pose_estimator_->inliers, f2w_frame0, ndi-1, ndi, NULL);
    esba.doSBA(3,1.0e-4,SBA_SPARSE_CHOLESKY);
    if (doPointPlane)
      addPointCloudProjections(f0, f1, esba, pointcloud_matches_, f2w_frame0, f2w_frame1, ndi-1, ndi, NULL);
  }


  // find transform between two frames
  // first frame given by <frameId>, second is <n> frames ahead
  // returns second frame id
  int voSt::findTransform(int frameId, int n, Vector4d &trans, Quaterniond &qr,
                           Matrix<double,6,6> &prec)
  {
    // find frame with index frameId
    int fi = 0;
    for (; fi < (int)frames.size(); fi++)
      {
        if (frameId == frames[fi].frameId)
          break;
      }
    if (fi >= (int)frames.size())
      return -1;                // didn't find the frame

    // now check for nth frame ahead
    int fi1 = fi+n;
    if (fi1 >= (int)frames.size())
      fi1 = frames.size()-1;

    // get transform between nodes
    Node &nd0 = sba.nodes[fi];
    Node &nd1 = sba.nodes[fi1];
    transformN2N(trans,qr,nd0,nd1);
    // just do a diagonal precision matrix for now
    prec.setIdentity();
    prec(3,3) = prec(4,4) = prec(5,5) = 10;

    return frames[fi1].frameId;
  }


  // find node index of associated frameId
  int voSt::findNode(int frameId)
  {
    int fi = 0;
    for (; fi < (int)frames.size(); fi++)
      {
        if (frameId == frames[fi].frameId)
          return fi;
      }
    return -1;
  }

  // add connections between frames, based on keypoint matches
  void addProjections(fc::Frame &f0, fc::Frame &f1, 
                      std::vector<fc::Frame, Eigen::aligned_allocator<fc::Frame> > &frames,
                      SysSBA &sba, const std::vector<cv::DMatch> &inliers,
                      const Matrix<double,3,4>& f2w, int ndi0, int ndi1, std::vector<int>* ipts)
  {
    // set up array to kill duplicate matches
    vector<bool> matched0(f0.ipts.size(),0);
    vector<bool> matched1(f1.ipts.size(),0);
    
    // Whether the frame we are adding is stereo or not.
    // Not sure this would do the right thing in the case of stereo-mono matches.
    bool stereo = f1.isStereo;

    // add points and projections
    for (int i=0; i<(int)inliers.size(); i++)
      {
        int i0 = inliers[i].queryIdx;
        int i1 = inliers[i].trainIdx;

        if (matched0[i0]) continue;
        if (matched1[i1]) continue;

        if (f0.goodPts[i0] == 0) continue;

        matched0[i0] = true;
        matched1[i1] = true;

        int pti;

        if (f0.ipts[i0] < 0 && f1.ipts[i1] < 0)    // new point
          {
            pti = sba.tracks.size();
            f0.ipts[i0] = pti;
            f1.ipts[i1] = pti;

            Vector4d pt;
            pt.head(3) = f2w*f0.pts[i0]; // transform to RW coords
            pt(3) = 1.0;
            sba.addPoint(pt);
            if (ipts)
              ipts->push_back(-1);  // external point index

            Vector3d ipt = getProjection(f0, i0);
            sba.addProj(ndi0, pti, ipt, stereo);

            // projected point, ul,vl,ur
            ipt = getProjection(f1, i1);
            sba.addProj(ndi1, pti, ipt, stereo);
          }

        else if (f0.ipts[i0] >= 0 && f1.ipts[i1] >= 0) // merge two tracks
          {
            if (f0.ipts[i0] != f1.ipts[i1]) // different tracks
              {
                int tri = sba.mergeTracksSt(f0.ipts[i0],f1.ipts[i1]);
                if (tri >= 0)   // successful merge
                  {
                    // update the ipts in frames that connect to this track
                    ProjMap &prjs = sba.tracks[tri].projections;
                    for(ProjMap::iterator itr = prjs.begin(); itr != prjs.end(); itr++)
                      {
                        Proj &prj = itr->second;  
                        if (tri == f0.ipts[i0])
                          substPointRef(frames[prj.ndi].ipts, tri, f1.ipts[i1]);
                        else
                          substPointRef(frames[prj.ndi].ipts, tri, f0.ipts[i0]);
                      }
                  }
              }
          }

        else if (f1.ipts[i1] < 0)                 // add to previous point track
          {
            pti = f0.ipts[i0];
            f1.ipts[i1] = pti;
          
            // projected point, ul,vl,ur
            Vector3d ipt = getProjection(f1, i1);
            sba.addProj(ndi1, pti, ipt, stereo);
          }
        else if (f0.ipts[i0] < 0)                 // add to previous point track
          {
            pti = f1.ipts[i1];
            f0.ipts[i0] = pti;
          
            // projected point, ul,vl,ur
            Vector3d ipt = getProjection(f0, i0);
            sba.addProj(ndi0, pti, ipt, stereo);
          }
      }
  }
  
  // Pointcloud matches, copied from above. Think of a more elegant way of doing this.
  void addPointCloudProjections(fc::Frame &f0, fc::Frame &f1, 
                      SysSBA &sba, const std::vector<cv::DMatch> &inliers,
                      const Matrix<double,3,4>& f2w_frame0, 
                      const Matrix<double,3,4>& f2w_frame1, 
                      int ndi0, int ndi1, std::vector<int>* ipts)
  {
    // add points and projections
    double covariance = 1.0;
    Matrix3d covar;
    covar <<  covariance, 0, 0,
      0, covariance, 0, 
      0, 0, covariance;

    covariance = 100.0;
    Matrix3d cv2;
    cv2 <<  covariance, 0, 0,
      0, covariance, 0, 
      0, 0, covariance;

    for (int i=0; i<(int)inliers.size(); i++)
      {
        int i0 = inliers[i].queryIdx;
        int i1 = inliers[i].trainIdx;

        int pti;

        if (f0.pl_ipts[i0] < 0)  // new point
          {
            pti = sba.tracks.size();
            f0.pl_ipts[i0] = pti;

            Vector4d pt;
            pt.head<3>() = f2w_frame0*f0.pl_pts[i0]; // transform to RW coords
            pt(3) = 1.0;

            //            cout << "f0 point: " << pt.transpose() << endl;

            sba.addPoint(pt);
            if (ipts)
              ipts->push_back(-1);  // external point index

            sba.addStereoProj(ndi0, pti, f0.pl_kpts[i0]);
            sba.setProjCovariance(ndi0, pti, cv2);
          }
        if (f1.pl_ipts[i1] < 0) // new point
          {
            pti = sba.tracks.size();
            f1.pl_ipts[i1] = pti;
            
            Vector4d pt;
            pt.head<3>() = f2w_frame1*f1.pl_pts[i1]; // transform to RW coords
            pt(3) = 1.0;

            //            cout << "f1 point: " << pt.transpose() << endl;

            sba.addPoint(pt);
            if (ipts)
              ipts->push_back(-1);  // external point index
            
            sba.addStereoProj(ndi1, pti, f1.pl_kpts[i1]);
            sba.setProjCovariance(ndi1, pti, cv2);
          }
        
        // Add point-to-plane projections
        
        // First, figure out normals in world coordinate frame:
        Vector3d normal0 = f0.pl_normals[i0].head<3>();
        Vector3d normal1 = f1.pl_normals[i1].head<3>();
        
        // Then add the forward and backward projections.
        sba.addPointPlaneMatch(ndi0, f0.pl_ipts[i0], normal0, ndi1, f1.pl_ipts[i1], normal1);
        
        // Add covariance for just whichever projections we have.
        sba.setProjCovariance(ndi1, f0.pl_ipts[i0], covar);
        sba.setProjCovariance(ndi0, f1.pl_ipts[i1], covar);
      }
  }
  
  Vector3d getProjection(fc::Frame &frame, int index)
  {
    Vector3d proj;
    proj(0) = frame.kpts[index].pt.x;
    proj(1) = frame.kpts[index].pt.y;
    proj(2) = proj(0)-frame.disps[index];
    return proj;
  }

  // substitute point references
  void substPointRef(std::vector<int> &ipts, int tri0, int tri1)
  {
    for (int i=0; i<(int)ipts.size(); i++)
      {
        if (ipts[i] == tri1)
          ipts[i] = tri0;
      }
  }
                    
} // end namespace vslam

