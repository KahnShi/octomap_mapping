/*
 * Copyright (c) 2012, D. Kuhner, P. Ruchti, University of Freiburg
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <octomap_server/TruckOctomapServer.h>
#include <string>

using namespace octomap;
using namespace octomath;
using namespace octomap_server;

TruckOctomapServer::TruckOctomapServer() :
  OctomapServer()
{

  ros::NodeHandle private_nh("~");

  private_nh.param("resolution", m_res, 0.1);
  private_nh.param("frame_id", m_worldFrameId, (std::string)"/map");

  init_param();
}

TruckOctomapServer::~TruckOctomapServer() {
}

void TruckOctomapServer::init_param()
{
  //roof.assign((int)round(1.0/m_res), (int)round(1.5/m_res), (int)round(1.0/m_res));
  roof[0] = (int)round(1.0/m_res); roof[1] = (int)round(1.5/m_res); roof[2] = (int)round(1.0/m_res);
  //base.assign((int)round(2.5/m_res), (int)round(1.5/m_res), (int)round(1.5/m_res));
  base[0] = (int)round(2.5/m_res); base[1] = (int)round(1.5/m_res); base[2] = (int)round(1.0/m_res);
  //cargo.assign((int)round(1.5/m_res), (int)round(1.5/m_res), (int)round(1.0/m_res));
  cargo[0] = (int)round(1.5/m_res); cargo[1] = (int)round(1.5/m_res); cargo[2] = (int)round(0.5/m_res);

  step_value = (float)m_res / 2.0f;
}


void TruckOctomapServer::publishTruckFullOctoMap(const ros::Time& rostime)
{
  OctomapServer *b = this;
  this->publishFullOctoMap(rostime);
}

void TruckOctomapServer::publishTruckAll(const ros::Time& rostime)
{
  OctomapServer *b = this;
  this->publishAll(rostime);
}

void TruckOctomapServer::WriteTruckOctree(Pose6D rot_mat)
{
  printf("Resolution is %f\n", m_res);

  // Truck's roof above drivers
  for (int x=-roof[0]; x<roof[0]; x++) {
    for (int y=-roof[1]; y<roof[1]; y++) {
      for (int z=-roof[2]; z<roof[2]; z++) {
        Vector3 end_vec = rot_mat.transform(Vector3((float) x*step_value+0.75f, (float) y*step_value, (float) z*step_value+1.25f));
        point3d endpoint (end_vec.x(), end_vec.y(), end_vec.z());
        m_octree->updateNode(endpoint, true); // integrate 'occupied' measurement
      }
    }
  }

  // Truck's whole base
  for (int x=-base[0]; x<base[0]; x++) {
    for (int y=-base[1]; y<base[1]; y++) {
      for (int z=-base[2]; z<base[2]; z++) {
        Vector3 end_vec = rot_mat.transform(Vector3((float) x*step_value, (float) y*step_value, (float) z*step_value+0.5f));
        point3d endpoint (end_vec.x(), end_vec.y(), end_vec.z());
        m_octree->updateNode(endpoint, true); // integrate 'occupied' measurement
      }
    }
  }


  // insert some measurements of free cells
  //Cargo: Truck's region above landing area
  for (int x=-cargo[0]; x<cargo[0]; x++) {
    for (int y=-cargo[1]; y<cargo[1]; y++) {
      for (int z=-cargo[2]; z<cargo[2]; z++) {
        Vector3 end_vec = rot_mat.transform(Vector3((float) x*step_value-0.5f, (float) y*step_value, (float) z*step_value+1.25f));
        point3d endpoint (end_vec.x(), end_vec.y(), end_vec.z());
        m_octree->updateNode(endpoint, false);  // integrate 'free' measurement
      }
    }
  }

}

