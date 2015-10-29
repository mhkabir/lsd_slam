/**
 * This file is part of LSD-SLAM.
 *
 * Copyright 2013 Jakob Engel <engelj at in dot tum dot de> (Technical University of Munich)
 * For more information see <http://vision.in.tum.de/lsdslam>
 *
 * LSD-SLAM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * LSD-SLAM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with LSD-SLAM. If not, see <http://www.gnu.org/licenses/>.
 */

#include "Frame.h"
#include "FramePoseStruct.h"

namespace lsd_slam {

int privateFramePoseStructAllocCount   = 0;

int FramePoseStruct::cacheValidCounter = 0;

FramePoseStruct::FramePoseStruct(Frame* frame, const Sim3& cam_pose) {
  cacheValidFor       = -1;
  isOptimized         = false;
  isRegisteredToGraph = false;
  hasUnmergedPose     = false;
  isInGraph           = false;
  trackingParent      = nullptr;
  this->graphVertex   = nullptr;
  this->frame         = frame;
  frameID             = frame->id();

  thisToParent_raw    = Sim3();
  camToWorld          = cam_pose;
  camToWorld_new      = cam_pose;

  privateFramePoseStructAllocCount++;

  if (enablePrintDebugInfo && printMemoryDebugInfo)
    printf("ALLOCATED pose %d, now there are %d\n",
           frameID, privateFramePoseStructAllocCount);
}

FramePoseStruct::~FramePoseStruct() {
  privateFramePoseStructAllocCount--;
  if (enablePrintDebugInfo && printMemoryDebugInfo)
    printf("DELETED pose %d, now there are %d\n",
           frameID, privateFramePoseStructAllocCount);
}

void FramePoseStruct::setPoseGraphOptResult(Sim3 camToWorld) {
  if (!isInGraph)
    return;

  camToWorld_new  = camToWorld;
  hasUnmergedPose = true;
}

void FramePoseStruct::applyPoseGraphOptResult() {
  if (!hasUnmergedPose)
    return;

  camToWorld      = camToWorld_new;
  isOptimized     = true;
  hasUnmergedPose = false;
  cacheValidCounter++;
}

void FramePoseStruct::invalidateCache() {
  cacheValidFor = -1;
}

Sim3 FramePoseStruct::getCamToWorld(int recursionDepth) {
  // prevent stack overflow
  assert(recursionDepth < 5000);

  // if the node is in the graph, it's absolute pose is only changed by optimization.
  if (isOptimized) return camToWorld;

  // return chached pose, if still valid.
  if (cacheValidFor == cacheValidCounter)
    return camToWorld;

  // return id if there is no parent (very first frame)
  if (trackingParent == nullptr)
    return camToWorld = Sim3();

  // abs. pose is computed from the parent's abs. pose, and cached.
  cacheValidFor     = cacheValidCounter;

  return camToWorld = trackingParent->getCamToWorld(recursionDepth+1) * thisToParent_raw;
}

}
