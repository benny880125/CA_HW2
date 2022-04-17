#include "kinematics.h"

#include <algorithm>
#include <iostream>
#include "utils.h"
void forwardKinematics(const Posture& posture, Bone* bone) {
  // TODO (FK)
  // You should set these variables:
  //     bone->startPosition = Eigen::Vector3f::Zero();
  //     bone->endPosition = Eigen::Vector3f::Zero();
  //     bone->rotation = Eigen::Quaternionf::Identity();
  // The sample above just set everything to initial state
  // Hint:
  //   1. posture.translations, posture.rotations
  // Note:
  //   1. This function will be called with bone == root bone of the skeleton

  // Write your code here
  bone->startPosition = Eigen::Vector3f::Zero();
  bone->endPosition = Eigen::Vector3f::Zero();
  bone->rotation = posture.rotations[0];

  std::vector<bool> flag(30,0);
  std::vector<Bone*> roots;
  
  Bone* root = bone->child;
  Bone* next = bone->child;
  
  while (next != nullptr || root->sibling != nullptr) {
    next = root;
    if (flag[root->idx] == true && root->sibling != nullptr) {
      root = root->sibling;
      next = root;
    } else {
      while (next != nullptr && flag[next->idx] == false) {
        if (next->sibling != nullptr) {
          root = next;
        }
        next->rotation = next->parent->rotation * next->rotationParentCurrent * posture.rotations[next->idx];
        next->startPosition = posture.translations[next->parent->idx] + next->parent->endPosition;
        next->endPosition = next->startPosition + next->rotation * next->direction * next->length;
        flag[next->idx] = true;
        next = next->child;
      }
    }
  }
  // move to unvisited bone
  next = bone->child;
  while (flag[next->idx] == true) {
    if (next->sibling != nullptr) {
      next = next->sibling;
    } else
      next = next->child;  
  }
  root = next;

  while (next != nullptr || root->sibling != nullptr) {
    next = root;
    if (flag[root->idx] == true && root->sibling != nullptr) {
      root = root->sibling;
      next = root;
    } else {
      while (next != nullptr && flag[next->idx] == false) {
        if (next->sibling != nullptr) {
          root = next;
        }
        next->rotation = next->parent->rotation * next->rotationParentCurrent * posture.rotations[next->idx];
        next->startPosition = posture.translations[next->parent->idx] + next->parent->endPosition;
        next->endPosition = next->startPosition + next->rotation * next->direction * next->length;
        flag[next->idx] = true;
        next = next->child;
      }
    }
  }
}

Motion motionWarp(const Motion& motion, int oldKeyframe, int newKeyframe) {
  Motion newMotion = motion;
  int totalFrames = static_cast<int>(motion.size());
  int totalBones = static_cast<int>(motion.posture(0).rotations.size());
  for (int i = 0; i < totalFrames; ++i) {
    // Maybe set some per=Frame variables here
    double frame;
    if (i > oldKeyframe) {
      frame = newKeyframe + (i - oldKeyframe) * ((double)((totalFrames - 1) - newKeyframe) / (double)((totalFrames - 1) - oldKeyframe));
    } else
      frame = i * ((double)newKeyframe / (double)oldKeyframe);
    
    int lower = floor(frame);
    int upper = ceil(frame);
    double fromstart = frame - lower;
    for (int j = 0; j < totalBones; ++j) {
      // TODO (Time warping)
      // original: |--------------|---------------|
      // new     : |------------------|-----------|
      // OR
      // original: |--------------|---------------|
      // new     : |----------|-------------------|
      // You should set these variables:
      //     newMotion.posture(i).translations[j] = Eigen::Vector3f::Zero();
      //     newMotion.posture(i).rotations[j] = Eigen::Quaternionf::Identity();
      // The sample above just set to initial state
      // Hint:
      //   1. Your should scale the frames before and after key frames.
      //   2. You can use linear interpolation with translations.
      //   3. You should use spherical linear interpolation for rotations.

      // Write your code here
      Eigen::Vector3f t1 = motion.posture(lower).translations[j];
      Eigen::Vector3f t2 = motion.posture(upper).translations[j];
      Eigen::Quaternionf r1 = motion.posture(lower).rotations[j];
      Eigen::Quaternionf r2 = motion.posture(upper).rotations[j];
      newMotion.posture(i).translations[j] = t1 + fromstart * (t2 - t1);
      newMotion.posture(i).rotations[j] = r1.slerp(fromstart, r2);
    }
  }
  //std::cout << std::endl;
  return newMotion;
}

Motion motionBlend(const Motion& motionA, const Motion& motionB) {
  Motion newMotion;
  constexpr int blendFrameCount = 20;
  constexpr float blendFactor = 1.0f / blendFrameCount;
  constexpr int matchRange = 10;
  float difference[matchRange] = {};
  // TODO (Bonus)
  // motionA: |--------------|--matchRange--|--blendFrameCount--|
  // motionB:                               |--blendFrameCount--|--------------|
  // The starting frame of `blendFrameCount` can be in `matchRange`
  // Hint:
  //   1. Find motionB's starting posture
  //   2. Match it with the minimum cost posture in `matchRange`
  //   3. Find to translation and rotation offset between matched motionA and motionB's start
  //   4. Begin from the matched frame, blend `blendFrameCount` of frames,
  //      with a blendFactor from 1 / `blendFrameCount` to 1
  //   5. Add remaining motionB to newMotion
  // Note:
  //   1. The offset found in 3 should apply to 4 and 5
  //   2. A simple cost function is translation offsets between two posture.
  //   3. A better one considered both translations and rotations.
  //   4. Your animation should smoothly change from motionA to motionB.
  //   5. You can adjust those `constexpr`s above by yourself if you need.

  // Write your code here
  return newMotion;
}
