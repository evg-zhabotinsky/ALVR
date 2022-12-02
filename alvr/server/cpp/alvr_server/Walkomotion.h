#pragma once

#include "TrackedDevice.h"

class Walkomotion {
  public:
    Walkomotion();
    void OnPositionUpdated(uint64_t targetTimestampNs, double x, double z);
    void TransformPose(AlvrDeviceMotion *motion);
    void Recenter();
    void SetVirtPosition(double x, double z);
    void ResetState();
  private:
    void UpdateReference();
    double ref_x, ref_z, ref_r, ref_a, ref_vx, ref_vz, ref_dx, ref_dz;
    bool ref_i;
    double center_x, center_z, phys_x, phys_z;
};