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
    void PhysToVirt(double px, double pz, double *vx, double *vz);
    void UpdateReference();

    double m_safeRadius, m_marginRadius, m_walkRadius, m_turnRadius, m_minRadius;
    double m_dragLength;

    double m_refX, m_refZ, m_refR, m_refA, m_refVX, m_refVZ, m_refDX, m_refDZ;
    bool m_refI;
    double m_centerX, m_centerZ, m_physX, m_physZ, m_virtX, m_virtZ;
};
