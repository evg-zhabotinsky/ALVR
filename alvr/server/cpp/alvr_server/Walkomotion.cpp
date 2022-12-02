#include "Logger.h"
#include "Walkomotion.h"
#include "include/openvr_math.h"

Walkomotion::Walkomotion() {
    ResetState();
}

void Walkomotion::ResetState() {
    ref_x = 0;
    ref_z = 0;
    ref_i = false;
    ref_r = 0;
    ref_a = 0;
    ref_vx = 0;
    ref_vz = 0;
    ref_dx = 0;
    ref_dz = 0;
    center_x = 0;
    center_z = 0;
}

void Walkomotion::SetVirtPosition(double x, double z) {
    ref_vx = x;
    ref_vz = z;
}

void Walkomotion::Recenter() {
    center_x += phys_x;
    center_z += phys_z;
    ref_x += phys_x;
    ref_z += phys_z;
}

void Walkomotion::OnPositionUpdated(uint64_t targetTimestampNs, double x, double z) {
    (void)targetTimestampNs;
    ref_a = targetTimestampNs / 10e9;
    double offset = fmod(targetTimestampNs / 1e9, 4);
    int index = int(offset);
    offset -= index;
    ref_x = index==0 ? offset : 0;
    ref_r = index==1 ? offset : 0;
    ref_z = index==2 ? offset : 0;
}

void Walkomotion::TransformPose(AlvrDeviceMotion *motion) {
    auto refRot = vrmath::quaternionFromRotationY(ref_a);

    auto orientation = vr::HmdQuaternion_t{
        motion->orientation.w, motion->orientation.x, motion->orientation.y, motion->orientation.z};
    orientation = refRot * orientation;
    motion->orientation = {float(orientation.x), float(orientation.y), float(orientation.z), float(orientation.w)};

    auto position = vr::HmdVector3d_t{motion->position[0], motion->position[1], motion->position[2]};
    position.v[0] += ref_x;
    position.v[1] += ref_r;
    position.v[2] += ref_z;
    position = vrmath::quaternionRotateVector(refRot, position);
    motion->position[0] = position.v[0];
    motion->position[1] = position.v[1];
    motion->position[2] = position.v[2];

    motion->linearVelocity[0] = 0;
    motion->linearVelocity[1] = 0;
    motion->linearVelocity[2] = 0;

    motion->angularVelocity[0] = 0;
    motion->angularVelocity[1] = 0;
    motion->angularVelocity[2] = 0;
}