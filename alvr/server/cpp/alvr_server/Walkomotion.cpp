#include "Logger.h"
#include "Walkomotion.h"
#include "include/openvr_math.h"
#include <cmath>

constexpr double pi = 3.141592653589793238462643383279502884L;

inline double sqr(double x) {
    return x * x;
}
inline double negif(double x, bool f) {
    return f ? -x : x;
}

Walkomotion::Walkomotion() {
    ResetState();
}

void Walkomotion::ResetState() {
    m_safeRadius = 3.0 / 2 - .7;
    m_marginRadius = m_safeRadius - .1;
    m_walkRadius = m_marginRadius - .2;
    m_turnRadius = m_walkRadius / 2;
    m_minRadius = .1;
    m_dragLength = m_minRadius / 2;
    Info("TRACEPARAM %f %f %f %f %f %f\n", m_safeRadius, m_marginRadius, m_walkRadius, m_turnRadius, m_minRadius, m_dragLength);

    m_refI = false;
    m_refR = m_safeRadius * 1;//000;
    m_refA = pi / 2;
    m_refX = m_refR;
    m_refZ = 0;
    m_refVX = 0;
    m_refVZ = 0;
    m_refDX = 0;
    m_refDZ = -1;

    m_centerX = 0;
    m_centerZ = 0;
    m_physX = 0;
    m_physZ = 0;
    m_virtX = 0;
    m_virtZ = 0;
}

void Walkomotion::SetVirtPosition(double x, double z) {
    m_refVX += x - m_virtX;
    m_refVZ += z - m_virtZ;
    m_virtX = x;
    m_virtZ = z;
}

void Walkomotion::Recenter() {
    m_centerX += m_physX;
    m_centerZ += m_physZ;
    m_refX -= m_physX;
    m_refZ -= m_physZ;
    m_physX = 0;
    m_physZ = 0;
}

void Walkomotion::OnPositionUpdated(uint64_t targetTimestampNs, double x, double z) {
    (void)targetTimestampNs;
    m_physX = x - m_centerX;
    m_physZ = z - m_centerZ;

    // Phys to virt and check distance from the dragged point
    double frameX = m_physX - m_refX;
    double frameZ = m_physZ - m_refZ;
    double frameR = negif(sqrt(sqr(frameX) + sqr(frameZ)) - m_refR, m_refI);
    double frameA = atan2(frameX, frameZ) - m_refA;
    if (frameA > pi) {
        frameA -= 2 * pi;
    } else if (frameA <= -pi) {
        frameA += 2 * pi;
    }
    frameA *= negif(m_refR, m_refI);
    double newVX = m_refVX + m_refDX * frameA - m_refDZ * frameR;
    double newVZ = m_refVZ + m_refDZ * frameA + m_refDX * frameR;
    double frameDX = newVX - m_virtX;
    double frameDZ = newVZ - m_virtZ;
    double len = sqrt(sqr(frameDX) + sqr(frameDZ));
    if (len <= m_dragLength) {
        return;
    }

    // Moved far enough, drag the reference
    frameDX /= len;
    frameDZ /= len;
    m_virtX += frameDX * (len - m_dragLength);
    m_virtZ += frameDZ * (len - m_dragLength);

    // Virt to phys with direction
    frameX = newVX - m_refVX;
    frameZ = newVZ - m_refVZ;
    frameR = negif(frameZ * m_refDX - frameX * m_refDZ, m_refI) + m_refR;
    frameA = negif(frameX * m_refDX + frameZ * m_refDZ, m_refI) / m_refR + m_refA;
    double frameDR = negif(frameDZ * m_refDX - frameDX * m_refDZ, m_refI);
    double frameDA = negif(frameDX * m_refDX + frameDZ * m_refDZ, m_refI) / m_refR;
    double cosA = cos(frameA);
    double sinA = sin(frameA);
    double physDX = frameDR * sinA + frameR * cosA * frameDA;
    double physDZ = frameDR * cosA - frameR * sinA * frameDA;
    len = sqrt(sqr(physDX) + sqr(physDZ));
    physDX /= len;
    physDZ /= len;

    // Update non-curvature-dependent frame params
    m_refVX = newVX;
    m_refVZ = newVZ;
    m_refDX = frameDX;
    m_refDZ = frameDZ;

    // Compute the curvature and the direction of the frame
    double leftX = physDZ;
    double leftZ = -physDX;
    double leftD = -m_physX * leftX - m_physZ * leftZ; // Distance to room center along left direction
    m_refI = leftD < 0; // We normally turn towards center
    leftD = negif(leftD, m_refI); // Now is positive
    double D2 = sqr(m_physX) + sqr(m_physZ); // Direct distance to room center, squared
    bool away = physDX * m_physX + physDZ * m_physZ > 0; // Moving away from room center
    m_refR = 0;
    if (D2 >= sqr(m_safeRadius)) {
        m_refR = m_minRadius; // In unsafe area, make tightest allowed turn
    } else if (away) {
        // Max turn radius to avoid unsafe area
        double t = (sqr(m_safeRadius) - D2) / (2 * (m_safeRadius - leftD));
        if (t < m_minRadius) {
            m_refR = m_minRadius; // Will inevitably hit unsafe area, make the tightest allowed turn
        } else if (t < m_turnRadius) {
            m_refR = t; // Need a tighter-than-preferred turn to avoid unsafe area
        }
    }

    if (m_refR == 0) {
        // No risk of touching the unsafe area
        if (D2 >= sqr(m_marginRadius)) {
            m_refR = m_turnRadius; // In margin area, make the tightest preferred turn
        } else if (away) {
            // Max turn radius to avoid the margin area
            double t = (sqr(m_marginRadius) - D2) / (2 * (m_marginRadius - leftD));
            if (t < m_turnRadius) {
                m_refR = m_turnRadius; // Make the tightest preferred turn to avoid margin area
            } else if (t > m_walkRadius) {
                m_refR = m_walkRadius; // Orbit at that radius to avoid tighter turns later
            } else {
                m_refR = t; // Make the loosest turn to avoid the margin area
            }
        }
    }

    if (m_refR == 0) {
        m_refR = m_walkRadius; // No risk of hitting the margin area
        if (sqr(m_physX - negif(m_walkRadius, m_refI) * leftX) +
            sqr(m_physZ - negif(m_walkRadius, m_refI) * leftZ) < sqr(2 * m_walkRadius))
        {
            m_refI = !m_refI; // Turn away from center to get a more centered orbit later
        }
    }

    // Update remaining frame params
    m_refX = m_physX + negif(m_refR, m_refI) * leftX;
    m_refZ = m_physZ + negif(m_refR, m_refI) * leftZ;
    m_refA = atan2(negif(leftX, !m_refI), negif(leftZ, !m_refI));
    Info("TRACE %f %f %f %f %f %f %f %f %f %f %f %f %f\n",
        m_refX, m_refZ, m_refR, m_refA, negif(1, m_refI), m_refVX, m_refVZ, m_refDX, m_refDZ, m_physX, m_physZ, m_virtX, m_virtZ);
}

void Walkomotion::TransformPose(AlvrDeviceMotion *motion) {
    // Unpack the motion
    auto position = vr::HmdVector3d_t{
        motion->position[0], motion->position[1], motion->position[2]};
    auto orientation = vr::HmdQuaternion_t{
        motion->orientation.w, motion->orientation.x, motion->orientation.y, motion->orientation.z};
    auto velocity = vr::HmdVector3d_t{
        motion->linearVelocity[0], motion->linearVelocity[1], motion->linearVelocity[2]};
    auto rotVel = vr::HmdVector3d_t{  // Direction is axis (forward), magnitude is rad/s (CCW / CW???)
        motion->angularVelocity[0], motion->angularVelocity[1], motion->angularVelocity[2]};

    auto pos = position - vr::HmdVector3d_t{m_centerX, 0, m_centerZ};
    auto look = vrmath::quaternionRotateVector(orientation, {0, 0, -1});

    // Compute the transform
    double frameX = position.v[0] - m_centerX - m_refX;
    double frameZ = position.v[2] - m_centerZ - m_refZ;
    double frameR = negif(sqrt(sqr(frameX) + sqr(frameZ)) - m_refR, m_refI);
    double frameA = atan2(frameX, frameZ) - m_refA;
    if (frameA > pi) {
        frameA -= 2 * pi;
    } else if (frameA <= -pi) {
        frameA += 2 * pi;
    }
    auto refRot = vrmath::quaternionFromRotationY((pi / 2 - m_refA - negif(pi / 2, m_refI)) + frameA - atan2(m_refDZ, m_refDX));
    frameA *= negif(m_refR, m_refI);
    frameX = m_refVX + m_refDX * frameA - m_refDZ * frameR;
    frameZ = m_refVZ + m_refDZ * frameA + m_refDX * frameR;


    // Apply the transform
    position.v[0] = frameX;
    position.v[2] = frameZ;
    orientation = refRot * orientation;

    // Update the motion
    motion->position[0] = position.v[0];
    motion->position[1] = position.v[1];
    motion->position[2] = position.v[2];

    motion->orientation = {float(orientation.x), float(orientation.y), float(orientation.z), float(orientation.w)};

    motion->linearVelocity[0] = 0;
    motion->linearVelocity[1] = 0;
    motion->linearVelocity[2] = 0;

    motion->angularVelocity[0] = 0;
    motion->angularVelocity[1] = 0;
    motion->angularVelocity[2] = 0;

    auto virtPos = position;
    auto virtLook = vrmath::quaternionRotateVector(orientation, {0, 0, -1});
    look = look * (1 / sqrt(sqr(look.v[0]) + sqr(look.v[2])));
    virtLook = virtLook * (1 / sqrt(sqr(virtLook.v[0]) + sqr(virtLook.v[2])));
    Info("TRACE %f %f %f %f %f %f %f %f\n", pos.v[0], pos.v[2], look.v[0], look.v[2], virtPos.v[0], virtPos.v[2], virtLook.v[0], virtLook.v[2]);
}
