#pragma once

#include <stdint.h>

// Mixer for the BlueROV2-style 6-thruster "vectored" frame:
// 4 horizontal thrusters at 45 deg in the corners + 2 vertical on the sides.
//
//            FRONT
//      T2 //      \\ T1        T1 front-right   T2 front-left
//          |  T6 T5 |          T3 rear-right    T4 rear-left
//      T4 \\      // T3        T5 vertical right
//            REAR              T6 vertical left
//
// The motor factors are the ArduSub SUB_FRAME_VECTORED matrix
// (ArduPilot libraries/AP_Motors/AP_Motors6DOF.cpp). They assume BlueROV2
// thruster orientations; fix any thruster that spins the wrong way with the
// per-motor "reversed" setting, never by editing the matrix.
//
// With only side-mounted vertical thrusters this frame controls roll but not
// pitch, exactly like the BlueROV2 standard frame.
class ThrusterMixer {
public:
    static constexpr uint8_t NUM_MOTORS = 6;

    // All inputs in [-1, 1]. Body frame: + roll = right side down,
    // + yaw = clockwise from above, + climb = up, + forward, + lateral = right.
    struct Input {
        float roll    = 0.0f;
        float pitch   = 0.0f;
        float yaw     = 0.0f;
        float climb   = 0.0f;
        float forward = 0.0f;
        float lateral = 0.0f;
    };

    // Writes NUM_MOTORS outputs in [-1, 1]. If any motor would saturate, all
    // outputs are scaled down together so the commanded direction is kept.
    static void mix(const Input& in, float out[NUM_MOTORS]);
};
