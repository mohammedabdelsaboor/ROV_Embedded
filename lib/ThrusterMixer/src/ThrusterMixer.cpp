#include "ThrusterMixer.hpp"

#include <math.h>

namespace {

struct Factors {
    float roll, pitch, yaw, climb, forward, lateral;
};

// clang-format off
// ArduSub SUB_FRAME_VECTORED
//                                 roll   pitch  yaw    climb  forward lateral
const Factors FACTORS[ThrusterMixer::NUM_MOTORS] = {
    /* T1 front-right    */ {  0.0f, 0.0f,  1.0f,  0.0f, -1.0f,  1.0f },
    /* T2 front-left     */ {  0.0f, 0.0f, -1.0f,  0.0f, -1.0f, -1.0f },
    /* T3 rear-right     */ {  0.0f, 0.0f, -1.0f,  0.0f,  1.0f,  1.0f },
    /* T4 rear-left      */ {  0.0f, 0.0f,  1.0f,  0.0f,  1.0f, -1.0f },
    /* T5 vertical right */ {  1.0f, 0.0f,  0.0f, -1.0f,  0.0f,  0.0f },
    /* T6 vertical left  */ { -1.0f, 0.0f,  0.0f, -1.0f,  0.0f,  0.0f },
};
// clang-format on

}  // namespace

void ThrusterMixer::mix(const Input& in, float out[NUM_MOTORS]) {
    float largest = 1.0f;
    for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
        const Factors& f = FACTORS[i];
        out[i] = f.roll * in.roll + f.pitch * in.pitch + f.yaw * in.yaw + f.climb * in.climb +
                 f.forward * in.forward + f.lateral * in.lateral;
        if (fabsf(out[i]) > largest) {
            largest = fabsf(out[i]);
        }
    }
    for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
        out[i] /= largest;
    }
}
