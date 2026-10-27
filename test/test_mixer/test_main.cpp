#include <unity.h>
#include <ThrusterMixer.hpp>

void setUp() {}
void tearDown() {}

namespace {
constexpr float EPS = 1e-5f;
enum { T1, T2, T3, T4, T5, T6 };
}  // namespace

void test_neutral_is_all_zero() {
    float out[ThrusterMixer::NUM_MOTORS];
    ThrusterMixer::mix(ThrusterMixer::Input(), out);
    for (float v : out) {
        TEST_ASSERT_FLOAT_WITHIN(EPS, 0.0f, v);
    }
}

void test_forward_uses_horizontal_only() {
    ThrusterMixer::Input in;
    in.forward = 0.5f;
    float out[ThrusterMixer::NUM_MOTORS];
    ThrusterMixer::mix(in, out);
    TEST_ASSERT_FLOAT_WITHIN(EPS, -0.5f, out[T1]);
    TEST_ASSERT_FLOAT_WITHIN(EPS, -0.5f, out[T2]);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 0.5f, out[T3]);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 0.5f, out[T4]);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 0.0f, out[T5]);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 0.0f, out[T6]);
}

void test_climb_uses_vertical_only() {
    ThrusterMixer::Input in;
    in.climb = 1.0f;
    float out[ThrusterMixer::NUM_MOTORS];
    ThrusterMixer::mix(in, out);
    for (int i = T1; i <= T4; ++i) {
        TEST_ASSERT_FLOAT_WITHIN(EPS, 0.0f, out[i]);
    }
    TEST_ASSERT_FLOAT_WITHIN(EPS, -1.0f, out[T5]);
    TEST_ASSERT_FLOAT_WITHIN(EPS, -1.0f, out[T6]);
}

void test_roll_is_differential_on_verticals() {
    ThrusterMixer::Input in;
    in.roll = 0.3f;
    float out[ThrusterMixer::NUM_MOTORS];
    ThrusterMixer::mix(in, out);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 0.3f, out[T5]);
    TEST_ASSERT_FLOAT_WITHIN(EPS, -0.3f, out[T6]);
}

void test_saturation_scales_all_outputs_together() {
    ThrusterMixer::Input in;
    in.forward = 1.0f;
    in.lateral = 1.0f;  // T1/T4 cancel, T2/T3 would hit 2.0
    float out[ThrusterMixer::NUM_MOTORS];
    ThrusterMixer::mix(in, out);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 0.0f, out[T1]);
    TEST_ASSERT_FLOAT_WITHIN(EPS, -1.0f, out[T2]);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 1.0f, out[T3]);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 0.0f, out[T4]);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_neutral_is_all_zero);
    RUN_TEST(test_forward_uses_horizontal_only);
    RUN_TEST(test_climb_uses_vertical_only);
    RUN_TEST(test_roll_is_differential_on_verticals);
    RUN_TEST(test_saturation_scales_all_outputs_together);
    return UNITY_END();
}
