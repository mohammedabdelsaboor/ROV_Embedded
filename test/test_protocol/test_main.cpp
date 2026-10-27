#include <unity.h>
#include <RovProtocol.hpp>

void setUp() {}
void tearDown() {}

void test_motion_roundtrip() {
    rov::MotionCmd in;
    in.surge = 1000;
    in.sway  = -1000;
    in.heave = 123;
    in.yaw   = -1;
    rov::CanFrame frame;
    rov::encode(in, frame);
    TEST_ASSERT_EQUAL_UINT16(rov::ID_CMD_MOTION, frame.id);
    TEST_ASSERT_EQUAL_UINT8(8, frame.len);

    rov::MotionCmd out;
    TEST_ASSERT_TRUE(rov::decode(frame, out));
    TEST_ASSERT_EQUAL_INT16(in.surge, out.surge);
    TEST_ASSERT_EQUAL_INT16(in.sway, out.sway);
    TEST_ASSERT_EQUAL_INT16(in.heave, out.heave);
    TEST_ASSERT_EQUAL_INT16(in.yaw, out.yaw);
}

void test_state_roundtrip_and_gain_clamp() {
    rov::StateCmd in;
    in.armed = true;
    in.mode  = rov::Mode::Stabilize;
    in.gain  = 250;
    in.seq   = 77;
    rov::CanFrame frame;
    rov::encode(in, frame);

    rov::StateCmd out;
    TEST_ASSERT_TRUE(rov::decode(frame, out));
    TEST_ASSERT_TRUE(out.armed);
    TEST_ASSERT_EQUAL(static_cast<int>(rov::Mode::Stabilize), static_cast<int>(out.mode));
    TEST_ASSERT_EQUAL_UINT8(100, out.gain);
    TEST_ASSERT_EQUAL_UINT8(77, out.seq);
}

void test_status_roundtrip() {
    rov::StatusTlm in;
    in.flags    = rov::status::ARMED | rov::status::IMU_OK;
    in.supplyMv = 15800;
    in.imuTemp  = -250;
    rov::CanFrame frame;
    rov::encode(in, frame);

    rov::StatusTlm out;
    TEST_ASSERT_TRUE(rov::decode(frame, out));
    TEST_ASSERT_EQUAL_UINT8(in.flags, out.flags);
    TEST_ASSERT_EQUAL_UINT16(15800, out.supplyMv);
    TEST_ASSERT_EQUAL_INT16(-250, out.imuTemp);
}

void test_thruster_roundtrip() {
    rov::ThrusterTlm in;
    const int8_t     values[rov::NUM_THRUSTERS] = {-100, -1, 0, 1, 50, 100};
    for (int i = 0; i < rov::NUM_THRUSTERS; ++i) {
        in.output[i] = values[i];
    }
    rov::CanFrame frame;
    rov::encode(in, frame);

    rov::ThrusterTlm out;
    TEST_ASSERT_TRUE(rov::decode(frame, out));
    TEST_ASSERT_EQUAL_INT8_ARRAY(values, out.output, rov::NUM_THRUSTERS);
}

void test_decode_rejects_wrong_id_or_short_frame() {
    rov::MotionCmd motion;
    rov::CanFrame  frame;
    rov::encode(motion, frame);

    rov::StateCmd state;
    TEST_ASSERT_FALSE(rov::decode(frame, state));

    frame.len = 4;
    TEST_ASSERT_FALSE(rov::decode(frame, motion));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_motion_roundtrip);
    RUN_TEST(test_state_roundtrip_and_gain_clamp);
    RUN_TEST(test_status_roundtrip);
    RUN_TEST(test_thruster_roundtrip);
    RUN_TEST(test_decode_rejects_wrong_id_or_short_frame);
    return UNITY_END();
}
