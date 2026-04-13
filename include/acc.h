#pragma once
#include <cstdint>

enum class AccState : uint8_t
{
    FREE_CRUISE = 0,
    FOLLOWING = 1,
    EMERGENCY = 2
};

struct AccInput
{
    float ego_speed;
    float target_distance;
    float set_speed;
    bool vehicle_detected;
};

struct AccOutput
{
    float throttle_cmd;
    float brake_cmd;
};

class AdaptiveCruiseControl
{
public:
    AdaptiveCruiseControl();

    void reset();
    AccOutput update(const AccInput &input);
    AccState getState() const { return state_; }

private:
    AccState state_;
    float speed_error_integral_;
    float last_distance_error_;

    static constexpr float FOLLOWING_THRESHOLD = 50.0f;
    static constexpr float FOLLOWING_HYSTERESIS = 10.0f;
    static constexpr float EMERGENCY_THRESHOLD = 8.0f;
    static constexpr float SAFE_TIME_HEADWAY = 2.0f;
    static constexpr float MIN_SAFE_GAP = 5.0f;

    static constexpr float KP_SPEED = 0.05f;
    static constexpr float KI_SPEED = 0.01f;
    static constexpr float KP_DISTANCE = 0.02f;
    static constexpr float KD_DISTANCE = 0.05f;
    static constexpr float MAX_INTEGRAL = 1.0f;

    static constexpr float CYCLE_TIME_S = 0.01f;

    AccOutput freeCruise(const AccInput &input);
    AccOutput following(const AccInput &input);
    AccOutput emergency();
    void updateState(const AccInput &input);
    static float clamp(float v, float lo, float hi);
};

extern "C"
{
    void *acc_create();
    void acc_destroy(void *handle);
    void acc_reset(void *handle);
    void acc_update(void *handle,
                    float ego_speed, float target_distance,
                    float set_speed, int vehicle_detected,
                    float *throttle_out, float *brake_out);
    int acc_get_state(void *handle);
}
