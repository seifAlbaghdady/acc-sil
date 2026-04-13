#include "acc.h"
#include <algorithm>
#include <cmath>

AdaptiveCruiseControl::AdaptiveCruiseControl()
    : state_(AccState::FREE_CRUISE), speed_error_integral_(0.0f), last_distance_error_(0.0f)
{
}

void AdaptiveCruiseControl::reset()
{
    state_ = AccState::FREE_CRUISE;
    speed_error_integral_ = 0.0f;
    last_distance_error_ = 0.0f;
}

AccOutput AdaptiveCruiseControl::update(const AccInput &input)
{
    updateState(input);

    switch (state_)
    {
    case AccState::FREE_CRUISE:
        return freeCruise(input);
    case AccState::FOLLOWING:
        return following(input);
    case AccState::EMERGENCY:
        return emergency();
    default:
        return {0.0f, 0.0f};
    }
}

void AdaptiveCruiseControl::updateState(const AccInput &input)
{
    switch (state_)
    {

    case AccState::FREE_CRUISE:
        if (input.vehicle_detected &&
            input.target_distance < FOLLOWING_THRESHOLD)
        {
            state_ = AccState::FOLLOWING;
        }
        break;

    case AccState::FOLLOWING:
        if (input.target_distance < EMERGENCY_THRESHOLD)
        {
            state_ = AccState::EMERGENCY;
        }
        else if (!input.vehicle_detected ||
                 input.target_distance >
                     FOLLOWING_THRESHOLD + FOLLOWING_HYSTERESIS)
        {
            state_ = AccState::FREE_CRUISE;
            speed_error_integral_ = 0.0f;
        }
        break;

    case AccState::EMERGENCY:
        if (input.vehicle_detected &&
            input.target_distance > EMERGENCY_THRESHOLD &&
            input.ego_speed < 2.0f)
        {
            state_ = AccState::FOLLOWING;
        }
        break;
    }
}

// ── Control laws ────────────────────────────────────────────

AccOutput AdaptiveCruiseControl::freeCruise(const AccInput &input)
{
    float error = input.set_speed - input.ego_speed;

    speed_error_integral_ += error * CYCLE_TIME_S;
    speed_error_integral_ = clamp(speed_error_integral_,
                                  -MAX_INTEGRAL, MAX_INTEGRAL);

    float control = KP_SPEED * error + KI_SPEED * speed_error_integral_;

    AccOutput out;
    out.throttle_cmd = clamp(control, 0.0f, 1.0f);
    out.brake_cmd = (error < -2.0f)
                        ? clamp(-KP_SPEED * error, 0.0f, 0.3f)
                        : 0.0f;
    return out;
}

AccOutput AdaptiveCruiseControl::following(const AccInput &input)
{
    float safe_distance = SAFE_TIME_HEADWAY * input.ego_speed + MIN_SAFE_GAP;
    float distance_error = input.target_distance - safe_distance;
    float d_error = (distance_error - last_distance_error_) / CYCLE_TIME_S;
    last_distance_error_ = distance_error;

    float control = KP_DISTANCE * distance_error + KD_DISTANCE * d_error;

    AccOutput out;
    if (control > 0.0f)
    {
        out.throttle_cmd = clamp(control, 0.0f, 0.5f);
        out.brake_cmd = 0.0f;
    }
    else
    {
        out.throttle_cmd = 0.0f;
        out.brake_cmd = clamp(-control, 0.0f, 0.8f);
    }
    return out;
}

AccOutput AdaptiveCruiseControl::emergency()
{
    return {0.0f, 1.0f};
}

float AdaptiveCruiseControl::clamp(float v, float lo, float hi)
{
    return std::max(lo, std::min(hi, v));
}

extern "C"
{

    void *acc_create()
    {
        return new AdaptiveCruiseControl();
    }

    void acc_destroy(void *handle)
    {
        delete static_cast<AdaptiveCruiseControl *>(handle);
    }

    void acc_reset(void *handle)
    {
        static_cast<AdaptiveCruiseControl *>(handle)->reset();
    }

    void acc_update(void *handle,
                    float ego_speed, float target_distance,
                    float set_speed, int vehicle_detected,
                    float *throttle_out, float *brake_out)
    {
        AccInput in{ego_speed, target_distance, set_speed,
                    static_cast<bool>(vehicle_detected)};
        AccOutput out = static_cast<AdaptiveCruiseControl *>(handle)->update(in);
        *throttle_out = out.throttle_cmd;
        *brake_out = out.brake_cmd;
    }

    int acc_get_state(void *handle)
    {
        return static_cast<int>(
            static_cast<AdaptiveCruiseControl *>(handle)->getState());
    }
}
