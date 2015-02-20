#ifndef A4988_HPP
#define A4988_HPP

#include <iostream>
#include <cstdint>
#include <cstdlib>
#include <math.h>

#include "MockDriver.hpp"

static const float MICROSECONDS_PER_SECOND = 1000000.0f;

template <typename T>
T constrain(T value, T minimum, T maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

template <typename T>
int sgn(T value)
{
    return (value > T(0)) - (value < T(0));
}

template <typename T>
T max(T a, T b)
{
    return (a > b) ? a : b;
}

typedef MockDriver Driver;

class A4988 : private Driver
{
public:
    A4988(float max_speed=1.0f, float acceleration=0.0f, uint8_t microstep_denominator=16);

    void reset();

    void setSleep(bool sleep);
    bool isSleeping() const;

    void setEnabled(bool enabled);
    bool isEnabled() const;

    void setPosition(long position);
    long position() const;
    void moveTo(long position);
    void moveBy(long displacement);
    long displacementToTarget() const;
    bool isClockwiseToTarget() const;
    int  directionToTarget() const;

    // TODO: max and min position.

    void  runAtVelocity(float steps_per_second);

    float velocity() const;
    int   movementDirection() const;
    bool  isMovementClockwise() const;

    void  setMaxSpeed(float steps_per_second);
    float maxSpeed() const;

    void  setAcceleration(float steps_per_second_per_second);
    float acceleration() const;

    void    setMicrostep(uint8_t denominator);
    uint8_t microstep() const;

    bool poll();

    void stop();

private:
    bool runSpeed();
    bool isStepDue(unsigned long time);
    void updateTrajectory();
    float nextStepInterval(float step_interval, long n);
    long stoppingDistance(float velocity, float acceleration);
    void forwardStep();
    void backwardStep();

    uint8_t microstep_;
    long current_position_;
    long target_position_;
    float current_velocity_;
    float target_velocity_;
    float max_speed_;
    float acceleration_;
    float step_interval_;
    float min_step_interval_;
    float initial_step_interval_;
    unsigned long last_step_time_;
    bool is_positioning_;
    long n_;

    bool positioningTrajectory();
    bool runningTrajectory();

    bool isAccelerating();

    bool isDecelerating();

    int nextDirection();
};

A4988::A4988(float max_speed, float acceleration, uint8_t microstep_denominator) :
        Driver(),
        microstep_(microstep_denominator),
        current_position_(0L),
        target_position_(0L),
        current_velocity_(0.0f),
        target_velocity_(0.0f),
        max_speed_(max_speed),
        acceleration_(0.0f),  // Must be different to the value we use below
        step_interval_(0.0f),
        min_step_interval_(MICROSECONDS_PER_SECOND / max_speed_),
        initial_step_interval_(0.0f),
        last_step_time_(0UL),
        is_positioning_(true),
        n_(0L)
{
    setAcceleration(acceleration);
}

void A4988::reset()
{
    this->writeReset();
}

void A4988::setSleep(bool sleep)
{
    this->writeNotSleep(sleep ? 0 : 1);
}

bool A4988::isSleeping() const
{
    return this->readNotSleep() != 1;
}

void A4988::setEnabled(bool enabled)
{
    this->writeNotEnable(enabled ? 0 : 1);
}

bool A4988::isEnabled() const
{
    return this->readNotEnable() != 1;
}

void A4988::forwardStep()
{
    this->writeForwardStep();
}

void A4988::backwardStep()
{
    this->writeBackwardStep();
}

void A4988::setPosition(long position)
{
    // TODO: Validate against position limits
    target_position_ = current_position_ = position;
    n_ = 0;
    step_interval_ = 0L;
}

long A4988::position() const
{
    return current_position_;
}

void A4988::moveTo(long position)
{
    is_positioning_ = true;
    if (target_position_ != position)
    {
        target_position_ = position;
        updateTrajectory();
    }
}

void A4988::stop()
{
    // Irrespective of which mode we were in previously, we're now positioning
    is_positioning_ = true;

    if (current_velocity_ != 0.0f)
    {
        // Given current kinematics, what is the displacement to the nearest point at
        // which we can stop?
        long s = movementDirection() * stoppingDistance(current_velocity_, acceleration_);

        // Set the step number to the negative number of steps away from stop. This ensures
        // we decelerate to stop.
        n_ = -std::abs(s);

        // The target position is the current position plus the displacement to stop
        target_position_ = current_position_ + s;
        updateTrajectory();
    }
}

void A4988::moveBy(long displacement)
{
    moveTo(current_position_ + displacement);
}

/**
*  +ve : target is anticlockwise from current position
*  -ve : target is clockwise from current position
*/
long A4988::displacementToTarget() const
{
    return target_position_ - current_position_;
}

bool A4988::isClockwiseToTarget() const
{
    return directionToTarget() == 1;
}

int A4988::directionToTarget() const
{
    return sgn(displacementToTarget());
}

void A4988::runAtVelocity(float v)
{
    if (v == 0.0f) {
        stop();
        return;
    }

    is_positioning_ = false;

    target_velocity_ = constrain(v, -max_speed_, max_speed_);

    if (current_velocity_ != target_velocity_)
    {

        if (target_velocity_ > current_velocity_) {
            // Need to accelerate
            //long target_velocity_stopping_distance = stoppingDistance(target_velocity_, acceleration_);
            long current_velocity_stopping_distance = stoppingDistance(current_velocity_, acceleration_);
            n_ = current_velocity_stopping_distance;

        }
        else if (target_velocity_ < current_velocity_) {
            // Need to decelerate
            //long target_velocity_stopping_distance = stoppingDistance(target_velocity_, acceleration_);
            long current_velocity_stopping_distance = stoppingDistance(current_velocity_, acceleration_);
            n_ =  -current_velocity_stopping_distance;
        }

        updateTrajectory();
    }
}

float A4988::velocity() const {
    return current_velocity_;
}

int A4988::movementDirection() const {
    return sgn(current_velocity_);
}

bool A4988::isMovementClockwise() const {
    return movementDirection() == 1;
}

void A4988::setMaxSpeed(float speed)
{
    if (max_speed_ != speed)
    {
        max_speed_ = speed;
        min_step_interval_ = MICROSECONDS_PER_SECOND / speed;
        // Recompute _n from current speed and adjust speed if accelerating or cruising
        if (n_ != 0)
        {
            n_ = stoppingDistance(current_velocity_, acceleration_);
            updateTrajectory();
        }
    }
}

float A4988::maxSpeed() const
{
    return max_speed_;
}

void A4988::setAcceleration(float acceleration)
{
    if (acceleration <= 0.0)
        return;

    if (acceleration != acceleration_)
    {
        // Recompute n_ per Equation 17
        n_ = static_cast<long>(n_ * (acceleration_ / acceleration));
        // New c0 per Equation 7
        initial_step_interval_ = float(sqrt(1.0 / acceleration) * MICROSECONDS_PER_SECOND);
        acceleration_ = acceleration;
        updateTrajectory();
    }
}

float A4988::acceleration() const {
    return acceleration_;
}

void A4988::setMicrostep(uint8_t denominator)
{
    switch (denominator)
    {
        case 1:
        case 2:
        case 4:
        case 8:
        case 16:
            microstep_ = denominator;
    }
}

uint8_t A4988::microstep() const
{
    return microstep_;
}

bool A4988::runSpeed() {
    if (step_interval_ == 0L)
    {
        return false;
    }

    unsigned long time = this->clock();
    if (isStepDue(time))
    {
        if (isMovementClockwise()) // Or clockwiseTo()
        {
            // Clockwise
            current_position_ += 1;
            forwardStep();
        }
        else
        {
            // Anticlockwise
            current_position_ -= 1;
            backwardStep();
        }

        last_step_time_ = time;
        return true;
    }
    return false;
}

bool A4988::poll()
{
    //std::cout << "n_ = " << n_ << std::endl;
    bool has_stepped = runSpeed();

    if (has_stepped)
    {
        updateTrajectory();
    }
    return current_velocity_ != 0.0 || displacementToTarget() != 0; // TODO: Running mode?
}


bool A4988::isStepDue(unsigned long time)
{
    unsigned long next_step_time = last_step_time_ + static_cast<unsigned long>(step_interval_);
    // Gymnastics to detect wrapping of either the nextStepTime and/or the current time
    return ((next_step_time >= last_step_time_) && ((time >= next_step_time) || (time < last_step_time_)))
        || ((next_step_time <  last_step_time_) && ((time >= next_step_time) && (time < last_step_time_)));
}

void A4988::updateTrajectory()
{
    if (is_positioning_) {
        bool done = positioningTrajectory();
    }
    else // running
    {
        bool done = runningTrajectory();
    }
}

// TODO: make static
float A4988::nextStepInterval(float current_step_interval, long n) {
    float step_interval = current_step_interval - ((2.0f * current_step_interval) / ((4.0f * n) + 1)); // Equation 13
    return max(step_interval, min_step_interval_);
};

bool A4988::runningTrajectory() {
    // Trajectory from the current velocity to a target velocity
    long st = sgn(target_velocity_) * stoppingDistance(target_velocity_, acceleration_);
    long cv = sgn(current_velocity_) * stoppingDistance(current_velocity_, acceleration_);

    if (st == cv)
    {
        step_interval_ = MICROSECONDS_PER_SECOND / fabsf(target_velocity_);
        current_velocity_ = target_velocity_;
        n_ = 0;
        return true;
    }

    // Need to accelerate or decelerate
    int dir;
    if (n_ == 0)
    {
        // Initial step from stopped
        step_interval_ = initial_step_interval_;
        dir = sgn(target_velocity_);
    }
    else
    {
        // Subsequent steps.
        step_interval_ = nextStepInterval(step_interval_, n_);
        dir = (current_velocity_ == 0.0f) ? sgn(target_velocity_) : movementDirection();
    }
    current_velocity_ = dir * MICROSECONDS_PER_SECOND / step_interval_;

    ++n_;

    return false;
}

bool A4988::positioningTrajectory() {
    long stopping_distance = stoppingDistance(current_velocity_, acceleration_);
    int movement_direction = movementDirection();
    int direction_to_target = directionToTarget();

    if (direction_to_target == 0 && stopping_distance <= 1)
    {
        // We are at the target and it's time to stop
        step_interval_ = 0;
        current_velocity_ = 0.0f;
        n_ = 0;
        return true;
    }
    else if (movement_direction == direction_to_target) {


        // n_ is a step number on a ramp from zero
        // Note: n_ is -ve : deceleration
        //       n_ is   0 : start/stop
        //       n_ is +ve : acceleration

        if (direction_to_target > 0) {
            if (isAccelerating()) {
                if (stopping_distance >= displacementToTarget()) {
                    n_ = -stopping_distance; // Decelerate to stop in n steps
                }
            }
            else if (isDecelerating()) {
                if ((stopping_distance < displacementToTarget())) {
                    n_ = -n_; // Start acceleration
                }
            }
        }
        else if (direction_to_target < 0) {
            if (isAccelerating()) {
                if (stopping_distance >= -displacementToTarget()) {
                    n_ = -stopping_distance; // Start deceleration
                }
            }
            else if (isDecelerating()) {
                if (stopping_distance < -displacementToTarget()) {
                    n_ = -n_; // Start acceleration
                }
            }
        }
    }
    else if (direction_to_target != 0 && stopping_distance == 0L) // Stopped, but not at target.
    {
        n_ = 0;
        current_velocity_ = 0.0f;
    }
    else if (movement_direction != direction_to_target)
    {
        n_ = -stopping_distance;
    }

    int dir;
    if (n_ == 0)
    {
        // Initial step from stopped
        step_interval_ = initial_step_interval_;
        dir = directionToTarget();
    }
    else
    {
        // Subsequent steps.
        step_interval_ = nextStepInterval(step_interval_, n_);
        dir = (current_velocity_ == 0.0f) ? directionToTarget() : movementDirection();
    }
    current_velocity_ = dir * MICROSECONDS_PER_SECOND / step_interval_;

    ++n_;

    return false;
}

bool A4988::isDecelerating() {
    return n_ < 0;
}

bool A4988::isAccelerating() {
    return n_ > 0;
}

/**
*  Number of steps to decelerate from velocity to zero at acceleration.
*/
long A4988::stoppingDistance(float velocity, float acceleration) {
    // Equation 16
    return (long)((velocity * velocity) / (2.0 * acceleration));
}

#endif