#pragma once

#include "SCurveAcceleration.hpp"

class SCurveMotionProfile {
public:
    using Constraints = SCurveAcceleration::Constraints;
    using Step = SCurveAcceleration::Step;

private:
    SCurveAcceleration accelerationProfile;
    Constraints constraints;

    float displacement;
    float cruiseDisplacement;
    float cruiseTime;
    float accelerationTime;
    float accelerationDisplacement;

public:
    SCurveMotionProfile(Constraints constraints, float displacement) : accelerationProfile(constraints), constraints(constraints), displacement(displacement) {
        accelerationTime = accelerationProfile.totalTime();
        accelerationDisplacement = accelerationProfile.totalDisplacement();
        cruiseDisplacement = displacement - accelerationDisplacement * 2.0f;
        cruiseTime = cruiseDisplacement / constraints.velocity;
    }

    Step stepAtTime(float t) {
        if (t < accelerationTime) {
            return accelerationProfile.stepAtT(t);
        }

        t -= accelerationTime;

        if (t < cruiseTime) {
            return Step(accelerationDisplacement + constraints.velocity * t, constraints.velocity);
        }

        t -= cruiseTime;

        Step step = accelerationProfile.stepAtT(accelerationTime - t);

        step.dist = accelerationDisplacement + cruiseDisplacement + (accelerationDisplacement - step.dist);
        step.acceleration *= -1;
        step.jerk *= -1;

        return step;
    }

    Step stepAtDisplacement(float s) {
        if (s < accelerationDisplacement) {
            return accelerationProfile.stepAtDisplacement(s);
        }

        s -= accelerationDisplacement;

        if (s < cruiseDisplacement) {
            return Step(accelerationDisplacement + s, constraints.velocity);
        }

        s -= cruiseDisplacement;

        Step step = accelerationProfile.stepAtDisplacement(accelerationDisplacement - s);

        step.dist = accelerationDisplacement + cruiseDisplacement + (accelerationDisplacement - step.dist);
        step.acceleration *= -1;
        step.jerk *= -1;

        return step;
    }

    float timeAtDisplacement(float s) {
        if (s < accelerationDisplacement) {
            return accelerationProfile.timeAtDisplacement(s);
        }

        s -= accelerationDisplacement;

        if (s < cruiseDisplacement) {
            return accelerationTime + s / constraints.velocity;
        }

        s -= cruiseDisplacement;

        return accelerationTime + cruiseTime + accelerationTime - accelerationProfile.timeAtDisplacement(accelerationDisplacement - s);
    }

    float totalTime() {
        return accelerationTime + accelerationTime + cruiseTime;
    }
};
