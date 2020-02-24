/*
 * Copyright (c) 2018-2019 REV Robotics
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of REV Robotics nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <cmath>
#include "sparkmax/rev/CANSparkMaxLowLevel.h"

namespace rev {

class CANSparkMax;
  
class CANSensor {
public:

    explicit CANSensor(CANSparkMax& device) { m_device = &device; }

    /**
     * Set the phase of the CANSensor so that it is set to be in
     * phase with the motor itself. This only works for quadrature
     * encoders and analog sensors. This will throw an error
     * if the user tries to set the inversion of the hall effect. 
     */ 
    virtual CANError SetInverted(bool inverted) = 0;

    /**
     * Get the phase of the CANSensor. This will just return false
     * if the user tries to get the inversion of the hall effect.
     */
    virtual bool GetInverted() const = 0;

    /** The range of the feedback sensor can be specified if the sensor
     * has a limited range, such as an absolute encoder or analog device. 
     * The default range is -infinity to infinity. This is NOT the same
     * as setting soft limits, as this will only check to ensure that 
     * setpoint commands for the controller fall within this range. 
     * 
     * @param minRange The lower bound on the range of the sensor
     * @param maxRange The upper bound on the range of the sensor
     */
    // CANError SetFeedbackSensorRange(float minRange = -INFINITY, float maxRange = INFINITY) {
    //     auto status = c_SparkMax_SetFeedbackDeviceRange(static_cast<c_SparkMax_handle>(m_device->m_sparkMax), minRange, maxRange);
    //     return static_cast<CANError>(status);
    // }

protected:
    CANSparkMax* m_device;

// Friend so that the PIDcontroller knows what SensorType to send to the controller when changing
// feedback devices
friend class CANPIDController;

    /**
     * Get the ID of the sensor that is connected to the SparkMax through
     * the encoder port on the front of the controller (not the top port).
     * 
     * @return The ID of the sensor
     */
    virtual int GetID() const = 0;

    enum class FeedbackSensorType {
        kNoSensor = 0,
        kHallSensor = 1,
        kQuadrature = 2,
        kSensorless = 3,
        kAnalog = 4,
        kAltQuadrature = 5,
    };
};

}  // namespace rev