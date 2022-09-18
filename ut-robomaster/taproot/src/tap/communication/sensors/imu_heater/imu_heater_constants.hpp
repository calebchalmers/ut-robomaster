/*****************************************************************************/
/********** !!! WARNING: CODE GENERATED BY TAPROOT. DO NOT EDIT !!! **********/
/*****************************************************************************/

/*
 * Copyright (c) 2020-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of Taproot.
 *
 * Taproot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Taproot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Taproot.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef TAPROOT_IMU_HEATER_CONSTANTS_HPP_
#define TAPROOT_IMU_HEATER_CONSTANTS_HPP_

#include "tap/communication/gpio/pwm.hpp"

namespace tap::communication::sensors::imu_heater::bound_ports
{
    static constexpr tap::gpio::Pwm::Timer IMU_HEATER_TIMER = tap::gpio::Pwm::Timer::TIMER10;
}  // tap::sensors

#endif  // TAPROOT_IMU_HEATER_CONSTANTS_HPP_
