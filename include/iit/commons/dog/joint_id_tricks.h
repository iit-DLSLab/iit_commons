/*
 * Copyright (c) 2015-2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * Authors: Marco Frigerio, Michele Focchi, Marco Camurri
 *
 * This file is part of iit_commons, a library for
 * algebra, kinematics and dynamics for quadruped robots.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */


#pragma once


#include "declarations.h"
#include "leg_data_map.h"

namespace iit {
namespace dog {

inline JointIdentifiers toJointID(LegID leg, LegJoints j) {
    return orderedJointIDs[ leg * 3 + j ];
}

inline LegID toLegID(JointIdentifiers j) {
    static LegID legs[] = {LF, RF, LH, RH};
    return legs[ static_cast<int>(j) / 3 ];
}

inline bool isHAA(JointIdentifiers j) {
    return ((static_cast<int>(j) % 3) == 0);
}

//it assumes the legs are always the first joints
inline void getLegJointState(LegID leg, const JointState& jstate, rbd::Vector3d& vecout) {
    // WARNING: assuming three joints per leg
    vecout = (jstate.block<3, 1>(leg * 3, 0));
}

inline rbd::Vector3d getLegJointState(LegID leg, const JointState& jstate) {
    // WARNING: assuming three joints per leg
    return jstate.block<3, 1>(leg * 3, 0);
}

/**
 * @brief setLegJointState sets the portion of a JointState  variable with the
 * joint state of a specific leg.
 * @param[in] leg the identifier of the leg
 * @param[in] vecin the state of the joints of that leg
 * @param[out] jstate the full joint state variable to be set
 */
inline void setLegJointState(LegID leg, const rbd::Vector3d& vecin,  JointState& jstate)
{
    // WARNING: assuming three joints per leg
    jstate.block<3,1>(leg*3,0) = vecin;
}

inline bool belongsTo(LegID leg, const JointIdentifiers jointID)
{

    if ((leg==dog::LF) &&
            ((jointID == dog::LF_HAA)||(jointID == dog::LF_HFE)||(jointID == dog::LF_KFE)))
                return true;
    if ((leg==dog::RF) &&
            ((jointID == dog::RF_HAA)||(jointID == dog::RF_HFE)||(jointID == dog::RF_KFE)))
                return true;
    if ((leg==dog::LH) &&
            ((jointID == dog::LH_HAA)||(jointID == dog::LH_HFE)||(jointID == dog::LH_KFE)))
                return true;
    if ((leg==dog::RH) &&
            ((jointID == dog::RH_HAA)||(jointID == dog::RH_HFE)||(jointID == dog::RH_KFE)))
                return true;

    return false;
}

}
}

