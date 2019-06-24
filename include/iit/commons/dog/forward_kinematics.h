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

/**
 * A forward kinematics interface for quadrupeds.
 */
class ForwardKinematics
{
public:
    virtual ~ForwardKinematics() {}

    virtual Vector3d getFootPosLF(const JointState& q) = 0;
    virtual Vector3d getFootPosRF(const JointState& q) = 0;
    virtual Vector3d getFootPosLH(const JointState& q) = 0;
    virtual Vector3d getFootPosRH(const JointState& q) = 0;
    virtual Vector3d getFootPos  (const JointState& q, const LegID& leg) = 0;
    inline virtual dog::LegDataMap<Vector3d> getFeetPos  (const JointState& q)
    {
        dog::LegDataMap<dog::Vector3d> feetPos(Vector3d::Zero());
        feetPos[dog::LF] = getFootPosLF(q);
        feetPos[dog::RF] = getFootPosRF(q);
        feetPos[dog::LH] = getFootPosLH(q);
        feetPos[dog::RH] = getFootPosRH(q);
        return feetPos;
    }
    virtual Matrix3d getFootOrientation(const JointState& q, const LegID& leg) = 0;
    virtual Vector3d getShinPos(const JointState& q,
                                const double& contact_pos,
                                const LegID& leg) = 0;


};


}
}

