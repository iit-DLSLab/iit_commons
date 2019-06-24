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

#include <iit/rbd/rbd.h>

/*! \mainpage IIT Commons Library
 *
 * \section intro_sec Introduction
 *
 * This library contains many useful tools for algebra, rotations, kinematics
 * and dynamics of a quadruped robot...
 *
 * \section install_sec Installation
 * this is a CMake project, therefore you can build and install this package as
 * usual:
 * \code{.sh}
 * mkdir build
 * cd build
 * cmake ..
 * make
 * sudo make install
 * \endcode
 * this will build and install the library in <code>/usr/local</code>
 */

namespace iit {
namespace dog {

static const int JointSpaceDimension = 12;
static const int jointsCount = 12;
static const int jointsLegCount = 3; //number of joints per leg
static const int contactConstrCount = 3; //number of constraint for each contact point

//index to define the joint state
static const int baseJoints = 0;
static const int activeJoints = 6;

static const int fbjointsCount = 18;
typedef Eigen::Matrix<double, fbjointsCount, 1> Column18d;
typedef Column18d FloatingBaseJointState;

/** The total number of rigid bodies of this robot, including the base */
static const int linksCount  = 13;

typedef Eigen::Matrix<double, 12, 1> Column12d;
typedef Column12d JointState;
typedef Eigen::Array<bool, 3, 1> LegJointBool;

typedef iit::rbd::Vector3d Vector3d;
typedef iit::rbd::Matrix33d Matrix3d;
typedef iit::rbd::PlainMatrix<double, 3, 3 > FootJac;

typedef Eigen::Vector3d LegJointState;


enum JointIdentifiers {
    LF_HAA = 0
    , LF_HFE
    , LF_KFE
    , RF_HAA
    , RF_HFE
    , RF_KFE
    , LH_HAA
    , LH_HFE
    , LH_KFE
    , RH_HAA
    , RH_HFE
    , RH_KFE
};


enum LinkIdentifiers {
    TRUNK = 0
    , LF_HIPASSEMBLY
    , LF_UPPERLEG
    , LF_LOWERLEG
    , RF_HIPASSEMBLY
    , RF_UPPERLEG
    , RF_LOWERLEG
    , LH_HIPASSEMBLY
    , LH_UPPERLEG
    , LH_LOWERLEG
    , RH_HIPASSEMBLY
    , RH_UPPERLEG
    , RH_LOWERLEG
};

static const JointIdentifiers orderedJointIDs[jointsCount] =
    {LF_HAA,LF_HFE,LF_KFE,RF_HAA,RF_HFE,RF_KFE,LH_HAA,LH_HFE,LH_KFE,RH_HAA,RH_HFE,RH_KFE};

static const LinkIdentifiers orderedLinkIDs[linksCount] =
    {TRUNK,LF_HIPASSEMBLY,LF_UPPERLEG,LF_LOWERLEG,RF_HIPASSEMBLY,RF_UPPERLEG,RF_LOWERLEG,LH_HIPASSEMBLY,LH_UPPERLEG,LH_LOWERLEG,RH_HIPASSEMBLY,RH_UPPERLEG,RH_LOWERLEG};



}
}

