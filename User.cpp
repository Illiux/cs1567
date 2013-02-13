//
//  User.cpp
//  Platform
//
//  Created by Corrigan Farley on 2/8/13.
//  Copyright (c) 2013 Corrigan Farley. All rights reserved.
//

#include "User.h"

#include <cmath>
#include <Eigen/Geometry>
#include <boost/math/constants/constants.hpp>

#define sign(x) ((x > 0) - (x < 0))

using namespace Eigen;
using boost::math::constants::pi;

const Vector3d User::UP(0, 1, 0);
const double User::TURN_SPEED = pi<double>()/8.0; // rads/10ms
const double User::WALK_SPEED = 0.05; // meters/10ms

User::User(sc_module_name nm) : sc_module(nm), heading(1, 0, 0)
{
		 SC_THREAD(think);
}

void User::think()
{
		 while (true) {
					// TODO: Use an actual angle class for calculations
					wait(10, SC_MS); // 10ms tick
					double snd_yaw = this->snd_yaw.read(); // read to local var

					// If the sound origin is anything but 0, we need to orient
					if (snd_yaw != 0) {
							 double turnspeed = sign(snd_yaw) * TURN_SPEED;
							 double dyaw = (abs(snd_yaw) > TURN_SPEED) ? turnspeed : snd_yaw;
							 // Rotate while maintaining pitch
							 AngleAxis<double> rotation(-dyaw, UP);
							 heading = rotation * heading;
							 cur_yaw.write(std::atan2(heading.z(), heading.x()));
					}
					if (walk.read()) {
							 position += (heading * WALK_SPEED).homogeneous();
							 // TODO: Collision detection here
					}
		 }
}

double User::nearest_wall(double yaw) const
{
		 AngleAxis<double> rotation(yaw, UP);
		 Vector3d rayheading = rotation * this->heading;
		 return World::get_instance().dist_to_wall(position, rayheading);
}
