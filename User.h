//
//  User.h
//  Platform
//
//  Created by Corrigan Farley on 2/8/13.
//  Copyright (c) 2013 Corrigan Farley. All rights reserved.
//

#pragma once
#ifndef __Platform__User__
#define __Platform__User__

#include <systemc.h>
#include <Eigen/Core>

#include "World.h"

class User : public sc_module
{
public:
		 const static double TURN_SPEED;
		 const static double WALK_SPEED;
		 const static Eigen::Vector3d UP;

		 User(sc_module_name nm);

		 SC_HAS_PROCESS(User);

		 // We're not modelling communication along wires, we can use standard C++ types
		 sc_in<double> snd_yaw; // relative to the user (in radians)
		 sc_in<bool> walk;
		 sc_out<double> cur_yaw; // relative to world

		 void think();

		 /** Finds distance to nearest wall in some heading. -1 if no wall */
		 double nearest_wall(const Eigen::Vector3d& heading) const;
		 /** Finds distance to nearest wall based on a rotation around head tilt.
			* For instance, nearest_wall(pi/2) would find the nearest wall on the
			* user's left side (input is in radians).
			*/
		 // Currently uses UP for head tilt
		 double nearest_wall(double yaw) const;
protected:
		 Eigen::Vector3d heading;
		 Eigen::Vector4d position;
};

inline double User::nearest_wall(const Eigen::Vector3d& heading) const
{
		 return World::get_instance().dist_to_wall(position, heading);
}

#endif /* defined(__Platform__User__) */
