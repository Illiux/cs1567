//
//  World.cpp
//  Platform
//
//  Created by Corrigan Farley on 2/13/13.
//  Copyright (c) 2013 Corrigan Farley. All rights reserved.
//

#include "World.h"
#include <boost/foreach.hpp>
#include <Eigen/LU>
#define foreach BOOST_FOREACH

using namespace Eigen;
using std::list;

World* World::instance = NULL;

// TODO: Needs more testing
double World::dist_to_wall(const Vector4d& origin, const Vector3d& heading) const
{
		 list<double> dists;
		 foreach(const Wall& wall, walls) {
					// Direction points p2 -> p1
					Vector2d diff = (wall.p1 - wall.p2);
					Vector2d d0 = diff.normalized();
					Vector2d p0 = wall.p2;
					double tmax = diff.x()/d0.x();

					// Reduce input ray to 2D
					Vector2d d1(origin.x(), origin.z());
					Vector2d p1(heading.x(), heading.z());

					// Build matrix representation
					Matrix<double, 2, 2> a;
					Matrix<double, 2, 1> b;
					a << d0.y(), d0.x(), d1.y(), d1.x();
					b << -p0.y() - p0.x(), -p1.y() - p1.x();

					// Solve
					Matrix<double, 2, 1> res = a.fullPivLu().solve(b);
					if ((a*res).isApprox(b)) {
							 // Check our solution is within range
							 // Calculate t for each original equation
							 double t0, t1;
							 t0 = res.x()/d0.x();
							 t1 = res.x()/d1.x();

							 // t1 is input ray, t0 is wall
							 if ((t1 < 0) || (t0 < 0) || (t0 > tmax)) {
										return -1;
							 }

							 // Valid t, find distance from origin to intersect and return
							 return res.x()/res.normalized().x();
					} else {
							 // Unsolvable (e.g. no intersection)
							 return -1;
					}
		 }
		 // No walls to collide with
		 return -1;
}
