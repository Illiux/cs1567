//
//  World.h
//  Platform
//
//  Created by Corrigan Farley on 2/13/13.
//  Copyright (c) 2013 Corrigan Farley. All rights reserved.
//

#pragma once
#ifndef __Platform__World__
#define __Platform__World__

#include <Eigen/Core>
#include <boost/ptr_container/ptr_list.hpp>

class World
{
		 struct Wall
		 {
					Wall(Eigen::Vector2d p1, Eigen::Vector2d p2) : p1(p1), p2(p2) {}
					Eigen::Vector2d p1, p2;
		 };

public:
		 World() {};
		 /** Constructs a new world. If setmain is true the new instance is set as the
			* main instance.
			*/
		 World(bool setmain);

		 /** Get main instance. NULL if not yet initialized. */
		 static World& get_instance() { return *instance; }
		 
		 double dist_to_wall(const Eigen::Vector4d& origin,
												 const Eigen::Vector3d& heading) const;
		 void add_wall(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2);
protected:
		 static World* instance;
		 boost::ptr_list<Wall> walls;
};

inline void World::add_wall(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2)
{
		 walls.push_back(new Wall(p1, p2));
}

inline World::World(bool setmain)
{
		 if (setmain) instance = this;
}

#endif /* defined(__Platform__World__) */
