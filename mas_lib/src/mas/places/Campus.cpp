/**
 *  Campus.h
 *
 *  Version: 1.2.4
 *  Created on: 04/04/2016
 *  Modified on: 17/08/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mas/places/Campus.h"

namespace mas
{
	namespace places
	{

		/**
		 *
		 */
		Campus::Campus(int id, std::string name, geometry_msgs::Polygon boundary, double x, double y, double theta) : Place(id, name, boundary, x, y, theta)
		{
		}

		/**
		 *
		 */
		Campus::Campus(int id, std::string name, geometry_msgs::Polygon boundary, geometry_msgs::Pose pose_msg) : Place(id, name, boundary, pose_msg)
		{
		}

		/**
		 *
		 */
		Campus::Campus(const mas_msgs::Place::ConstPtr& campus_msg) : Place(campus_msg)
		{
		}

		/**
		 *
		 */
		Campus::Campus(mas_msgs::Place campus_msg) : Place(campus_msg)
		{
		}

		/**
		 *
		 */
		Campus::~Campus() 
		{
		}

		/**
		 *
		 */
		int Campus::getType() 
		{
			return CAMPUS;
		}
		
	}
}
