/**
 *  Building.h
 *
 *  Version: 1.2.4
 *  Created on: 04/04/2016
 *  Modified on: 17/08/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mas/places/Building.h"

namespace mas
{
	namespace places
	{

		/**
		 *
		 */
		Building::Building(int id, std::string name, Campus campus, geometry_msgs::Polygon boundary, double x, double y, double theta) : Place(id, name, boundary, x, y, theta), campus_(campus)
		{
		}

		/**
		 *
		 */
		Building::Building(int id, std::string name, Campus campus, geometry_msgs::Polygon boundary, geometry_msgs::Pose pose_msg) : Place(id, name, boundary, pose_msg), campus_(campus)
		{
		}

		/**
		 *
		 */
		Building::Building(const mas_msgs::Place::ConstPtr& building_msg) : Place(building_msg), campus_(mas_msgs::Place()) //campus_(building_msg->parent) 
		{
			// verificar se tem o campus 
		}

		/**
		 *
		 */
		Building::Building(mas_msgs::Place building_msg) : Place(building_msg), campus_(mas_msgs::Place()) //campus_(building_msg.parent) 
		{
			// verificar se tem o campus
		}

		/**
		 *
		 */
		Building::~Building() 
		{
		}

		/**
		 *
		 */
		int Building::getType() 
		{
			return BUILDING;
		}

		/**
		 *
		 */
		Campus Building::getCampus() 
		{
			return campus_;
		}


		/**
		 *
		 */
		mas_msgs::Place Building::toMsg() 
		{
			mas_msgs::Place building_msg = Place::toMsg();
			//building_msg.campus = campus_.toMsg();
			return building_msg;
		}

		/**
		 *
		 */
		void Building::operator=(const Building& building)
		{
			Place::operator=(building);
			campus_ = building.campus_;
		}
		
	}
}
