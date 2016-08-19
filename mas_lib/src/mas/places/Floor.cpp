/**
 *  Floor.h
 *
 *  Version: 1.2.4
 *  Created on: 04/04/2016
 *  Modified on: 17/08/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mas/places/Floor.h"

namespace mas
{
	namespace places
	{

		/**
		 *
		 */
		Floor::Floor(int id, std::string name, Building building, geometry_msgs::Polygon boundary, double x, double y, double theta) : Place(id, name, boundary, x, y, theta), building_(building)
		{
		}

		/**
		 *
		 */
		Floor::Floor(int id, std::string name, Building building, geometry_msgs::Polygon boundary, geometry_msgs::Pose pose_msg) : Place(id, name, boundary, pose_msg), building_(building)
		{
		}

		/**
		 *
		 */
		Floor::Floor(const mas_msgs::Place::ConstPtr& floor_msg) : Place(floor_msg), building_(mas_msgs::Place()) //floor_(floor_msg->parent) 
		{
			// verificar se tem o building 
		}

		/**
		 *
		 */
		Floor::Floor(mas_msgs::Place floor_msg) : Place(floor_msg), building_(mas_msgs::Place()) //floor_(floor_msg.parent) 
		{
			// verificar se tem o building
		}

		/**
		 *
		 */
		Floor::~Floor() 
		{
		}

		/**
		 *
		 */
		Building Floor::getBuilding() 
		{
			return building_;
		}

		/**
		 *
		 */
		int Floor::getType() 
		{
			return FLOOR;
		}


		/**
		 *
		 */
		mas_msgs::Place Floor::toMsg() 
		{
			mas_msgs::Place building_msg = Place::toMsg();
			//building_msg.building = building_.toMsg();
			return building_msg;
		}

		/**
		 *
		 */
		void Floor::operator=(const Floor& floor)
		{
			Place::operator=(floor);
			building_ = floor.building_;
		}	
		
	}
}
