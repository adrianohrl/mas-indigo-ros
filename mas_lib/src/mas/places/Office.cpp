/**
 *  Office.h
 *
 *  Version: 1.2.4
 *  Created on: 04/04/2016
 *  Modified on: 17/08/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mas/places/Office.h"

namespace mas
{
	namespace places
	{

		/**
		 *
		 */
		Office::Office(int id, std::string name, Floor floor, geometry_msgs::Polygon boundary, double x, double y, double theta) : Place(id, name, boundary, x, y, theta), floor_(floor)
		{
		}

		/**
		 *
		 */
		Office::Office(int id, std::string name, Floor floor, geometry_msgs::Polygon boundary, geometry_msgs::Pose pose_msg) : Place(id, name, boundary, pose_msg), floor_(floor)
		{
		}

		/**
		 *
		 */
		Office::Office(const mas_msgs::Place::ConstPtr& office_msg) : Place(office_msg), floor_(mas_msgs::Place()) //office_(office_msg->parent) 
		{
			// verificar se tem o floor 
		}

		/**
		 *
		 */
		Office::Office(mas_msgs::Place office_msg) : Place(office_msg), floor_(mas_msgs::Place()) //office_(office_msg.parent) 
		{
			// verificar se tem o floor
		}

		/**
		 *
		 */
		Office::~Office() 
		{
		}

		/**
		 *
		 */
		Floor Office::getFloor() 
		{
			return floor_;
		}

		/**
		 *
		 */
		int Office::getType() 
		{
			return OFFICE;
		}


		/**
		 *
		 */
		mas_msgs::Place Office::toMsg() 
		{
			mas_msgs::Place floor_msg = Place::toMsg();
			//floor_msg.floor = floor_.toMsg();
			return floor_msg;
		}

		/**
		 *
		 */
		void Office::operator=(const Office& office)
		{
			Place::operator=(office);
			floor_ = office.floor_;
		}
		
	}
}
