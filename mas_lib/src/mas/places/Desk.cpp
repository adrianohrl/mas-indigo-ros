/**
 *  Desk.h
 *
 *  Version: 1.2.4
 *  Created on: 04/04/2016
 *  Modified on: 17/08/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mas/places/Desk.h"

namespace mas
{
	namespace places
	{

		/**
		 *
		 */
		Desk::Desk(int id, std::string name, Office office, geometry_msgs::Polygon boundary, double x, double y, double theta) : Place(id, name, boundary, x, y, theta), office_(office)
		{
		}

		/**
		 *
		 */
		Desk::Desk(int id, std::string name, Office office, geometry_msgs::Polygon boundary, geometry_msgs::Pose pose_msg) : Place(id, name, boundary, pose_msg), office_(office)
		{
		}

		/**
		 *
		 */
		Desk::Desk(const mas_msgs::Place::ConstPtr& desk_msg) : Place(desk_msg), office_(mas_msgs::Place()) //desk_(desk_msg->parent) 
		{
			// verificar se tem o office 
		}

		/**
		 *
		 */
		Desk::Desk(mas_msgs::Place desk_msg) : Place(desk_msg), office_(mas_msgs::Place()) //desk_(desk_msg.parent) 
		{
			// verificar se tem o office
		}

		/**
		 *
		 */
		Desk::~Desk() 
		{
		}

		/**
		 *
		 */
		int Desk::getType() 
		{
			return DESK;
		}

		/**
		 *
		 */
		Office Desk::getOffice() 
		{
			return office_;
		}


		/**
		 *
		 */
		mas_msgs::Place Desk::toMsg() 
		{
			mas_msgs::Place office_msg = Place::toMsg();
			//office_msg.office = office_.toMsg();
			return office_msg;
		}

		/**
		 *
		 */
		void Desk::operator=(const Desk& desk)
		{
			Place::operator=(desk);
			office_ = desk.office_;
		}
		
	}
}
