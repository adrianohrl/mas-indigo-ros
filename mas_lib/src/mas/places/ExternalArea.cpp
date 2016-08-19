/**
 *  ExternalArea.h
 *
 *  Version: 1.2.4
 *  Created on: 04/04/2016
 *  Modified on: 17/08/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mas/places/ExternalArea.h"

namespace mas
{
	namespace places
	{

		/**
		 *
		 */
		ExternalArea::ExternalArea(int id, std::string name, Campus campus, geometry_msgs::Polygon boundary, double x, double y, double theta) : Place(id, name, boundary, x, y, theta), campus_(campus)
		{
		}

		/**
		 *
		 */
		ExternalArea::ExternalArea(int id, std::string name, Campus campus, geometry_msgs::Polygon boundary, geometry_msgs::Pose pose_msg) : Place(id, name, boundary, pose_msg), campus_(campus)
		{
		}

		/**
		 *
		 */
		ExternalArea::ExternalArea(const mas_msgs::Place::ConstPtr& external_area_msg) : Place(external_area_msg), campus_(mas_msgs::Place())//campus_(external_area_msg->parent)
		{
			// verificar se tem o campus 
		}

		/**
		 *
		 */
		ExternalArea::ExternalArea(mas_msgs::Place external_area_msg) : Place(external_area_msg), campus_(mas_msgs::Place())//campus_(external_area_msg.parent)
		{
			// verificar se tem o campus
		}

		/**
		 *
		 */
		ExternalArea::~ExternalArea() 
		{
		}

		/**
		 *
		 */
		Campus ExternalArea::getCampus() 
		{
			return campus_;
		}

		/**
		 *
		 */
		int ExternalArea::getType() 
		{
			return EXTERNAL_AREA;
		}


		/**
		 *
		 */
		mas_msgs::Place ExternalArea::toMsg() 
		{
			mas_msgs::Place external_area_msg = Place::toMsg();
			// external_area_msg.campus = campus_.toMsg();
			return external_area_msg;
		}

		/**
		 *
		 */
		void ExternalArea::operator=(const ExternalArea& external_area)
		{
			Place::operator=(external_area);
			campus_ = external_area.campus_;
		}	
	
	}
}
