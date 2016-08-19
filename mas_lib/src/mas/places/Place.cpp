/**
 *  Place.h
 *
 *  Version: 1.2.4
 *  Created on: 03/04/2016
 *  Modified on: 17/08/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mas/places/Place.h"

namespace mas
{
	namespace places
		{

		/**
		 *
		 */
		Place::Place(int id, std::string name, geometry_msgs::Polygon boundary, double x, double y, double theta) : Location(x, y, theta)
		{
			id_ = id;
			name_ = name;
			boundary_ = boundary;
		}

		/**
		 *
		 */
		Place::Place(int id, std::string name, geometry_msgs::Polygon boundary, geometry_msgs::Pose pose_msg) : Location(pose_msg)
		{
			id_ = id;
			name_ = name;
			boundary_ = boundary;
		}

		/**
		 *
		 */
		Place::Place(const mas_msgs::Place::ConstPtr& place_msg) : Location(place_msg->location) 
		{
			if (!isValidType(place_msg->type)) 
			{
				return;
			}
			id_ = place_msg->id;
			name_ = place_msg->name;
			boundary_ = place_msg->boundary;
		}

		/**
		 *
		 */
		Place::Place(mas_msgs::Place place_msg) : Location(place_msg.location)
		{
			if (!isValidType(place_msg.type)) 
			{
				return;
			}
			id_ = place_msg.id;
			name_ = place_msg.name;
			boundary_ = place_msg.boundary;	
		}

		/**
		 *
		 */
		Place::~Place() 
		{
		}

		/**
		 *
		 */
		int Place::getId() 
		{
			return id_;
		}

		/**
		 *
		 */
		std::string Place::getName() 
		{
			return name_;
		}

		/**
		 *
		 */
		geometry_msgs::Polygon Place::getBoundary() 
		{
			return boundary_;
		}

		/**
		 *
		 */
		bool Place::isValidType(int type) 
		{
			return type == getType();
		}

		/**
		 *
		 */
		int Place::getType() 
		{
			return PLACE;
		}

		/**
		 *
		 */
		void Place::setBoundary(geometry_msgs::Polygon boundary) 
		{
			boundary_ = boundary;
		}

		/**
		 *
		 */
		mas_msgs::Place Place::toMsg() 
		{
			mas_msgs::Place place_msg;
			place_msg.type = getType();
			place_msg.id = id_;
			place_msg.name = name_;
			place_msg.location = Location::toMsg();
			place_msg.boundary = boundary_;
			return place_msg;
		}

		/**
		 *
		 */
		bool Place::operator==(const Place& place)
		{
			return id_ == place.id_;
		}

		/**
		 *
		 */
		bool Place::operator!=(const Place& place) 
		{
			return id_ != place.id_;
		}

		/**
		 *
		 */
		void Place::operator=(const Place& place) 
		{
			Location::operator=(place);
			id_ = place.id_;
			name_ = place.name_;
			boundary_ = place.boundary_;
		}

	}
}
