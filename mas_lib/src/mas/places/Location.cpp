/**
 *  Location.h
 *
 *  Version: 1.2.4
 *  Created on: 03/04/2016
 *  Modified on: 17/08/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mas/places/Location.h"

namespace mas
{
	namespace places
	{

		/**
		 *
		 */
		Location::Location(double x, double y, double theta)
		{
			x_ = x;
			y_ = y;
			theta_ = theta;
		}

		/**
		 *
		 */
		Location::Location(geometry_msgs::Pose pose_msg)
		{
			x_ = pose_msg.position.x;
			y_ = pose_msg.position.y;
			theta_ = tf::getYaw(pose_msg.orientation);
		}

		/**
		 *
		 */
		Location::Location(const mas_msgs::Location::ConstPtr& location_msg) 
		{
			x_ = location_msg->x;
			y_ = location_msg->y;
			theta_ = location_msg->theta;	
		}

		/**
		 *
		 */
		Location::Location(mas_msgs::Location location_msg) 
		{
			x_ = location_msg.x;
			y_ = location_msg.y;
			theta_ = location_msg.theta;	
		}

		/**
		 *
		 */
		Location::~Location() 
		{
		}

		/**
		 *
		 */
		double Location::getX() 
		{
			return x_;
		}

		/**
		 *
		 */
		double Location::getY() 
		{
			return y_;
		}

		/**
		 *
		 */
		double Location::getTheta() 
		{
			return theta_;
		}

		/**
		 *
		 */
		geometry_msgs::Pose Location::getPose() 
		{
			geometry_msgs::Pose pose_msg;
			pose_msg.position.x = x_;
			pose_msg.position.y = y_;
			pose_msg.orientation = tf::createQuaternionMsgFromYaw(theta_);
			return pose_msg;
		}

		/**
		 *
		 */
		void Location::setPose(double x, double y, double theta) 
		{
			x_ = x;
			y_ = y;
			theta_ = theta;
		}

		/**
		 *
		 */
		void Location::setPose(geometry_msgs::Pose pose_msg) 
		{
			x_ = pose_msg.position.x;
			y_ = pose_msg.position.y;
			theta_ = tf::getYaw(pose_msg.orientation);
		}

		/**
		 *
		 */
		void Location::setPose(mas_msgs::Location location_msg) 
		{
			x_ = location_msg.x;
			y_ = location_msg.y;
			theta_ = location_msg.theta;
		}

		/**
		 *
		 */
		void Location::setPose(Location location) 
		{
			x_ = location.x_;
			y_ = location.y_;
			theta_ = location.theta_;
		}

		/**
		 *
		 */
		mas_msgs::Location Location::toMsg() 
		{
			mas_msgs::Location location_msg;
			location_msg.x = x_;
			location_msg.y = y_;
			location_msg.theta = theta_;
			return location_msg;
		}

		/**
		 *
		 */
		void Location::operator=(const Location& location)
		{
			x_ = location.x_;
			y_ = location.y_;
			theta_ = location.theta_;
		}
	
	}
}	
