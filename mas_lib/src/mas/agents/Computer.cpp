/**
 *  Computer.cpp
 *
 *  Version: 1.2.4
 *  Created on: 04/08/2015
 *  Modified on: 17/08/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mas/agents/Computer.h"

namespace mas
{
	namespace agents
	{

		/**
		 *
		 */
		Computer::Computer() : Agent()
		{
		}

		/**
		 *
		 */
		Computer::Computer(int id, std::string hostname, bool mobile, double x, double y, double theta) : Agent(id, x, y, theta)
		{
			hostname_ = hostname;
			mobile_ = mobile;
		}

		/**
		 *
		 */
		Computer::Computer(int id, std::string hostname, bool mobile, geometry_msgs::Pose pose_msg) : Agent(id, pose_msg)
		{
			hostname_ = hostname;
			mobile_ = mobile;	
		}

		/**
		 *
		 */
		Computer::Computer(int id, std::string hostname, bool mobile, places::Location location) : Agent(id, location)
		{
			hostname_ = hostname;
			mobile_ = mobile;	
		}

		/**
		 *
		 */
		Computer::Computer(const mas_msgs::Agent::ConstPtr& computer_msg) : Agent(computer_msg)
		{
			hostname_ = computer_msg->name;
			mobile_ = computer_msg->mobile;
		}

		/**
		 *
		 */
		Computer::Computer(mas_msgs::Agent computer_msg) : Agent(computer_msg)
		{
			hostname_ = computer_msg.name;
			mobile_ = computer_msg.mobile;		
		}

		/**
		 *
		 */
		Computer::~Computer() 
		{
		}

		/**
		 *
		 */
		std::string Computer::getHostname() 
		{
			return hostname_;
		}

		/**
		 *
		 */
		bool Computer::isMobile() 
		{
			return mobile_;
		}

		/**
		 *
		 */
		int Computer::getType() 
		{
			return COMPUTER;
		}

		/**
		 *
		 */
		std::string Computer::getClassName() 
		{
		  return "computer";
		}

		/**
		 *
		 */
		void Computer::setHostname(std::string hostname) 
		{
			hostname_ = hostname;
		}

		/**
		 *
		 */
		void Computer::setMobile(bool mobile) 
		{
			mobile_ = mobile;
		}

		/**
		 *
		 */
		mas_msgs::Agent Computer::toMsg() 
		{
			mas_msgs::Agent computer_msg = Agent::toMsg();
			computer_msg.name = hostname_;
			computer_msg.mobile = mobile_;
			return computer_msg;
		}

		/**
		 *
		 */
		std::string Computer::toString() 
		{
			return Agent::toString() +
					", hostname: " + hostname_ +
					"}";
		}

		/**
		 *
		 */
		bool Computer::operator==(const Computer& computer)
		{
			return hostname_ == computer.hostname_;
		}

		/**
		 *
		 */
		void Computer::operator=(const Computer& computer) 
		{
			Agent::operator=(computer);
			hostname_ = computer.hostname_;
			mobile_ = computer.mobile_;
		}
		
	}
}
