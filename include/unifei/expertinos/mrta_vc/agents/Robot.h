/**
 *  Robot.h
 *
 *  Version: 1.0.0.0
 *  Created on: 05/04/2016
 *  Modified on: *********
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef ROBOT_H_
#define ROBOT_H_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "unifei/expertinos/mrta_vc/agents/Computer.h"

#define ROBOT_BEACON_INTERVAL_DURATION 3.0
#define MAXIMUM_ROBOT_BEACON_ABSENCE_DURATION 5 * ROBOT_BEACON_INTERVAL_DURATION

namespace unifei 
{
	namespace expertinos
	{
		namespace mrta_vc
		{
			namespace agents
			{
				class Robot : public Computer
				{

				public:
					Robot(int id, std::string hostname, bool holonomic, bool mobile = true, double x = 0, double y = 0, double theta = 0);
					Robot(int id, std::string hostname, bool holonomic, bool mobile, geometry_msgs::Pose pose_msg);
					Robot(int id, std::string hostname, bool holonomic, bool mobile, unifei::expertinos::mrta_vc::places::Location location);
					Robot(const ::mrta_vc::Agent::ConstPtr& robot_msg);
					Robot(::mrta_vc::Agent robot_msg);		
					~Robot();

					bool isHolonomic();
					double getVelX();
					double getVelY();
					double getVelTheta();
					geometry_msgs::Twist getVelocity();
					ros::Time getLastBeaconTimestamp();
					void setVelocity(double x = 0, double y = 0, double theta = 0);
					void setVelocity(geometry_msgs::Twist twist_msg);
					void setLastBeaconTimestamp(ros::Time last_beacon_timestamp = ros::Time::now());
					bool isLogged();
					::mrta_vc::Agent toMsg();
					std::string toString();
					void operator=(const Robot& Robot);
					
				protected:
					Robot();
					
					int getType();
					void setHolonomic(bool holonomic);

				private:
					bool holonomic_;
					double vel_x_;
					double vel_y_;
					double vel_theta_;
					ros::Time last_beacon_timestamp_;

					std::string getClassName();
					
				};
			}
		}
	}
}		
					
#endif /* ROBOT_H_ */
