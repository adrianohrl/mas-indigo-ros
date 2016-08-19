/**
 *  Robot.h
 *
 *  Version: 1.2.4
 *  Created on: 05/04/2016
 *  Modified on: 17/08/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef AGENTS_ROBOT_H_
#define AGENTS_ROBOT_H_

#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <mas_msgs/Robot.h>
#include "mas/agents/Computer.h"
#include "mas/tasks/Skill.h"
#include "mas/tasks/Task.h"

#define ROBOT_BEACON_INTERVAL_DURATION 1.0
#define MAXIMUM_ROBOT_BEACON_ABSENCE_DURATION 2 * ROBOT_BEACON_INTERVAL_DURATION

namespace mas 
{
	namespace agents
	{
		class Robot : public Computer
		{

		public:
			Robot();
			Robot(int id, std::string hostname, bool holonomic, bool mobile = true, double x = 0.0, double y = 0.0, double theta = 0.0);
			Robot(int id, std::string hostname, bool holonomic, bool mobile, geometry_msgs::Pose pose_msg);
			Robot(int id, std::string hostname, bool holonomic, bool mobile, mas::places::Location location);
			Robot(const mas_msgs::Agent::ConstPtr& robot_msg);
			Robot(mas_msgs::Agent robot_msg);
			virtual ~Robot();

			std::vector<mas::tasks::Skill> getSkills();
			bool isHolonomic();
			double getVelX();
			double getVelY();
			double getVelTheta();
			geometry_msgs::Twist getVelocity();
			ros::Time getLastBeaconTimestamp();
			void setSkills(std::vector<mas::tasks::Skill> skills);
			void setVelocity(double x = 0.0, double y = 0.0, double theta = 0.0);
			void setVelocity(geometry_msgs::Twist twist_msg);
			void setLastBeaconTimestamp(ros::Time last_beacon_timestamp = ros::Time::now());
			bool isLogged();
			static bool isNotLoggedAnyMore(Robot robot);
			double getUtility(mas::tasks::Task task);
			virtual mas_msgs::Agent toMsg();
			virtual std::string toString();
			virtual void operator=(const Robot& robot);

		protected:
			virtual int getType();
			void addSkill(mas::tasks::Skill skill);
			void removeSkill(mas::tasks::Skill skill);
			void setHolonomic(bool holonomic);

		private:
			std::vector<mas::tasks::Skill> skills_;
			bool holonomic_;
			double vel_x_;
			double vel_y_;
			double vel_theta_;
			ros::Time last_beacon_timestamp_;

			virtual std::string getClassName();
			
		};
	}
}		

#endif /* AGENTS_ROBOT_H_ */
