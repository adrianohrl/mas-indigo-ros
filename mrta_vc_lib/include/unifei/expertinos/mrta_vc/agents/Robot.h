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

#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "unifei/expertinos/mrta_vc/agents/Computer.h"
#include "unifei/expertinos/mrta_vc/tasks/Skill.h"
#include "unifei/expertinos/mrta_vc/tasks/Task.h"

#define ROBOT_BEACON_INTERVAL_DURATION 1.0
#define MAXIMUM_ROBOT_BEACON_ABSENCE_DURATION 2 * ROBOT_BEACON_INTERVAL_DURATION

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
					Robot();
					Robot(int id, std::string hostname, bool holonomic, bool mobile = true, double x = 0.0, double y = 0.0, double theta = 0.0);
					Robot(int id, std::string hostname, bool holonomic, bool mobile, geometry_msgs::Pose pose_msg);
					Robot(int id, std::string hostname, bool holonomic, bool mobile, unifei::expertinos::mrta_vc::places::Location location);
					Robot(const ::mrta_vc::Agent::ConstPtr& robot_msg);
					Robot(::mrta_vc::Agent robot_msg);
					virtual ~Robot();

					std::vector<unifei::expertinos::mrta_vc::tasks::Skill> getSkills();
					bool isHolonomic();
					double getVelX();
					double getVelY();
					double getVelTheta();
					geometry_msgs::Twist getVelocity();
					ros::Time getLastBeaconTimestamp();
					void setSkills(std::vector<unifei::expertinos::mrta_vc::tasks::Skill> skills);
					void setVelocity(double x = 0.0, double y = 0.0, double theta = 0.0);
					void setVelocity(geometry_msgs::Twist twist_msg);
					void setLastBeaconTimestamp(ros::Time last_beacon_timestamp = ros::Time::now());
					bool isLogged();
					static bool isNotLoggedAnyMore(Robot robot);
					double getUtility(unifei::expertinos::mrta_vc::tasks::Task task);
					virtual ::mrta_vc::Agent toMsg();
					virtual std::string toString();
					virtual void operator=(const Robot& robot);

				protected:
					virtual int getType();
					void addSkill(unifei::expertinos::mrta_vc::tasks::Skill skill);
					void removeSkill(unifei::expertinos::mrta_vc::tasks::Skill skill);
					void setHolonomic(bool holonomic);

				private:
					std::vector<unifei::expertinos::mrta_vc::tasks::Skill> skills_;
					bool holonomic_;
					double vel_x_;
					double vel_y_;
					double vel_theta_;
					ros::Time last_beacon_timestamp_;

					virtual std::string getClassName();
					
				};
			}
		}
	}
}		

#endif /* ROBOT_H_ */
