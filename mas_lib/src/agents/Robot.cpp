/**
 *  Robot.cpp
 *
 *  Version: 1.2.2
 *  Created on: 04/08/2015
 *  Modified on: 17/08/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "unifei/expertinos/mrta_vc/agents/Robot.h"

/**
 *
 */
unifei::expertinos::mrta_vc::agents::Robot::Robot() : unifei::expertinos::mrta_vc::agents::Computer()
{
	vel_x_ = 0.0;
	vel_y_ = 0.0;
	vel_theta_ = 0.0;
}

/**
 *
 */
unifei::expertinos::mrta_vc::agents::Robot::Robot(int id, std::string hostname, bool holonomic, bool mobile, double x, double y, double theta) : unifei::expertinos::mrta_vc::agents::Computer(id, hostname, mobile, x, y, theta)
{
	holonomic_ = holonomic;
	vel_x_ = 0.0;
	vel_y_ = 0.0;
	vel_theta_ = 0.0;
}

/**
 *
 */
unifei::expertinos::mrta_vc::agents::Robot::Robot(int id, std::string hostname, bool holonomic, bool mobile, geometry_msgs::Pose pose_msg) : unifei::expertinos::mrta_vc::agents::Computer(id, hostname, mobile, pose_msg)
{
	holonomic_ = holonomic;
	vel_x_ = 0.0;
	vel_y_ = 0.0;
	vel_theta_ = 0.0;
}

/**
 *
 */
unifei::expertinos::mrta_vc::agents::Robot::Robot(int id, std::string hostname, bool holonomic, bool mobile, unifei::expertinos::mrta_vc::places::Location location) : unifei::expertinos::mrta_vc::agents::Computer(id, hostname, mobile, location)
{
	holonomic_ = holonomic;
	vel_x_ = 0.0;
	vel_y_ = 0.0;
	vel_theta_ = 0.0;
}

/**
 *
 */
unifei::expertinos::mrta_vc::agents::Robot::Robot(const mas_msgs::Agent::ConstPtr& robot_msg) : unifei::expertinos::mrta_vc::agents::Computer(robot_msg)
{
	holonomic_ = robot_msg->holonomic;
	vel_x_ = 0.0;
	vel_y_ = 0.0;
	vel_theta_ = 0.0;
	for (int i = 0; i < robot_msg->skills.size(); i++)
	{
		skills_.push_back(unifei::expertinos::mrta_vc::tasks::Skill(robot_msg->skills.at(i)));
	}
}

/**
 *
 */
unifei::expertinos::mrta_vc::agents::Robot::Robot(mas_msgs::Agent robot_msg) : unifei::expertinos::mrta_vc::agents::Computer(robot_msg)
{
	holonomic_ = robot_msg.holonomic;
	vel_x_ = 0.0;
	vel_y_ = 0.0;
	vel_theta_ = 0.0;
	for (int i = 0; i < robot_msg.skills.size(); i++)
	{
		skills_.push_back(unifei::expertinos::mrta_vc::tasks::Skill(robot_msg.skills.at(i)));
	}
}

/**
 *
 */
unifei::expertinos::mrta_vc::agents::Robot::~Robot() 
{
}

/**
 *
 */
std::vector<unifei::expertinos::mrta_vc::tasks::Skill> unifei::expertinos::mrta_vc::agents::Robot::getSkills()
{
	return skills_;
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::agents::Robot::isHolonomic()
{
	return holonomic_;
}

/**
 *
 */
double unifei::expertinos::mrta_vc::agents::Robot::getVelX() 
{
	return vel_x_;
}

/**
 *
 */
double unifei::expertinos::mrta_vc::agents::Robot::getVelY() 
{
	return vel_y_;
}

/**
 *
 */
double unifei::expertinos::mrta_vc::agents::Robot::getVelTheta() 
{
	return vel_theta_;
}

/**
 *
 */
geometry_msgs::Twist unifei::expertinos::mrta_vc::agents::Robot::getVelocity() 
{
	geometry_msgs::Twist twist_msg;
	twist_msg.linear.x = vel_x_;
	twist_msg.linear.y = vel_y_;
	twist_msg.angular.z = vel_theta_;
	return twist_msg;
}

/**
 *
 */
ros::Time unifei::expertinos::mrta_vc::agents::Robot::getLastBeaconTimestamp() 
{
	return last_beacon_timestamp_;
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::agents::Robot::isLogged()
{
	return (ros::Time::now() - last_beacon_timestamp_).toSec() <= MAXIMUM_ROBOT_BEACON_ABSENCE_DURATION;
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::agents::Robot::isNotLoggedAnyMore(unifei::expertinos::mrta_vc::agents::Robot robot)
{
	return !robot.isLogged();
}


/**
 *
 */
int unifei::expertinos::mrta_vc::agents::Robot::getType() 
{
	return ROBOT;
}

/**
 *
 */
std::string unifei::expertinos::mrta_vc::agents::Robot::getClassName() 
{
	return "robot";
}

/**
 *
 */
void unifei::expertinos::mrta_vc::agents::Robot::setSkills(std::vector<unifei::expertinos::mrta_vc::tasks::Skill> skills)
{
	skills_ = skills;
}

/**
 *
 */
void unifei::expertinos::mrta_vc::agents::Robot::addSkill(unifei::expertinos::mrta_vc::tasks::Skill skill)
{
	for (int i = 0; i < skills_.size(); i++)
	{
		if (skill.equals(skills_.at(i)))
		{
			skills_.at(i).setLevel(skill.getLevel());
			return;
		}
	}
	skills_.push_back(skill);
}

/**
 *
 */
void unifei::expertinos::mrta_vc::agents::Robot::removeSkill(unifei::expertinos::mrta_vc::tasks::Skill skill)
{
	for (int i = 0; i < skills_.size(); i++)
	{
		if (skill.equals(skills_.at(i)))
		{
			skills_.erase(skills_.begin() + i);
			return;
		}
	}
}

/**
 *
 */
void unifei::expertinos::mrta_vc::agents::Robot::setHolonomic(bool holonomic)
{
	holonomic_ = holonomic;
}

/**
 *
 */
void unifei::expertinos::mrta_vc::agents::Robot::setVelocity(double x, double y, double theta) 
{
	vel_x_ = x;
	vel_y_= y;
	vel_theta_ = theta;
}

/**
 *
 */
void unifei::expertinos::mrta_vc::agents::Robot::setVelocity(geometry_msgs::Twist twist_msg) 
{
	vel_x_ = twist_msg.linear.x;
	vel_y_= twist_msg.linear.y;
	vel_theta_ = twist_msg.angular.z;
}

/**
 *
 */
void unifei::expertinos::mrta_vc::agents::Robot::setLastBeaconTimestamp(ros::Time last_beacon_timestamp) 
{
	if (last_beacon_timestamp > last_beacon_timestamp_ && last_beacon_timestamp <= ros::Time::now())
	{
		last_beacon_timestamp_ = last_beacon_timestamp;
	}
}

/**
 * This function returns this robot utility for the input task.
 */
double unifei::expertinos::mrta_vc::agents::Robot::getUtility(unifei::expertinos::mrta_vc::tasks::Task task)
{
	double utility = 0.0;
	std::vector<unifei::expertinos::mrta_vc::tasks::Skill> desired_skills = task.getDesiredSkills();
	int number_of_desired_skills = desired_skills.size();
	int number_of_skills = getSkills().size();
	if (number_of_skills < number_of_desired_skills)
	{
		return 0.0;
	}
	for (int i = 0; i < number_of_desired_skills; i++)
	{
		unifei::expertinos::mrta_vc::tasks::Skill desired_skill(desired_skills.at(i));
		for (int j = 0; j < number_of_skills; j++)
		{
			unifei::expertinos::mrta_vc::tasks::Skill skill(getSkills().at(j));
			if (skill == desired_skill)
			{
				if (skill.compareTo(desired_skill) < 0.0)
				{
					return 0.0;
				}
				utility += skill.getLevel() / desired_skill.getLevel();
				break;
			}
			if (j == number_of_skills - 1)
			{
				return 0.0;
			}
		}
	}
	return utility * number_of_desired_skills / number_of_skills;
}

/**
 *
 */
mas_msgs::Agent unifei::expertinos::mrta_vc::agents::Robot::toMsg() 
{
	mas_msgs::Agent robot_msg = unifei::expertinos::mrta_vc::agents::Computer::toMsg();
	robot_msg.holonomic = holonomic_;
	robot_msg.velocity.linear.x = vel_x_;
	robot_msg.velocity.linear.y = vel_y_;
	robot_msg.velocity.angular.z = vel_theta_;
	for( int i = 0; i < skills_.size(); i++)
	{
		robot_msg.skills.push_back(skills_.at(i).toMsg());
	}
	return robot_msg;
}

/**
 *
 */
std::string unifei::expertinos::mrta_vc::agents::Robot::toString() 
{
	std::string aux_str = unifei::expertinos::mrta_vc::agents::Computer::toString();
	std::stringstream aux_ss;
	aux_ss << vel_x_ << ", " << vel_y_ << ", " << vel_theta_;
	return aux_str.substr(0, aux_str.length() - 1) + ", velocity: (" + aux_ss.str() + ")}";
}

/**
 *
 */
void unifei::expertinos::mrta_vc::agents::Robot::operator=(const unifei::expertinos::mrta_vc::agents::Robot& robot) 
{
	unifei::expertinos::mrta_vc::agents::Computer::operator=(robot);
	holonomic_ = robot.holonomic_;
	vel_x_ = robot.vel_x_;
	vel_y_ = robot.vel_y_;
	vel_theta_ = robot.vel_theta_;
	skills_.clear();
	for (int i = 0; i < robot.skills_.size(); i++)
	{
		skills_.push_back(robot.skills_.at(i));
	}
}
