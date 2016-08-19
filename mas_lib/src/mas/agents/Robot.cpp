/**
 *  Robot.cpp
 *
 *  Version: 1.2.4
 *  Created on: 04/08/2015
 *  Modified on: 17/08/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mas/agents/Robot.h"

namespace mas
{
	namespace agents
	{

		/**
		 *
		 */
		Robot::Robot() : Computer()
		{
			vel_x_ = 0.0;
			vel_y_ = 0.0;
			vel_theta_ = 0.0;
		}

		/**
		 *
		 */
		Robot::Robot(int id, std::string hostname, bool holonomic, bool mobile, double x, double y, double theta) : Computer(id, hostname, mobile, x, y, theta)
		{
			holonomic_ = holonomic;
			vel_x_ = 0.0;
			vel_y_ = 0.0;
			vel_theta_ = 0.0;
		}

		/**
		 *
		 */
		Robot::Robot(int id, std::string hostname, bool holonomic, bool mobile, geometry_msgs::Pose pose_msg) : Computer(id, hostname, mobile, pose_msg)
		{
			holonomic_ = holonomic;
			vel_x_ = 0.0;
			vel_y_ = 0.0;
			vel_theta_ = 0.0;
		}

		/**
		 *
		 */
		Robot::Robot(int id, std::string hostname, bool holonomic, bool mobile, places::Location location) : Computer(id, hostname, mobile, location)
		{
			holonomic_ = holonomic;
			vel_x_ = 0.0;
			vel_y_ = 0.0;
			vel_theta_ = 0.0;
		}

		/**
		 *
		 */
		Robot::Robot(const mas_msgs::Agent::ConstPtr& robot_msg) : Computer(robot_msg)
		{
			holonomic_ = robot_msg->holonomic;
			vel_x_ = 0.0;
			vel_y_ = 0.0;
			vel_theta_ = 0.0;
			for (int i = 0; i < robot_msg->skills.size(); i++)
			{
				skills_.push_back(tasks::Skill(robot_msg->skills.at(i)));
			}
		}

		/**
		 *
		 */
		Robot::Robot(mas_msgs::Agent robot_msg) : Computer(robot_msg)
		{
			holonomic_ = robot_msg.holonomic;
			vel_x_ = 0.0;
			vel_y_ = 0.0;
			vel_theta_ = 0.0;
			for (int i = 0; i < robot_msg.skills.size(); i++)
			{
				skills_.push_back(tasks::Skill(robot_msg.skills.at(i)));
			}
		}

		/**
		 *
		 */
		Robot::~Robot() 
		{
		}

		/**
		 *
		 */
		std::vector<tasks::Skill> Robot::getSkills()
		{
			return skills_;
		}

		/**
		 *
		 */
		bool Robot::isHolonomic()
		{
			return holonomic_;
		}

		/**
		 *
		 */
		double Robot::getVelX() 
		{
			return vel_x_;
		}

		/**
		 *
		 */
		double Robot::getVelY() 
		{
			return vel_y_;
		}

		/**
		 *
		 */
		double Robot::getVelTheta() 
		{
			return vel_theta_;
		}

		/**
		 *
		 */
		geometry_msgs::Twist Robot::getVelocity() 
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
		ros::Time Robot::getLastBeaconTimestamp() 
		{
			return last_beacon_timestamp_;
		}

		/**
		 *
		 */
		bool Robot::isLogged()
		{
			return (ros::Time::now() - last_beacon_timestamp_).toSec() <= MAXIMUM_ROBOT_BEACON_ABSENCE_DURATION;
		}

		/**
		 *
		 */
		bool Robot::isNotLoggedAnyMore(Robot robot)
		{
			return !robot.isLogged();
		}


		/**
		 *
		 */
		int Robot::getType() 
		{
			return ROBOT;
		}

		/**
		 *
		 */
		std::string Robot::getClassName() 
		{
			return "robot";
		}

		/**
		 *
		 */
		void Robot::setSkills(std::vector<tasks::Skill> skills)
		{
			skills_ = skills;
		}

		/**
		 *
		 */
		void Robot::addSkill(tasks::Skill skill)
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
		void Robot::removeSkill(tasks::Skill skill)
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
		void Robot::setHolonomic(bool holonomic)
		{
			holonomic_ = holonomic;
		}

		/**
		 *
		 */
		void Robot::setVelocity(double x, double y, double theta) 
		{
			vel_x_ = x;
			vel_y_= y;
			vel_theta_ = theta;
		}

		/**
		 *
		 */
		void Robot::setVelocity(geometry_msgs::Twist twist_msg) 
		{
			vel_x_ = twist_msg.linear.x;
			vel_y_= twist_msg.linear.y;
			vel_theta_ = twist_msg.angular.z;
		}

		/**
		 *
		 */
		void Robot::setLastBeaconTimestamp(ros::Time last_beacon_timestamp) 
		{
			if (last_beacon_timestamp > last_beacon_timestamp_ && last_beacon_timestamp <= ros::Time::now())
			{
				last_beacon_timestamp_ = last_beacon_timestamp;
			}
		}

		/**
		 * This function returns this robot utility for the input task.
		 */
		double Robot::getUtility(tasks::Task task)
		{
			double utility = 0.0;
			std::vector<tasks::Skill> desired_skills = task.getDesiredSkills();
			int number_of_desired_skills = desired_skills.size();
			int number_of_skills = getSkills().size();
			if (number_of_skills < number_of_desired_skills)
			{
				return 0.0;
			}
			for (int i = 0; i < number_of_desired_skills; i++)
			{
				tasks::Skill desired_skill(desired_skills.at(i));
				for (int j = 0; j < number_of_skills; j++)
				{
					tasks::Skill skill(getSkills().at(j));
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
		mas_msgs::Agent Robot::toMsg() 
		{
			mas_msgs::Agent robot_msg = Computer::toMsg();
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
		std::string Robot::toString() 
		{
			std::string aux_str = Computer::toString();
			std::stringstream aux_ss;
			aux_ss << vel_x_ << ", " << vel_y_ << ", " << vel_theta_;
			return aux_str.substr(0, aux_str.length() - 1) + ", velocity: (" + aux_ss.str() + ")}";
		}

		/**
		 *
		 */
		void Robot::operator=(const Robot& robot) 
		{
			Computer::operator=(robot);
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
		
	}
}
