/**
 *  This source file implements the Robot class.
 *
 *  Version: 1.4.0
 *  Created on: 04/08/2015
 *  Modified on: 13/12/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mas/agents/robot.h"

namespace mas
{
namespace agents
{

/**
   * @brief Robot::Robot
   */
Robot::Robot()
    : Computer(), vel_x_(0.0), vel_y_(0.0), vel_theta_(0.0),
      last_beacon_timestamp_(ros::Time::now())
{
}

/**
 * @brief Robot::Robot
 * @param id
 * @param hostname
 * @param holonomic
 * @param mobile
 * @param x
 * @param y
 * @param theta
 */
Robot::Robot(int id, std::string hostname, bool holonomic, bool mobile,
             double x, double y, double theta)
    : Computer(id, hostname, mobile, x, y, theta), holonomic_(holonomic),
      vel_x_(0.0), vel_y_(0.0), vel_theta_(0.0),
      last_beacon_timestamp_(ros::Time::now())
{
}

/**
 * @brief Robot::Robot
 * @param id
 * @param hostname
 * @param holonomic
 * @param mobile
 * @param pose_msg
 */
Robot::Robot(int id, std::string hostname, bool holonomic, bool mobile,
             const geometry_msgs::Pose& pose_msg)
    : Computer(id, hostname, mobile, pose_msg), holonomic_(holonomic),
      vel_x_(0.0), vel_y_(0.0), vel_theta_(0.0),
      last_beacon_timestamp_(ros::Time::now())
{
}

/**
 * @brief Robot::Robot
 * @param id
 * @param hostname
 * @param holonomic
 * @param mobile
 * @param location
 */
Robot::Robot(int id, std::string hostname, bool holonomic, bool mobile,
             const places::Location& location)
    : Computer(id, hostname, mobile, location), holonomic_(holonomic),
      vel_x_(0.0), vel_y_(0.0), vel_theta_(0.0),
      last_beacon_timestamp_(ros::Time::now())
{
}

/**
 * @brief Robot::Robot
 * @param robot
 */
Robot::Robot(const Robot& robot)
    : Computer(robot), holonomic_(robot.holonomic_), vel_x_(robot.vel_x_),
      vel_y_(robot.vel_y_), vel_theta_(robot.vel_theta_),
      last_beacon_timestamp_(robot.last_beacon_timestamp_)
{
  std::vector<tasks::Skill*> skills(robot.getSkills());
  for (int i(0); i < skills.size(); i++)
  {
    skills_.push_back(new tasks::Skill(*skills[i]));
  }
}

/**
 * @brief Robot::Robot
 * @param robot_msg
 */
Robot::Robot(const mas_msgs::Agent::ConstPtr& robot_msg)
    : Computer(robot_msg), holonomic_(robot_msg->holonomic), vel_x_(0.0),
      vel_y_(0.0), vel_theta_(0.0), last_beacon_timestamp_(ros::Time::now())
{
  for (int i(0); i < robot_msg->skills.size(); i++)
  {
    skills_.push_back(new tasks::Skill(robot_msg->skills[i]));
  }
}

/**
 * @brief Robot::Robot
 * @param robot_msg
 */
Robot::Robot(const mas_msgs::Agent& robot_msg)
    : Computer(robot_msg), holonomic_(robot_msg.holonomic), vel_x_(0.0),
      vel_y_(0.0), vel_theta_(0.0), last_beacon_timestamp_(ros::Time::now())
{
  for (int i(0); i < robot_msg.skills.size(); i++)
  {
    skills_.push_back(new tasks::Skill(robot_msg.skills[i]));
  }
}

/**
 * @brief Robot::~Robot
 */
Robot::~Robot()
{
  for (int i(0); i < skills_.size(); i++)
  {
    if (skills_[i])
    {
      delete skills_[i];
      skills_[i] = NULL;
    }
  }
}

/**
 * @brief Robot::getSkills
 * @return
 */
std::vector<tasks::Skill*> Robot::getSkills() const { return skills_; }

/**
 * @brief Robot::isHolonomic
 * @return
 */
bool Robot::isHolonomic() const { return holonomic_; }

/**
 * @brief Robot::getVelX
 * @return
 */
double Robot::getVelX() const { return vel_x_; }

/**
 * @brief Robot::getVelY
 * @return
 */
double Robot::getVelY() const { return vel_y_; }

/**
 * @brief Robot::getVelTheta
 * @return
 */
double Robot::getVelTheta() const { return vel_theta_; }

/**
 * @brief Robot::getVelocity
 * @return
 */
geometry_msgs::Twist Robot::getVelocity() const
{
  geometry_msgs::Twist twist_msg;
  twist_msg.linear.x = vel_x_;
  twist_msg.linear.y = vel_y_;
  twist_msg.angular.z = vel_theta_;
  return twist_msg;
}

/**
 * @brief Robot::getLastBeaconTimestamp
 * @return
 */
ros::Time Robot::getLastBeaconTimestamp() const
{
  return last_beacon_timestamp_;
}

/**
 * @brief Robot::isLogged
 * @return
 */
bool Robot::isLogged() const
{
  return (ros::Time::now() - last_beacon_timestamp_).toSec() <=
         MAXIMUM_ROBOT_BEACON_ABSENCE_DURATION;
}

/**
 * @brief Robot::isNotLoggedAnyMore
 * @param robot
 * @return
 */
bool Robot::isNotLoggedAnyMore(Robot* robot) { return !robot->isLogged(); }

/**
 * @brief Robot::getType
 * @return
 */
int Robot::getType() const { return ROBOT; }

/**
 * @brief Robot::getClassName
 * @return
 */
std::string Robot::getClassName() const { return "robot"; }

/**
 * @brief Robot::setSkills
 * @param skills
 */
void Robot::setSkills(const std::vector<tasks::Skill*>& skills)
{
  for (int i(0); i < skills_.size(); i++)
  {
    if (skills_[i])
    {
      delete skills_[i];
      skills_[i] = NULL;
    }
  }
  skills_ = skills;
}

/**
 * @brief Robot::addSkill
 * @param skill
 */
void Robot::addSkill(tasks::Skill* skill)
{
  for (int i(0); i < skills_.size(); i++)
  {
    if (*skill == *skills_[i])
    {
      skills_[i]->setLevel(skill->getLevel());
      return;
    }
  }
  skills_.push_back(skill);
}

/**
 * @brief Robot::removeSkill
 * @param skill
 */
void Robot::removeSkill(const tasks::Skill& skill)
{
  for (int i(0); i < skills_.size(); i++)
  {
    if (skill == *skills_[i])
    {
      if (skills_[i])
      {
        delete skills_[i];
        skills_[i] = NULL;
      }
      skills_.erase(skills_.begin() + i);
      return;
    }
  }
}

/**
 * @brief Robot::setVelocity
 * @param x
 * @param y
 * @param theta
 */
void Robot::setVelocity(double x, double y, double theta)
{
  vel_x_ = x;
  vel_y_ = y;
  vel_theta_ = theta;
}

/**
 * @brief Robot::setVelocity
 * @param twist_msg
 */
void Robot::setVelocity(const geometry_msgs::Twist& twist_msg)
{
  vel_x_ = twist_msg.linear.x;
  vel_y_ = twist_msg.linear.y;
  vel_theta_ = twist_msg.angular.z;
}

/**
 * @brief Robot::setLastBeaconTimestamp
 * @param last_beacon_timestamp
 */
void Robot::setLastBeaconTimestamp(const ros::Time& last_beacon_timestamp)
{
  if (last_beacon_timestamp > last_beacon_timestamp_ &&
      last_beacon_timestamp <= ros::Time::now())
  {
    last_beacon_timestamp_ = last_beacon_timestamp;
  }
}

/**
 * @brief Robot::getUtility
 * @param task
 * @return
 */
double Robot::getUtility(const tasks::Task& task) const
{
  double utility(0.0);
  std::vector<tasks::Skill*> desired_skills(task.getSkills());
  int number_of_desired_skills(desired_skills.size());
  int number_of_skills(skills_.size());
  if (number_of_skills < number_of_desired_skills)
  {
    return 0.0;
  }
  for (int i(0); i < number_of_desired_skills; i++)
  {
    tasks::Skill* desired_skill = desired_skills[i];
    for (int j(0); j < number_of_skills; j++)
    {
      tasks::Skill* skill = skills_[j];
      if (*skill == *desired_skill)
      {
        if (skill->compareTo(*desired_skill) < 0.0)
        {
          return 0.0;
        }
        utility += skill->getLevel() / desired_skill->getLevel();
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
 * @brief Robot::to_msg
 * @return
 */
mas_msgs::Agent Robot::to_msg() const
{
  mas_msgs::Agent robot_msg(Computer::to_msg());
  robot_msg.holonomic = holonomic_;
  robot_msg.velocity.linear.x = vel_x_;
  robot_msg.velocity.linear.y = vel_y_;
  robot_msg.velocity.angular.z = vel_theta_;
  for (int i(0); i < skills_.size(); i++)
  {
    if (skills_[i])
    {
      robot_msg.skills.push_back(skills_[i]->to_msg());
    }
  }
  return robot_msg;
}

/**
 * @brief Robot::str
 * @return
 */
std::string Robot::str() const
{
  std::stringstream skills_ss;
  if (!skills_.empty())
  {
    tasks::Skill* skill = skills_[0];
    if (skill)
    {
      skills_ss << "0 " << skill->str();
    }
    for (int i(1); i < skills_.size(); i++)
    {
      skill = skills_[i];
      if (skill)
      {
        skills_ss << ", " << i << " " << skill->str();
      }
    }
  }
  std::string aux_str(Computer::str());
  std::stringstream aux_ss;
  aux_ss << vel_x_ << ", " << vel_y_ << ", " << vel_theta_;
  return aux_str.substr(0, aux_str.length() - 1) + ", velocity: (" +
         aux_ss.str() + ")" +
         (!skills_ss.str().empty() ? ", skills: {" + skills_ss.str() + "}"
                                   : "") +
         "}";
}

/**
 * @brief Robot::operator =
 * @param robot
 */
void Robot::operator=(const Robot& robot)
{
  Computer::operator=(robot);
  holonomic_ = robot.holonomic_;
  vel_x_ = robot.vel_x_;
  vel_y_ = robot.vel_y_;
  vel_theta_ = robot.vel_theta_;
  skills_.clear();
  for (int i(0); i < robot.skills_.size(); i++)
  {
    if (skills_[i])
    {
      skills_.push_back(robot.skills_[i]);
    }
  }
}
}
}
