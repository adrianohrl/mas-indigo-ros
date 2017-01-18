/**
 *  This header file defines the Robot class.
 *
 *  Version: 1.4.0
 *  Created on: 05/04/2016
 *  Modified on: 13/12/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _AGENTS_ROBOT_H_
#define _AGENTS_ROBOT_H_

#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <mas_msgs/Robot.h>
#include "mas/agents/computer.h"
#include "mas/tasks/skill.h"
#include "mas/tasks/task.h"

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
  Robot(int id, std::string hostname, bool holonomic, bool mobile = true,
        double x = 0.0, double y = 0.0, double theta = 0.0);
  Robot(int id, std::string hostname, bool holonomic, bool mobile,
        const geometry_msgs::Pose& pose_msg);
  Robot(int id, std::string hostname, bool holonomic, bool mobile,
        const mas::places::Location& location);
  Robot(const Robot& robot);
  Robot(const mas_msgs::Agent::ConstPtr& robot_msg);
  Robot(const mas_msgs::Agent& robot_msg);
  virtual ~Robot();

  std::vector<mas::tasks::Skill*> getSkills() const;
  bool isHolonomic() const;
  double getVelX() const;
  double getVelY() const;
  double getVelTheta() const;
  geometry_msgs::Twist getVelocity() const;
  ros::Time getLastBeaconTimestamp() const;
  void addSkill(tasks::Skill* skill);
  void removeSkill(const tasks::Skill& skill);
  void setSkills(const std::vector<mas::tasks::Skill*>& skills);
  void setVelocity(double x = 0.0, double y = 0.0, double theta = 0.0);
  void setVelocity(const geometry_msgs::Twist& twist_msg);
  void setLastBeaconTimestamp(
      const ros::Time& last_beacon_timestamp = ros::Time::now());
  bool isLogged() const;
  static bool isNotLoggedAnyMore(Robot* robot);
  double getUtility(const mas::tasks::Task& task) const;
  virtual mas_msgs::Agent to_msg() const;
  virtual std::string str() const;
  virtual void operator=(const Robot& robot);

protected:
  virtual int getType() const;

private:
  std::vector<mas::tasks::Skill*> skills_;
  bool holonomic_;
  double vel_x_;
  double vel_y_;
  double vel_theta_;
  ros::Time last_beacon_timestamp_;

  virtual std::string getClassName() const;
};

struct RobotComparator
{
  Robot* robot_;
  RobotComparator(Robot* robot) : robot_(robot) {}
  bool operator()(Robot* robot) { return *robot_ == *robot; }
  bool operator()(const Robot& robot) { return *robot_ == robot; }
};
}
}

#endif /* _AGENTS_ROBOT_H_ */
