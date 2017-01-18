/**
 *  This header file defines the Allocation class.
 *
 *  Version: 1.4.0
 *  Created on: 04/08/2015
 *  Modified on: 13/12/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *           Heverton Machado Soares (sm.heverton@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _TASKS_ALLOCATION_H_
#define _TASKS_ALLOCATION_H_

#include <mas_msgs/Allocation.h>
#include "mas/agents/robot.h"
#include "mas/tasks/task.h"
#include "mas/tasks/allocation_satisfactions.h"
#include "mas/tasks/allocation_states.h"

namespace mas
{
namespace tasks
{
class Allocation
{
public:
  Allocation();
  Allocation(Task* task, const std::vector<agents::Robot*>& robots,
             AllocationSatisfactionEnum satisfaction,
             const ros::Time& allocation_timestamp,
             const ros::Time& dispatch_timestamp,
             const ros::Time& start_timestamp, const ros::Time& end_timestamp);
  Allocation(Task* task, const std::vector<agents::Robot*>& robots =
                             std::vector<agents::Robot*>());
  Allocation(const mas_msgs::Allocation::ConstPtr& allocation_msg);
  Allocation(const mas_msgs::Allocation& allocation_msg);
  ~Allocation();

  Task* getTask() const;
  std::vector<agents::Robot*> getRobots() const;
  AllocationStateEnum getState() const;
  AllocationSatisfactionEnum getSatisfaction() const;
  ros::Time getAllocationTimestamp() const;
  ros::Time getDispatchTimestamp() const;
  ros::Time getStartTimestamp() const;
  ros::Time getEndTimestamp() const;
  bool wasAllocated() const;
  bool wasDispatched() const;
  bool wasAccepted() const;
  bool isExecuting() const;
  bool isFinished() const;
  bool wasSucceeded() const;
  bool wasAborted() const;
  bool wasCancelled() const;
  bool wasEvaluated() const;
  void setSatisfaction(AllocationSatisfactionEnum satisfaction);
  void allocate(const std::vector<agents::Robot*>& robots);
  void dispatch();
  void start();
  void end();
  void abort();
  void cancel();
  bool finish(AllocationStateEnum state);
  bool isInvolved(const agents::Robot& robot) const;
  bool isInvolved(const agents::Person& person) const;
  virtual mas_msgs::Allocation to_msg() const;
  virtual std::string str() const;
  const char* c_str() const;
  void operator=(const Allocation& allocation);
  bool operator==(const Allocation& allocation) const;
  bool operator!=(const Allocation& allocation) const;
  int compareTo(const Allocation& allocation) const;

private:
  Task* task_;
  std::vector<agents::Robot*> robots_;
  AllocationStateEnum state_;
  AllocationSatisfactionEnum satisfaction_;
  ros::Time allocation_timestamp_;
  ros::Time dispatch_timestamp_;
  ros::Time start_timestamp_;
  ros::Time end_timestamp_;

  void addRobots(const std::vector<agents::Robot*>& robots);
  void addRobot(agents::Robot* robot);
  void removeRobot(const agents::Robot& robot);
  bool isValid(AllocationStateEnum state);
  void setState(AllocationStateEnum state);
  bool hasStateChanged(AllocationStateEnum state) const;
  void setAllocationTimestamp(
      const ros::Time& allocation_timestamp = ros::Time::now());
  void
  setDispatchTimestamp(const ros::Time& dispatch_timestamp = ros::Time::now());
  void setStartTimestamp(const ros::Time& start_timestamp = ros::Time::now());
  void setEndTimestamp(const ros::Time& end_timestamp = ros::Time::now());
};
}
}

#endif /* _TASKS_ALLOCATION_H_ */
