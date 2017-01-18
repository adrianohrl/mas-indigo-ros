/**
 *  This header file defines the TaskPriorityEnum enumerateds and the
 *TaskPriorities class.
 *
 *  Version: 1.4.0
 *  Created on: 04/04/2016
 *  Modified on: 13/12/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _TASKS_TASK_PRIORITIES_H_
#define _TASKS_TASK_PRIORITIES_H_

#include <string>
#include <vector>

namespace mas
{
namespace tasks
{
namespace priorities
{
enum TaskPriorityEnum
{
  LOW,
  NORMAL,
  IMPORTANT,
  CRITICAL
};
}

typedef priorities::TaskPriorityEnum TaskPriorityEnum;

class TaskPriorities
{
public:
  static TaskPriorityEnum toEnumerated(int code);
  static TaskPriorityEnum toEnumerated(std::string name);
  static bool isValid(std::string name);
  static int toCode(TaskPriorityEnum enumerated);
  static std::string str(TaskPriorityEnum enumerated);
  static const char* c_str(TaskPriorityEnum enumerated);
  static TaskPriorityEnum getDefault();
  static std::vector<TaskPriorityEnum> getAll();
  static int compare(TaskPriorityEnum priority1, TaskPriorityEnum priority2);
};
}
}

#endif /* _TASKS_TASK_PRIORITIES_H_ */
