/**
 *  This header file defines the Resource class.
 *
 *  Version: 1.4.0
 *  Created on: 04/08/2015
 *  Modified on: 13/12/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *           Heverton Machado Soares (sm.heverton@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _TASKS_RESOURCE_H_
#define _TASKS_RESOURCE_H_

#include <string>
#include <mas_msgs/Resource.h>

namespace mas
{
namespace tasks
{
class Resource
{
public:
  Resource(std::string name);
  Resource(int id, std::string name, std::string description);
  Resource(const Resource& resource);
  Resource(const mas_msgs::Resource::ConstPtr& resource_msg);
  Resource(const mas_msgs::Resource& resource_msg);
  ~Resource();

  int getId() const;
  std::string getName() const;
  std::string getDescription() const;
  void setDescription(std::string description);
  virtual mas_msgs::Resource to_msg() const;
  virtual std::string str() const;
  const char* c_str() const;
  void operator=(const Resource& resource);
  bool operator==(const Resource& resource) const;
  bool operator!=(const Resource& resource) const;

private:
  int id_;
  std::string name_;
  std::string description_;
};
}
}

#endif /* _TASKS_RESOURCE_H_ */
