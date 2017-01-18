/**
 *  This header file defines the Robot Capability Resource class.
 *
 *  Version: 1.4.0
 *  Created on: 16/09/2016
 *  Modified on: 14/12/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *           Heverton Machado Soares (sm.heverton@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _ROBOTS_CAPABILITY_RESOURCE_H_
#define _ROBOTS_CAPABILITY_RESOURCE_H_

#include <string>
#include <mrs_msgs/Resource.h>

namespace mrs
{
namespace robots
{
class Resource
{
public:
  Resource(std::string name);
  Resource(int id, std::string name, std::string description);
  Resource(const Resource& resource);
  Resource(const mrs_msgs::Resource::ConstPtr& resource_msg);
  Resource(const mrs_msgs::Resource& resource_msg);
  ~Resource();

  int getId() const;
  std::string getName() const;
  std::string getDescription() const;
  void setDescription(std::string description);
  virtual mrs_msgs::Resource to_msg() const;
  virtual Resource* clone() const;
  virtual std::string str() const;
  const char* c_str() const;
  virtual void operator=(const Resource& resource);
  virtual bool operator==(const Resource& resource) const;
  bool operator!=(const Resource& resource) const;

private:
  int id_;
  std::string name_;
  std::string description_;
};
}
}

#endif /* _ROBOTS_CAPABILITY_RESOURCE_H_ */
