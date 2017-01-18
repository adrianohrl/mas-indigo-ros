/**
 *  This header file defines the Robot Capabilty class.
 *
 *  Version: 1.4.0
 *  Created on: 16/09/2016
 *  Modified on: 14/12/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *           Heverton Machado Soares (sm.heverton@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _ROBOTS_CAPABILITY_H_
#define _ROBOTS_CAPABILITY_H_

#include <mrs_msgs/Capability.h>
#include "mrs/robots/resource.h"
#include "mrs/robots/levels/level_converter.h"

namespace mrs
{
namespace robots
{
class Capability
{
public:
  Capability(int id, const Resource& resource, Level lower_level,
             Level upper_level);
  Capability(const Capability& capability);
  Capability(const mrs_msgs::Capability::ConstPtr& capability_msg);
  Capability(const mrs_msgs::Capability& capability_msg);
  ~Capability();

  int getId() const;
  Level getLowerLevel() const;
  Level getUpperLevel() const;
  Resource* getResource() const;
  bool isSufficient(Level level) const;
  bool isSufficient(const Capability& capability) const;
  void setId(int id);
  void setLowerLevel(Level lower_level);
  void setUpperLevel(Level upper_level);
  virtual mrs_msgs::Capability to_msg() const;
  virtual Capability* clone() const;
  virtual std::string str() const;
  const char* c_str() const;
  virtual void operator=(const Capability& capability);
  virtual bool operator==(const Capability& capability) const;
  bool operator!=(const Capability& capability) const;

private:
  int id_;
  Resource* resource_;
  Level lower_level_, upper_level_;
};
}
}

#endif /* ROBOTS_CAPABILITY_H_ */
