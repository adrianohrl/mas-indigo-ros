/**
 *  This header file defines the Location class.
 *
 *  Version: 1.4.0
 *  Created on: 02/04/2016
 *  Modified on: 13/12/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _PLACES_LOCATION_H_
#define _PLACES_LOCATION_H_

#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include <mas_msgs/Location.h>

namespace mas
{
namespace places
{
class Location
{

public:
  Location(double x = 0.0, double y = 0.0, double theta = 0.0);
  Location(const Location& location);
  Location(const geometry_msgs::Pose& pose_msg);
  Location(const mas_msgs::Location::ConstPtr& location_msg);
  Location(const mas_msgs::Location& location_msg);
  virtual ~Location();

  double getX() const;
  double getY() const;
  double getTheta() const;
  geometry_msgs::Pose getPose() const;
  void setPose(double x, double y, double theta = 0.0);
  void setPose(const geometry_msgs::Pose& pose_msg);
  void setPose(const mas_msgs::Location& location_msg);
  void setPose(const Location& location);
  mas_msgs::Location to_msg() const;
  virtual void operator=(const Location& location);

private:
  double x_;
  double y_;
  double theta_;
};
}
}

#endif /* _PLACES_LOCATION_H_ */
