/**
 *  This header file defines the Office class.
 *
 *  Version: 1.4.0
 *  Created on: 02/04/2016
 *  Modified on: 13/12/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _PLACES_OFFICE_H_
#define _PLACES_OFFICE_H_

#include "mas/places/floor.h"

namespace mas
{
namespace places
{
class Office : public Place
{

public:
  Office(int id, std::string name, Floor* floor,
         const geometry_msgs::Polygon& boundary, double x, double y,
         double theta = 0.0);
  Office(int id, std::string name, Floor* floor,
         const geometry_msgs::Polygon& boundary,
         const geometry_msgs::Pose& pose_msg);
  Office(const mas_msgs::Place::ConstPtr& place_msg);
  Office(const mas_msgs::Place& place_msg);
  virtual ~Office();

  Floor* getFloor() const;
  virtual mas_msgs::Place to_msg() const;
  virtual void operator=(const Office& office);

protected:
  virtual int getType() const;

private:
  Floor* floor_;
};
}
}

#endif /* _PLACES_OFFICE_H_ */
