/**
 *  This header file defines the Desk class.
 *
 *  Version: 1.4.0
 *  Created on: 02/04/2016
 *  Modified on: 13/12/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _PLACES_DESK_H_
#define _PLACES_DESK_H_

#include "mas/places/office.h"

namespace mas
{
namespace places
{
class Desk : public Place
{

public:
  Desk(int id, std::string name, Office* office,
       const geometry_msgs::Polygon& boundary, double x, double y,
       double theta = 0.0);
  Desk(int id, std::string name, Office* office,
       const geometry_msgs::Polygon& boundary,
       const geometry_msgs::Pose& pose_msg);
  Desk(const mas_msgs::Place::ConstPtr& place_msg);
  Desk(const mas_msgs::Place& place_msg);
  virtual ~Desk();

  Office* getOffice() const;
  virtual mas_msgs::Place to_msg() const;
  virtual void operator=(const Desk& desk);

protected:
  virtual int getType() const;

private:
  Office* office_;
};
}
}

#endif /* _PLACES_DESK_H_ */
