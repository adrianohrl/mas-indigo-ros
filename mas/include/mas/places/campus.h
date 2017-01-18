/**
 *  This header file defines the Campus class.
 *
 *  Version: 1.4.0
 *  Created on: 02/04/2016
 *  Modified on: 13/12/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _PLACES_CAMPUS_H_
#define _PLACES_CAMPUS_H_

#include "mas/places/place.h"

namespace mas
{
namespace places
{
class Campus : public Place
{

public:
  Campus(int id, std::string name, const geometry_msgs::Polygon& boundary,
         double x, double y, double theta = 0.0);
  Campus(int id, std::string name, const geometry_msgs::Polygon& boundary,
         const geometry_msgs::Pose& pose_msg);
  Campus(const mas_msgs::Place::ConstPtr& place_msg);
  Campus(const mas_msgs::Place& campus_msg);
  virtual ~Campus();

protected:
  virtual int getType() const;
};
}
}

#endif /* _PLACES_CAMPUS_H_ */
