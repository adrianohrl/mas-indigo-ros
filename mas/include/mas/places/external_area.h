/**
 *  This header file defines the ExternalArea class.
 *
 *  Version: 1.4.0
 *  Created on: 02/04/2016
 *  Modified on: 13/12/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _PLACES_EXTERNAL_AREA_H_
#define _PLACES_EXTERNAL_AREA_H_

#include "mas/places/campus.h"

namespace mas
{
namespace places
{
class ExternalArea : public Place
{

public:
  ExternalArea(int id, std::string name, Campus* campus,
               const geometry_msgs::Polygon& boundary, double x, double y,
               double theta = 0.0);
  ExternalArea(int id, std::string name, Campus* campus,
               const geometry_msgs::Polygon& boundary,
               const geometry_msgs::Pose& pose_msg);
  ExternalArea(const mas_msgs::Place::ConstPtr& place_msg);
  ExternalArea(const mas_msgs::Place& place_msg);
  virtual ~ExternalArea();

  Campus* getCampus() const;
  virtual mas_msgs::Place to_msg() const;
  virtual void operator=(const ExternalArea& external_area);

protected:
  virtual int getType() const;

private:
  Campus* campus_;
};
}
}

#endif /* _PLACES_EXTERNAL_AREA_H_ */
