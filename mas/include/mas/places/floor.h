/**
 *  This header file defines the Floor class.
 *
 *  Version: 1.4.0
 *  Created on: 02/04/2016
 *  Modified on: 13/12/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _PLACES_FLOOR_H_
#define _PLACES_FLOOR_H_

#include "mas/places/building.h"

namespace mas
{
namespace places
{
class Floor : public Place
{

public:
  Floor(int id, std::string name, Building* building,
        const geometry_msgs::Polygon& boundary, double x, double y,
        double theta = 0.0);
  Floor(int id, std::string name, Building* building,
        const geometry_msgs::Polygon& boundary,
        const geometry_msgs::Pose& pose_msg);
  Floor(const mas_msgs::Place::ConstPtr& place_msg);
  Floor(const mas_msgs::Place& place_msg);
  virtual ~Floor();

  Building* getBuilding() const;
  virtual mas_msgs::Place to_msg() const;
  virtual void operator=(const Floor& floor);

protected:
  virtual int getType() const;

private:
  Building* building_;
};
}
}

#endif /* _PLACES_FLOOR_H_ */
