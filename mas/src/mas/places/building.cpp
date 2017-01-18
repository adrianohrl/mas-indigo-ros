/**
 *  This source file implements the Building class.
 *
 *  Version: 1.4.0
 *  Created on: 04/04/2016
 *  Modified on: 13/12/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mas/places/building.h"

namespace mas
{
namespace places
{

/**
 * @brief Building::Building
 * @param id
 * @param name
 * @param campus
 * @param boundary
 * @param x
 * @param y
 * @param theta
 */
Building::Building(int id, std::string name, Campus* campus,
                   const geometry_msgs::Polygon& boundary, double x, double y,
                   double theta)
    : Place(id, name, boundary, x, y, theta), campus_(campus)
{
}

/**
 * @brief Building::Building
 * @param id
 * @param name
 * @param campus
 * @param boundary
 * @param pose_msg
 */
Building::Building(int id, std::string name, Campus* campus,
                   const geometry_msgs::Polygon& boundary,
                   const geometry_msgs::Pose& pose_msg)
    : Place(id, name, boundary, pose_msg), campus_(campus)
{
}

/**
 * @brief Building::Building
 * @param building_msg
 */
Building::Building(const mas_msgs::Place::ConstPtr& building_msg)
    : Place(building_msg), campus_(NULL)
{
  // verificar se tem o campus
}

/**
 * @brief Building::Building
 * @param building_msg
 */
Building::Building(const mas_msgs::Place& building_msg)
    : Place(building_msg), campus_(NULL)
{
  // verificar se tem o campus
}

/**
 * @brief Building::~Building
 */
Building::~Building()
{
  if (campus_)
  {
    delete campus_;
    campus_ = NULL;
  }
}

/**
 * @brief Building::getType
 * @return
 */
int Building::getType() const { return BUILDING; }

/**
 * @brief Building::getCampus
 * @return
 */
Campus* Building::getCampus() const { return campus_; }

/**
 * @brief Building::to_msg
 * @return
 */
mas_msgs::Place Building::to_msg() const
{
  mas_msgs::Place building_msg = Place::to_msg();
  // building_msg.campus = campus_->to_msg();
  return building_msg;
}

/**
 * @brief Building::operator =
 * @param building
 */
void Building::operator=(const Building& building)
{
  Place::operator=(building);
  campus_ = building.campus_;
}
}
}
