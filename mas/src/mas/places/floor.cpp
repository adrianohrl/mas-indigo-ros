/**
 *  This source file implements the Floor class.
 *
 *  Version: 1.4.0
 *  Created on: 04/04/2016
 *  Modified on: 13/12/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mas/places/floor.h"

namespace mas
{
namespace places
{

/**
 * @brief Floor::Floor
 * @param id
 * @param name
 * @param building
 * @param boundary
 * @param x
 * @param y
 * @param theta
 */
Floor::Floor(int id, std::string name, Building* building,
             const geometry_msgs::Polygon& boundary, double x, double y, double theta)
    : Place(id, name, boundary, x, y, theta), building_(building)
{
}

/**
 * @brief Floor::Floor
 * @param id
 * @param name
 * @param building
 * @param boundary
 * @param pose_msg
 */
Floor::Floor(int id, std::string name, Building* building,
             const geometry_msgs::Polygon& boundary,
             const geometry_msgs::Pose& pose_msg)
    : Place(id, name, boundary, pose_msg), building_(building)
{
}

/**
 * @brief Floor::Floor
 * @param floor_msg
 */
Floor::Floor(const mas_msgs::Place::ConstPtr& floor_msg)
    : Place(floor_msg), building_(NULL)
{
  // verificar se tem o building
}

/**
 * @brief Floor::Floor
 * @param floor_msg
 */
Floor::Floor(const mas_msgs::Place& floor_msg)
    : Place(floor_msg), building_(NULL)
{
  // verificar se tem o building
}

/**
 * @brief Floor::~Floor
 */
Floor::~Floor()
{
  if (building_)
  {
    delete building_;
    building_ = NULL;
  }
}

/**
 * @brief Floor::getBuilding
 * @return
 */
Building* Floor::getBuilding() const { return building_; }

/**
 * @brief Floor::getType
 * @return
 */
int Floor::getType() const { return FLOOR; }

/**
 * @brief Floor::to_msg
 * @return
 */
mas_msgs::Place Floor::to_msg() const
{
  mas_msgs::Place building_msg = Place::to_msg();
  // building_msg.building = building_.to_msg();
  return building_msg;
}

/**
 * @brief Floor::operator =
 * @param floor
 */
void Floor::operator=(const Floor& floor)
{
  Place::operator=(floor);
  building_ = floor.building_;
}
}
}
