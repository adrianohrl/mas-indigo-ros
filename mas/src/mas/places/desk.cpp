/**
 *  This source file implements the Desk class.
 *
 *  Version: 1.4.0
 *  Created on: 04/04/2016
 *  Modified on: 13/12/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mas/places/desk.h"

namespace mas
{
namespace places
{

/**
 * @brief Desk::Desk
 * @param id
 * @param name
 * @param office
 * @param boundary
 * @param x
 * @param y
 * @param theta
 */
Desk::Desk(int id, std::string name, Office* office,
           const geometry_msgs::Polygon& boundary, double x, double y,
           double theta)
    : Place(id, name, boundary, x, y, theta), office_(office)
{
}

/**
 * @brief Desk::Desk
 * @param id
 * @param name
 * @param office
 * @param boundary
 * @param pose_msg
 */
Desk::Desk(int id, std::string name, Office* office,
           const geometry_msgs::Polygon& boundary,
           const geometry_msgs::Pose& pose_msg)
    : Place(id, name, boundary, pose_msg), office_(office)
{
}

/**
 * @brief Desk::Desk
 * @param desk_msg
 */
Desk::Desk(const mas_msgs::Place::ConstPtr& desk_msg)
    : Place(desk_msg), office_(NULL)
{
  // verificar se tem o office
}

/**
 * @brief Desk::Desk
 * @param desk_msg
 */
Desk::Desk(const mas_msgs::Place& desk_msg) : Place(desk_msg), office_(NULL)
{
  // verificar se tem o office
}

/**
 * @brief Desk::~Desk
 */
Desk::~Desk()
{
  if (office_)
  {
    delete office_;
    office_ = NULL;
  }
}

/**
 * @brief Desk::getType
 * @return
 */
int Desk::getType() const { return DESK; }

/**
 * @brief Desk::getOffice
 * @return
 */
Office* Desk::getOffice() const { return office_; }

/**
 * @brief Desk::to_msg
 * @return
 */
mas_msgs::Place Desk::to_msg() const
{
  mas_msgs::Place office_msg = Place::to_msg();
  // office_msg.office = office_.to_msg();
  return office_msg;
}

/**
 * @brief Desk::operator =
 * @param desk
 */
void Desk::operator=(const Desk& desk)
{
  Place::operator=(desk);
  office_ = desk.office_;
}
}
}
