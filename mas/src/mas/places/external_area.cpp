/**
 *  This source file implements the ExternalArea class.
 *
 *  Version: 1.4.0
 *  Created on: 04/04/2016
 *  Modified on: 13/12/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mas/places/external_area.h"

namespace mas
{
namespace places
{

/**
 * @brief ExternalArea::ExternalArea
 * @param id
 * @param name
 * @param campus
 * @param boundary
 * @param x
 * @param y
 * @param theta
 */
ExternalArea::ExternalArea(int id, std::string name, Campus* campus,
                           const geometry_msgs::Polygon& boundary, double x,
                           double y, double theta)
    : Place(id, name, boundary, x, y, theta), campus_(campus)
{
}

/**
 * @brief ExternalArea::ExternalArea
 * @param id
 * @param name
 * @param campus
 * @param boundary
 * @param pose_msg
 */
ExternalArea::ExternalArea(int id, std::string name, Campus* campus,
                           const geometry_msgs::Polygon& boundary,
                           const geometry_msgs::Pose& pose_msg)
    : Place(id, name, boundary, pose_msg), campus_(campus)
{
}

/**
 * @brief ExternalArea::ExternalArea
 * @param external_area_msg
 */
ExternalArea::ExternalArea(const mas_msgs::Place::ConstPtr& external_area_msg)
    : Place(external_area_msg), campus_(NULL)
{
  // verificar se tem o campus
}

/**
 * @brief ExternalArea::ExternalArea
 * @param external_area_msg
 */
ExternalArea::ExternalArea(const mas_msgs::Place& external_area_msg)
    : Place(external_area_msg), campus_(NULL)
{
  // verificar se tem o campus
}

/**
 * @brief ExternalArea::~ExternalArea
 */
ExternalArea::~ExternalArea()
{
  if (campus_)
  {
    delete campus_;
    campus_ = NULL;
  }
}

/**
 * @brief ExternalArea::getCampus
 * @return
 */
Campus* ExternalArea::getCampus() const { return campus_; }

/**
 * @brief ExternalArea::getType
 * @return
 */
int ExternalArea::getType() const { return EXTERNAL_AREA; }

/**
 * @brief ExternalArea::to_msg
 * @return
 */
mas_msgs::Place ExternalArea::to_msg() const
{
  mas_msgs::Place external_area_msg = Place::to_msg();
  // external_area_msg.campus = campus_.to_msg();
  return external_area_msg;
}

/**
 * @brief ExternalArea::operator =
 * @param external_area
 */
void ExternalArea::operator=(const ExternalArea& external_area)
{
  Place::operator=(external_area);
  campus_ = external_area.campus_;
}
}
}
