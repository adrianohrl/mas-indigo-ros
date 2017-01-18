/**
 *  This source file implements the Campus class.
 *
 *  Version: 1.4.0
 *  Created on: 04/04/2016
 *  Modified on: 13/12/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mas/places/campus.h"

namespace mas
{
namespace places
{

/**
 * @brief Campus::Campus
 * @param id
 * @param name
 * @param boundary
 * @param x
 * @param y
 * @param theta
 */
Campus::Campus(int id, std::string name, const geometry_msgs::Polygon& boundary,
               double x, double y, double theta)
    : Place(id, name, boundary, x, y, theta)
{
}

/**
 * @brief Campus::Campus
 * @param id
 * @param name
 * @param boundary
 * @param pose_msg
 */
Campus::Campus(int id, std::string name, const geometry_msgs::Polygon& boundary,
               const geometry_msgs::Pose& pose_msg)
    : Place(id, name, boundary, pose_msg)
{
}

/**
 * @brief Campus::Campus
 * @param campus_msg
 */
Campus::Campus(const mas_msgs::Place::ConstPtr& campus_msg) : Place(campus_msg)
{
}

/**
 * @brief Campus::Campus
 * @param campus_msg
 */
Campus::Campus(const mas_msgs::Place& campus_msg) : Place(campus_msg) {}

/**
 * @brief Campus::~Campus
 */
Campus::~Campus() {}

/**
 * @brief Campus::getType
 * @return
 */
int Campus::getType() const { return CAMPUS; }
}
}
