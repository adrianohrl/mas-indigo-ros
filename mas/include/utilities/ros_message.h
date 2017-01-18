/**
 *  This header file defines the templated ROSMessage abstract class (interface).
 *  The classes which enhance this interface must implement all its methods.
 *
 *  Version: 1.4.0
 *  Created on: 12/12/2016
 *  Modified on: 12/12/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_ROS_MESSAGE_H_
#define _UTILITIES_ROS_MESSAGE_H_

#include <string>
#include <vector>

namespace utilities
{

/**
 * E type of a ROS message.
 */
template <typename E> class ROSMessage
{
public:
  /**
   * @brief to_msg converts this object to an <E> ROS message.
   * @return an <E> ROS message.
   */
  virtual E to_msg() const = 0;
  /**
   * @brief operator = assigns the fields of this object based on the
   * given <E> ROS message.
   * @param msg an <E> ROS message.
   */
  virtual void operator=(const E& msg) = 0;
  /**
   * @brief operator == compares this object to a given <E> ROS message.
   * @param msg an <E> ROS message.
   * @return true if this object is equals to the given <E> ROS message.
   */
  virtual bool operator==(const E& msg) const = 0;
  /**
   * @brief operator == compares this object to a given <E> ROS message.
   * @param msg an <E> ROS message.
   * @return true if this object is equals to the given <E> ROS message.
   */
  virtual bool operator!=(const E& msg) const;
};

/**
 *
 */
template<typename E>
bool ROSMessage<E>::operator!=(const E& msg) const
{
  return !operator==(msg);
}
}

#endif // _UTILITIES_ROS_MESSAGE_H_
