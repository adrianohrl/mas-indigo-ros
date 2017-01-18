/**
 *  This header file defines the TimeManipulator helper class.
 *
 *  Version: 1.4.0
 *  Created on: 16/05/2016
 *  Modified on: 14/12/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef UTILITIES_TIME_MANIPULATOR_H_
#define UTILITIES_TIME_MANIPULATOR_H_

#include <ros/ros.h>
#include "utilities/math_manipulator.h"
#include "utilities/string_manipulator.h"

namespace utilities
{
class TimeManipulator
{

public:
  static bool isTime(std::string time);
  static bool isTime(const ros::Time& time);
  static bool isDeadline(std::string deadline);
  static bool isDeadline(const ros::Time& deadline);
  static bool isDuration(std::string duration);
  static bool isDuration(const ros::Duration& duration);
  static ros::Time getTime(std::string time);
  static ros::Time getTime(int month, int day, int year, int hours = 0,
                           int minutes = 0, float seconds = 0.0);
  static ros::Time getDeadline(const ros::Duration& duration);
  static ros::Duration getDuration(std::string duration);
  static ros::Duration getDuration(int hours, int minutes, float seconds = 0.0);
  static std::string str(const ros::Time& time);
  static std::string str(const ros::Duration& duration);

private:
  static bool hasDateSyntax(std::string date);
  static bool hasTimeSyntax(std::string time);
  static bool hasTimestampSyntax(std::string timestamp);
  static bool hasDurationSyntax(std::string duration);
  static bool isValidDuration(int hours, int minutes, float seconds = 0.0);
  static bool isValidDate(int month, int day, int year);
  static bool isValidTime(int hours, int minutes, float seconds = 0.0);
  static bool isLeapYear(int year);
  static bool isFutureTime(ros::Time time);
  static bool isFutureTime(int month, int day, int year, int hours, int minutes,
                           float seconds = 0.0);
  static double getTimestamp(int month, int day, int year, int hours,
                             int minutes, float seconds = 0.0);
  static double getDurationInSeconds(int hours, int minutes,
                                     float seconds = 0.0);
};
}

#endif /* UTILITIES_TIME_MANIPULATOR_H_ */
