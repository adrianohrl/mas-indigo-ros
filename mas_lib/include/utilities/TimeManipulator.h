/**
 *  TimeManipulator.h
 *
 *  Version: 1.2.4
 *  Created on: 16/05/2016
 *  Modified on: 17/08/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef UTILITIES_TIME_MANIPULATOR_H_
#define UTILITIES_TIME_MANIPULATOR_H_

#include <ros/ros.h>
#include "utilities/MathManipulator.h"
#include "utilities/StringManipulator.h"

namespace utilities
{
	class TimeManipulator
	{

	public:
		static bool isTime(std::string time);
		static bool isTime(ros::Time time);
		static bool isDeadline(std::string deadline);
		static bool isDeadline(ros::Time deadline);
		static bool isDuration(std::string duration);
		static bool isDuration(ros::Duration duration);
		static ros::Time getTime(std::string time);
		static ros::Time getTime(int month, int day, int year, int hours = 0, int minutes = 0, float seconds = 0.0);
		static ros::Time getDeadline(ros::Duration duration);
		static ros::Duration getDuration(std::string duration);
		static ros::Duration getDuration(int hours, int minutes, float seconds = 0.0);
		static std::string toString(ros::Time time);
		static std::string toString(ros::Duration duration);

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
		static bool isFutureTime(int month, int day, int year, int hours, int minutes, float seconds = 0.0);
		static double getTimestamp(int month, int day, int year, int hours, int minutes, float seconds = 0.0);
		static double getDurationInSeconds(int hours, int minutes, float seconds = 0.0);

	};
}

#endif /* UTILITIES_TIME_MANIPULATOR_H_ */
