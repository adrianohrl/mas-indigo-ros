/**
 *  TimeManipulator.h
 *
 *  Version: 1.0.0.0
 *  Created on: 16/05/2016
 *  Modified on: *********
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef TIME_MANIPULATOR_H_
#define TIME_MANIPULATOR_H_

#include <ros/ros.h>
#include "unifei/expertinos/mrta_vc/utilities/StringManipulator.h"
#include "unifei/expertinos/mrta_vc/utilities/MathManipulator.h"

namespace unifei
{
	namespace expertinos
	{
            namespace mrta_vc
            {
                  namespace utilities
                  {
                        class TimeManipulator
                        {
                        
                        public:
                              static bool isDeadline(std::string answer);
                              static bool isTimestamp(std::string answer);
                              static bool isDuration(std::string answer);
															static ros::Time getTime(std::string answer);
                              static ros::Duration getDuration(std::string answer);
                              static std::string toString(ros::Time timestamp);
                              static std::string toString(ros::Duration duration);

                        private:
                              static bool hasDateSyntax(std::string date);
                              static bool hasTimeSyntax(std::string time);
															static bool isValidDuration(int hours, int minutes, float seconds = 0.0);
                              static bool isValidDate(int month, int day, int year);
                              static bool isValidTime(int hours, int minutes, float seconds = 0.0);
                              static bool isLeapYear(int year);
                              static bool isFuture(int month, int day, int year, int hours, int minutes, float seconds = 0.0);
															static double getTimestamp(int month, int day, int year, int hours, int minutes, float seconds = 0.0);
															static double getSecondsDuration(int hours, int minutes, float seconds = 0.0);

												};
                  }
            }
	}
}		
					
#endif /* TIME_MANIPULATOR_H_ */
