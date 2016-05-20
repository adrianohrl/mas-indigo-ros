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

#include <string>
#include <ros/ros.h>
#include <time.h>
#include "unifei/expertinos/mrta_vc/utilities/StringManipulator.h"

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
                              static bool isDuration(std::string answer);

                        private:
                              static bool hasDateSyntax(std::vector<std::string> split_answer, int vector_position);
                              static bool hasTimeSyntax(std::vector<std::string> split_answer, int vector_position);
                              static bool hasCorrectDateIntervals(std::vector<std::string> date);
                              static bool hasCorrectTimeIntervals(std::vector<std::string> time);
                              static bool isLeapYear(std::string year); 
                              static bool isFuture(std::vector<std::string> time_str, std::vector<std::string> date);
                        };
                  }
            }
	}
}		
					
#endif /* TIME_MANIPULATOR_H_ */
