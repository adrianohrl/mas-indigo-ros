/**
 *  TimeManipulator.cpp
 *
 *  Version: 0.0.0.0
 *  Created on: 16/05/2016
 *  Modified on: *********
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "unifei/expertinos/mrta_vc/utilities/TimeManipulator.h"

bool unifei::expertinos::mrta_vc::utilities::TimeManipulator::isDeadline(std::string answer)
{
	std::vector<std::string> split_answer, date, time;
 	bool time_comparator = false, date_comparator = false;

 	split_answer = unifei::expertinos::mrta_vc::utilities::StringManipulator::split(answer, ' ');

 	for(int i = 0; i < split_answer.size(); i++)
	{
		if(unifei::expertinos::mrta_vc::utilities::TimeManipulator::hasDateSyntax(split_answer, i))
		{
			date = unifei::expertinos::mrta_vc::utilities::StringManipulator::split(split_answer[i], '/');
			date_comparator = true;
		}

		else if(unifei::expertinos::mrta_vc::utilities::TimeManipulator::hasTimeSyntax(split_answer, i))
		{
			time = unifei::expertinos::mrta_vc::utilities::StringManipulator::split(split_answer[i], ':');
			time_comparator = true;
		}

		else
		{
			return false;
		}
	}

	if(time_comparator && unifei::expertinos::mrta_vc::utilities::TimeManipulator::hasCorrectTimeIntervals(time) == false)
	{
		return false;
	}

	if(date_comparator && unifei::expertinos::mrta_vc::utilities::TimeManipulator::hasCorrectDateIntervals(date) == false)
	{
		return false;
	}

  if(unifei::expertinos::mrta_vc::utilities::TimeManipulator::isFuture(time, date))
  {
    return true;
  }

  else if(unifei::expertinos::mrta_vc::utilities::TimeManipulator::isFuture(time, date) == false)
  {
    return false;
  }
}

bool unifei::expertinos::mrta_vc::utilities::TimeManipulator::isDuration(std::string answer)
{
  std::vector<std::string> auxiliar;
  int hours, minutes;
  float seconds;

  auxiliar = unifei::expertinos::mrta_vc::utilities::StringManipulator::split(answer, ':');

  hours = atoi(auxiliar[0].c_str());

  minutes = atoi(auxiliar[1].c_str());

  seconds = atof(auxiliar[2].c_str());

  if(hours >= 0 && (minutes >= 0 && minutes < 60) && (seconds >= 0 && seconds < 60))
  {
    return true;
  }

  else
  {
    return false;
  }
}

bool unifei::expertinos::mrta_vc::utilities::TimeManipulator::hasDateSyntax(std::vector<std::string> split_answer, int vector_position)
{
	if(std::count(split_answer[vector_position].begin(), split_answer[vector_position].end(), '/') == 2)
	{
		return true;
	}

	else
	{
		return false;
	}
}

bool unifei::expertinos::mrta_vc::utilities::TimeManipulator::hasTimeSyntax(std::vector<std::string> split_answer, int vector_position)
{
	if(std::count(split_answer[vector_position].begin(), split_answer[vector_position].end(), ':') >= 1)
	{
		return true;
	}

	else
	{
		return false;
	}
}

bool unifei::expertinos::mrta_vc::utilities::TimeManipulator::hasCorrectDateIntervals(std::vector<std::string> date)
{
  if(atoi(date[2].c_str()) < 0)
    {
      return false;
    }

    if(atoi(date[0].c_str()) < 0 || atoi(date[0].c_str()) > 12)
    {
      return false;
    }

    if((atoi(date[0].c_str()) == 1 || atoi(date[0].c_str()) == 3 || atoi(date[0].c_str()) == 5 || atoi(date[0].c_str()) == 7 || atoi(date[0].c_str()) == 8 || atoi(date[0].c_str()) == 10 || atoi(date[0].c_str()) == 12) && (atoi(date[1].c_str()) < 0 && atoi(date[1].c_str()) > 31))
    {
      return false;
    }

    if((atoi(date[0].c_str()) == 4 || atoi(date[0].c_str()) == 6 || atoi(date[0].c_str()) == 9 || atoi(date[0].c_str()) == 11) && (atoi(date[1].c_str()) < 0 && atoi(date[1].c_str()) > 30))
    {
      return false;
    }

    if((atoi(date[0].c_str()) == 2) && (atoi(date[1].c_str()) < 0 && atoi(date[1].c_str()) > 28))
    {
      return false;
    }

    else if((atoi(date[0].c_str()) == 2) && unifei::expertinos::mrta_vc::utilities::TimeManipulator::isLeapYear(date[3]) && (atoi(date[1].c_str()) < 0 && atoi(date[1].c_str()) > 29))
    {
      return false;
    }

    else
    {
    	return true;
    }
}

bool unifei::expertinos::mrta_vc::utilities::TimeManipulator::hasCorrectTimeIntervals(std::vector<std::string> time)
{
	if(atoi(time[0].c_str()) < 0 || atoi(time[0].c_str()) > 24)
    {
       	return false;
    }

    if(atoi(time[1].c_str()) < 0 || atoi(time[1].c_str()) >= 60)
    {
       	return false;
    }

    else if(atof(time[2].c_str()) < 0 || atof(time[2].c_str()) >= 60)
    {
       	return false;
    }

    else
    {
    	return true;
    }
}

bool unifei::expertinos::mrta_vc::utilities::TimeManipulator::isLeapYear(std::string year)
{
	if(atoi(year.c_str()) % 4 == 0 && (atoi(year.c_str()) % 400 == 0 || atoi(year.c_str()) % 100 != 0))
	{
		return true;
	}

	else
	{
		return false;
	}
}

bool unifei::expertinos::mrta_vc::utilities::TimeManipulator::isFuture(std::vector<std::string> time_str  , std::vector<std::string> date)
{
  time_t now;
  struct tm task_date;
  double seconds;

  time(&now);

  task_date = *localtime(&now);

  task_date.tm_hour = atoi(time_str[0].c_str()); task_date.tm_min = atoi(time_str[1].c_str()); task_date.tm_sec = atoi(time_str[2].c_str());
  task_date.tm_mon = atoi(date[0].c_str());  task_date.tm_mday = atoi(date[1].c_str()); task_date.tm_year = atoi(date[2].c_str());

  if(difftime(now, mktime(&task_date)) < 0)
  {
    return true;
  }

  else
  {
    return false;
  }
}
