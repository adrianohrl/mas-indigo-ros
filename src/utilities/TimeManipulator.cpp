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

/**
 *
 */
bool unifei::expertinos::mrta_vc::utilities::TimeManipulator::isDeadline(std::string answer)
{
	std::vector<std::string> split_answer, date, time;
	split_answer = unifei::expertinos::mrta_vc::utilities::StringManipulator::split(answer, ' ');
	if(split_answer.empty() || split_answer.size() > 2)
	{
		return false;
	}
	if(hasDateSyntax(split_answer[0]))
	{
		date = unifei::expertinos::mrta_vc::utilities::StringManipulator::split(split_answer[0], '/');
		if(!hasTimeSyntax(split_answer[1]))
		{
			return false;
		}
		time = unifei::expertinos::mrta_vc::utilities::StringManipulator::split(split_answer[1], ':');
	}
	else if(hasTimeSyntax(split_answer[0]))
	{
		time = unifei::expertinos::mrta_vc::utilities::StringManipulator::split(split_answer[0], ':');
		if(!hasDateSyntax(split_answer[1]))
		{
			return false;
		}
		date = unifei::expertinos::mrta_vc::utilities::StringManipulator::split(split_answer[1], '/');
	}
	else
	{
		return false;
	}
	int hours = atoi(time[0].c_str());
	int minutes = atoi(time[1].c_str());
	float seconds = 0;
	int month = atoi(date[0].c_str());
	int day = atoi(date[1].c_str());
	int year = atoi(date[2].c_str());
	if(time.size() == 3)
	{
		seconds = atof(time[2].c_str());
	}
	return isValidTime(hours, minutes, seconds) && isValidDate(month, day, year) && isFuture(month, day, year, hours, minutes, seconds);
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::utilities::TimeManipulator::isTimestamp(std::string answer)
{
	std::vector<std::string> split_answer, date, time;
	split_answer = unifei::expertinos::mrta_vc::utilities::StringManipulator::split(answer, ' ');
	if(split_answer.empty() || split_answer.size() > 2)
	{
		return false;
	}
	if(hasDateSyntax(split_answer[0]))
	{
		date = unifei::expertinos::mrta_vc::utilities::StringManipulator::split(split_answer[0], '/');
		if(!hasTimeSyntax(split_answer[1]))
		{
			return false;
		}
		time = unifei::expertinos::mrta_vc::utilities::StringManipulator::split(split_answer[1], ':');
	}
	else if(hasTimeSyntax(split_answer[0]))
	{
		time = unifei::expertinos::mrta_vc::utilities::StringManipulator::split(split_answer[0], ':');
		if(!hasDateSyntax(split_answer[1]))
		{
			return false;
		}
		date = unifei::expertinos::mrta_vc::utilities::StringManipulator::split(split_answer[1], '/');
	}
	else
	{
		return false;
	}
	int hours = atoi(time[0].c_str());
	int minutes = atoi(time[1].c_str());
	float seconds = 0;
	int month = atoi(date[0].c_str());
	int day = atoi(date[1].c_str());
	int year = atoi(date[2].c_str());
	if(time.size() == 3)
	{
		seconds = atof(time[2].c_str());
	}
	return isValidTime(hours, minutes, seconds) && isValidDate(month, day, year);
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::utilities::TimeManipulator::isDuration(std::string answer)
{
	std::vector<std::string> split_answer;
	if(!hasTimeSyntax(answer))
	{
		return false;
	}
	split_answer = unifei::expertinos::mrta_vc::utilities::StringManipulator::split(answer, ':');
	if(split_answer.empty() || split_answer.size() > 3)
	{
		return false;
	}
	int hours = atoi(split_answer[0].c_str());
	int minutes = atoi(split_answer[1].c_str());
	float seconds = 0.0;
	if(split_answer.size() == 3)
	{
		seconds = atof(split_answer[2].c_str());
	}
	return isValidDuration(hours, minutes, seconds);
}

/**
 *
 */
ros::Time unifei::expertinos::mrta_vc::utilities::TimeManipulator::getTime(std::string answer)
{
	std::vector<std::string> split_answer, date, time;
	split_answer = unifei::expertinos::mrta_vc::utilities::StringManipulator::split(answer, ' ');
	if(split_answer.empty() || split_answer.size() > 2)
	{
		return ros::Time();
	}
	if(hasDateSyntax(split_answer[0]))
	{
		date = unifei::expertinos::mrta_vc::utilities::StringManipulator::split(split_answer[0], '/');
		if(!hasTimeSyntax(split_answer[1]))
		{
			return ros::Time();
		}
		time = unifei::expertinos::mrta_vc::utilities::StringManipulator::split(split_answer[1], ':');
	}
	else if(hasTimeSyntax(split_answer[0]))
	{
		time = unifei::expertinos::mrta_vc::utilities::StringManipulator::split(split_answer[0], ':');
		if(!hasDateSyntax(split_answer[1]))
		{
			return ros::Time();
		}
		date = unifei::expertinos::mrta_vc::utilities::StringManipulator::split(split_answer[1], '/');
	}
	else
	{
		return ros::Time();
	}
	int hours = atoi(time[0].c_str());
	int minutes = atoi(time[1].c_str());
	float seconds = 0;
	int month = atoi(date[0].c_str());
	int day = atoi(date[1].c_str());
	int year = atoi(date[2].c_str());
	if(time.size() == 3)
	{
		seconds = atof(time[2].c_str());
	}
	return ros::Time(getTimestamp(month, day, year, hours, minutes, seconds));
}

/**
 *
 */
ros::Duration unifei::expertinos::mrta_vc::utilities::TimeManipulator::getDuration(std::string answer)
{
	std::vector<std::string> split_answer;
	if(!hasTimeSyntax(answer))
	{
		return ros::Duration();
	}
	split_answer = unifei::expertinos::mrta_vc::utilities::StringManipulator::split(answer, ':');
	if(split_answer.empty() || split_answer.size() > 3)
	{
		return ros::Duration();
	}
	int hours = atoi(split_answer[0].c_str());
	int minutes = atoi(split_answer[1].c_str());
	float seconds = 0.0;
	if(split_answer.size() == 3)
	{
		seconds = atof(split_answer[2].c_str());
	}
	return ros::Duration(getSecondsDuration(hours, minutes, seconds));
}

/**
 *
 */
std::string unifei::expertinos::mrta_vc::utilities::TimeManipulator::toString(ros::Time timestamp)
{
	long minutes_unix = unifei::expertinos::mrta_vc::utilities::MathManipulator::getUnsignedDivision((long)floor(timestamp.toSec()), 60);
	int seconds = unifei::expertinos::mrta_vc::utilities::MathManipulator::getUnsignedRest((long)floor(timestamp.toSec()), 60);
	long hours_unix = unifei::expertinos::mrta_vc::utilities::MathManipulator::getUnsignedDivision(minutes_unix, 60);
	int minutes = unifei::expertinos::mrta_vc::utilities::MathManipulator::getUnsignedRest(minutes_unix, 60);
	long days_unix = unifei::expertinos::mrta_vc::utilities::MathManipulator::getUnsignedDivision(hours_unix, 24);
	int hours = unifei::expertinos::mrta_vc::utilities::MathManipulator::getUnsignedRest(hours_unix, 24);
	long periods_of_400_years = unifei::expertinos::mrta_vc::utilities::MathManipulator::getUnsignedDivision(days_unix, 146097);
	int days_in_400_years_period = unifei::expertinos::mrta_vc::utilities::MathManipulator::getUnsignedRest(days_unix, 146097);
	if (days_in_400_years_period >= 32 * 1461 + 789)
	{
		days_in_400_years_period++;
	}
	if (days_in_400_years_period >= 57 * 1461 + 789)
	{
		days_in_400_years_period++;
	}
	if (days_in_400_years_period >= 82 * 1461 + 789)
	{
		days_in_400_years_period++;
	}
	int periods_of_4_years = days_in_400_years_period / 1461;
	int days_in_4_years_period = days_in_400_years_period % 1461;
	if (days_in_4_years_period >= 59)
	{
		days_in_4_years_period++;
	}
	if (days_in_4_years_period >= 425)
	{
		days_in_4_years_period++;
	}
	if (days_in_4_years_period >= 1157)
	{
		days_in_4_years_period++;
	}
	int year_in_4_years_period = days_in_4_years_period / 366;
	int year_days = days_in_4_years_period % 366;
	int year = year_in_4_years_period + periods_of_4_years * 4 + periods_of_400_years * 400 + 1970;
	int months_table[] = {31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
	int month_counter = 0;
	while(year_days > months_table[month_counter])
	{
		year_days -= months_table[month_counter];
		month_counter++;
	}
	int month = month_counter + 1;
	int day = year_days + 1;
	std::stringstream ss;
	ss << month << "/" << day << "/" << year << " " << hours << ":" << minutes << ":" << seconds;
	return ss.str();
}

/**
 *
 */
std::string unifei::expertinos::mrta_vc::utilities::TimeManipulator::toString(ros::Duration duration)
{
	int seconds_duration = floor(duration.toSec());
	int hours = (seconds_duration / 3600);
	int minutes = (seconds_duration -(3600 * hours)) / 60;
	int seconds = seconds_duration - (3600 * hours) - (minutes * 60);
	std::stringstream ss;
	ss << hours << ":" << minutes << ":" << seconds;
	return ss.str();
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::utilities::TimeManipulator::hasDateSyntax(std::string date)
{
	return std::count(date.begin(), date.end(), '/') == 2;
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::utilities::TimeManipulator::hasTimeSyntax(std::string time)
{
	int counter = std::count(time.begin(), time.end(), ':');
	return counter == 1 || counter == 2;
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::utilities::TimeManipulator::isValidDuration(int hours, int minutes, float seconds)
{
	return hours >= 0 && minutes >= 0 && minutes < 60 && seconds >= 0 && seconds < 60;
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::utilities::TimeManipulator::isValidDate(int month, int day, int year)
{
	return year > 0 && month > 0 && month <= 12 && day > 0 &&
			(((month == 1 || month == 3 || month == 5 || month == 7 || month == 8 || month == 10 || month == 12) && day <= 31) ||
			 ((month == 4 || month == 6 || month == 9 || month == 11) && day <= 30) ||
			 (month == 2 && ((isLeapYear(year) && day <= 29) || day <= 28)));
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::utilities::TimeManipulator::isValidTime(int hour, int minutes, float seconds)
{
	return hour >= 0 && hour < 24 && minutes >= 0 && minutes < 60 && seconds >= 0 && seconds < 60;
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::utilities::TimeManipulator::isLeapYear(int year)
{
	return year % 4 == 0 && (year % 400 == 0 || year % 100 != 0);
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::utilities::TimeManipulator::isFuture(int month, int day, int year, int hours, int minutes, float seconds)
{
	double timestamp = getTimestamp(month, day, year, hours, minutes, seconds);
	return ros::Time::now() < ros::Time(timestamp);
}

/**
 *
 */
double unifei::expertinos::mrta_vc::utilities::TimeManipulator::getTimestamp(int month, int day, int year, int hours, int minutes, float seconds)
{
	long years_since_1970 = year - 1970;
	long periods_of_400_years = unifei::expertinos::mrta_vc::utilities::MathManipulator::getUnsignedDivision(years_since_1970, 400);
	int year_in_400_years_period = unifei::expertinos::mrta_vc::utilities::MathManipulator::getUnsignedRest(years_since_1970, 400);
	int periods_of_4_years_in_400 = year_in_400_years_period / 4;
	int year_in_4_years_period = year_in_400_years_period % 4;
	int days_in_years_before_4_years_period = 365 * year_in_4_years_period + (year_in_4_years_period == 3 ? 1 : 0);
	long year_days = day - 1;
	int months_table[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
	for (int i = 0; i < month - 1; i++)
	{
		year_days += months_table[i];
	}
	long days_since_01_01_1970 = year_days + days_in_years_before_4_years_period + periods_of_4_years_in_400 * 1461 + periods_of_400_years * 146097;
	if (year_in_4_years_period == 2 && month > 2)
	{
		days_since_01_01_1970++;
	}
	if (year_in_400_years_period > 130 || (year_in_400_years_period == 130 && month > 2))
	{
		days_since_01_01_1970--;
	}
	if (year_in_400_years_period > 230 || (year_in_400_years_period == 230 && month > 2))
	{
		days_since_01_01_1970--;
	}
	if (year_in_400_years_period > 330 || (year_in_400_years_period == 330 && month > 2))
	{
		days_since_01_01_1970--;
	}
	return seconds + 60 * (double)minutes + 60 * 60 * (double)hours + 60 * 60 * 24 * (double)days_since_01_01_1970;
}

/**
 *
 */
double unifei::expertinos::mrta_vc::utilities::TimeManipulator::getSecondsDuration(int hours, int minutes, float seconds)
{
	return 60 * 60 * (double)hours + 60 * (double)minutes + (double)seconds;
}
