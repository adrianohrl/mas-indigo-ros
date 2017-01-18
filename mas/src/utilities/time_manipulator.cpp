/**
 *  This source file implements the TimeManipulator class.
 *
 *  Version: 1.4.0
 *  Created on: 16/05/2016
 *  Modified on: 13/12/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "utilities/time_manipulator.h"

namespace utilities
{

/**
 * @brief TimeManipulator::isTime
 * @param time
 * @return
 */
bool TimeManipulator::isTime(std::string time)
{
  return TimeManipulator::isTime(TimeManipulator::getTime(time));
}

/**
 * @brief TimeManipulator::isTime
 * @param time
 * @return
 */
bool TimeManipulator::isTime(const ros::Time& time)
{
  return time.isValid();
}

/**
 * @brief TimeManipulator::isDeadline
 * @param deadline
 * @return
 */
bool TimeManipulator::isDeadline(std::string deadline)
{
  return TimeManipulator::isDeadline(TimeManipulator::getTime(deadline));
}

/**
 * @brief TimeManipulator::isDeadline
 * @param time
 * @return
 */
bool TimeManipulator::isDeadline(const ros::Time& time)
{
  return TimeManipulator::isTime(time) && TimeManipulator::isFutureTime(time);
}

/**
 * @brief TimeManipulator::isDuration
 * @param duration
 * @return
 */
bool TimeManipulator::isDuration(std::string duration)
{
  return TimeManipulator::isDuration(getDuration(duration));
}

/**
 * @brief TimeManipulator::isDuration
 * @param duration
 * @return
 */
bool TimeManipulator::isDuration(const ros::Duration& duration)
{
  return !duration.isZero();
}

/**
 * @brief TimeManipulator::getTime
 * @param answer
 * @return
 */
ros::Time TimeManipulator::getTime(std::string answer)
{
  if (!TimeManipulator::hasTimestampSyntax(answer))
  {
    return ros::Time();
  }
  std::vector<std::string> splitted, date, time;
  splitted = StringManipulator::split(answer, ' ');
  if (TimeManipulator::hasDateSyntax(splitted[0]) && TimeManipulator::hasTimeSyntax(splitted[1]))
  {
    date = StringManipulator::split(splitted[0], '/');
    time = StringManipulator::split(splitted[1], ':');
  }
  else
  {
    date = StringManipulator::split(splitted[1], '/');
    time = StringManipulator::split(splitted[0], ':');
  }
  int hours = atoi(time[0].c_str());
  int minutes = atoi(time[1].c_str());
  float seconds = time.size() == 3 ? atof(time[2].c_str()) : 0.0;
  int month = atoi(date[0].c_str());
  int day = atoi(date[1].c_str());
  int year = atoi(date[2].c_str());
  return TimeManipulator::getTime(month, day, year, hours, minutes, seconds);
}

/**
 * @brief TimeManipulator::getTime
 * @param month
 * @param day
 * @param year
 * @param hours
 * @param minutes
 * @param seconds
 * @return
 */
ros::Time TimeManipulator::getTime(int month, int day, int year, int hours, int minutes, float seconds)
{
  double timestamp = TimeManipulator::getTimestamp(month, day, year, hours, minutes, seconds);
  return ros::Time(timestamp);
}

/**
 * @brief TimeManipulator::getDeadline
 * @param duration
 * @return
 */
ros::Time TimeManipulator::getDeadline(const ros::Duration& duration)
{
  return ros::Time::now() + duration;
}

/**
 * @brief TimeManipulator::getDuration
 * @param duration
 * @return
 */
ros::Duration TimeManipulator::getDuration(std::string duration)
{
  if(!TimeManipulator::hasDurationSyntax(duration))
  {
    return ros::Duration();
  }
  std::vector<std::string> splitted;
  splitted = StringManipulator::split(duration, ':');
  int hours = atoi(splitted[0].c_str());
  int minutes = atoi(splitted[1].c_str());
  float seconds = splitted.size() == 3 ? atof(splitted[2].c_str()) : 0.0;
  return TimeManipulator::getDuration(hours, minutes, seconds);
}

/**
 * @brief TimeManipulator::getDuration
 * @param hours
 * @param minutes
 * @param seconds
 * @return
 */
ros::Duration TimeManipulator::getDuration(int hours, int minutes, float seconds)
{
  double duration = TimeManipulator::getDurationInSeconds(hours, minutes, seconds);
  return ros::Duration(duration);
}

/**
 * @brief TimeManipulator::toString
 * @param time
 * @return
 */
std::string TimeManipulator::str(const ros::Time& time)
{
  double tenths = time.toSec() - floor(time.toSec());
  long minutes_unix = MathManipulator::getUnsignedDivision((long) floor(time.toSec()), 60);
  int seconds = MathManipulator::getUnsignedRest((long) floor(time.toSec()), 60);
  long hours_unix = MathManipulator::getUnsignedDivision(minutes_unix, 60);
  int minutes = MathManipulator::getUnsignedRest(minutes_unix, 60);
  long days_unix = MathManipulator::getUnsignedDivision(hours_unix, 24);
  int hours = MathManipulator::getUnsignedRest(hours_unix, 24);
  long periods_of_400_years = MathManipulator::getUnsignedDivision(days_unix, 146097);
  int days_in_400_years_period = MathManipulator::getUnsignedRest(days_unix, 146097);
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
    year_days -= months_table[month_counter++];
  }
  int month = month_counter + 1;
  int day = year_days + 1;
  std::stringstream ss;
  ss << (month < 10 ? "0" : "") << month << (day < 10 ? "/0" : "/") << day << (year < 10 ? "/0" : "/") << year;
  ss << (hours < 10 ? " 0" : " ") << hours << (minutes < 10 ? ":0" : ":") << minutes;
  if (seconds + tenths > 0)
  {
    ss << (seconds < 10 ? ":0" : ":") << seconds + tenths;
  }
  return ss.str();
}

/**
 * @brief TimeManipulator::str
 * @param duration
 * @return
 */
std::string TimeManipulator::str(const ros::Duration& duration)
{
  double seconds = duration.toSec();
  int hours = floor(seconds / 3600);
  int minutes = floor(seconds / 60);
  seconds = (seconds / 60 - minutes) * 60;
  minutes -= hours * 60;
  std::stringstream ss;
  ss << hours << (minutes < 10 ? ":0" : ":") << minutes;
  if (seconds > 0)
  {
    ss << (seconds < 10 ? ":0" : ":") << seconds;
  }
  return ss.str();
}

/**
 * @brief TimeManipulator::hasDateSyntax
 * @param date
 * @return
 */
bool TimeManipulator::hasDateSyntax(std::string date)
{
  return std::count(date.begin(), date.end(), '/') == 2;
}

/**
 * @brief TimeManipulator::hasTimeSyntax
 * @param time
 * @return
 */
bool TimeManipulator::hasTimeSyntax(std::string time)
{
  int counter = std::count(time.begin(), time.end(), ':');
  return (counter == 1 || counter == 2);
}

/**
 * @brief TimeManipulator::hasTimestampSyntax
 * @param timestamp
 * @return
 */
bool TimeManipulator::hasTimestampSyntax(std::string timestamp)
{
  std::vector<std::string> splitted;
  splitted = StringManipulator::split(timestamp, ' ');
  return splitted.size() == 2 &&
      ((TimeManipulator::hasDateSyntax(splitted[0]) && TimeManipulator::hasTimeSyntax(splitted[1])) ||
      (TimeManipulator::hasDateSyntax(splitted[1]) && TimeManipulator::hasTimeSyntax(splitted[0])));
}

/**
 * @brief TimeManipulator::hasDurationSyntax
 * @param duration
 * @return
 */
bool TimeManipulator::hasDurationSyntax(std::string duration)
{
  return TimeManipulator::hasTimeSyntax(duration) &&
      std::count(duration.begin(), duration.end(), '/') == 0 &&
      std::count(duration.begin(), duration.end(), ' ') == 0;
}

/**
 * @brief TimeManipulator::isValidDuration
 * @param hours
 * @param minutes
 * @param seconds
 * @return
 */
bool TimeManipulator::isValidDuration(int hours, int minutes, float seconds)
{
  return hours >= 0 && minutes >= 0 && minutes < 60 && seconds >= 0 && seconds < 60;
}

/**
 * @brief TimeManipulator::isValidDate
 * @param month
 * @param day
 * @param year
 * @return
 */
bool TimeManipulator::isValidDate(int month, int day, int year)
{
  return year > 0 && month > 0 && month <= 12 && day > 0 &&
      (((month == 1 || month == 3 || month == 5 || month == 7 || month == 8 || month == 10 || month == 12) && day <= 31) ||
       ((month == 4 || month == 6 || month == 9 || month == 11) && day <= 30) ||
       (month == 2 && ((TimeManipulator::isLeapYear(year) && day <= 29) || day <= 28)));
}

/**
 * @brief TimeManipulator::isValidTime
 * @param hour
 * @param minutes
 * @param seconds
 * @return
 */
bool TimeManipulator::isValidTime(int hour, int minutes, float seconds)
{
  return hour >= 0 && hour < 24 && minutes >= 0 && minutes < 60 && seconds >= 0 && seconds < 60;
}

/**
 * @brief TimeManipulator::isLeapYear
 * @param year
 * @return
 */
bool TimeManipulator::isLeapYear(int year)
{
  return year % 4 == 0 && (year % 400 == 0 || year % 100 != 0);
}

/**
 * @brief TimeManipulator::isFutureTime
 * @param time
 * @return
 */
bool TimeManipulator::isFutureTime(ros::Time time)
{
  return time > ros::Time::now();
}

/**
 * @brief TimeManipulator::isFutureTime
 * @param month
 * @param day
 * @param year
 * @param hours
 * @param minutes
 * @param seconds
 * @return
 */
bool TimeManipulator::isFutureTime(int month, int day, int year, int hours, int minutes, float seconds)
{
  ros::Time time(TimeManipulator::getTimestamp(month, day, year, hours, minutes, seconds));
  return TimeManipulator::isFutureTime(time);
}

/**
 * @brief TimeManipulator::getTimestamp
 * @param month
 * @param day
 * @param year
 * @param hours
 * @param minutes
 * @param seconds
 * @return
 */
double TimeManipulator::getTimestamp(int month, int day, int year, int hours, int minutes, float seconds)
{
  long years_since_1970 = year - 1970;
  long periods_of_400_years = MathManipulator::getUnsignedDivision(years_since_1970, 400);
  int year_in_400_years_period = MathManipulator::getUnsignedRest(years_since_1970, 400);
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
  return seconds + 60 * (double) minutes + 60 * 60 * (double) hours + 60 * 60 * 24 * (double) days_since_01_01_1970;
}

/**
 * @brief TimeManipulator::getDurationInSeconds
 * @param hours
 * @param minutes
 * @param seconds
 * @return
 */
double TimeManipulator::getDurationInSeconds(int hours, int minutes, float seconds)
{
  return (double) (3600 * hours + 60 * minutes + seconds);
}

}
