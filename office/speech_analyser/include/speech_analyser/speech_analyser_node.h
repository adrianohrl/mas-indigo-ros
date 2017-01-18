/**
 *  This feader file defines the SpeechAnalyserNode class, which is based on the
 *ROSNode helper class. It controls the speech_analyser_node.
 *
 *  Version: 1.4.0
 *  Created on: 01/04/2016
 *  Modified on: 16/12/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _SPEECH_ANALYSER_NODE_H_
#define _SPEECH_ANALYSER_NODE_H_

#include <stdlib.h> /* srand, rand */
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <utilities/time_manipulator.h>
#include <utilities/ros_node.h>

namespace speech_analyser
{
class SpeechAnalyserNode : public utilities::ROSNode
{

public:
  SpeechAnalyserNode(ros::NodeHandle* nh);
  ~SpeechAnalyserNode();

private:
  ros::Subscriber questions_sub_;
  ros::Publisher answers_pub_;
  virtual void controlLoop();
  void questionsCallback(const std_msgs::String::ConstPtr& question_msg);
};
}

#endif /* _SPEECH_ANALYSER_NODE_H_ */
