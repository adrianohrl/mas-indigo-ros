/**
 *  SpeechAnalyserNode.h
 *
 *  Version: 0.0.0.0
 *  Created on: 01/04/2016
 *  Modified on: *********
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef SPEECH_ANALYSER_NODE_H_
#define SPEECH_ANALYSER_NODE_H_

#include <stdlib.h> /* srand, rand */
#include <ros/ros.h>
#include <unifei/expertinos/utilities/TimeManipulator.h> /* libmas */
#include "std_msgs/String.h"

namespace mrta_vc
{

class SpeechAnalyserNode
{

public:
  SpeechAnalyserNode(ros::NodeHandle nh);
  ~SpeechAnalyserNode();

  void spin();

private:
  ros::NodeHandle nh_;
  ros::Subscriber questions_sub_;
  ros::Publisher answers_pub_;

  void questionsCallback(const std_msgs::String::ConstPtr& question_msg);
};
}

#endif /* SPEECH_ANALYSER_NODE_H_ */
