/**
 *  SpeechAnalyserNode.h
 *
 *  Version: 1.2.4
 *  Created on: 01/04/2016
 *  Modified on: 17/08/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef SPEECH_ANALYSER_NODE_H_
#define SPEECH_ANALYSER_NODE_H_

#include <stdlib.h>     /* srand, rand */
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <utilities/TimeManipulator.h>

namespace speech_analyser 
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
