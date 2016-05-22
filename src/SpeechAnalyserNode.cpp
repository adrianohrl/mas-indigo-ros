/**
 *  SpeechAnalyserNode.cpp
 *
 *  Version: 0.0.0.0
 *  Created on: 01/04/2016
 *  Modified on: *********
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mrta_vc/SpeechAnalyserNode.h"

/**
 * Constructor
 */
mrta_vc::SpeechAnalyserNode::SpeechAnalyserNode(ros::NodeHandle nh) : nh_(nh)
{
  questions_sub_ = nh_.subscribe("questions", 1, &mrta_vc::SpeechAnalyserNode::questionsCallback, this);
  answers_pub_ = nh_.advertise<std_msgs::String>("answers", 1);
}

/**
 * Destructor
 */
mrta_vc::SpeechAnalyserNode::~SpeechAnalyserNode()
{
  questions_sub_.shutdown();
  answers_pub_.shutdown();
}

/**
 * 
 */
void mrta_vc::SpeechAnalyserNode::spin() 
{
	ROS_INFO("Speech Analyser Node is up and running!!!");
	ros::Rate loop_rate(10.0);
	while (nh_.ok()) 
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
}

/**
 *
 */
void mrta_vc::SpeechAnalyserNode::questionsCallback(const std_msgs::String::ConstPtr& question_msg)
{
	std_msgs::String answer_msg;
	std::vector<std::string> possible_answers;
	if (question_msg->data == "What should I do?")
	{
		possible_answers.push_back("bring");
		possible_answers.push_back("take");
		possible_answers.push_back("send");
	}
	else if (question_msg->data == "What?")
	{
		possible_answers.push_back("water");
		possible_answers.push_back("document");
		possible_answers.push_back("meal");
		possible_answers.push_back("coffee");
		possible_answers.push_back("gallon");
	}
	else if (question_msg->data == "From whom?" || question_msg->data == "To whom?")
	{
		possible_answers.push_back("Adriano Henrique Rossette Leite");
		possible_answers.push_back("Christiano Henrique Rezende");
		possible_answers.push_back("Luís Victor Pessiqueli Bonin");
		possible_answers.push_back("Héverton Soares");
		possible_answers.push_back("Audeliano Li");
	}
	else if (question_msg->data == "How urgent?")
	{
		possible_answers.push_back("low");
		possible_answers.push_back("normal");
		possible_answers.push_back("important");
		possible_answers.push_back("critical");
	}
	else if (question_msg->data == "What is the deadline?")
	{		
		possible_answers.push_back(unifei::expertinos::mrta_vc::utilities::TimeManipulator::toString(ros::Time::now() + ros::Duration(rand() % 31104000)));
		possible_answers.push_back(unifei::expertinos::mrta_vc::utilities::TimeManipulator::toString(ros::Duration(rand() % 86400)));
	}
	else
	{
		ROS_ERROR("Unknown question!!!");
		return;
	}
	answer_msg.data = possible_answers.at(rand() % possible_answers.size());
	answers_pub_.publish(answer_msg);
}
