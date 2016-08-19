/**
 *  SpeechAnalyserNode.cpp
 *
 *  Version: 1.2.4
 *  Created on: 01/04/2016
 *  Modified on: 17/08/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "speech_analyser/SpeechAnalyserNode.h"

using typename utilities::TimeManipulator;

namespace speech_analyser
{

	/**
	 * Constructor
	 */
	SpeechAnalyserNode::SpeechAnalyserNode(ros::NodeHandle nh) : nh_(nh)
	{
		questions_sub_ = nh_.subscribe("questions", 1, &SpeechAnalyserNode::questionsCallback, this);
		answers_pub_ = nh_.advertise<std_msgs::String>("answers", 1);
	}

	/**
	 * Destructor
	 */
	SpeechAnalyserNode::~SpeechAnalyserNode()
	{
		questions_sub_.shutdown();
		answers_pub_.shutdown();
	}

	/**
	 *
	 */
	void SpeechAnalyserNode::spin() 
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
	void SpeechAnalyserNode::questionsCallback(const std_msgs::String::ConstPtr& question_msg)
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
			srand(time(NULL));
			possible_answers.push_back(TimeManipulator::toString(ros::Time::now() + ros::Duration(rand() % 373248000))); // até 1 ano
			possible_answers.push_back(TimeManipulator::toString(ros::Time::now() + ros::Duration(rand() % 31104000))); // até 1 mês
			possible_answers.push_back(TimeManipulator::toString(ros::Time::now() + ros::Duration(rand() % 604800))); // até 1 semana
			possible_answers.push_back(TimeManipulator::toString(ros::Time::now() + ros::Duration(rand() % 86400))); // até 1 dia
			possible_answers.push_back(TimeManipulator::toString(ros::Time::now() + ros::Duration(rand() % 3600))); // até 1 hora
			possible_answers.push_back(TimeManipulator::toString(ros::Time::now() - ros::Duration(rand() % 373248000))); // até 1 ano atrás
			possible_answers.push_back(TimeManipulator::toString(ros::Time::now() - ros::Duration(rand() % 31104000))); // até 1 mês atrás
			possible_answers.push_back(TimeManipulator::toString(ros::Time::now() - ros::Duration(rand() % 604800))); // até 1 semana atrás
			possible_answers.push_back(TimeManipulator::toString(ros::Time::now() - ros::Duration(rand() % 86400))); // até 1 dia atrás
			possible_answers.push_back(TimeManipulator::toString(ros::Time::now() - ros::Duration(rand() % 3600))); // até 1 hora atrás
			possible_answers.push_back(TimeManipulator::toString(ros::Time()));
			possible_answers.push_back(TimeManipulator::toString(ros::Duration(rand() % 86400)));
			possible_answers.push_back(TimeManipulator::toString(ros::Duration(rand() % 43200)));
			possible_answers.push_back(TimeManipulator::toString(ros::Duration(rand() % 3600)));
			possible_answers.push_back(TimeManipulator::toString(ros::Duration()));
		}
		else
		{
			ROS_DEBUG("[SPEECH_ANALYSER] Unknown question!!!");
			return;
		}
		srand(time(NULL));
		answer_msg.data = possible_answers.at(rand() % possible_answers.size());
		//ROS_INFO("[SPEECH_ANALYSER] answer: %s", answer_msg.data.c_str());
		answers_pub_.publish(answer_msg);
	}
	
}
