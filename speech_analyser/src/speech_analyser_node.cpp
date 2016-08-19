/**
 *  main.cpp
 *
 *  Version: 1.2.4
 *  Created on: 01/04/2016
 *  Modified on: 17/08/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "speech_analyser/SpeechAnalyserNode.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "speech_analyser_node");
	ros::NodeHandle nh;
	speech_analyser::SpeechAnalyserNode node(nh);
	node.spin();
	return 0;
}
