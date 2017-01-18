/**
 *  This source file implements the main function that calls the Speech Analyser
 *node controller.
 *
 *  Version: 1.4.0
 *  Created on: 01/04/2016
 *  Modified on: 16/12/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "speech_analyser/speech_analyser_node.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "speech_analyser_node");
  ros::NodeHandle nh;
  speech_analyser::SpeechAnalyserNode node(&nh);
  node.spin();
  return 0;
}
