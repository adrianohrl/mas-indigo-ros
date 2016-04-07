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

#include <ros/ros.h>

namespace mrta_vc 
{

	class SpeechAnalyserNode 
	{

	public:
		/** Construtors */
		SpeechAnalyserNode(ros::NodeHandle nh);
		/** Destrutor */
		~SpeechAnalyserNode();

		/** métodos publicos relacionados ao gerenciamento do nó */
		void spin();

	private:
		/** atributos privados relacionados ao nó */
		ros::NodeHandle nh_;

	};

}

#endif /* SPEECH_ANALYSER_NODE_H_ */
