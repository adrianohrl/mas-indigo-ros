/**
 *  AbstractState.h
 *
 *  Version: 1.0.0.0
 *  Created on: 10/05/2016
 *  Modified on: *********
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef ABSTRACT_STATE_H_
#define ABSTRACT_STATE_H_

#include <string>
#include <ros/ros.h>
//#include "mrta_vc/state_machine/MachineController.h"

namespace mrta_vc
{
	namespace state_machine
	{
    class MachineController;

		class AbstractState 
		{

		public:
 			~AbstractState();

 			std::string getQuestion();
      std::string getMessage();
 			void setQuestion(std::string question);
      void setMessage(std::string message);
      virtual void process(std::string answer);

 		protected:
      AbstractState(MachineController* controller, std::string question = "");

      ros::NodeHandle getNodeHandle();
      MachineController* getController();

 		private:
      MachineController* controller_;

 			std::string question_;
      std::string message_;

      virtual void next();
		};
	}
}		
					
#endif /* ABSTRACT_STATE_H_ */
