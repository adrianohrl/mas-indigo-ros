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

#include "mrta_vc/state_machine/MachineController.h"

namespace mrta_vc
{
	namespace state_machine
	{
		class AbstractState 
		{

		public:
 			~AbstractState();

 			std::string getQuestion();
 			std::string getMessage();
 			std::string getAnswer();

 			void setQuestion(std::string question);
 			void setMessage(std::string message);
 			void setAnswer(std::string answer);

 			virtual void process(std::string answer);
 			virtual void next(); 			
 			virtual bool isValid();

 		protected:
            AbstractState(MachineController controller, std::string question = "");
			MachineController getController(); 			
 			ros::NodeHandle getNodeHandle();

 		private:
 			MachineController controller_;

 			std::string question_;
 			std::string message_;
 			std::string answer_;
		};
	}
}		
					
#endif /* ABSTRACT_STATE_H_ */
