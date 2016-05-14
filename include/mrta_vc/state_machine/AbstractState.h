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
      bool isFinalState();
 			void setQuestion(std::string question);
      void setMessage(std::string message);
      virtual void process(std::string answer);

 		protected:
      AbstractState(MachineController* controller, std::string question = "", bool final_state = false);

      ros::NodeHandle getNodeHandle();
      MachineController* getController();

 		private:
      MachineController* controller_;

 			std::string question_;
      std::string message_;
      bool final_state_;

      virtual void next(std::string answer);
		};
	}
}		
					
#endif /* ABSTRACT_STATE_H_ */
