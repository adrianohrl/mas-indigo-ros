/**
 *  Skill.h
 *
 *  Version: 1.2.2
 *  Created on: 04/08/2015
 *  Modified on: 17/08/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *           Heverton Machado Soares (sm.heverton@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef TASKS_SKILL_H_
#define TASKS_SKILL_H_

#include <mas_msgs/Skill.h>
#include "unifei/expertinos/mrta_vc/tasks/Resource.h"
#include "unifei/expertinos/mrta_vc/tasks/SkillLevels.h"

namespace unifei 
{
	namespace expertinos
	{
		namespace mrta_vc
		{
			namespace tasks
			{
				class Skill 
				{
        public:
          Skill(std::string resource_name, SkillLevelEnum level = SkillLevels::getDefault());
          Skill(int id, Resource resource, SkillLevelEnum level = SkillLevels::getDefault());
					Skill(const mas_msgs::Skill::ConstPtr& skill_msg);
					Skill(mas_msgs::Skill skill_msg);		
					~Skill();

          int getId();
					SkillLevelEnum getLevel();
					Resource getResource();
          bool isSufficient(SkillLevelEnum desired_level);
          bool isSufficient(Skill desired_skill);
          void setId(int id);
					void setLevel(SkillLevelEnum level);
          mas_msgs::Skill toMsg();
					std::string toString();
					int compareTo(Skill skill);
					bool equals(Skill skill);
					bool operator==(const Skill& skill);
					bool operator!=(const Skill& skill);
					void operator=(const Skill& skill);

				private:
          int id_;
					Resource resource_;
					SkillLevelEnum level_;
				};
			}
		}
	}
}		

#endif /* TASKS_SKILL_H_ */
