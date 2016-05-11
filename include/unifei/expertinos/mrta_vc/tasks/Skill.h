/**
 *  Skill.h
 *
 *  Version: 1.0.0.0
 *  Created on: 04/08/2015
 *  Modified on: *********
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *           Heverton Machado Soares (sm.heverton@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef SKILL_H_
#define SKILL_H_

#include "mrta_vc/Skill.h"
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
					Skill(const ::mrta_vc::Skill::ConstPtr& skill_msg);
					Skill(::mrta_vc::Skill skill_msg);		
					~Skill();

          int getId();
					SkillLevelEnum getLevel();
					Resource getResource();
          bool isSufficient(SkillLevelEnum desired_level);
          bool isSufficient(Skill desired_skill);
          void setId(int id);
					void setLevel(SkillLevelEnum level);
          ::mrta_vc::Skill toMsg();
          std::string toString();
					bool equals(Skill skill);
					int compareTo(Skill skill);
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

#endif /* SKILL_H_ */
