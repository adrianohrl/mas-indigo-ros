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

#include "Resource.h"
#include "mrta_vc/Skill.h"
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
					Skill(Resource resource, SkillLevelEnum level = SkillLevels::getDefault());
					Skill(const ::mrta_vc::Skill::ConstPtr& skill_msg);
					Skill(::mrta_vc::Skill skill_msg);		
					~Skill();

					SkillLevelEnum getLevel();
					Resource getResource();
					void setLevel(SkillLevelEnum level);
					bool isSufficient(Skill skill);
					int compareTo(Skill skill);
					::mrta_vc::Skill toMsg();

					bool operator==(const Skill& skill);
					bool operator!=(const Skill& skill);

				private:
					Resource resource_;
					SkillLevelEnum level_;
				};
			}
		}
	}
}		

#endif /* SKILL_H_ */
