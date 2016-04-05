/**
 *  ExternalArea.h
 *
 *  Version: 1.0.0.0
 *  Created on: 02/04/2016
 *  Modified on: *********
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef EXTERNAL_AREA_H_
#define EXTERNAL_AREA_H_

#include "unifei/expertinos/mrta_vc/places/Campus.h"


namespace unifei 
{
	namespace expertinos
	{
		namespace mrta_vc
		{
			namespace places
			{
				class ExternalArea : public Place
				{

				public:
					ExternalArea(int id, std::string name, Campus campus, geometry_msgs::Polygon boundary, double x, double y, double theta = 0);
					ExternalArea(int id, std::string name, Campus campus, geometry_msgs::Polygon boundary, geometry_msgs::Pose pose_msg);
					ExternalArea(const ::mrta_vc::Place::ConstPtr& place_msg);
					ExternalArea(::mrta_vc::Place place_msg);		
					~ExternalArea();

					Campus getCampus();
					::mrta_vc::Place toMsg();

				protected:
					int getType();

				private:
					Campus campus_;

				};
			}
		}
	}
}		
					
#endif /* EXTERNAL_AREA_H_ */
