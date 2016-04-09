/**
 *  Desk.h
 *
 *  Version: 1.0.0.0
 *  Created on: 02/04/2016
 *  Modified on: *********
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef DESK_H_
#define DESK_H_

#include "unifei/expertinos/mrta_vc/places/Office.h"


namespace unifei 
{
	namespace expertinos
	{
		namespace mrta_vc
		{
			namespace places
			{
				class Desk : public Place
				{

				public:
					Desk(int id, std::string name, Office office, geometry_msgs::Polygon boundary, double x, double y, double theta = 0);
					Desk(int id, std::string name, Office office, geometry_msgs::Polygon boundary, geometry_msgs::Pose pose_msg);
					Desk(const ::mrta_vc::Place::ConstPtr& place_msg);
					Desk(::mrta_vc::Place place_msg);		
					~Desk();

					Office getOffice();
					::mrta_vc::Place toMsg();
					void operator=(const Desk& desk);

				protected:
					int getType();

				private:
					Office office_;

				};
			}
		}
	}
}		
					
#endif /* DESK_H_ */
