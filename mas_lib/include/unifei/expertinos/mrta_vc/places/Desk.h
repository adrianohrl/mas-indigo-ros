/**
 *  Desk.h
 *
 *  Version: 1.2.2
 *  Created on: 02/04/2016
 *  Modified on: 17/08/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef PLACES_DESK_H_
#define PLACES_DESK_H_

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
					Desk(int id, std::string name, Office office, geometry_msgs::Polygon boundary, double x, double y, double theta = 0.0);
					Desk(int id, std::string name, Office office, geometry_msgs::Polygon boundary, geometry_msgs::Pose pose_msg);
					Desk(const mas_msgs::Place::ConstPtr& place_msg);
					Desk(mas_msgs::Place place_msg);		
					virtual ~Desk();

					Office getOffice();
					mas_msgs::Place toMsg();
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
					
#endif /* PLACES_DESK_H_ */
