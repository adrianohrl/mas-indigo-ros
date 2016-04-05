/**
 *  Colors.h
 *
 *  Version: 1.0.0.0
 *  Created on: 30/03/2016
 *  Modified on: ****
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef SKILL_LEVEL_ENUM_H_
#define SKILL_LEVEL_ENUM_H_

#include <string>
#include <vector>

namespace colors
{

	typedef enum 
	{
		ORANGE, 
		YELLOW, 
		BLUE, 
		GREEN, 
		RED, 
		BLACK, 
		PINK,
		PURPLE,
		NONE
	} ColorEnum;

	class Colors
	{

	public:

		static ColorEnum toColor(int color_code);
		static int toCode(ColorEnum color);
		static std::string toString(ColorEnum color);
		static ColorEnum getDefault();
		static std::vector<ColorEnum> getAll();

	};

};

typedef colors::ColorEnum Color;
typedef colors::Colors Colors;

#endif /* SKILL_LEVEL_ENUM_H_ */
