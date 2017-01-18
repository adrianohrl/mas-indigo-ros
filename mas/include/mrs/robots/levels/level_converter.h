/**
 *  This header file defines the Level Converter class, which is based on
 *  the Enum Converter Utility class.
 *
 *  Version: 1.4.0
 *  Created on: 16/09/2016
 *  Modified on: 14/12/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _CAPABILITY_LEVEL_CONVERTER_H_
#define _CAPABILITY_LEVEL_CONVERTER_H_

#include "utilities/enum_converter.h"
#include "mrs/robots/levels/level.h"

namespace mrs
{

  namespace robots
	{

    namespace levels
		{

      class LevelConverter : public utilities::EnumConverter<Level>
			{
			public:
        LevelConverter(int code);
        LevelConverter(std::string name);
        LevelConverter(Level level);
        virtual ~LevelConverter();
        using EnumConverter::getEnumerated;
        virtual Level getEnumerated(int code) const;
        virtual Level getEnumerated(std::string name) const;
        using EnumConverter::str;
        virtual std::string str(Level level) const;
        using EnumConverter::c_str;
        virtual const char* c_str(Level level) const;

        static Level toEnumerated(int code);
        static Level toEnumerated(std::string name);
        static std::string toString(Level level);
        static const char* toCString(Level level);
        static Level getDefault();
        static std::vector<Level> getAll();

        static Level getMinimum();
        static Level getMaximum();

			};

		}

    typedef typename levels::LevelConverter LevelConverter;

	}

}


#endif // _CAPABILITY_LEVEL_CONVERTER_H_
