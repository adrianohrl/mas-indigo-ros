/**
 *  This source file is used to test the CapabilityParser class.
 *
 *  Version: 1.4.0
 *  Created on: 01/04/2016
 *  Modified on: 16/12/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include <iostream>
#include <utilities/capability_expression_evaluator/capability_parser.h>

using namespace mrs::robots::levels;
using typename mrs::robots::Resource;
using typename mrs::robots::Capability;
using typename utilities::capability_expression_evaluator::CapabilityParser;

int main(int argc, char** argv)
{
  std::vector<std::string> expressions;
  expressions.push_back("velocity < 2");
  expressions.push_back("velocity <= 2");
  expressions.push_back("velocity == 2");
  expressions.push_back("velocity >= 2");
  expressions.push_back("velocity > 2");
  expressions.push_back("2 < velocity < 4");
  expressions.push_back("1 < velocity <= 4");
  expressions.push_back("1 <= velocity < 4");
  expressions.push_back("2 <= velocity <= 3");
  expressions.push_back("3 < velocity < 4"); // invalid
  expressions.push_back("2 < velocity < 1"); // invalid
  expressions.push_back("velocity <");
  expressions.push_back("velocity <=");
  expressions.push_back("velocity ==");
  expressions.push_back("velocity >=");
  expressions.push_back("velocity >");
  expressions.push_back("< velocity <");
  expressions.push_back("< velocity <=");
  expressions.push_back("<= velocity <");
  expressions.push_back("<= velocity <=");

  std::vector<Capability> capabilities;
  capabilities.push_back(Capability(1, Resource("velocity"), LOW, LOW));              // velocity < 2
  capabilities.push_back(Capability(2, Resource("velocity"), LOW, MODERATE));         // velocity <= 2
  capabilities.push_back(Capability(3, Resource("velocity"), MODERATE, MODERATE));    // velocity == 2
  capabilities.push_back(Capability(4, Resource("velocity"), MODERATE, RESOURCEFUL)); // velocity >= 2
  capabilities.push_back(Capability(4, Resource("velocity"), HIGH, RESOURCEFUL));     // velocity > 2
  capabilities.push_back(Capability(5, Resource("velocity"), HIGH, HIGH));            // 2 < velocity < 4
  capabilities.push_back(Capability(6, Resource("velocity"), MODERATE, RESOURCEFUL)); // 1 < velocity <= 4
  capabilities.push_back(Capability(7, Resource("velocity"), LOW, HIGH));             // 1 <= velocity < 4
  capabilities.push_back(Capability(8, Resource("velocity"), MODERATE, HIGH));        // 2 <= velocity <= 3
  capabilities.push_back(Capability(0, Resource(""), NONE, NONE));                    // invalid
  capabilities.push_back(Capability(0, Resource(""), NONE, NONE));                    // invalid
  capabilities.push_back(Capability(0, Resource(""), NONE, NONE));                    // invalid
  capabilities.push_back(Capability(0, Resource(""), NONE, NONE));                    // invalid
  capabilities.push_back(Capability(0, Resource(""), NONE, NONE));                    // invalid
  capabilities.push_back(Capability(0, Resource(""), NONE, NONE));                    // invalid
  capabilities.push_back(Capability(0, Resource(""), NONE, NONE));                    // invalid
  capabilities.push_back(Capability(0, Resource(""), NONE, NONE));                    // invalid
  capabilities.push_back(Capability(0, Resource(""), NONE, NONE));                    // invalid

  if (expressions.size() != capabilities.size())
  {
    std::cout << "\nNumber of expressions must be equal to the number of capabilities!!!\n\nQuiting...";
  }

  CapabilityParser parser;
  std::cout << "\nBegining test capability parser!!!";
  for (int i = 0; i < expressions.size(); i++)
  {
    std::cout << "\n\nTesting: " + capabilities[i].str() + "  ---->  " + expressions[i] + "";
    Capability *capability = parser.parse(expressions[i]);
    std::cout << "\nCapability comparison:    " << (capability && capabilities[i] == *capability ? "OK" : "NOT OK");
    std::string expression = parser.str(capabilities[i]);
    std::cout << "\nString comparison:        " << (expression != "" && expressions[i] == expression ? "OK" : "NOT OK");
    std::cout << "\nexpression: " + expression;
    delete capability;
    capability = NULL;
  }
  std::cout << "\n\nEnd of test!!!\n";
  return 0;
}
