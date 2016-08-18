uint8 type
uint32 id
string name				# When agent's type is Computer or Robot, this correspond to its hostname.
uint8 hierarchy_level			# Only persons use this parameter.
string login_name			# Only users use this parameter.
mrs_msgs/Location location
bool mobile				# This paremeter is not useful when agent's type is Person or User, because they are always true.
bool holonomic				# This paremeter is only needed when agent's type is Robot.
geometry_msgs/Twist velocity		# Only robots have velocity.
uint32 computer_id			# Only if this is an user
string computer_hostname		# Only if this is an user
bool computer_mobile			# Only if this is an user
mrs_msgs/Location computer_location 	# Only if this is an user
mrs_msgs/Skill[] skills			# Only if this is a robot

# Types:
# -1 -> NON_AGENT
#  0 -> AGENT
#  1 -> PERSON
#  2 -> COMPUTER
#  3 -> ROBOT
#  4 -> VOICE_COMMANDER

# Hierarchy Levels:
#  0 -> VISITOR
#  1 -> INTERN
#  2 -> EMPLOYEE
#  3 -> SUPERVISOR
#  4 -> MIDDLE_MANAGER
#  5 -> SENIOR_MANAGER
#  6 -> CEO

