#include "userSelector.h"

int main(int argc, char** argv)
{
	nite::Status rc = nite::STATUS_OK;
	
	UserSelector userSelector;
	rc = userSelector.init(argc, argv);
	if (rc != nite::STATUS_OK)
	{
		ROS_ERROR("Couldn't create user tracker!");
		return 1;
	}
	
	userSelector.run();
	
	return 0;
}
