#include "userSelector.h"

/**
 * TODO:
 * - GUI toggle goes here
 */
//#define USE_GUI
#ifdef USE_GUI
	#include "userViewer.h"
#endif

int main(int argc, char** argv)
{
	nite::Status rc = nite::STATUS_OK;
		
	UserSelector *userSelector = new UserSelector();
	
	rc = userSelector->init(argc, argv);
	if (rc != nite::STATUS_OK)
	{
		ROS_ERROR("Couldn't create user tracker!");
		return 1;
	}

#ifdef USE_GUI
	UserViewer *userViewer = new UserViewer();
	rc = userViewer->init(argc, argv);
	if (rc != nite::STATUS_OK)
	{
		ROS_ERROR("Couldn't initialize GUI");
		return 1;
	}
	userViewer->run(userSelector);
#else
	userSelector->run();
#endif
	
	
	delete userSelector;
	
#ifdef USE_GUI
	delete viewer;
#endif

	return 0;
}
