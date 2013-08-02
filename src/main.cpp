/**
 * NOTE:
 * - for now, GUI and text based are independently separated, switch by #define USE_GUI
 * - need to learn more about OpenGL
 * 
 * TODO:
 * - GUI
 * - stop tracking pose/gesture
 * - wrap activeUser object/variable
 * - review the whole scenario
 * - ROS joints publishing
 */

#define USE_GUI
#ifdef USE_GUI
	#include "userViewer.h"
#else
	#include "userSelector.h"
#endif

int main(int argc, char** argv)
{

#ifdef USE_GUI
	openni::Status rc = openni::STATUS_OK;
	
	UserViewer *userViewer = new UserViewer("User Viewer");
	rc = userViewer->init(argc, argv);
	if (rc != openni::STATUS_OK)
	{
		ROS_ERROR("Couldn't initialize GUI");
		return 1;
	}
	userViewer->run();
	
	delete userViewer;
#else
	nite::Status rc = nite::STATUS_OK;

	UserSelector *userSelector = new UserSelector();
	
	rc = userSelector->init(argc, argv);
	if (rc != nite::STATUS_OK)
	{
		ROS_ERROR("Couldn't create user tracker!");
		return 1;
	}
	
	userSelector->run(); // loop until SIGINT

	delete userSelector;
#endif
	return 0;
}
