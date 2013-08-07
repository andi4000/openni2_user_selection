/**
 * NOTE:
 * 
 * TODO:
 * - change start method, from wave to psi pose
 * - review the whole scenario
 * - ROS joints publishing
 * 
 * DONE:
 * - wrap activeUser object/variable --> cancelled
 * - GUI
 * - stop tracking pose/gesture
 * - for now, GUI and text based are independently separated, switch by #define USE_GUI
 * 
 */

#include "userViewer.h"
#include "userSelector.h"

int main (int argc, char** argv)
{
	bool bGUI = true;
	
	//woot, can't use argv for gui switch
	//if (argc > 1 && strcmp(argv[2], "-gui") == 0)
	
	UserSelector *userSelector = new UserSelector();
	
	if (bGUI)
	{
		openni::Status rc = openni::STATUS_OK;
		
		UserViewer *userViewer = new UserViewer("User Viewer");
		rc = userViewer->init(argc, argv, userSelector);
		if (rc != openni::STATUS_OK)
		{
			ROS_ERROR("Couldn't init UserViewer in GUI mode!");
			return 1;
		}
		ROS_WARN("Starting User Selection GUI");
		userViewer->run(); // loop until exit
		
		delete userViewer; // just to be safe
	}
	else
	{
		nite::Status rc = nite::STATUS_OK;
		rc = userSelector->init(argc, argv);
		if (rc != nite::STATUS_OK)
		{
			ROS_ERROR("Couldn't init UserSelector!");
			return 1;
		}
		ROS_WARN("Starting User Selection in text mode..");		
		userSelector->run(); // loop until SIGINT
	}
	
	delete userSelector; // just to be safe
	
	return 0;
}
