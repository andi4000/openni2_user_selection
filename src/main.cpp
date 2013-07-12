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

/**
#include "ros/ros.h"
#include "NiTE.h"

void updateUserState(const nite::UserData& user, unsigned long long ts)
{
	//if (user.isNew())
}

int main(int argc, char** argv){
	ros::init(argc, argv, "openni2_user_selection");
	ros::NodeHandle n;
	
	nite::UserTracker userTracker;
	nite::Status niteRc;
	
	nite::NiTE::initialize();
	
	niteRc = userTracker.create();
	if (niteRc != nite::STATUS_OK)
	{
		ROS_ERROR("Couldn't create user tracker!");
		return 3;
	}
	ROS_INFO("Start moving around to get detected!");
	
	nite::UserTrackerFrameRef userTrackerFrame;
	
	while(ros::ok())
	{
		niteRc = userTracker.readFrame(&userTrackerFrame);
		if (niteRc != nite::STATUS_OK)
		{
			ROS_ERROR("Failed to get next frame!");
			continue;
		}
		
		const nite::Array<nite::UserData>& users = userTrackerFrame.getUsers();
		for (int i = 0; i < users.getSize(); i++)
		{
			const nite::UserData& user = users[i];
			if (user.isNew())
			{
				ROS_INFO("User %d enters the scene", user.getId());
			}
			else if (user.isVisible())
			{
				ROS_INFO("User %d is visible", user.getId());
			}
			else if (!user.isVisible())
			{
				ROS_INFO("User %d is out of scene", user.getId());
			}
			else if (user.isLost())
			{
				ROS_INFO("User %d is lost", user.getId());
			}
		}
	}
	
	nite::NiTE::shutdown();
	ros::shutdown();
}
*/
