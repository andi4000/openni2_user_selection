#include "userSelector.h"

UserSelector::UserSelector()
{
	m_pUserTracker = new nite::UserTracker;
	m_pHandTracker = new nite::HandTracker;
	
}

nite::Status UserSelector::init(int argc, char** argv)
{
	ros::init(argc, argv, "openni2_user_selection");
	m_pNodeHandle = new ros::NodeHandle();
	
	nite::Status rc;
	rc = nite::NiTE::initialize();
	if (rc != nite::STATUS_OK)
		return rc;
		
	rc = m_pHandTracker->create();
	if (rc != nite::STATUS_OK)
		return rc;
	
	rc = m_pUserTracker->create();
	if (rc != nite::STATUS_OK)
		return rc;
	
	return nite::STATUS_OK; 
}

UserSelector::~UserSelector()
{
	m_pHandTracker->destroy();
	m_pUserTracker->destroy();
	delete m_pHandTracker;
	delete m_pUserTracker;
	
	nite::NiTE::shutdown();
	ros::shutdown();
}

void UserSelector::updateUserState(const nite::UserData& user, unsigned long long ts)
{
	if (user.isNew())
	{
		ROS_INFO("User %d enters the scene", user.getId());
	}
/**	else if (user.isVisible())
	{
		ROS_INFO("User %d is visible", user.getId());
	}
	else if (!user.isVisible())
	{
		ROS_INFO("User %d is out of scene", user.getId());
	}*/
	else if (user.isLost())
	{
		ROS_INFO("User %d is lost", user.getId());
	}
}

nite::UserId UserSelector::getUserIdFromPixel(nite::Point3f position, const nite::UserMap& userMap)
{
	//TODO: implement this!
	// - find out on point position, which user id is that
	// check UserViewer/Viewer.cpp lines 386++
	float x,y;
	m_pUserTracker->convertJointCoordinatesToDepth(position.x, position.y, position.z, &x, &y);
	ROS_INFO("Rx = %.2f Ry = %.2f Rz = %.2f", position.x, position.y, position.z);
	ROS_INFO("x = %.2f y = %.2f", x, y);
	
	const nite::UserId* pLabels = userMap.getPixels();
	
	
	
	for (int i = 0; i < 300000; ++i, ++pLabels){
		// this is stupid
		if (*pLabels == 1)
		{
			ROS_INFO("i= %d", i);
			break;
		}
		
	}
	ROS_INFO("pLabel = %d", *pLabels);
	
	return 0;
}

void UserSelector::run()
{
	nite::UserId activeUser;
	
	nite::HandTrackerFrameRef handTrackerFrame;
	nite::UserTrackerFrameRef userTrackerFrame;
	nite::Status rc;
	
	ROS_INFO("Start moving around to get detected!");
	ROS_INFO("or start waving to be tracked!");
	
	m_pHandTracker->startGestureDetection(nite::GESTURE_WAVE);
	
	while(ros::ok())
	{
		rc = m_pHandTracker->readFrame(&handTrackerFrame);
		if (rc != nite::STATUS_OK)
		{
			ROS_ERROR("Failed to get next frame!");
			continue;
		}
		
		rc = m_pUserTracker->readFrame(&userTrackerFrame);
		if (rc != nite::STATUS_OK)
		{
			ROS_ERROR("Failed to get next frame!");
			continue;
		}
		
		const nite::Array<nite::GestureData>& gestures = handTrackerFrame.getGestures();
		for (int i = 0; i < gestures.getSize(); ++i)
		{
			if (gestures[i].isInProgress())
			{
				ROS_INFO("Gesture detected!");
			}
			else if (gestures[i].isComplete())
			{
				activeUser = getUserIdFromPixel(gestures[i].getCurrentPosition(), userTrackerFrame.getUserMap());
				ROS_INFO("Gesture completed, now tracking User %d", activeUser);
				//todo: stop detecting
			}
		}
		
		const nite::Array<nite::UserData>& users = userTrackerFrame.getUsers();

		//const nite::UserMap& userLabels = userTrackerFrame.getUserMap();
		//const nite::UserId* pLabels = userLabels.getPixels();
		
		for (int i = 0; i < users.getSize(); ++i)
		{
			const nite::UserData& user = users[i];
			updateUserState(user, userTrackerFrame.getTimestamp());
		}
	}
}
