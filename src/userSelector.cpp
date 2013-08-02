#include "userSelector.h"

#define MAX_USERS 10

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
		
	m_activeUserId = 0;
	
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
/**
#define MAX_USERS 10
bool g_visibleUsers[MAX_USERS] = {false};
nite::SkeletonState g_skeletonStates[MAX_USERS] = {nite::SKELETON_NONE};
char g_userStatusLabels[MAX_USERS][100] = {{0}};

char g_generalMessage[100] = {0};

#define USER_MESSAGE(msg) {\
	sprintf(g_userStatusLabels[user.getId()], "%s", msg);\
	ROS_INFO("User #%d: %s", user.getId(), msg);\
	}
*/
//TODO: multiple global var declaration, fix!

void UserSelector::updateUserState(const nite::UserData& user, unsigned long long ts)
{
	//TODO: fix this
	/**
	if (user.isNew())
	{
		USER_MESSAGE("New");
	}
	else if (user.isVisible() && !g_visibleUsers[user.getId()])
		ROS_INFO("User #%d: Visible", user.getId());
	else if (!user.isVisible() && g_visibleUsers[user.getId()])
		ROS_INFO("User #%d: Out of scene", user.getId());
	else if (user.isLost())
	{
		USER_MESSAGE("Lost");
		if (user.getId() == m_activeUserId)
		{
			m_activeUserId = 0;
			ROS_WARN("User #%d: Lost, no longer active", user.getId());
		}
	}	
	g_visibleUsers[user.getId()] = user.isVisible();
	

	if(g_skeletonStates[user.getId()] != user.getSkeleton().getState())
	{
		switch(g_skeletonStates[user.getId()] = user.getSkeleton().getState())
		{
		case nite::SKELETON_NONE:
			USER_MESSAGE("Stopped tracking.")
			break;
		case nite::SKELETON_CALIBRATING:
			USER_MESSAGE("Calibrating...")
			break;
		case nite::SKELETON_TRACKED:
			USER_MESSAGE("Tracking!")
			break;
		case nite::SKELETON_CALIBRATION_ERROR_NOT_IN_POSE:
		case nite::SKELETON_CALIBRATION_ERROR_HANDS:
		case nite::SKELETON_CALIBRATION_ERROR_LEGS:
		case nite::SKELETON_CALIBRATION_ERROR_HEAD:
		case nite::SKELETON_CALIBRATION_ERROR_TORSO:
			USER_MESSAGE("Calibration Failed... :-|")
			break;
		}
	}
	*/
}

nite::UserId UserSelector::getUserIdFromPixel(nite::Point3f position, const nite::UserMap& userMap)
{
	float x,y;
	m_pUserTracker->convertJointCoordinatesToDepth(position.x, position.y, position.z, &x, &y);
	ROS_INFO("Rx = %.2f Ry = %.2f Rz = %.2f", position.x, position.y, position.z);
	const nite::UserId* pLabels = userMap.getPixels();
	nite::UserId userId = 0;
	
	for (int i = 0; i < userMap.getHeight(); ++i){
		for (int j = 0; j < userMap.getWidth(); ++j, ++pLabels){
			if (i == (int)y && j == (int)x)
				userId = *pLabels;
		}
	}

	return userId;
}

void UserSelector::run()
{
	nite::UserId gesturingUser = 0;
	
	nite::HandTrackerFrameRef handTrackerFrame;
	nite::UserTrackerFrameRef userTrackerFrame;
	nite::Status rc;
	
	ROS_INFO("Start moving around to get detected!");
	m_pHandTracker->startGestureDetection(nite::GESTURE_WAVE);
	//m_pHandTracker->startGestureDetection(nite::GESTURE_HAND_RAISE);
	
	bool bSwitch = true;

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
		
		const nite::Array<nite::UserData>& users = userTrackerFrame.getUsers();

		nite::UserId gesturingUserId;
		
		if (users.getSize() > 0)
		{
			const nite::Array<nite::GestureData>& gestures = handTrackerFrame.getGestures();
			
			for (int i = 0; i < gestures.getSize(); ++i)
			{
				if (gestures[i].isInProgress())
				{
					ROS_INFO("Gesture detected!");
				}
				else if (gestures[i].isComplete() && gestures[i].getType() == nite::GESTURE_WAVE)
				{

					gesturingUserId = getUserIdFromPixel(gestures[i].getCurrentPosition(), userTrackerFrame.getUserMap());
					ROS_INFO("gesture finished from id = %d", gesturingUserId);
					
					const nite::UserData* userTmp = userTrackerFrame.getUserById(gesturingUserId);
						
					if (m_activeUserId == 0 && userTmp->getSkeleton().getState() == nite::SKELETON_NONE)
					{
						//TODO: for above, is it better if != nite::SKELETON_TRACKED?
						m_activeUserId = gesturingUserId;
						ROS_WARN("User #%d: Now active", m_activeUserId);
						m_pUserTracker->startSkeletonTracking(m_activeUserId);
						//TODO: stopGestureDetection here?
					}
					else
					{
						ROS_WARN("not entering activation loop");
					}
					//ROS_INFO("active user id = %d", m_activeUserId);
					
					if (m_activeUserId != 0
								&& gesturingUserId == m_activeUserId 
								&& userTmp->getSkeleton().getState() == nite::SKELETON_TRACKED)
					{
						ROS_WARN("User #%d: No longer active", m_activeUserId);
						m_pUserTracker->stopSkeletonTracking(m_activeUserId);
						m_activeUserId = 0;
					}
					else
					{
						ROS_WARN("not entering the exit loop");
					}
				}
			}
			
			//TODO: exit pose
		}

		for (int i = 0; i < users.getSize(); ++i)
		{
			const nite::UserData& user = users[i];
			updateUserState(user, userTrackerFrame.getTimestamp());
			
			if (user.isNew())
			{
				// things here will be called only once for each user
				//TODO: maybe put startGestureDetection here? --> not a good idea, it will call it for each user
			}
			else if (!user.isLost())
			{
				// things here for when user is present on the FOV
				//DrawStatusLabel(m_pUserTracker, user);
				
				//if (users[i].getSkeleton().getState() == nite::SKELETON_TRACKED)
					//DrawSkeleton(m_pUserTracker, user);
			}
		}
		

	}
}
// woot, nasty brackets!

void UserSelector::updateFrame()
{
	
}
