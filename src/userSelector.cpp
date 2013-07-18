#include "userSelector.h"

/**
 * TODO:
 * - GUI
 * - stop tracking pose/gesture
 * - wrap activeUser object/variable
 * - review the whole scenario
 */

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
		ROS_WARN("User %d is lost", user.getId());
	}
	
	if (user.getId() == m_activeUserId && user.isLost())
	{
		ROS_WARN("Active user is lost");
		m_activeUserId = 0;
	}
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
	m_pHandTracker->startGestureDetection(nite::GESTURE_HAND_RAISE);
	
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

		for (int i = 0; i < users.getSize(); ++i)
		{
			const nite::UserData& user = users[i];
			updateUserState(user, userTrackerFrame.getTimestamp());
		}

		if (users.getSize() != 0){
	
			if (bSwitch){
				ROS_INFO("start waving to be tracked!");
				bSwitch = !bSwitch;
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
					if (gestures[i].getType() == nite::GESTURE_WAVE){
						gesturingUser = getUserIdFromPixel(gestures[i].getCurrentPosition(), userTrackerFrame.getUserMap());
						const nite::UserData* userTmp = userTrackerFrame.getUserById(gesturingUser);
						if (userTmp->getSkeleton().getState() != nite::SKELETON_TRACKED){
							m_activeUserId = gesturingUser;
							ROS_INFO("Gesture completed, now tracking User %d", m_activeUserId);
							m_pUserTracker->startSkeletonTracking(m_activeUserId);
							//m_pHandTracker->stopGestureDetection(nite::GESTURE_WAVE);
						}
					}
					
					/**
					 * TODO: exit pose, this seems redundant since wave starts with hand raise
					if (gestures[i].getType() == nite::GESTURE_HAND_RAISE){
						gesturingUser = getUserIdFromPixel(gestures[i].getCurrentPosition(), userTrackerFrame.getUserMap());
						if (gesturingUser == m_activeUserId){
							m_pUserTracker->stopSkeletonTracking(gesturingUser);
							m_activeUserId = 0;
						}
					}
					*/
				}
			}
			
			if (m_activeUserId != 0)
			{
				const nite::UserData* userTmp = userTrackerFrame.getUserById(m_activeUserId);
				if (userTmp->isVisible()){
					if (userTmp->getSkeleton().getState() == nite::SKELETON_TRACKED){
						const nite::SkeletonJoint& torso = userTmp->getSkeleton().getJoint(nite::JOINT_TORSO);
						if (torso.getPositionConfidence() > .5)
							ROS_INFO("Torso_%d: %.2f %.2f %.2f", m_activeUserId, torso.getPosition().x, torso.getPosition().y, torso.getPosition().z);
						else
							ROS_INFO("pos confidence %.2f", torso.getPositionConfidence());
					} else {
						ROS_INFO("still tracking..");
					}
				} else {
					//what?
				}
			}
		}
	}
}
// woot, nasty brackets!
