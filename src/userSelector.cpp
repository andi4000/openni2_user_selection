/**
 * TODO:
 * - fix joint rotation publishing
 * - also publish joint position in camera coordinate system
 * 
 * NOTES:
 * - foot joint (left and right) wont be exist if it's not visible in camera FOV, hence ROS' tf will be NaN
 * 
 * - init should be called once. if GUI is used, pass the userSelector object to userViewer 
 * then initialize userSelector inside userViewer
 * 
 * DONE:
 * make this class as an object that can be run independently for text-based interface
 * then this object could be passed to the userViewer as backbone for the GUI
 * 
 */

#include "userSelector.h"

UserSelector::UserSelector()
{
	
	m_pUserTrackerFrame = new nite::UserTrackerFrameRef;
	m_pHandTrackerFrame = new nite::HandTrackerFrameRef;	
	
	m_pUserTracker = new nite::UserTracker;
	m_pHandTracker = new nite::HandTracker;	
}

nite::Status UserSelector::init(int argc, char** argv, openni::Device* pDevice)
{
	ros::init(argc, argv, "openni2_user_selection");
	m_pNodeHandle = new ros::NodeHandle();
	
	nite::Status rc;
	rc = nite::NiTE::initialize();
	if (rc != nite::STATUS_OK)
		return rc;
		
	rc = m_pHandTracker->create(pDevice);
	if (rc != nite::STATUS_OK)
		return rc;
	
	rc = m_pUserTracker->create(pDevice);
	if (rc != nite::STATUS_OK)
		return rc;
		
	m_activeUserId = 0;
	
	return nite::STATUS_OK; 
}

UserSelector::~UserSelector()
{
	delete m_pHandTrackerFrame;
	delete m_pUserTrackerFrame;
	
	m_pHandTracker->destroy();
	m_pUserTracker->destroy();
	delete m_pHandTracker;
	delete m_pUserTracker;
	
	nite::NiTE::shutdown();
	ros::shutdown();
}

bool g_visibleUsers[MAX_USERS] = {false};
nite::SkeletonState g_skeletonStates[MAX_USERS] = {nite::SKELETON_NONE};
char g_userStatusLabels[MAX_USERS][100] = {{0}};

char g_generalMessage[100] = {0};

#define USER_MESSAGE(msg) {\
	sprintf(g_userStatusLabels[user.getId()], "%s", msg);\
	ROS_INFO("User #%d: %s", user.getId(), msg);\
	}

void UserSelector::updateUserState(const nite::UserData& user, unsigned long long ts)
{
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
			USER_MESSAGE("Stopped tracking.");
			if (user.getId() == m_activeUserId)
			{
				m_activeUserId = 0;
				ROS_WARN("User #%d: No longer active", user.getId());
			}
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
			USER_MESSAGE("Calibration Failed!")
			break;
		}
	}
	
}

char* UserSelector::getUserStatusLabel(nite::UserId id)
{
	return g_userStatusLabels[id];
}

nite::UserId UserSelector::getUserIdFromPixel(nite::Point3f position, const nite::UserMap& userMap)
{
	float x,y;
	m_pUserTracker->convertJointCoordinatesToDepth(position.x, position.y, position.z, &x, &y);
	//ROS_INFO("Rx = %.2f Ry = %.2f Rz = %.2f", position.x, position.y, position.z);
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
	ROS_INFO("Start moving around to get detected!");
	while(ros::ok())
	{
		updateFrame();
		detectionRoutine();
	}
}

void UserSelector::updateFrame()
{
	nite::Status rc;

	rc = m_pHandTracker->readFrame(m_pHandTrackerFrame);
	if (rc != nite::STATUS_OK)
	{
		ROS_ERROR("Failed to get next frame!");
		//continue;
		return;
	}
	
	rc = m_pUserTracker->readFrame(m_pUserTrackerFrame);
	if (rc != nite::STATUS_OK)
	{
		ROS_ERROR("Failed to get next frame!");
		//continue;
		return;
	}
	
}

void UserSelector::detectionRoutine()
{
	bool bUseWaveAsFocus = false;
	// user detection routine
	m_pHandTracker->startGestureDetection(nite::GESTURE_WAVE);
	
	const nite::Array<nite::UserData>& users = m_pUserTrackerFrame->getUsers();
	nite::UserId gesturingUserId;
	
	if (users.getSize() > 0)
	{
		const nite::Array<nite::GestureData>& gestures = m_pHandTrackerFrame->getGestures();
		
		for (int i = 0; i < gestures.getSize(); ++i)
		{
			if (gestures[i].isInProgress())
			{
				ROS_INFO("Gesture detected!");
			}
			else if (gestures[i].isComplete() && gestures[i].getType() == nite::GESTURE_WAVE)
			{

				gesturingUserId = getUserIdFromPixel(gestures[i].getCurrentPosition(), m_pUserTrackerFrame->getUserMap());
				ROS_INFO("gesture finished from id = %d", gesturingUserId);
				
				const nite::UserData* userTmp = m_pUserTrackerFrame->getUserById(gesturingUserId);
					
				if (bUseWaveAsFocus && m_activeUserId == 0 && userTmp->getSkeleton().getState() == nite::SKELETON_NONE)
				{
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

	//TODO: need to redesign this part, how can we pass info about which user whose skeleton we draw 
	for (int i = 0; i < users.getSize(); ++i)
	{
		const nite::UserData& user = users[i];
		updateUserState(user, m_pUserTrackerFrame->getTimestamp());
		
		if (user.isNew())
		{
			// things here will be called only once for each user
			if (!bUseWaveAsFocus)
				m_pUserTracker->startPoseDetection(user.getId(), nite::POSE_PSI);
				// this will start pose detection on every user available on scene
				// just in case there's some user that wants to "transfer" control to another user
				
				//TODO: should we stop pose detection once the user waves?
		}
		else if (!user.isLost())
		{
			// things here for when user is present on the FOV
			//DrawStatusLabel(m_pUserTracker, user);
			
			if (users[i].getSkeleton().getState() == nite::SKELETON_TRACKED)
				publishTransforms(user, "openni_depth_frame");
		}
		
		if (!bUseWaveAsFocus && m_activeUserId == 0)
		{
			const nite::PoseData& pose = user.getPose(nite::POSE_PSI);
			if (pose.isEntered()) //TODO: would this be isEntered or isHeld? isEntered is fine and fast until now
			{
				ROS_INFO("User #%d: Psi pose detected!", user.getId());
				m_activeUserId = user.getId();
				m_pUserTracker->startSkeletonTracking(user.getId());
				
			}
		}
	}
}

nite::UserTrackerFrameRef* UserSelector::getUserTrackerFrame()
{
	return m_pUserTrackerFrame;
}

nite::HandTrackerFrameRef* UserSelector::getHandTrackerFrame()
{
	return m_pHandTrackerFrame;
}

nite::UserTracker* UserSelector::getUserTracker()
{
	return m_pUserTracker;
}

nite::HandTracker* UserSelector::getHandTracker()
{
	return m_pHandTracker;
}

void UserSelector::publishTransform(const nite::UserData& user, nite::JointType jointType, const std::string& frame_id, const std::string& child_frame_id)
{
	//NOTE: this is real world coordinate system (in meter, hence the division by 1000)
	static tf::TransformBroadcaster br;
	
	const nite::SkeletonJoint& joint = user.getSkeleton().getJoint(jointType);
	float x = joint.getPosition().x / 1000.0;
	float y = joint.getPosition().y / 1000.0;
	float z = joint.getPosition().z / 1000.0;
	
	//ROS_INFO("%s x, y, z = %.2f, %.2f, %.2f", child_frame_id.c_str(), x, y, z);
	//TODO: finish this
	// - get orientation in quarternion --> unlike OpenNI 1.5, in NiTE2, orientation is already represented in quaternion
	// - check orientation values and sign! compare with result from OpenNI1
	// - publish 
	
	float qx = joint.getOrientation().x;
	float qy = joint.getOrientation().y;
	float qz = joint.getOrientation().z;
	float qw = joint.getOrientation().w;
	
	//ROS_INFO("%s (qx, qy, qz, qw) = %.4f, %.4f, %.4f", child_frame_id.c_str(), qx, qy, qz, qw);
	
	char child_frame_no[128];
	//snprintf(child_frame_no, sizeof(child_frame_no), "%s_%d", child_frame_id.c_str(), user.getId());
	snprintf(child_frame_no, sizeof(child_frame_no), "%s", child_frame_id.c_str());
	// we will only publish skeleton of one user, the active user
	
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(x, y, z));
	//transform.setRotation(tf::Quaternion(qx, -qy, -qz, qw));
	transform.setRotation(tf::Quaternion(1,1,1,1));
	
	//TODO: this is faulty! fix!
	// from ROS user tracker
	/**
	tf::Transform change_frame;
	change_frame.setOrigin(tf::Vector3(0, 0, 0));
	tf::Quaternion frame_rotation;
	frame_rotation.setEulerZYX(1.5708, 0, 1.5708); // 90° in radian
	change_frame.setRotation(frame_rotation);
	
	transform = change_frame * transform;
	*/
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_id, child_frame_no));
	
	
}

void UserSelector::publishTransforms(const nite::UserData& user, const std::string& frame_id)
{
	publishTransform(user, nite::JOINT_HEAD, frame_id, "head");
	publishTransform(user, nite::JOINT_NECK, frame_id, "neck");
	publishTransform(user, nite::JOINT_TORSO, frame_id, "torso");
	
	publishTransform(user, nite::JOINT_LEFT_SHOULDER, frame_id, "left_shoulder");
	publishTransform(user, nite::JOINT_LEFT_ELBOW, frame_id, "left_elbow");
	publishTransform(user, nite::JOINT_LEFT_HAND, frame_id, "left_hand");

	publishTransform(user, nite::JOINT_LEFT_HIP, frame_id, "left_hip");
	publishTransform(user, nite::JOINT_LEFT_KNEE, frame_id, "left_knee");
	publishTransform(user, nite::JOINT_LEFT_FOOT, frame_id, "left_foot");
	
	publishTransform(user, nite::JOINT_RIGHT_SHOULDER, frame_id, "right_shoulder");
	publishTransform(user, nite::JOINT_RIGHT_ELBOW, frame_id, "right_elbow");
	publishTransform(user, nite::JOINT_RIGHT_HAND, frame_id, "right_hand");
	
	publishTransform(user, nite::JOINT_RIGHT_HIP, frame_id, "right_hip");
	publishTransform(user, nite::JOINT_RIGHT_KNEE, frame_id, "right_knee");
	publishTransform(user, nite::JOINT_RIGHT_FOOT, frame_id, "right_foot");
}
