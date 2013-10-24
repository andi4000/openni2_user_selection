#ifndef _USERSELECTOR_H_
#define _USERSELECTOR_H_

#include "ros/ros.h"
#include "ros/package.h"
#include "tf/transform_broadcaster.h"
#include "kdl/frames.hpp"

#include "std_msgs/Bool.h"

#include "NiTE.h"

#define MAX_USERS 10

class UserSelector
{
	public:		
		enum SceneStatus
		{
			SCENE_EMPTY,
			SCENE_NOT_EMPTY,
			ACTIVE_USER_PRESENT
			//TODO: elaborate more on tis
		};
		UserSelector();
		~UserSelector();
		nite::Status init(int argc, char** argv, openni::Device* pDevice = NULL);
		void run();
		void updateFrame();
		void detectionRoutine();
		
		char* getUserStatusLabel(nite::UserId id);
		
		nite::UserTrackerFrameRef* getUserTrackerFrame();
		nite::HandTrackerFrameRef* getHandTrackerFrame();
		
		nite::UserTracker* getUserTracker();
		nite::HandTracker* getHandTracker();
		
		void publishTransform(const nite::UserData& user, nite::JointType jointType, const std::string& frame_id, const std::string& child_frame_id);
		void publishTransforms(const nite::UserData& user, const std::string& frame_id);
	private:
		void updateUserState(const nite::UserData& user, unsigned long long ts);
		nite::UserId getUserIdFromPixel(nite::Point3f position, const nite::UserMap& userMap);
		
		nite::UserTrackerFrameRef* 		m_pUserTrackerFrame;
		nite::HandTrackerFrameRef* 		m_pHandTrackerFrame;
		
		nite::UserTracker* 				m_pUserTracker;
		nite::HandTracker* 				m_pHandTracker;
		
		ros::NodeHandle* 				m_pNodeHandle;
		ros::Rate*						m_pRate;
		ros::Publisher*					m_pPub_ActiveUserPresent;
		ros::Publisher*					m_pPub_ActiveUserVisible;
		nite::UserId					m_activeUserId;
		nite::UserId					m_gesturingUser;
};

#endif
