#ifndef _USERSELECTOR_H_
#define _USERSELECTOR_H_

#include "ros/ros.h"
#include "NiTE.h"


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
		nite::Status init(int argc, char** argv);
		void run();
		void updateFrame();
		
	private:
		void updateUserState(const nite::UserData& user, unsigned long long ts);
		nite::UserId getUserIdFromPixel(nite::Point3f position, const nite::UserMap& userMap);
		
		nite::UserTracker* m_pUserTracker;
		nite::HandTracker* m_pHandTracker;
		ros::NodeHandle* m_pNodeHandle;
		nite::UserId m_activeUserId;
};

#endif
