
#ifndef _USERSELECTOR_VIEWER_H_
#define _USERSELECTOR_VIEWER_H_

#include "ros/ros.h"
#include "NiTE.h"

#define MAX_DEPTH 10000

class UserViewer
{
public:
	UserViewer(const char* strName);
	virtual ~UserViewer();
	
	virtual openni::Status init(int argc, char** argv);
	virtual openni::Status run(); // does not return
protected:
	virtual void DisplayCallback();
	virtual void DisplayPostDraw(){};
	
	virtual void OnKey(unsigned char key, int x, int y);
	
	virtual openni::Status InitOpenGL(int argc, char** argv);
	void InitOpenGLHooks();
	
	void Finalize();
	
private:
	UserViewer(const UserViewer&);
	UserViewer& operator=(UserViewer&);


	static UserViewer* 	ms_self;
		
	static void glutIdle();
	static void glutDisplay();
	static void glutKeyboard(unsigned char key, int x, int y);
	
	void updateUserState(const nite::UserData& user, unsigned long long ts);
	nite::UserId getUserIdFromPixel(nite::Point3f position, const nite::UserMap& userMap);
	
	float 					m_pDepthHist[MAX_DEPTH];
	char 					m_strSampleName[ONI_MAX_STR];
	openni::RGB888Pixel* 	m_pTexMap;
	unsigned int 			m_nTexMapX;
	unsigned int 			m_nTexMapY;
	
	openni::Device 			m_device;
	nite::UserTracker* 		m_pUserTracker;
	nite::HandTracker* 		m_pHandTracker;
	ros::NodeHandle*		m_pNodeHandle;
	nite::UserId			m_activeUserId;
	nite::UserId			m_gesturingUser;
	const nite::UserData*	m_pActiveUserData;
	
	nite::UserId			m_exitPosingUser;
	uint64_t				m_exitPoseTime;
};

#endif // _USERSELECTOR_VIEWER_H_
