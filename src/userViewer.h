
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
	virtual void Display();
	virtual void DisplayPostDraw(){};
	
	virtual openni::Status InitOpenGL(int argc, char** argv);
	void InitOpenGLHooks();
	
	void Finalize();
	
private:
	UserViewer(const UserViewer&);
	UserViewer& operator=(UserViewer&);
	
	static void glutIdle();
	static void glutDisplay();
	void updateUserState(const nite::UserData& user, unsigned long long ts);
	nite::UserId getUserIdFromPixel(nite::Point3f position, const nite::UserMap& pUserMap);

	static UserViewer* 	ms_self;
	
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
	
};

#endif // _USERSELECTOR_VIEWER_H_
