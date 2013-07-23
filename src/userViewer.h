
#ifndef _USERSELECTOR_VIEWER_H_
#define _USERSELECTOR_VIEWER_H_

#include "userSelector.h"

#define MAX_DEPTH 10000

class UserViewer
{
public:
	UserViewer(const char* strName);
	virtual ~UserViewer();
	
	virtual nite::Status init(int argc, char** argv);
	virtual nite::Status run();
protected:
	virtual void Display();
	virtual void DisplayPostDraw(){};
	
	virtual nite::Status InitOpenGL(int argc, char** argv);
	void InitOpenGLHooks();
	
	void Finalize();
	
private:
	UserViewer(const UserViewer&);
	UserViewer& operator=(UserViewer&);
	
	static UserViewer* ms_self;
	static void glutIdle();
	static void glutDisplay();
	
	float m_pDepthHist[MAX_DEPTH];
	char m_strSampleName[ONI_MAX_STR];
	openni::RGB888Pixel* m_pTexMap;
	unsigned int m_nTexMapX;
	unsigned int m_nTexMapY;
	
	openni::Device m_device;

	UserSelector* m_pUserSelector;
};

#endif // _USERSELECTOR_VIEWER_H_
