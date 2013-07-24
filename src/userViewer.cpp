
#include "userViewer.h"
#include <GL/glut.h>
#include "NiteSampleUtilities.h"

#define GL_WIN_SIZE_X 1280
#define GL_WIN_SIZE_Y 1024
#define TEXTURE_SIZE 512

#define DEFAULT_DISPLAY_MODE	DISPLAY_MODE_DEPTH

#define MIN_NUM_CHUNKS(data_size, chunk_size)	((((data_size)-1) / (chunk_size) + 1))
#define MIN_CHUNKS_SIZE(data_size, chunk_size)	(MIN_NUM_CHUNKS(data_size, chunk_size) * (chunk_size))

UserViewer* UserViewer::ms_self = NULL;

int g_nXRes = 0, g_nYRes = 0;

void UserViewer::glutIdle()
{
	glutPostRedisplay();
}

void UserViewer::glutDisplay()
{
	UserViewer::ms_self->Display();
}

UserViewer::UserViewer(const char* strName)
{
	ms_self = this;
	strncpy(m_strSampleName, strName, ONI_MAX_STR);
	m_pUserTracker = new nite::UserTracker;
	m_pHandTracker = new nite::HandTracker;
}

UserViewer::~UserViewer()
{
	Finalize();
	
	delete[] m_pTexMap;
	
	ms_self = NULL;
}

void UserViewer::Finalize()
{
	m_pHandTracker->destroy();
	m_pUserTracker->destroy();

	delete m_pHandTracker;
	delete m_pUserTracker;
	
	nite::NiTE::shutdown();
	openni::OpenNI::shutdown();
	ros::shutdown();	
}

openni::Status UserViewer::init(int argc, char** argv)
{
	m_pTexMap = NULL;
	ros::init(argc, argv, "openni2_user_selection");
	m_pNodeHandle = new ros::NodeHandle();
	
	openni::Status rc = openni::OpenNI::initialize();
	if (rc != openni::STATUS_OK)
	{
		ROS_ERROR("Failed to initialize OpenNI: %s", openni::OpenNI::getExtendedError());
		return rc;
	}
	
	const char* deviceUri = openni::ANY_DEVICE;
	for (int i = 1; i < argc-1; ++i)
	{
		if (strcmp(argv[i], "-device") == 0)
		{
			deviceUri = argv[i+1];
			break;
		}
	}
	
	rc = m_device.open(deviceUri);
	if (rc != openni::STATUS_OK)
	{
		ROS_ERROR("Failed to open device: %s", openni::OpenNI::getExtendedError());
		return rc;
	}
	
	nite::NiTE::initialize();
	
	if (m_pUserTracker->create(&m_device) != nite::STATUS_OK)
	{
		ROS_ERROR("Failed to create User Tracker");
		return openni::STATUS_ERROR;
	}
	
	if (m_pHandTracker->create(&m_device) != nite::STATUS_OK)
	{
		ROS_ERROR("Failed to create Hand Tracker");
		return openni::STATUS_ERROR;
	}
	
	m_activeUserId = 0;
	
	return InitOpenGL(argc, argv);
}

openni::Status UserViewer::run()
{
	glutMainLoop();
	return openni::STATUS_OK;
}

float Colors[][3] = {{1, 0, 0}, {0, 1, 0}, {1, 1, 1}};
int colorCount = 3;

#define MAX_USERS 10
bool g_visibleUsers[MAX_USERS] = {false};
nite::SkeletonState g_skeletonStates[MAX_USERS] = {nite::SKELETON_NONE};
char g_userStatusLabels[MAX_USERS][100] = {{0}};

char g_generalMessage[100] = {0};

#define USER_MESSAGE(msg) {\
	sprintf(g_userStatusLabels[user.getId()], "%s", msg);\
	ROS_INFO("User #%d:\t%s", user.getId(), msg);\
	}

//TODO: continue here
void UserViewer::updateUserState(const nite::UserData& user, unsigned long long ts)
{
	
}


void UserViewer::Display()
{
	
}


openni::Status UserViewer::InitOpenGL(int argc, char** argv)
{
	
	return openni::STATUS_OK;
}

void UserViewer::InitOpenGLHooks()
{
	
}

nite::UserId UserViewer::getUserIdFromPixel(nite::Point3f position, const nite::UserMap& pUserMap)
{
	nite::UserId userId;
	return userId;
}
