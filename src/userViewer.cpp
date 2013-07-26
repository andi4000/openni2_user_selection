/**
#if (defined _WIN32)
#define PRIu64 "llu"
#else
#define __STDC_FORMAT_MACROS
#include <inttypes.h>
#endif
*/

#include "userViewer.h"
#include <GL/glut.h>
#include "NiteSampleUtilities.h"

#define GL_WIN_SIZE_X 800
#define GL_WIN_SIZE_Y 600
#define TEXTURE_SIZE 512

#define DEFAULT_DISPLAY_MODE	DISPLAY_MODE_DEPTH

#define MIN_NUM_CHUNKS(data_size, chunk_size)	((((data_size)-1) / (chunk_size) + 1))
#define MIN_CHUNKS_SIZE(data_size, chunk_size)	(MIN_NUM_CHUNKS(data_size, chunk_size) * (chunk_size))

UserViewer* UserViewer::ms_self = NULL;

const int g_poseTimeoutToExit = 2000;

int g_nXRes = 0, g_nYRes = 0;

void UserViewer::glutIdle()
{
	glutPostRedisplay();
}

void UserViewer::glutDisplay()
{
	UserViewer::ms_self->DisplayCallback();
}

UserViewer::UserViewer(const char* strName)
{
	ms_self = this;
	strncpy(m_strSampleName, strName, ONI_MAX_STR);
	m_pUserTracker = new nite::UserTracker;
	m_pHandTracker = new nite::HandTracker;
	
	m_activeUserId = 0;
	m_exitPosingUser = 0;
	m_exitPoseTime = 0;
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
	
	
	return InitOpenGL(argc, argv);
}

openni::Status UserViewer::run()
{
	ROS_INFO("Start moving around to get detected");
	//TODO: need switch for this, so it can be put inside DisplayCallback
	m_pHandTracker->startGestureDetection(nite::GESTURE_WAVE);
	
	glutMainLoop();
	return openni::STATUS_OK;
}

float Colors[][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {1, 1, 1}};
int colorCount = 3;

#define MAX_USERS 10
bool g_visibleUsers[MAX_USERS] = {false};
nite::SkeletonState g_skeletonStates[MAX_USERS] = {nite::SKELETON_NONE};
char g_userStatusLabels[MAX_USERS][100] = {{0}};

char g_generalMessage[100] = {0};

#define USER_MESSAGE(msg) {\
	sprintf(g_userStatusLabels[user.getId()], "%s", msg);\
	ROS_INFO("User #%d: %s", user.getId(), msg);\
	}

void UserViewer::updateUserState(const nite::UserData& user, unsigned long long ts)
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
}

#ifndef USE_GLES
void glPrintString(void *font, const char *str)
{
	int i,l = (int)strlen(str);

	for(i=0; i<l; i++)
	{   
		glutBitmapCharacter(font,*str++);
	}   
}
#endif

void DrawStatusLabel(nite::UserTracker* pUserTracker, const nite::UserData& user)
{
	int color = user.getId() % colorCount;
	glColor3f(1.0f - Colors[color][0], 1.0f - Colors[color][1], 1.0f - Colors[color][2]);
	
	float x, y;
	pUserTracker->convertJointCoordinatesToDepth(user.getCenterOfMass().x, user.getCenterOfMass().y, user.getCenterOfMass().z, &x, &y);
	x *= GL_WIN_SIZE_X / (float)g_nXRes;
	y *= GL_WIN_SIZE_Y / (float)g_nYRes;
	
	char *msg = g_userStatusLabels[user.getId()];
	glRasterPos2i(x - ((strlen(msg)/2)*8), y);
	glPrintString(GLUT_BITMAP_HELVETICA_18, msg);
}

void DrawLimb(nite::UserTracker* pUserTracker, const nite::SkeletonJoint& joint1, const nite::SkeletonJoint& joint2, int color)
{
	float coordinates[6] = {0};
	pUserTracker->convertJointCoordinatesToDepth(joint1.getPosition().x, joint1.getPosition().y, joint1.getPosition().z, &coordinates[0], &coordinates[1]);
	pUserTracker->convertJointCoordinatesToDepth(joint2.getPosition().x, joint2.getPosition().y, joint2.getPosition().z, &coordinates[3], &coordinates[4]);

	coordinates[0] *= GL_WIN_SIZE_X/(float)g_nXRes;
	coordinates[1] *= GL_WIN_SIZE_Y/(float)g_nYRes;
	coordinates[3] *= GL_WIN_SIZE_X/(float)g_nXRes;
	coordinates[4] *= GL_WIN_SIZE_Y/(float)g_nYRes;

	if (joint1.getPositionConfidence() == 1 && joint2.getPositionConfidence() == 1)
	{
		glColor3f(1.0f - Colors[color][0], 1.0f - Colors[color][1], 1.0f - Colors[color][2]);
	}
	else if (joint1.getPositionConfidence() < 0.5f || joint2.getPositionConfidence() < 0.5f)
	{
		return;
	}
	else
	{
		glColor3f(.5, .5, .5);
	}
	glPointSize(2);
	glVertexPointer(3, GL_FLOAT, 0, coordinates);
	glDrawArrays(GL_LINES, 0, 2);

	glPointSize(10);
	if (joint1.getPositionConfidence() == 1)
	{
		glColor3f(1.0f - Colors[color][0], 1.0f - Colors[color][1], 1.0f - Colors[color][2]);
	}
	else
	{
		glColor3f(.5, .5, .5);
	}
	glVertexPointer(3, GL_FLOAT, 0, coordinates);
	glDrawArrays(GL_POINTS, 0, 1);

	if (joint2.getPositionConfidence() == 1)
	{
		glColor3f(1.0f - Colors[color][0], 1.0f - Colors[color][1], 1.0f - Colors[color][2]);
	}
	else
	{
		glColor3f(.5, .5, .5);
	}
	glVertexPointer(3, GL_FLOAT, 0, coordinates+3);
	glDrawArrays(GL_POINTS, 0, 1);
}

void DrawSkeleton(nite::UserTracker* pUserTracker, const nite::UserData& userData)
{
	DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_HEAD), userData.getSkeleton().getJoint(nite::JOINT_NECK), userData.getId() % colorCount);

	DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER), userData.getSkeleton().getJoint(nite::JOINT_LEFT_ELBOW), userData.getId() % colorCount);
	DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_LEFT_ELBOW), userData.getSkeleton().getJoint(nite::JOINT_LEFT_HAND), userData.getId() % colorCount);

	DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER), userData.getSkeleton().getJoint(nite::JOINT_RIGHT_ELBOW), userData.getId() % colorCount);
	DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_RIGHT_ELBOW), userData.getSkeleton().getJoint(nite::JOINT_RIGHT_HAND), userData.getId() % colorCount);

	DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER), userData.getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER), userData.getId() % colorCount);

	DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER), userData.getSkeleton().getJoint(nite::JOINT_TORSO), userData.getId() % colorCount);
	DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER), userData.getSkeleton().getJoint(nite::JOINT_TORSO), userData.getId() % colorCount);

	DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_TORSO), userData.getSkeleton().getJoint(nite::JOINT_LEFT_HIP), userData.getId() % colorCount);
	DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_TORSO), userData.getSkeleton().getJoint(nite::JOINT_RIGHT_HIP), userData.getId() % colorCount);

	DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_LEFT_HIP), userData.getSkeleton().getJoint(nite::JOINT_RIGHT_HIP), userData.getId() % colorCount);


	DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_LEFT_HIP), userData.getSkeleton().getJoint(nite::JOINT_LEFT_KNEE), userData.getId() % colorCount);
	DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_LEFT_KNEE), userData.getSkeleton().getJoint(nite::JOINT_LEFT_FOOT), userData.getId() % colorCount);

	DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_RIGHT_HIP), userData.getSkeleton().getJoint(nite::JOINT_RIGHT_KNEE), userData.getId() % colorCount);
	DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_RIGHT_KNEE), userData.getSkeleton().getJoint(nite::JOINT_RIGHT_FOOT), userData.getId() % colorCount);
}

nite::UserId UserViewer::getUserIdFromPixel(nite::Point3f position, const nite::UserMap& userMap)
{
	nite::UserId userId = 0;
	float x, y;
	m_pUserTracker->convertJointCoordinatesToDepth(position.x, position.y, position.z, &x, &y);
	const nite::UserId* pUserLabels = userMap.getPixels();
	
	for (int i = 0; i < userMap.getHeight(); ++i)
	{
		for (int j = 0; j < userMap.getWidth(); ++j, ++pUserLabels)
		{
			if (i == (int)y && j == (int)x)
				userId = *pUserLabels;
		}
	}
	
	return userId;
}

void UserViewer::DisplayCallback()
{
	nite::UserTrackerFrameRef userTrackerFrame;
	nite::HandTrackerFrameRef handTrackerFrame;
	openni::VideoFrameRef depthFrame;
	
	nite::Status rc;

	rc = m_pHandTracker->readFrame(&handTrackerFrame);
	if (rc != nite::STATUS_OK)
	{
		ROS_ERROR("HandTracker read frame failed");
		return;
	}
	
	rc = m_pUserTracker->readFrame(&userTrackerFrame);
	if (rc != nite::STATUS_OK)
	{
		ROS_ERROR("UserTracker read frame failed!");
		return;
	}
	
	depthFrame = userTrackerFrame.getDepthFrame();

	if (m_pTexMap == NULL)
	{
		// texture map init
		m_nTexMapX = MIN_CHUNKS_SIZE(depthFrame.getVideoMode().getResolutionX(), TEXTURE_SIZE);
		m_nTexMapY = MIN_CHUNKS_SIZE(depthFrame.getVideoMode().getResolutionY(), TEXTURE_SIZE);
		m_pTexMap = new openni::RGB888Pixel[m_nTexMapX * m_nTexMapY];
	}
	
	const nite::UserMap& userLabels = userTrackerFrame.getUserMap();
	
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glOrtho(0, GL_WIN_SIZE_X, GL_WIN_SIZE_Y, 0, -1.0, 1.0);
	
	if (depthFrame.isValid())
	{
		calculateHistogram(m_pDepthHist, MAX_DEPTH, depthFrame);
	}
	
	memset(m_pTexMap, 0, m_nTexMapX * m_nTexMapY * sizeof(openni::RGB888Pixel));
	
	float factor[3] = {0, 0, 0};
	
	if (depthFrame.isValid())
	{
		// begin drawing depth
		const nite::UserId* pLabels = userLabels.getPixels();
		
		const openni::DepthPixel* pDepthRow = (const openni::DepthPixel*)depthFrame.getData();
		openni::RGB888Pixel* pTexRow = m_pTexMap + depthFrame.getCropOriginY() * m_nTexMapY;
		int rowSize = depthFrame.getStrideInBytes() / sizeof(openni::DepthPixel);
		
		for (int y = 0; y < depthFrame.getHeight(); ++y)
		{
			const openni::DepthPixel* pDepth = pDepthRow;
			openni::RGB888Pixel* pTex = pTexRow + depthFrame.getCropOriginX();
			
			for (int x = 0; x < depthFrame.getWidth(); ++x, ++pDepth, ++pTex, ++pLabels)
			{
				if (*pDepth != 0)
				{
					if (*pLabels == 0)
					{
						factor[0] = Colors[colorCount][0];
						factor[1] = Colors[colorCount][1];
						factor[2] = Colors[colorCount][2];
					}
					else
					{
						factor[0] = Colors[*pLabels % colorCount][0];
						factor[1] = Colors[*pLabels % colorCount][1];
						factor[2] = Colors[*pLabels % colorCount][2];
					} // end if pLabels
					
					int nHistValue = m_pDepthHist[*pDepth];
					/**
					if (x == depthFrame.getWidth()/2 && y == depthFrame.getHeight()/2)
						ROS_INFO("%d", nHistValue);
					*/
					pTex->r = nHistValue * factor[0];
					pTex->g = nHistValue * factor[1];
					pTex->b = nHistValue * factor[2];
					
					factor[0] = factor[1] = factor[2] = 1;
				} // end if *pDepth
			} // end for int x
			
			pDepthRow += rowSize;
			pTexRow += m_nTexMapX;
		} // end for int y
	} // end if depthFrame.isValid()
	
	glTexParameteri(GL_TEXTURE_2D, GL_GENERATE_MIPMAP_SGIS, GL_TRUE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, m_nTexMapX, m_nTexMapY, 0, GL_RGB, GL_UNSIGNED_BYTE, m_pTexMap);

	// Display the OpenGL texture map
	glColor4f(1,1,1,1);

	glEnable(GL_TEXTURE_2D);
	glBegin(GL_QUADS);

	g_nXRes = depthFrame.getVideoMode().getResolutionX();
	g_nYRes = depthFrame.getVideoMode().getResolutionY();

	// upper left
	glTexCoord2f(0, 0);
	glVertex2f(0, 0);
	// upper right
	glTexCoord2f((float)g_nXRes/(float)m_nTexMapX, 0);
	glVertex2f(GL_WIN_SIZE_X, 0);
	// bottom right
	glTexCoord2f((float)g_nXRes/(float)m_nTexMapX, (float)g_nYRes/(float)m_nTexMapY);
	glVertex2f(GL_WIN_SIZE_X, GL_WIN_SIZE_Y);
	// bottom left
	glTexCoord2f(0, (float)g_nYRes/(float)m_nTexMapY);
	glVertex2f(0, GL_WIN_SIZE_Y);

	glEnd();
	glDisable(GL_TEXTURE_2D);
	
	
	// user detection routine
	
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
			DrawStatusLabel(m_pUserTracker, user);
			
			if (users[i].getSkeleton().getState() == nite::SKELETON_TRACKED)
				DrawSkeleton(m_pUserTracker, user);
		}
	}
	
	glutSwapBuffers();
}

void UserViewer::OnKey(unsigned char key, int x, int y)
{
	switch (key)
	{
		case 27:	// ESC
		Finalize();
		exit (1);
	}
}

void UserViewer::glutKeyboard(unsigned char key, int x, int y)
{
	UserViewer::ms_self->OnKey(key, x, y);
}
openni::Status UserViewer::InitOpenGL(int argc, char** argv)
{
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
	glutInitWindowSize(GL_WIN_SIZE_X, GL_WIN_SIZE_Y);
	glutCreateWindow(m_strSampleName);
	glutSetCursor(GLUT_CURSOR_NONE);
	
	InitOpenGLHooks();
	
	glDisable(GL_DEPTH_TEST);
	glEnable(GL_TEXTURE_2D);
	glEnableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_COLOR_ARRAY);
	
	return openni::STATUS_OK;
}

void UserViewer::InitOpenGLHooks()
{
	glutKeyboardFunc(glutKeyboard);
	glutDisplayFunc(glutDisplay);
	glutIdleFunc(glutIdle);
}
