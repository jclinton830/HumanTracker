#include "main.h"
#include "glut.h"

#include <tchar.h>
#include <iostream>
#include <strsafe.h>

#include <cmath>
#include <cstdio>
#include <ctime>

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include "SerialPort.h"
#include <iterator>
#include <algorithm>
#include <minwindef.h>
#include <vector>

#ifdef max
#undef max
#endif
#ifdef min
#undef min
#endif

#include <math.h>
#include <Windows.h>
#include <Ole2.h>

#include <Kinect.h>


//#define IDC_STATUS 1001
// We'll be using buffer objects to store the kinect point cloud
GLuint vboId;
GLuint cboId;

INT64                   m_nLastCounter;
double                  m_fFreq;
DWORD                   m_nFramesSinceUpdate;
INT64                   m_nStartTime;
double fps = 0.0;
INT64 nTime = 0;
INT64 nStartTime = 0;
float TicToc;
HWND                    m_hWnd;

// Intermediate Buffers
unsigned char rgbimage[colorwidth*colorheight * 4];    // Stores RGB color image
ColorSpacePoint depth2rgb[width*height];             // Maps depth pixels to rgb pixels
CameraSpacePoint depth2xyz[width*height];			 // Maps depth pixels to 3d coordinates

// Body tracking variables
BOOLEAN tracked;							// Whether we see a body
Joint joints[JointType_Count];				// List of joints in the tracked body

// Kinect Variables
IKinectSensor* sensor;             // Kinect sensor
IMultiSourceFrameReader* reader;   // Kinect data source
ICoordinateMapper* mapper;         // Converts between depth, color, and 3d coordinates

const CameraSpacePoint& lwrist = joints[JointType_WristLeft].Position;;
const CameraSpacePoint& rwrist = joints[JointType_WristRight].Position;;
const CameraSpacePoint& lelbow = joints[JointType_ElbowLeft].Position;;
const CameraSpacePoint& relbow = joints[JointType_ElbowRight].Position;;
const CameraSpacePoint& lshoulder = joints[JointType_ShoulderLeft].Position;;
const CameraSpacePoint& rshoulder = joints[JointType_ShoulderRight].Position;;
const CameraSpacePoint& lhand = joints[JointType_HandLeft].Position;;
const CameraSpacePoint& rhand = joints[JointType_HandRight].Position;;
const CameraSpacePoint& lhip = joints[JointType_HipLeft].Position;;
const CameraSpacePoint& rhip = joints[JointType_HipRight].Position;;
const CameraSpacePoint& lknee = joints[JointType_KneeLeft].Position;;
const CameraSpacePoint& rknee = joints[JointType_KneeRight].Position;;
const CameraSpacePoint& lankle = joints[JointType_AnkleLeft].Position;;
const CameraSpacePoint& rankle = joints[JointType_AnkleRight].Position;;
const CameraSpacePoint& lfoot = joints[JointType_FootLeft].Position;;
const CameraSpacePoint& rfoot = joints[JointType_FootRight].Position;;
const CameraSpacePoint& spineshoulder = joints[JointType_SpineShoulder].Position;;
const CameraSpacePoint& spinebase = joints[JointType_SpineBase].Position;;
const CameraSpacePoint& spinemid = joints[JointType_SpineMid].Position;;
const CameraSpacePoint& neck = joints[JointType_Neck].Position;;
const CameraSpacePoint& head = joints[JointType_Head].Position;;
const CameraSpacePoint& lthumb = joints[JointType_ThumbLeft].Position;;
const CameraSpacePoint& rthumb = joints[JointType_ThumbRight].Position;;
const CameraSpacePoint& lhandtip = joints[JointType_HandTipLeft].Position;;
const CameraSpacePoint& rhandtip = joints[JointType_HandTipRight].Position;;

CameraSpacePoint inithead;
CameraSpacePoint finhead;

//String for getting the output from arduino
char output[MAX_DATA_LENGTH];

/*Portname must contain these backslashes, and remember to
replace the following com port*/
char *port_name = "\\\\.\\COM3";

//String for incoming data
char incomingData[MAX_DATA_LENGTH];



bool fall;
bool not_fall;
bool not_fall_activity;

bool initKinect() {
	if (FAILED(GetDefaultKinectSensor(&sensor))) {
		return false;
	}
	if (sensor) {
		sensor->get_CoordinateMapper(&mapper);

		sensor->Open();
		sensor->OpenMultiSourceFrameReader(
			FrameSourceTypes::FrameSourceTypes_Body,
			&reader);
		return reader;
	}
	else {
		return false;
	}
}

void getBodyData(IMultiSourceFrame* frame) 
{
	IBodyFrameReader* bodyframeReader = NULL;
	IBodyFrame* bodyframe;
	IBodyFrameReference* frameref = NULL;

	frame->get_BodyFrameReference(&frameref);
	frameref->AcquireFrame(&bodyframe);
	if (frameref) frameref->Release();

	if (!bodyframe) return;

	//getting timespan
	TicToc = bodyframe->get_RelativeTime(&nTime);
	if (!nStartTime)
	{
		nStartTime = nTime;
	}
	TicToc = (nTime - nStartTime)/10000.f;


	double scale = 0.01;
	TicToc = (int)(TicToc / scale) * scale;
	//std::cout << "Time Span: " << (int)TicToc << std::endl;
	LARGE_INTEGER qpcNow = { 0 };

	if (m_fFreq)
	{
		if (QueryPerformanceCounter(&qpcNow))
		{
			if (m_nLastCounter)
			{
				m_nFramesSinceUpdate++;
				fps = m_fFreq * m_nFramesSinceUpdate / double(qpcNow.QuadPart - m_nLastCounter);
			}
		}
	}
	//std::cout << "FPS: " << fps << std::endl;

	//getting joint data
	IBody* body[BODY_COUNT] = { 0 };
	bodyframe->GetAndRefreshBodyData(BODY_COUNT, body);
	for (int i = 0; i < BODY_COUNT; i++) {
		body[i]->get_IsTracked(&tracked);
		if (tracked) {
			body[i]->GetJoints(JointType_Count, joints);
			break;
		}
	}
	if (bodyframe) bodyframe->Release();
	
}

void getKinectData()
{
	IMultiSourceFrame* frame = NULL;
	if (SUCCEEDED(reader->AcquireLatestFrame(&frame))) {
		glUnmapBuffer(GL_ARRAY_BUFFER);
		getBodyData(frame);
	}
	if (frame) frame->Release();
}

void transferData(std::string activity)
{
	SerialPort arduino(port_name);
	if (arduino.isConnected())
	{
		std::cout << "Connection Established" << std::endl;
		activity = "FALL";
		if (activity == "FALL")
		{
			char fall_data[] = "at+send=1\r";
			arduino.writeSerialPort(fall_data, 10);
		}

	}
	else
	{
		std::cout << "ERROR, check port name" << std::endl;
	}
	return;
}

void drawBoundingBox()
{
	//tolerances 
	CameraSpacePoint Tolerance;
	Tolerance.X = 0.1;
	Tolerance.Y = 0.1;

	//vertices of the bounding box
	CameraSpacePoint TopLeftVertice;
	CameraSpacePoint TopRightVertice; 
	CameraSpacePoint BottomLeftVertice;
	CameraSpacePoint BottomRightVertice;

	//variables used to find the boundaries of the bounding box
	CameraSpacePoint initminFeet;
	CameraSpacePoint initminLeft;
	CameraSpacePoint initmaxRight;
	
	//calculating boundaries
	initmaxRight.X = std::max(rknee.X, std::max(rfoot.X, std::max(relbow.X, rhand.X)));
	initminLeft.X = std::min(lknee.X, std::min(lfoot.X, std::min(lelbow.X, lhand.X)));
	inithead.Y = head.Y;
	initminFeet.Y = std::min(lfoot.Y, rfoot.Y);
	//head2toe.Y = head.Y - initminFeet.Y;

	//coordinates od the vertices 
	TopLeftVertice.X = initminLeft.X - Tolerance.X;
	TopLeftVertice.Y = head.Y + Tolerance.Y;
	TopLeftVertice.Z = spineshoulder.Z;

	TopRightVertice.X = initmaxRight.X + Tolerance.X;
	TopRightVertice.Y = head.Y + Tolerance.Y;
	TopRightVertice.Z = spineshoulder.Z;

	BottomLeftVertice.X = initminLeft.X -Tolerance.X; 
	BottomLeftVertice.Y = initminFeet.Y - Tolerance.Y;
	BottomLeftVertice.Z = spineshoulder.Z;

	BottomRightVertice.X = initmaxRight.X + Tolerance.X;
	BottomRightVertice.Y = initminFeet.Y - Tolerance.Y;
	BottomRightVertice.Z = spineshoulder.Z;

	//drwaing the bounding box
	glColor3f(0.f, 1.f, 0.f);
	glBegin(GL_LINES);
	glLineWidth(10.0f);
	glVertex3f(TopLeftVertice.X, TopLeftVertice.Y, TopLeftVertice.Z);
	glVertex3f(TopRightVertice.X, TopRightVertice.Y, TopRightVertice.Z);
	glVertex3f(TopRightVertice.X, TopRightVertice.Y, TopRightVertice.Z);
	glVertex3f(BottomRightVertice.X, BottomRightVertice.Y, BottomRightVertice.Z);
	glVertex3f(BottomRightVertice.X, BottomRightVertice.Y, BottomRightVertice.Z);
	glVertex3f(BottomLeftVertice.X, BottomLeftVertice.Y, BottomLeftVertice.Z);
	glVertex3f(BottomLeftVertice.X, BottomLeftVertice.Y, BottomLeftVertice.Z);
	glVertex3f(TopLeftVertice.X, TopLeftVertice.Y, TopLeftVertice.Z);
	glEnd();

	//fall = true;
	//not_fall = false;
	//not_fall_activity = false;
	float delta_head;

	int counter = TicToc;
	//std::cout << "time: "<< (int)TicToc << std::endl;
	float max_threshold = 0.25;
	float min_threshold = 0.005;

	float mid_threshold = 1.1;	

	//if (fall == true)
	//{
	//	transferData("FALL");
	//}

	if (counter % 4000 == 0)
	{
		delta_head = abs(head.Y - std::min(lfoot.Y, rfoot.Y));
		//std::cout << "height difference: " << delta_head << std::endl;

		if (delta_head < max_threshold)
		{
			//fall intitiated
			std::cout << "Fall initiated" << std::endl;
			if ((delta_head > min_threshold) && (delta_head < max_threshold))
			{
				//fall detected
				std::cout << "FALL DETECTED" << std::endl;
				transferData("FALL");
				return;
			}
			else
			{
				std::cout << "NOT A FALL" << std::endl;				
				return;
			}
		}

		if ((delta_head > max_threshold) && (delta_head < mid_threshold))
		{
			//not a fall scenario
			std::cout << "SEATED" << std::endl;		
			return;
		}

	}
}


void drawKinectData() {
	
	getKinectData();

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_COLOR_ARRAY);
	glEnable(GL_PROGRAM_POINT_SIZE);

	glBindBuffer(GL_ARRAY_BUFFER, vboId);
	glVertexPointer(3, GL_FLOAT, 0, NULL);

	glBindBuffer(GL_ARRAY_BUFFER, cboId);
	glColorPointer(3, GL_FLOAT, 0, NULL);

	glLineWidth(5.0f);
	glPointSize(15.0f);

	glDrawArrays(GL_POINTS, 0, width*height);
	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_COLOR_ARRAY);

	for (int i = 0; i < BODY_COUNT; i++) {
		if (tracked){

			//Draw Joints
			glBegin(GL_POINTS);
			glColor3f(0.f, 0.f, 1.f);
			glVertex3f(lthumb.X, lthumb.Y, lthumb.Z);
			glVertex3f(rthumb.X, rthumb.Y, rthumb.Z);
			glVertex3f(lhandtip.X, lhandtip.Y, lhandtip.Z);
			glVertex3f(rhandtip.X, rhandtip.Y, rhandtip.Z);
			glVertex3f(lhand.X, lhand.Y, lhand.Z);
			glVertex3f(rhand.X, rhand.Y, rhand.Z);
			glVertex3f(lwrist.X, lwrist.Y, lwrist.Z);
			glVertex3f(rwrist.X, rwrist.Y, rwrist.Z);
			glVertex3f(lelbow.X, lelbow.Y, lelbow.Z);
			glVertex3f(relbow.X, relbow.Y, relbow.Z);
			glVertex3f(lshoulder.X, lshoulder.Y, lshoulder.Z);
			glVertex3f(rshoulder.X, rshoulder.Y, rshoulder.Z);
			glVertex3f(spineshoulder.X, spineshoulder.Y, spineshoulder.Z);
			glVertex3f(neck.X, neck.Y, neck.Z);
			glVertex3f(head.X, head.Y, head.Z);
			glVertex3f(spinemid.X, spinemid.Y, spinemid.Z);
			glVertex3f(spinebase.X, spinebase.Y, spinebase.Z);
			glVertex3f(lhip.X, lhip.Y, lhip.Z);
			glVertex3f(rhip.X, rhip.Y, rhip.Z);
			glVertex3f(lknee.X, lknee.Y, lknee.Z);
			glVertex3f(rknee.X, rknee.Y, rknee.Z);
			glVertex3f(lankle.X, lankle.Y, lankle.Z);
			glVertex3f(rankle.X, rankle.Y, rankle.Z);
			glVertex3f(lfoot.X, lfoot.Y, lfoot.Z);
			glVertex3f(rfoot.X, rfoot.Y, rfoot.Z);
			glEnd();

			// Draw skeleton
			glBegin(GL_LINES);
			glColor3f(1.f, 0.f, 0.f);
			//left upper body lines
			glVertex3f(lhandtip.X, lhandtip.Y, lhandtip.Z);
			glVertex3f(lhand.X, lhand.Y, lhand.Z);
			glVertex3f(lthumb.X, lthumb.Y, lthumb.Z);
			glVertex3f(lhand.X, lhand.Y, lhand.Z);
			glVertex3f(lhand.X, lhand.Y, lhand.Z);
			glVertex3f(lwrist.X, lwrist.Y, lwrist.Z);
			glVertex3f(lwrist.X, lwrist.Y, lwrist.Z);
			glVertex3f(lelbow.X, lelbow.Y, lelbow.Z);
			glVertex3f(lelbow.X, lelbow.Y, lelbow.Z);
			glVertex3f(lshoulder.X, lshoulder.Y, lshoulder.Z);
			glVertex3f(lshoulder.X, lshoulder.Y, lshoulder.Z);
			glVertex3f(spineshoulder.X, spineshoulder.Y, spineshoulder.Z);
			glEnd();
			//right upper body lines
			glBegin(GL_LINES);
			glColor3f(1.f, 0.f, 0.f);
			glVertex3f(spineshoulder.X, spineshoulder.Y, spineshoulder.Z);
			glVertex3f(rshoulder.X, rshoulder.Y, rshoulder.Z);
			glVertex3f(rshoulder.X, rshoulder.Y, rshoulder.Z);
			glVertex3f(relbow.X, relbow.Y, relbow.Z);
			glVertex3f(relbow.X, relbow.Y, relbow.Z);
			glVertex3f(rwrist.X, rwrist.Y, rwrist.Z);
			glVertex3f(rwrist.X, rwrist.Y, rwrist.Z);
			glVertex3f(rhand.X, rhand.Y, rhand.Z);
			glVertex3f(rhand.X, rhand.Y, rhand.Z);
			glVertex3f(rthumb.X, rthumb.Y, rthumb.Z);
			glVertex3f(rhand.X, rhand.Y, rhand.Z);
			glVertex3f(rhandtip.X, rhandtip.Y, rhandtip.Z);
			glEnd();
			//mid upper body lines
			glBegin(GL_LINES);
			glColor3f(1.f, 0.f, 0.f);
			glVertex3f(spineshoulder.X, spineshoulder.Y, spineshoulder.Z);
			glVertex3f(neck.X, neck.Y, neck.Z);
			glVertex3f(neck.X, neck.Y, neck.Z);
			glVertex3f(head.X, head.Y, head.Z);
			glEnd();
			//mid center body lines
			glBegin(GL_LINES);
			glColor3f(1.f, 0.f, 0.f);
			glVertex3f(spineshoulder.X, spineshoulder.Y, spineshoulder.Z);
			glVertex3f(spinemid.X, spinemid.Y, spinemid.Z);
			glVertex3f(spinemid.X, spinemid.Y, spinemid.Z);
			glVertex3f(spinebase.X, spinebase.Y, spinebase.Z);
			glEnd();
			//left lower body lines
			glBegin(GL_LINES);
			glColor3f(1.f, 0.f, 0.f);
			glVertex3f(spinebase.X, spinebase.Y, spinebase.Z);
			glVertex3f(lhip.X, lhip.Y, lhip.Z);
			glVertex3f(lhip.X, lhip.Y, lhip.Z);
			glVertex3f(lknee.X, lknee.Y, lknee.Z);
			glVertex3f(lknee.X, lknee.Y, lknee.Z);
			glVertex3f(lankle.X, lankle.Y, lankle.Z);
			glVertex3f(lankle.X, lankle.Y, lankle.Z);
			glVertex3f(lfoot.X, lfoot.Y, lfoot.Z);
			glEnd();
			//right lower body lines 
			glBegin(GL_LINES);
			glColor3f(1.f, 0.f, 0.f);
			glVertex3f(spinebase.X, spinebase.Y, spinebase.Z);
			glVertex3f(rhip.X, rhip.Y, rhip.Z);
			glVertex3f(rhip.X, rhip.Y, rhip.Z);
			glVertex3f(rknee.X, rknee.Y, rknee.Z);
			glVertex3f(rknee.X, rknee.Y, rknee.Z);
			glVertex3f(rankle.X, rankle.Y, rankle.Z);
			glVertex3f(rankle.X, rankle.Y, rankle.Z);
			glVertex3f(rfoot.X, rfoot.Y, rfoot.Z);
			glEnd();


			drawBoundingBox();

		}
	}
	
}

int main(int argc, char* argv[]) 
{	
	if (!init(argc, argv)) return 1;
	if (!initKinect()) return 1;

	// OpenGL setup
	glClearColor(0, 0, 0, 0);
	glClearDepth(1.0f);

	// Set up array buffers
	const int dataSize = width*height * 3 * 4;
	glGenBuffers(1, &vboId);
	glBindBuffer(GL_ARRAY_BUFFER, vboId);
	glBufferData(GL_ARRAY_BUFFER, dataSize, 0, GL_DYNAMIC_DRAW);
	glGenBuffers(1, &cboId);
	glBindBuffer(GL_ARRAY_BUFFER, cboId);
	glBufferData(GL_ARRAY_BUFFER, dataSize, 0, GL_DYNAMIC_DRAW);

	// Camera setup
	glViewport(0, 0, width, height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45, width / (GLdouble)height, 0.1, 1000);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(0, 0, 0, 0, 0, 1, 0, 1, 0);

	// Main loop
	execute();

	return 0;
}