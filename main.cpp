
// Shervin Rahimimanesh
// CSCI 699 
// HW 2
#include <fstream>
#include <stdlib.h>
#include <iostream>
#include <cstdio>
#include <cassert>
#include <vector>

#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>

#include <GL/glut.h>
#include <GL/gl.h>

#define _USE_MATH_DEFINES
#include <math.h>

// -------------
// Pound Defines
// -------------

// These three define the axis numbering for SensAble position vectors.
// The x-axis is typically left-right, with right positive.
// The y-axis is typically up-down, with up positive.
// The z-axis is typically in-out, with out positive.
#define X  0
#define Y  1
#define Z  2

// Vertical force for gravity compensation, in newtons.
#define GRAVITY_COMPENSATION_FORCE 0.15

// Wall and floor parameters, in millimeters.
#define LEFTWALLX  -120.0
#define FLOORY      -65.0
#define RIGHTWALLX  120.0

// Ball parameters, in millimeters.
#define BALLX  -55.0
#define BALLY   50.0
#define BALLZ  -25.0
#define BALLR   35.0

// Box parameters, in millimeters.
#define BOXX  80.0
#define BOXY  60.0
#define BOXZ  10.0
#define BOXL  40.0

// Button parameters, in millimeters
#define BUTTONXMIN  (LEFTWALLX + 60.0)
#define BUTTONXMAX  (BUTTONXMIN + 120.0)
#define BUTTONZMIN  -10.0
#define BUTTONZMAX   60.0

// Small offset to be used for rendering shadows and button.
#define SMALL_OFFSET 0.05



// SensAble
static HHD ghHD = HD_INVALID_HANDLE;
static HDSchedulerHandle gSchedulerCallback = HD_INVALID_HANDLE;

// Graphics - all dimensions are in millimeters
bool drawProxy = true;
float proxyRadius =  7.0;
float deviceRadius = 6.5;

// Haptic Device Record
struct DeviceDisplayState {
    HHD m_hHD;
	hduVector3Dd proxyPosition;
    hduVector3Dd devicePosition;
    hduVector3Dd force;
};

double radius;
hduVector3Dd radius_hat;
hduVector3Dd ball = hduVector3Dd(BALLX, BALLY, BALLZ);
std::vector<double> face{ BOXX - BOXL / 2, BOXX + BOXL / 2, BOXY - BOXL / 2, BOXY + BOXL / 2, BOXZ - BOXL / 2, BOXZ + BOXL / 2 };


// -------------------
// Function Prototypes
// -------------------
void callbackScheduler(void);
HDCallbackCode HDCALLBACK hapticCallback(void *data);
HDCallbackCode HDCALLBACK deviceStateCallback(void *pUserData);
void exitHandler(void);
void renderScene(void);
void changeSize(int w, int h);
void createGLUTMenus(void);
void setupGraphics(void);
void processMenuEvents(int option);
void processNormalKeys(unsigned char key, int x, int y);
void processButtons(HDint current, HDint last);
double max3(double a, double b, double c);
double min6(double a, double b, double c, double d, double e, double f);
double sind(double thetad);
double cosd(double thetad);
double tand(double thetad);


// --------------------
// Function Definitions
// --------------------

/*******************************************************************************
 hapticCallback()
 Main callback that sets the force that the user will feel.  It gets the current
 position and velocity of the device.
 This is what you want to edit to change the system's haptic feedback.
*******************************************************************************/
HDCallbackCode HDCALLBACK hapticCallback(void *data)
{
	// Local variables.
    hduVector3Dd position;
	hduVector3Dd velocity;
	hduVector3Dd force;
    hduVector3Dd proxyPosition;
	HDint currentButtonState;
	HDint lastButtonState;
	double stiffness = 0.5;  // Units are newtons per millimeter.

	// Get the handle for the current haptic device.
    HHD hHD = hdGetCurrentDevice();

	// Begin the haptic frame for this device.
    hdBeginFrame(hHD);

	// Get its position and velocity and store them in hduVector3Dd variables.
    hdGetDoublev(HD_CURRENT_POSITION, position);  // Units are millimeters.
	hdGetDoublev(HD_CURRENT_VELOCITY, velocity);  // Units are millimeters per second.

	// Get its current and previous button states and store them in HDint variables.
	hdGetIntegerv(HD_CURRENT_BUTTONS, &currentButtonState);
	hdGetIntegerv(HD_LAST_BUTTONS, &lastButtonState);

	// Process the buttons.  This function is defined below.
	processButtons(currentButtonState, lastButtonState);

	// Use the position and velocity measurements to compute the force the user should feel.
	// This is the code you need to change to create different haptic virtual environments.
	
	// Step 1
	// ------
	// First compute the location of the proxy.  This is the point where we will show a 
	// small sphere to represent the user's position in the virtual environment.  While the
	// user can move the device to any location, the proxy follows geometrical constraints 
	// and does not go inside of virtual objects.  If the user is not touching anything,
	// the proxy should be at the same location as the device.

	proxyPosition = position;
	int wall_x_left = LEFTWALLX;
	int wall_x_right = RIGHTWALLX;
	int floor_y = FLOORY; 
	
	if (position[X] < wall_x_left) {

		proxyPosition[X] = wall_x_left;
	}
	else if (position[X] > wall_x_right) {

		proxyPosition[X] = wall_x_right;
	}
	if (position[Y] < floor_y) {

		proxyPosition[Y] = floor_y;
	}	
	
	int ball_radius = BALLR;
	radius = position.distance(ball);

	if (radius < ball_radius) {
		radius_hat = (position - ball) / radius;
		proxyPosition = radius_hat*(ball_radius) + ball;
	}

	if (face[0] < position[X] && 
		position[X] < face[1] &&
		face[2] < position[Y] && 
		position[Y] < face[3] &&
		face[4] < position[Z] && 
		position[Z] < face[5]) {
		
		float distance[6];
		distance[0] = position[X] - face[0];
		distance[1] = face[1] - position[X];
		distance[2] = position[Y] - face[2];
		distance[3] = face[3] - position[Y];
		distance[4] = position[Z] - face[4];
		distance[5] = face[5] - position[Z];
		
		int near = 0;
		int n = 80;

		for (int i = 0; i < face.size(); i++) {

			if (distance[i] < n) {
				near = i;
				n = distance[i];
			}
		}
		if (near == 0) {

			proxyPosition[X] = face[0];
		}
		else if (near == 1) {

			proxyPosition[X] = face[1];
		}
		else if (near == 2) {

			proxyPosition[Y] = face[2];
		}
		else if (near == 3) {

			proxyPosition[Y] = face[3];
		}
		else if (near == 4) {

			proxyPosition[Z] = face[4];
		}
		else if(near == 5){

			proxyPosition[Z] = face[5];
		}
	}

	// Step 2
	// ------
	// Use the device position and the proxy position to compute the force you want the user to feel.
	// Typically, we use a spring force that pulls device toward the proxy, so that the force is 
	// proportional to the magnitude of the separation.

	force = stiffness * (proxyPosition - position);
	
	int button_x_min = BUTTONXMIN;
	int button_x_max = BUTTONXMAX;
	int button_z_min = BUTTONZMIN; 
	int button_z_max = BUTTONZMAX;

	if (position[X] > button_x_min &&
		position[X] < button_x_max &&
		position[Z] > button_z_min &&
		position[Z] < button_z_max &&
		position[Y] < floor_y) {

		
		if (position[Y] < 0 && position[Y] <= -68.0) {

			proxyPosition[Y] = -68.0;
			force = stiffness * (proxyPosition - position);
		}
	
			/*
				if (position[Y] == -68.5) {

				force = stiffness * (proxyPosition - position);

				if (position[Y] < -68.0) {

					force = 5 * (proxyPosition - position);
				}
			*/
	}
	

	// Add gravity compensation; a constant vertical force to make the device feel lighter.
	force[Y] += GRAVITY_COMPENSATION_FORCE;  // Units are newtons.


	// Step 3
	// ------
	// Output the force and store the chosen proxy location.

    // Send the force to the device.
	hdSetDoublev(HD_CURRENT_FORCE, force);
	
	// Send the proxy position to the device as though it was a torque.
	// Omnis have no torque output capabilities, so this merely acts to store the
	// proxy position for the programmer, so that the graphics thread can easily
	// access it.  This is a hack that would not be safe to use on a haptic device
	// that can actually output torque.
	hdSetDoublev(HD_CURRENT_TORQUE, proxyPosition);
	
	// End this haptic device's frame.
    hdEndFrame(hHD);

	// Handle errors.
    HDErrorInfo error;
    if (HD_DEVICE_ERROR(error = hdGetError())) {
        hduPrintError(stderr, &error, "Error during scheduler callback.");
        if (hduIsSchedulerError(&error)) {
            return HD_CALLBACK_DONE;
        }
    }

	// End the callback.
    return HD_CALLBACK_CONTINUE;
}

/******************************************************************************
 renderScene()
 This draws the objects to the display.
******************************************************************************/
void renderScene(void) {
	double angle;

    // Get the current state of the haptic device, including proxyPosition, devicePosition, and force.
    DeviceDisplayState state;
    hdScheduleSynchronous(deviceStateCallback, &state, HD_MIN_SCHEDULER_PRIORITY);

	//Clear the buffers to begin preparation for drawing.
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Draw a sphere at the haptic device's location.
	// Save the current settings.
	glPushMatrix();

	// Translate to where we want to draw the device's sphere
	glTranslatef(state.devicePosition[X], state.devicePosition[Y], state.devicePosition[Z]);			

	// Set the color in RGB.
	glColor3f(0.5, 0.0, 0.0);

	// Draw a small sphere there.
	glutSolidSphere(deviceRadius, 20, 20);

	// Return to the previous graphics settings.
	glPopMatrix();

	// Only draw a device shadow if we're not drawing the proxy and if the device is above the floor.
	if ((!drawProxy) && (state.devicePosition[Y] > FLOORY)) {
		// Draw a shadow below the device, right on top of the floor.
		glPushMatrix();
		glColor3f(0.0, 0.0, 0.0);
		glBegin(GL_TRIANGLE_FAN);
			glVertex3f(state.devicePosition[X], FLOORY + 2 * SMALL_OFFSET, state.devicePosition[Z]);
			angle = 0.0;
			for (angle = 0; angle <= 360.0; angle += 10.0) {
				glVertex3f(state.devicePosition[X] + deviceRadius*sind(angle), FLOORY + 2 * SMALL_OFFSET, state.devicePosition[Z] + deviceRadius*cosd(angle));
			}
		glEnd();
		glPopMatrix();
	}

	if (drawProxy) {
		// Draw a similar sphere at the proxy location, using the same five steps.
		glPushMatrix();
		glTranslatef(state.proxyPosition[X],state.proxyPosition[Y],state.proxyPosition[Z]);
		glColor3f(0.0, 0.0, 0.5);
		glutSolidSphere(proxyRadius, 20, 20);
		glPopMatrix();

		// Only draw the proxy shadow if the proxy is above the floor.
		if (state.proxyPosition[Y] > FLOORY) {
			// Draw a shadow below the proxy, right on top of the floor.
			glPushMatrix();
			glColor3f(0.0, 0.0, 0.0);
			glBegin(GL_TRIANGLE_FAN);
				glVertex3f(state.proxyPosition[X], FLOORY + 2 * SMALL_OFFSET, state.proxyPosition[Z]);
				for (angle = 0; angle <= 360.0; angle += 10.0) {
					glVertex3f(state.proxyPosition[X] + proxyRadius*sind(angle), FLOORY + 2 * SMALL_OFFSET, state.proxyPosition[Z] + proxyRadius*cosd(angle));
				}
			glEnd();
			glPopMatrix();
		}
	}

	// Draw a solid golden ball.
	glPushMatrix();
	glTranslatef(BALLX, BALLY, BALLZ);
	glColor3f(0.7, 0.5, 0.0);
	glutSolidSphere(BALLR,20,20);
	glPopMatrix();

	// Draw a solid purple box.
	glPushMatrix();
	glTranslatef(BOXX, BOXY, BOXZ);
	glColor3f(0.4, 0.0, 0.4);
	glutSolidCube(BOXL);
	glPopMatrix();

	// Draw a green rectangle for the button.
	glPushMatrix();
	glBegin(GL_QUADS);
		glColor3f(0.0, 0.7, 0.3);
		glVertex3f( BUTTONXMIN, FLOORY + SMALL_OFFSET, BUTTONZMIN);
		glVertex3f( BUTTONXMIN, FLOORY + SMALL_OFFSET, BUTTONZMAX);
		glVertex3f( BUTTONXMAX, FLOORY + SMALL_OFFSET, BUTTONZMAX);
		glVertex3f( BUTTONXMAX, FLOORY + SMALL_OFFSET, BUTTONZMIN);
	glEnd();
	glPopMatrix();

	// Draw rectangles for the walls and floor.
	glPushMatrix();
	glBegin(GL_QUADS);
		glColor3f(0.7, 0.7, 0.7);
		glVertex3f( RIGHTWALLX,  300.0,  100.0);
		glVertex3f( RIGHTWALLX,  300.0, -100.0);
		glVertex3f( RIGHTWALLX, FLOORY, -100.0);
		glVertex3f( RIGHTWALLX, FLOORY,  100.0);
		glColor3f(0.8, 0.8, 0.8);
		glVertex3f( RIGHTWALLX, FLOORY,  100.0);
		glVertex3f( RIGHTWALLX, FLOORY, -100.0);
		glVertex3f( LEFTWALLX,  FLOORY, -100.0);
		glVertex3f( LEFTWALLX,  FLOORY,  100.0);
		glColor3f(0.5, 0.5, 0.5);
		glVertex3f( LEFTWALLX,  300.0, -100.0);
		glVertex3f( LEFTWALLX,  300.0,  100.0);
		glVertex3f( LEFTWALLX, FLOORY,  100.0);
		glVertex3f( LEFTWALLX, FLOORY, -100.0);
	glEnd();
	glPopMatrix();

	// Swap the double buffer for the next draw.
	glutSwapBuffers();										
}

/*******************************************************************************
 callbackScheduler()
 Schedules the haptics and graphics callbacks.
*******************************************************************************/
void callbackScheduler(void)
{
    printf("Starting haptics callback.\n");
    gSchedulerCallback = hdScheduleAsynchronous(hapticCallback, 0, HD_DEFAULT_SCHEDULER_PRIORITY);

    HDErrorInfo error;
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to initialize the haptic device");
        fprintf(stderr, "\nPress any key to quit.\n");
        getchar();
        exit(-1);
    }

    printf("Starting graphics callback.\n\n");
    printf("Click in the graphics window and press Esc to quit.\n");

    glutMainLoop(); // Enter GLUT main loop.
}


/*******************************************************************************
 deviceStateCallback()
 Client callback used by the graphics main loop function.
 Use this callback synchronously.
 Gets data, in a thread safe manner, that is constantly being modified by the 
 haptics thread. 
 Note that the device torque is being used to hold the position of the proxy.
 This is not a safe thing to do when using a device that can output torques.
*******************************************************************************/
HDCallbackCode HDCALLBACK deviceStateCallback(void *pUserData)
{
    DeviceDisplayState *pDisplayState = static_cast<DeviceDisplayState *>(pUserData);

    hdGetDoublev(HD_CURRENT_POSITION, pDisplayState->devicePosition);
    hdGetDoublev(HD_CURRENT_FORCE, pDisplayState->force);
	hdGetDoublev(HD_CURRENT_TORQUE, pDisplayState->proxyPosition);

    // execute this only once.
    return HD_CALLBACK_DONE;
}

/******************************************************************************
 exitHandler()
 This handler gets called when the process is exiting. Ensures that HDAPI is
 properly shut down
******************************************************************************/
void exitHandler(void)
{
    hdStopScheduler();
    hdUnschedule(gSchedulerCallback);

    if (ghHD != HD_INVALID_HANDLE) {
        hdDisableDevice(ghHD);
        ghHD = HD_INVALID_HANDLE;
    }
}

/******************************************************************************
 createGLUTMenus()
 Method to create mouse right-click menu.
******************************************************************************/
void createGLUTMenus(void) {

	int submenu;

	// Create a new menu.
	submenu = glutCreateMenu(processMenuEvents);

	// Add items to the menu.  The string is the name, and the number is the identifying integer for processMenuEvents.
	glutAddMenuEntry("Enlarge Proxy Radius",1);	
	glutAddMenuEntry("Diminish Proxy Radius",2);

	// Attach the menu to the right mouse button.
	glutAttachMenu(GLUT_RIGHT_BUTTON);
}


/******************************************************************************
 processMenuEvents()
 Method to process menu selection.
******************************************************************************/
void processMenuEvents(int option) {

	switch (option) {
		case 1:
			// Increase the proxy's radius, and tell the user.
			proxyRadius += 1.0;
			printf("Increased the proxy's radius.\n");
			break;									
		case 2:
			// Reduce the proxy's radius, if possible, and tell the user.
			if (proxyRadius > 1.0) {
				proxyRadius -= 1.0;
				printf("Reduced the proxy's radius.\n");
			} else {
				printf("The proxy cannot get any smaller.\n");
			}
			break;
		default:
			// Tell the user we received an unknown option.
			printf("Received unknown menu event %d.\n", option);
			break;
	}
}


/******************************************************************************
 processNormalKeys()
 Method to handle keyboard input.
******************************************************************************/
void processNormalKeys(unsigned char key, int x, int y) {
	DeviceDisplayState state;

	switch (key) {
	
		case 27:										//27 is the escape key
		case 'q':
		case 'Q':
			exit(0);									//close the simulation
			break;
		case ' ':
		    // Get the current state of the haptic device, including proxyPosition, devicePosition, and force.
			hdScheduleSynchronous(deviceStateCallback, &state, HD_MIN_SCHEDULER_PRIORITY);
			printf("The haptic device is at x = %.2f mm, y = %.2f mm, and z = %.2f mm.\n", state.devicePosition[X], state.devicePosition[Y], state.devicePosition[Z]);
			break;
		case 'f':
		case'F':
			// do something else if the f key is depressed.
			printf("You pressed the f key.\n");
			break;
		case 'p':
		case 'P':
			drawProxy = !drawProxy;
			if (drawProxy) {
				printf("Showing proxy.\n");
			} else {
				printf("Hiding proxy.\n");
			}
			break;
		default:
			printf("You pressed the %c key, for which there is no dedicated action.\n", key);
			break;
	}
}

/******************************************************************************
 processButtons
******************************************************************************/
void processButtons(HDint current, HDint last)
{
	// Check to see whether the stylus button states have changed since last haptic frame.
	if (current != last) {
		// A button has been pressed or released.
		//printf("A stylus button was pressed or released: %x %x %x\n", lastButtonState, currentButtonState);

		// Figure out what happened and respond.
		switch (current - last) {
			case 1:
				//printf("Button 1 was pressed\n");
				break;
			case -1:
				//printf("Button 1 was released\n");
				break;
			case 2:
				//printf("Button 2 was pressed\n");
				//drawProxy = !drawProxy;
				//if (drawProxy) {
				//	printf("Showing proxy.\n");
				//} else {
				//	printf("Hiding proxy.\n");
				//}
				break;
			case -2:
				//printf("Button 2 was released\n");
				break;
			case 3:
				//printf("Both buttons were pressed\n");
				break;
			case -3:
				//printf("Both buttons were released\n");
				break;
			default:
				//printf("Something unexpected happened to the buttons.\n");
				break;
		}
	}
}

/******************************************************************************
 changeSize()
 Method to resize the openGL window.
******************************************************************************/
void changeSize(int w, int h) {

	// Prevent a divide by zero when the window is too short
	// (you can't make a window of zero width).
	if (h == 0) {
		h = 1;
	}

	float ratio = 1.0 * w / h;

	// Reset the coordinate system before modifying.
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	
	// Set the viewport to be the entire window.
	glViewport(0, 0, w, h);

    HDdouble maxWorkspace[6];
    hdGetDoublev(HD_MAX_WORKSPACE_DIMENSIONS, maxWorkspace);

    // Low/left/back point of device workspace.
    hduVector3Dd LLB(maxWorkspace[0], maxWorkspace[1], maxWorkspace[2]);
    // Top/right/front point of device workspace.
    hduVector3Dd TRF(maxWorkspace[3], maxWorkspace[4], maxWorkspace[5]);

	// Find the device workspace size
    HDdouble centerScreen[3];
    centerScreen[0] = (TRF[0] + LLB[0])/2.0;
    centerScreen[1] = (TRF[1] + LLB[1])/2.0;
    centerScreen[2] = (TRF[2] + LLB[2])/2.0;

    HDdouble screenDims[3];
    screenDims[0] = TRF[0] - LLB[0];
    screenDims[1] = TRF[1] - LLB[1];
    screenDims[2] = TRF[2] - LLB[2];

	// Set the correct perspective.
	gluPerspective(45,ratio,1,1000);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	// look at the device workspace
	gluLookAt(0.0,0.0,max3(screenDims[0],screenDims[1],screenDims[2]), 
			  0.0,0.0,0.0,
			  0.0f,1.0f,0.0f);
}

/******************************************************************************
 max3()
 A helper function to evaluate the maximum of three doubles.
******************************************************************************/
double max3(double a, double b, double c)
{
	if((a > b) && (a > c)) {
		return a;
	} else if((b > a) && (b > c)) {
		return b;
	} else {
		return c;
	}
}

/******************************************************************************
 min6()
 A helper function to evaluate the minimum of six doubles.
******************************************************************************/
double min6(double a, double b, double c, double d, double e, double f)
{
	if ((((a < b) && (a < c)) && (a < d)) && ((a < e) && (a < f))) {
		return a;
	} else if (((b < c) && (b < d)) && ((b < e) && (b < f))) {
		return b;
	} else if (((c < d) && (c < e)) && (c < f)) {
		return c;
	} else if ((d < e) && (d < f)) {
		return d;
	} else if (e < f) {
		return e;
	} else {
		return f;
	}
}

// Trigonometric functions in degrees.
double sind(double thetad)
{
	return(sin(thetad * M_PI / 180));
}
double cosd(double thetad)
{
	return(cos(thetad * M_PI / 180));
}
double tand(double thetad)
{
	return(tan(thetad * M_PI / 180));
}

/******************************************************************************    
 Set up graphics pipeline and lights.
******************************************************************************/
void setupGraphics(void)
{
	// Set background color to be white.
	glClearColor(1.0f, 1.0f, 1.0f, 0.5f);

	// Set the depth buffer clear depth and enable depth testing.
	glClearDepth(1.0f);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);								// Set type of depth testing to do

	// Set the perspective to use.
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

	// Enable smooth shading, so that polygons and triangles can have interpolated colors.
	glShadeModel(GL_SMOOTH);

	// Enable color material, so that object color can be specified with glColor.
	glEnable(GL_COLOR_MATERIAL);

	// Turn on lighting.
	glEnable(GL_LIGHTING);

    // Set the zeroth light's position and color, then enable.
    GLfloat lightZeroPosition[] = { 100.0, 100.0, 100.0, 0.0 };
    GLfloat lightZeroColor[] = { 0.8, 0.8, 0.8, 1.0 };
    glLightfv(GL_LIGHT0, GL_POSITION, lightZeroPosition);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightZeroColor);
    glEnable(GL_LIGHT0);

	// Set the first light's position and color, then enable.
    GLfloat lightOnePosition[] = { -20.0, 0.0, 20.0, 0.0 };
    glLightfv(GL_LIGHT1, GL_POSITION, lightOnePosition);
	GLfloat lightOneColor[] = { 1.0, 1.0, 1.0, 1.0 };
    glLightfv(GL_LIGHT1, GL_DIFFUSE, lightOneColor);
    glEnable(GL_LIGHT1);
}

/******************************************************************************
 main()
 This is the main program entry point.
******************************************************************************/
void main(int argc, char **argv) {

	// Tell the user we're starting up.
    printf("Starting application.\n");
    
	// Set up the exit handler.
    atexit(exitHandler);

    // Initialize the device.  This needs to be called before any other
    // actions are performed on the device.
    ghHD = hdInitDevice(HD_DEFAULT_DEVICE);

	// Check for errors with the haptic device initialization.
    HDErrorInfo error;
    if (HD_DEVICE_ERROR(error = hdGetError())) {
        hduPrintError(stderr, &error, "Failed to initialize the haptic device.");
        fprintf(stderr, "\nPress any key to quit.\n");
        getchar();
        exit(-1);
    }

	// Tell the user what device we found.
	printf("Found a %s.\n", hdGetString(HD_DEVICE_MODEL_TYPE));
    
	// Set up the device for force output and max force clamping.
    hdEnable(HD_FORCE_OUTPUT);
    hdEnable(HD_MAX_FORCE_CLAMPING);

	// Start the haptic device scheduler.
    hdStartScheduler();

	// Check for errors with the scheduler.
    if (HD_DEVICE_ERROR(error = hdGetError())) {
        hduPrintError(stderr, &error, "Failed to start scheduler.");
        fprintf(stderr, "\nPress any key to quit.\n");
        getchar();
        exit(-1);
    }

	// Initialize a glut project.
	glutInit(&argc, argv);

	// Set up the display mode.
	glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);

	// Set up the window.
	glutInitWindowPosition(400,200);					//Set the initial window position
	glutInitWindowSize(800,800);						//Set the initial window size
	glutCreateWindow("CSCI 699: Haptic Interfaces - Assignment 2 - Spring 2022");		//Give our window a name
	
	// Set up functions to handle key hits, display, and window reshaping.
	glutKeyboardFunc(processNormalKeys);				//Register processNormalKeys to deal with keyboard input
	glutDisplayFunc(renderScene);						//Register display function
	glutReshapeFunc(changeSize);						//Register resize function
	glutIdleFunc(renderScene);							//Register Idle callback.
														//This tells opengl to loop the renderScene
														//method even when the standard inputs like
														//mouse and keyboard are not acting.

	// Set up the menus.
	createGLUTMenus();									//Create right click menu

	// Set up graphics.
	setupGraphics();

	// Pre-openGL user I/O at console window 
	printf("\n\nFirmly hold the handle of the haptic device.\n Then press enter to start the application.\n");	//Prompt user for "enter" key
    getchar();

    // Start the haptics and graphics callbacks.
	callbackScheduler();
}
