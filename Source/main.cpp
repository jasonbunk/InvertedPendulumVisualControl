/*
 * Simulations with OpenGL visualizations
 *
 * Author: Jason Bunk
 * Web page: http://sites.google.com/site/jasonbunk
 * License: Apache License Version 2.0, January 2004
 * Copyright (c) 2015 Jason Bunk
 */

#include "stdafx.h"
#include "main.h"
#include "SFML_Inputs.h"
#include "Utils/CaseInsensitiveSearch.h"

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <list>
#include <deque>
#include <algorithm>
using std::cout; using std::endl;

#include "InitSimulations.h"
#include "SimplerGameSimSystem.h"

#include "Visualization/DrawPlane.h"
#include "Visualization/GlutFontRender.h"


/*extern*/ WindowProjectionInfo global_window_info = WindowProjectionInfo(); //declared in stdafx.h
/*extern*/ bool SystemIsPaused = true; //declared in stdafx.h
/*extern*/ bool RenderVisualsUsingSFML = true;



#define RENDERED_FONT_SIZE_FPS_AND_INFO 13.0f


#define USE_SIMULATED_TIMER 1

//-------------------------------------------------------------
#include "TryIncludeJPhysics.h" //for timer
#if USE_SIMULATED_TIMER
#include "Utils/Clock.h"
myclock mytimer;
#else
TIMER mytimer;
#endif
#include <list>
#include <time.h>
std::list<double> frameTimes(11, 1.0);
double DTIME = 0.0;
double DTIME_AVG = 0.0; //average framerate
//-------------------------------------------------------------


// Left button presses place a control point.
void myMouseFunc( int button, int state, int x, int y )
{
#ifdef BUILD_WITH_SFML
	if(button == sf::Mouse::Left)
	{
        gGameSystem.SetMouseL(state==1);
	}
	else if(button == sf::Mouse::Right)
	{
        gGameSystem.SetMouseR(state==1);
	}
#endif
}

void myMouseMotionFunc( int x, int y )	//called both for MotionFunc and PassiveMotionFunc
{
	phys::point truepos(static_cast<float>(x), static_cast<float>(global_window_info.WindowHeight - y));

	phys::point newpos;
	newpos.x = ((float)x)/((float)(global_window_info.WindowWidth-1)) - 0.5f;	//right now, range from -0.50 to 0.50 as a kind of % distance from center
	newpos.y = (1.0f-((float)y)/((float)(global_window_info.WindowHeight-1))) - 0.5f; // Flip y-value since y position is from top row.


	//newpos.x *= 2.0f*gGameSystem.GetUniverseLimitsXXX();
	//newpos.y *= 2.0f*gGameSystem.GetUniverseLimitsYYY();


	newpos.x *= (gGameSystem.camera_rotation.r / gGameSystem.camera_original_r) * 2.0f * global_window_info.WindowWidth_half_f;
	newpos.y *= (gGameSystem.camera_rotation.r / gGameSystem.camera_original_r) * 2.0f * global_window_info.WindowHeight_half_f;

	newpos.x -= (gGameSystem.camera_pos.x + gGameSystem.camera_pos_game_offset.x);
	newpos.y -= (gGameSystem.camera_pos.y + gGameSystem.camera_pos_game_offset.y);


	if(gGameSystem.GetMouseDown_R())
	{
		gGameSystem.camera_rotation.theta -= 0.007*(truepos.y - gGameSystem.GetMousePos_Wincoords().y);

		gGameSystem.camera_rotation.phi += 0.007*(truepos.x - gGameSystem.GetMousePos_Wincoords().x);
	}


	gGameSystem.SetMousePos(truepos, newpos);
}


void updatePhysics(void)
{
#if USE_SIMULATED_TIMER
	DTIME = mytimer.getTimeSinceLastMeasurement();
#else
	DTIME = mytimer.seconds();
	mytimer.reset();
#endif

	frameTimes.pop_front();
	frameTimes.push_back(DTIME);
	DTIME_AVG = 0.0;
	std::list<double>::iterator It = frameTimes.begin();
	for(; It != frameTimes.end(); It++) {DTIME_AVG += (*It);}
	DTIME_AVG /= frameTimes.size();


	gGameSystem.Update(DTIME);
}


/*extern*/ GLdouble* last_used_modelMatrix_saved[16];	//declared in stdafx.h, should be an array of length 16
/*extern*/ GLdouble* last_used_projMatrix_saved[16];	//declared in stdafx.h, should be an array of length 16
/*extern*/ GLint* last_used_viewport_saved[4];		//declared in stdafx.h, should be an array of length 4

static inline void GL_START_DRAWING_TRANSLATION(phys::vec3 camera_offset_xyz, phys::vec3_spherical camera_rotation)
{
	glPushMatrix();
	glMatrixMode(GL_MODELVIEW);

	glTranslated(0.0, 0.0, -camera_rotation.r);
	glRotated(camera_rotation.theta *physmath::RAD_TO_DEG,	1.0, 0.0, 0.0);
	glRotated(camera_rotation.phi *physmath::RAD_TO_DEG,		0.0, 0.0, 1.0);
	glTranslated(camera_offset_xyz.x, camera_offset_xyz.y, camera_offset_xyz.z);

	glMatrixMode(GL_MODELVIEW);

	glGetDoublev(GL_MODELVIEW_MATRIX, global_window_info.last_used_modelMatrix);
	glGetDoublev(GL_PROJECTION_MATRIX, global_window_info.last_used_projMatrix);
	glGetIntegerv(GL_VIEWPORT, global_window_info.last_used_viewport);
}
static inline void GL_FINISH_DRAWING_TRANSLATION()
{
	glPopMatrix();
}


static bool draw_tic_tock = false;


void displayFunc(void)
{
	if(gGameSystem.physics_system != nullptr)
		gGameSystem.physics_system->SetCameraPosGameOffset(gGameSystem.camera_pos_game_offset);


	//x is like pitch (around up and down)
	//y is like yaw (around left to right)
	//z is like roll

	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);	//clear normal buffer to black (not framebuffer yet, but that'll be cleared too.)
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


	//gGameSystem.Draw_MousePointer();
	draw_tic_tock = true; // !draw_tic_tock;
	const double DrawnGridLightness = 0.08;
	
	
if(draw_tic_tock)
{
	GL_START_DRAWING_TRANSLATION(gGameSystem.camera_pos + gGameSystem.camera_pos_game_offset, gGameSystem.camera_rotation);
	
	
	//gGameSystem.physics_system->DrawSystemStuff();
	gGameSystem.Draw_EverythingExceptMousePointer();
	
	if(gGameSystem.physics_system != nullptr)
	{
		gGameSystem.physics_system->DrawSystemStuff();
		draw_planes::Draw(draw_planes::xy, 101, gGameSystem.physics_system->GetGridWidth_ForDrawingPlanes()*50.0, gGameSystem.physics_system->GetOriginForDrawingPlanes(), 1.0, DrawnGridLightness);
	}
	else
		draw_planes::Draw(draw_planes::xy, 101, 50.0, phys::vec3(), 1.0, DrawnGridLightness);


	GL_FINISH_DRAWING_TRANSLATION();

	//gGameSystem.Draw_MousePointer();



	const float writeX = -1.0f * global_window_info.WindowWidth_half_f;	//*gGameSystem.GetUniverseLimitsXXX();
	float writePos = 1.0f	* global_window_info.WindowHeight_half_f;			//gGameSystem.GetUniverseLimitsYYY();
	float WI = -0.055f	* global_window_info.WindowHeight_half_f;	//gGameSystem.window.GetPixelsHeight_f();


	glColor4ub(255,255,255,255);
	char buffer[4096];

#if 0
	sprintf____s (buffer, "Simulation");
	renderBitmapString(315.0, 325.0, buffer, 18.0f);
	sprintf____s (buffer, "(author: Jason Bunk)");
	renderBitmapString(346.0, 305.0, buffer, 18.0f);
#endif

	sprintf____s (buffer, "fps: %f", ((1.0/DTIME_AVG)));
	renderBitmapString(writeX, (writePos+=WI), buffer, RENDERED_FONT_SIZE_FPS_AND_INFO);


	sprintf____s (buffer, "elapsed time (s): %f", gGameSystem.GetTimeElapsed());
	renderBitmapString(writeX, (writePos+=WI), buffer, RENDERED_FONT_SIZE_FPS_AND_INFO);


#if 0
	sprintf____s (buffer, "window width (Mm): %f", (2.0f * gGameSystem.GetUniverseLimitsXXX()));
	renderBitmapString(writeX, (writePos+=WI), buffer, RENDERED_FONT_SIZE_FPS_AND_INFO);
#endif
	sprintf____s (buffer, "camera distance: %f", static_cast<float>(gGameSystem.camera_rotation.r));
	renderBitmapString(writeX, (writePos+=WI), buffer, RENDERED_FONT_SIZE_FPS_AND_INFO);


	int line_for_module_printed = 0;
	if(gGameSystem.physics_system != nullptr) {
		while(line_for_module_printed < 100 && gGameSystem.physics_system->FormatTextInfo(buffer, line_for_module_printed)) {
			renderBitmapString(writeX, (writePos+=WI), buffer, RENDERED_FONT_SIZE_FPS_AND_INFO);
			line_for_module_printed++;
		}
	}


	if(gGameSystem.cheats.timescale != 1.0f)
	{
		if(gGameSystem.cheats.timescale > 0.99f && gGameSystem.cheats.timescale < 1.01f)
			gGameSystem.cheats.timescale = 1.0f;
		else
		{
			sprintf____s (buffer, "timescale: %f", gGameSystem.cheats.timescale);
			renderBitmapString(writeX, (writePos+=WI), buffer, RENDERED_FONT_SIZE_FPS_AND_INFO);
		}
	}
}

	glFlush();
}


void setupOpenGLSmoothRendering()
{
	glClearColor( 0.0f, 0.0f, 0.0f, 1.0f );

	glShadeModel( GL_SMOOTH );	// The default value of GL_SMOOTH is usually better
//	glEnable( GL_DEPTH_TEST );	// Depth testing must be turned off

	// Make big points and wide lines.  (This may be commented out if desired.)
	glPointSize(3.5f);
	glLineWidth(1.0f);

	// The following commands should induce OpenGL to create round points and
	//	antialias points and lines.  (This is implementation dependent unfortunately, and
	//  may slow down rendering considerably.)
	//  You may comment these out if you wish.
	glEnable(GL_POINT_SMOOTH);
	glEnable(GL_LINE_SMOOTH);

	glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);	// Make round points, not square points
	glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);		// Antialias the lines

	glEnable(GL_DEPTH_TEST);   // Enable depth testing for z-culling
}


void FakeResizeWindow(int w, int h, double zoom_factor)
{
	std::cout << std::string("new window: dimensions: pixels(") << w << std::string(",") << h << std::string(")... zoom: ") << zoom_factor << std::endl;


	global_window_info.WindowWidth = (w>1) ? w : 2;
	global_window_info.WindowHeight = (h>1) ? h : 2;

	global_window_info.WindowWidth_half_f  = static_cast<float>(global_window_info.WindowWidth)*0.5f;
	global_window_info.WindowHeight_half_f = static_cast<float>(global_window_info.WindowHeight)*0.5f;


	glViewport(0, 0, (GLsizei) w, (GLsizei) h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	// 2D plane, center of window is (0,0), top left corner is (-X,Y)
	//gluOrtho2D(-gGameSystem.GetUniverseLimitsXXX(), gGameSystem.GetUniverseLimitsXXX(), -gGameSystem.GetUniverseLimitsYYY(), gGameSystem.GetUniverseLimitsYYY());

	//glOrtho(-gGameSystem.GetUniverseLimitsXXX(), gGameSystem.GetUniverseLimitsXXX(),
	//		-gGameSystem.GetUniverseLimitsYYY(), gGameSystem.GetUniverseLimitsYYY(), -1.0f, 1.0f);


	glDisable(GL_DEPTH_TEST);   // Enable depth testing for z-culling

	if(gGameSystem.physics_system == nullptr)
	{
	  gluPerspective( 60.0, // field of view in degrees
		 static_cast<double>(w) / static_cast<double>(h), // aspect ratio
		 0.001 * zoom_factor, // Z near
		 10000.0 * zoom_factor); // Z far
	}
	else
	{
	  gluPerspective( 60.0, // field of view in degrees
		 static_cast<double>(w) / static_cast<double>(h), // aspect ratio
		 gGameSystem.physics_system->GetMinDist_ForCameraClipping() * zoom_factor, // Z near
		 gGameSystem.physics_system->GetMaxDist_ForCameraClipping() * zoom_factor); // Z far
	}

	glDisable(GL_DEPTH_TEST);   // Enable depth testing for z-culling


	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();


	if(gGameSystem.IsInitialized()==false) {
		gGameSystem.Init();
	}

	glEnable(GL_DEPTH_TEST);   // Enable depth testing for z-culling
}

void resizeWindow(int w, int h) //called when the window is actually resized; calling this yourself won't actually change it...
{
	FakeResizeWindow(w,h,1.0);
}


int main(int argc, char** argv)
{
	srand(time(NULL));


	cout<<"Launching InvertedPendulumVisualControl ...\nCreated in 2015 by Jason Bunk."<<endl;
	cout<<"----------"<<endl;
	cout<<"main arguments:   [render-windows?]"<<endl;
	cout<<"simulation arguments:  [SIMULATIONTYPE-NUMBER]  [SIM-OPTION]"<<endl;
	cout<<"    simtypes: 0 == video-based sim,  1 == full demo w/optional webcam,  2 == optimization,  3 == jphysics simulation"<<endl;
	cout<<"    full demo (sim 1) SIM-OPTION arguments:   [WEBCAM? default:yes]   [calibrate webcam?]"<<endl;
	
	if(argc < 2) {
		exit(0);
	} else if(argc >= 3) {
		RenderVisualsUsingSFML = (atoi(argv[1]) != 0);
	}
	sf::Window* sfml_cl_window = nullptr;
	
	bool do_experimental_old_video_card_fix = false;

	if(RenderVisualsUsingSFML) {
		global_window_info.WindowWidth = 800;
		global_window_info.WindowHeight = 700;
	} else {
		global_window_info.WindowWidth = 2;
		global_window_info.WindowHeight = 2;
	}
	
	if(RenderVisualsUsingSFML) {
		global_window_info.WindowWidth_half_f  = static_cast<float>(global_window_info.WindowWidth)*0.5f;
		global_window_info.WindowHeight_half_f = static_cast<float>(global_window_info.WindowHeight)*0.5f;
//==========================================================================================================================

#ifdef BUILD_WITH_SFML
		// Request a 32-bits depth buffer when creating the window
		sf::ContextSettings contextSettings;
		contextSettings.depthBits = 32;

		// Create the main window
		sfml_window = new sf::RenderWindow(sf::VideoMode(global_window_info.WindowWidth, global_window_info.WindowHeight), "Simulation (with SFML)", sf::Style::Default, contextSettings);

		////sfml_window->setVerticalSyncEnabled(true); //VSYNC

		// Make it the active window for OpenGL calls
		sfml_window->setActive();
		sfml_window->setMouseCursorVisible(true);
		sfml_window->setKeyRepeatEnabled(false);
#endif
		setupOpenGLSmoothRendering();

//-------------------------------------------------------------------------------------

		if(do_experimental_old_video_card_fix)
		{
			cout<<"glewExperimental == true"<<endl;
			glewExperimental = GL_TRUE;
		}

#if THISPLATFORM == PLATFORM_WINDOWS
		GLenum err = glewInit();
		if (GLEW_OK != err)
		{
		  // Problem: glewInit failed, something is seriously wrong.
			cout<<"GLEW init error! "<<phys::to_string(glewGetErrorString(err))<<endl;
		}
		cout<<"Status: Using GLEW "<<phys::to_string(glewGetString(GLEW_VERSION))<<endl;
#endif
	} else {
		sfml_cl_window = new sf::Window(sf::VideoMode(global_window_info.WindowWidth, global_window_info.WindowHeight), "Simulation (with SFML)", sf::Style::Default);
		sfml_cl_window->setActive();
		sfml_cl_window->setMouseCursorVisible(true);
		sfml_cl_window->setKeyRepeatEnabled(false);
		sfml_cl_window->display();
	}
	
#if USE_SIMULATED_TIMER
	//mytimer.fixed_time_step___if_less_than_zero_all_clocks_are_realtime = 0.01;
	mytimer.restart();
#endif
	StartSimulation(argc, argv);
	
	
	if(RenderVisualsUsingSFML) {
#ifdef BUILD_WITH_SFML
		resizeWindow(global_window_info.WindowWidth, global_window_info.WindowHeight);
		
		if(font_for_debugstuff.loadFromFile("Fonts/OpenSans-Bold.ttf")==false) {
			if(font_for_debugstuff.loadFromFile("../Fonts/OpenSans-Bold.ttf")==false) {
				cout<<"Warning!! Could not load font Fonts/OpenSans-Bold.ttf from file!"<<endl;
			}
		}

		// Process events
		sf::Event event;

		// Start game loop
		while(sfml_window->isOpen())
		{
			while(sfml_window->pollEvent(event))
			{
				switch(event.type)
				{
				case sf::Event::Closed:
					sfml_window->close(); // Close window : exit
					break;
				case sf::Event::KeyPressed:
					key_up_or_down_func(event.key.code,true);
					break;
				case sf::Event::KeyReleased:
					key_up_or_down_func(event.key.code,false);
					break;
				case sf::Event::MouseButtonPressed:
					myMouseFunc(event.mouseButton.button, 1, event.mouseButton.x, event.mouseButton.y); //remnant of GLUT:  1 == down
					break;
				case sf::Event::MouseButtonReleased:
					myMouseFunc(event.mouseButton.button, 0, event.mouseButton.x, event.mouseButton.y); //remnant of GLUT:  0 == up
					break;
				case sf::Event::MouseMoved:
					myMouseMotionFunc(event.mouseMove.x, event.mouseMove.y);
					break;
				case sf::Event::Resized:
					resizeWindow(event.size.width, event.size.height);
					break;
				case sf::Event::TextEntered:
					gGameSystem.GetKeyboard()->last_unicode_key_entered = event.text.unicode;
					break;
				}
			}

			updatePhysics();
			displayFunc();
			sfml_window->display();
		}
#endif
	} else {
		sf::Event event;
		while(true) {
			updatePhysics();
			sfml_cl_window->pollEvent(event);
		}
	}
	return 0;					// If using freeglut, this line is never reached
}




void ExitSimulation()
{
#ifdef BUILD_WITH_SFML
	if(sfml_window != nullptr) {
		sfml_window->close();
	}
	exit(0);
#else
	exit(0);
#endif
}
