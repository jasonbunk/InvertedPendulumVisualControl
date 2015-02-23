/*
 * Render text onscreen.
 *
 * Author: Jason Bunk
 * Web page: http://sites.google.com/site/jasonbunk
 * License: Apache License Version 2.0, January 2004
 * Copyright (c) 2015 Jason Bunk
 */
#include "stdafx.h"
#include "GlutFontRender.h"
#include "TryIncludeJPhysics.h"
#include "IncludeGameEngine.h"
#include "main.h"



/*extern*/ sf::RenderWindow* sfml_window = nullptr;
/*extern*/ sf::Font font_for_debugstuff = sf::Font();



void renderBitmapString(
		float posx,
		float posy,
		const char *string,
		float charheight/*=18.0f*/,
		float* returned_width_of_printed_pixels/*=NULL*/)
{
	if(returned_width_of_printed_pixels != NULL)
		*returned_width_of_printed_pixels = 0.0f;

	sf::Text text(string, font_for_debugstuff);
	text.setColor(sf::Color(255, 255, 255, 255));

	float pixelheight = physmath::RoundFloatToInteger(charheight);
	text.setCharacterSize(physmath::RoundFloatToInt(pixelheight));

	//text.setPosition(	posx + gGameSystem.GetUniverseLimitsXXX(),
	//					(gGameSystem.GetUniverseLimitsYYY() * 2.0f) - (pixelheight + posy + gGameSystem.GetUniverseLimitsYYY()));
	
	text.setPosition(	posx + global_window_info.WindowWidth_half_f,
						(global_window_info.WindowHeight_half_f * 2.0f) - (pixelheight + posy + global_window_info.WindowHeight_half_f));

	//text.setPosition(350.0f, 350.0f);

	// Draw some text on top of our OpenGL object
	sfml_window->pushGLStates();
	sfml_window->draw(text);
	sfml_window->popGLStates();

#ifdef COMPILE_WITH_FREEGLUT
	const char *c;
	glRasterPos2f(posx, posy);

	if(ssmall)
	{
		for (c=string; *c != '\0'; c++)
		{
			glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, *c);

			if(returned_width_of_printed_pixels != NULL)
				*returned_width_of_printed_pixels += ((float)glutBitmapWidth(GLUT_BITMAP_HELVETICA_12, *c));
		}
	}
	else
	{
		for (c=string; *c != '\0'; c++)
		{
			glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, *c);

			if(returned_width_of_printed_pixels != NULL)
				*returned_width_of_printed_pixels += ((float)glutBitmapWidth(GLUT_BITMAP_HELVETICA_18, *c));
		}
	}
#endif
#ifdef __COMPILE_WITH_SDL_

	GLuint stringTex = 0;

	if(font_arial != NULL)
	{
		SDL_Surface* sdtextured = 0;
		stringTex = TextToTexture(font_arial, 255, 255, 255, string, &sdtextured);

		float foxx = static_cast<float>(last_drawn_font_tex_w) * 1.2f * gGameSystem.GetGLZoom();
		float foyy = static_cast<float>(last_drawn_font_tex_h) * 1.25f * gGameSystem.GetGLZoom();

		if(ssmall)
		{
			foxx *= 0.72f;
			foyy *= 0.72f;
		}

		glColor3ub( 255, 255, 255 );
		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, stringTex);
		glBegin(GL_QUADS);
			glTexCoord2f( 0.0f, 1.0f ); glVertex2f( posx, posy );
			glTexCoord2f( 1.0f, 1.0f ); glVertex2f( posx+foxx, posy );
			glTexCoord2f( 1.0f, 0.0f ); glVertex2f( posx+foxx, posy+foyy );
			glTexCoord2f( 0.0f, 0.0f ); glVertex2f( posx, posy+foyy );
		glEnd();

		glFinish();

		SDL_FreeSurface(sdtextured);

		//delete the texture we just drew (we'll never use it again)
		glDeleteTextures(1, &stringTex);
	}
	else
		cout<<"renderBitmapString() - Failed to load font!!"<<endl;
#endif
}
