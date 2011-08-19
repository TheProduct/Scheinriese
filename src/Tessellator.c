//
//  Tessellator.c
//  Scheinriese
//
//  Created by Dennis Paul on 17.08.11.
//  Copyright 2011 The Product. All rights reserved.
//

#include "Tessellator.h"

/* ------------------------ */

void CALLBACK tessBeginCB(GLenum which);
void CALLBACK tessEndCB();
void CALLBACK tessErrorCB(GLenum errorCode);
void CALLBACK tessVertexCB(const GLvoid *data);
void CALLBACK tessVertexCB2(const GLvoid *data);
void CALLBACK tessCombineCB(const GLdouble newVertex[3], const GLdouble *neighborVertex[4],
                            const GLfloat neighborWeight[4], GLdouble **outData);

void CALLBACK tessBeginCB(GLenum which)
{
//	std::cout << "begin" << std::endl;
    glBegin(which);
}



void CALLBACK tessEndCB()
{
    glEnd();
}



void CALLBACK tessVertexCB(const GLvoid *data)
{
    // cast back to double type
    const GLdouble *ptr = (const GLdouble*)data;
	
//	std::cout << ptr[0] << std::endl;
    glVertex3dv(ptr);
}

void CALLBACK tessErrorCB(GLenum errorCode)
{
    const GLubyte *errorStr;
	
    errorStr = gluErrorString(errorCode);
//	std::cout << "[ERROR]: " << errorStr << std::endl;
}


GLuint tessellate(GLdouble theData[], int theSize)
{
    //GLuint id = glGenLists(1);  // create a display list
    //if(!id) return id;          // failed to create a list, return 0
	
    GLUtesselator *tess = gluNewTess(); // create a tessellator
    if(!tess) return 0;  // failed to create tessellation object, return 0
	
    // define concave quad data (vertices only)
    //  0    2
    //  \ \/ /
    //   \3 /
    //    \/
    //    1
    
	// https://devel.nuclex.org/framework/browser/graphics/fonts/Nuclex.Fonts.Content.TrueTypeImporter/trunk/Source/VectorFonts/FreeTypeFontTessellator.cpp
    
    // register callback functions
    gluTessCallback(tess, GLU_TESS_BEGIN, (void (CALLBACK *)())tessBeginCB);
    gluTessCallback(tess, GLU_TESS_END, (void (CALLBACK *)())tessEndCB);
    gluTessCallback(tess, GLU_TESS_ERROR, (void (CALLBACK *)())tessErrorCB);
    gluTessCallback(tess, GLU_TESS_VERTEX, (void (CALLBACK *)())tessVertexCB);
	
	gluTessBeginPolygon(tess, 0);                   // with NULL data
	gluTessBeginContour(tess);
    int i = 0;
	for (i = 0; i < theSize; i+=3) {
		//GLdouble myData[3] = {theData[i+0],theData[i+1],theData[i+2]};
		GLdouble myData1[3] = {100,100,100};
		gluTessVertex(tess, myData1, myData1);
	}
	gluTessEndContour(tess);
    gluTessEndPolygon(tess);
	
    // tessellate and compile a concave quad into display list
    // gluTessVertex() takes 3 params: tess object, pointer to vertex coords,
    // and pointer to vertex data to be passed to vertex callback.
    // The second param is used only to perform tessellation, and the third
    // param is the actual vertex data to draw. It is usually same as the second
    // param, but It can be more than vertex coord, for example, color, normal
    // and UV coords which are needed for actual drawing.
    // Here, we are looking at only vertex coods, so the 2nd and 3rd params are
    // pointing same address.
    //glNewList(id, GL_COMPILE);
	
    gluDeleteTess(tess);        // delete after tessellation
	
    return 0;//id;      // return handle ID of a display list
}


/* ------------------------ */

