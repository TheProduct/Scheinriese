//
//  Tessellator.h
//  Scheinriese
//
//  Created by Dennis Paul on 17.08.11.
//  Copyright 2011 The Product. All rights reserved.
//

#include <stdio.h>
#include <GLUT/glut.h>

#ifndef Scheinriese_Tessellator_h
#define Scheinriese_Tessellator_h

void CALLBACK tessBeginCB(GLenum which);
void CALLBACK tessEndCB();
void CALLBACK tessErrorCB(GLenum errorCode);
void CALLBACK tessVertexCB(const GLvoid *data);
void CALLBACK tessVertexCB2(const GLvoid *data);
void CALLBACK tessCombineCB(const GLdouble newVertex[3], const GLdouble *neighborVertex[4],
                            const GLfloat neighborWeight[4], GLdouble **outData);

#endif
