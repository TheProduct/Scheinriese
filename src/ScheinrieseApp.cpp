#include "cinder/app/AppBasic.h"
#include "cinder/gl/gl.h"
#include "cinder/gl/Texture.h"
#include "CinderOpenCv.h"
#include "Kinect.h"
#include "libfreenect.h"
#include "SimpleGUI.h"
#include <GLUT/glut.h>

using namespace ci;
using namespace ci::app;
using namespace std;
using namespace mowa::sgui;

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
	std::cout << "begin" << std::endl;
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
	
	std::cout << ptr[0] << std::endl;
    glVertex3dv(ptr);
}

void CALLBACK tessErrorCB(GLenum errorCode)
{
    const GLubyte *errorStr;
	
    errorStr = gluErrorString(errorCode);
	std::cout << "[ERROR]: " << errorStr << std::endl;
}


GLuint tessellate1(GLdouble theData[], int theSize)
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
	for (int i = 0; i < theSize; i+=3) {
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


class ScheinrieseApp : public AppBasic {
    
public:
	void setup();
	void mouseDown( MouseEvent event );	
	void update();
	void draw();
    
private:
    void findContours(ImageSourceRef theDepthImage);

    SimpleGUI* mGui;
	float threshold;
	float blur;
    
    Kinect mKinect;
	bool hasKinect;
    
    ImageSourceRef mDepthImage;
	gl::Texture mDepthTexture;
    gl::Texture mColorTexture;
    gl::Texture mPreviewTexture;
    
	vector<vector<cv::Point> > mContours;
};

void ScheinrieseApp::setup()
{   
    // GUI
	mGui = new SimpleGUI(this);
	mGui->addParam("Threshold", &threshold, 0, 255, 100);
	mGui->addParam("Blur", &blur, 0, 20, 10);
    
	// KINECT
	hasKinect = false;
	console() << "### INFO: There are " << Kinect::getNumDevices() << " Kinects connected." << std::endl;
	if (Kinect::getNumDevices() >= 1) {
		mKinect = Kinect( Kinect::Device() );
		mKinect.setTilt(10);
		hasKinect = true;
	}

	// OPENCV
	blur = 11;
	threshold = 0;
}

void ScheinrieseApp::update()
{
    if (!hasKinect) {
		return;
	}

	if( mKinect.checkNewDepthFrame() ) {
		mDepthImage = mKinect.getDepthImage();
		mDepthTexture = mDepthImage;
	}
	
	if( mKinect.checkNewVideoFrame() ) {
		mColorTexture = mKinect.getVideoImage();
	}	
	
	if (mDepthImage) {
		findContours(mDepthImage);
	}
}

void ScheinrieseApp::draw()
{
	gl::clear( Color( 0, 0, 0 ) ); 
	
	
	// KINECT
	if (!hasKinect) {
		return;
	}
	glColor3f( 1.0f, 1.0f, 1.0f );
	gl::setMatricesWindow( getWindowWidth(), getWindowHeight() );
	if( mDepthTexture ) {
		gl::draw( mDepthTexture );
	}
	if( mColorTexture ) {
		gl::draw( mColorTexture, Vec2i( 640, 0 ) );
	}
	
	// OPENCV
	gl::setMatricesWindow( getWindowWidth(), getWindowHeight() );
	glColor3f( 1.0f, 1.0f, 1.0f );
	if( mPreviewTexture ) {
		gl::draw( mPreviewTexture, Vec2i( 0, 480 ) );
	}
	glPushMatrix();
	glTranslatef(640,480,0);
	for (int i = 0; i < mContours.size(); i++) {
		glColor3f(0.0f,1.0f,1.0f);
		glBegin( GL_LINE_STRIP );
		for (int j = 0; j < mContours[i].size(); j+=10) {
			glVertex2f( mContours[i][j].x, mContours[i][j].y);
		}
		glEnd();
	}
	glPopMatrix();
    
    
	//for (int i = 0; i < mContours.size(); i++) {
	//		vector<p2t::Point*> polyline;
	//		for (int j = 0; j < mContours[i].size(); j+=1) {
	//			polyline.push_back(new p2t::Point(mContours[i][j].x, mContours[i][j].y));
	//		}
	//		if (polyline.size() > 9) {
	//		cout << "Number of line points = " << polyline.size() << endl;
	//		CDT* cdt = new CDT(polyline);
	//		cdt->Triangulate();
	//		//vector<p2t::Triangle*> triangles = cdt->GetTriangles();
	//		//cout << "Number of triangles = " << triangles.size() << endl;
	//		}
	//	}
	
	gl::setMatricesWindow( getWindowWidth(), getWindowHeight() );
	glPushMatrix();
	glTranslatef(200,200,0);
	GLdouble quad1[4*3] = { -100,300,0, 0,0,0, 100,300,0, 0,200,0 };
	tessellate1(quad1, 12);
	glPopMatrix();
	
	for (int i = 0; i < mContours.size(); i++) {
		gl::setMatricesWindow( getWindowWidth(), getWindowHeight() );
		glPushMatrix();
		for (int j = 0; j < mContours[i].size(); j+=1) {
			
		}
        glPopMatrix();
	}
	
	// GUI
	mGui->draw();
}

void ScheinrieseApp::mouseDown( MouseEvent event )
{
}

void ScheinrieseApp::findContours(ImageSourceRef theDepthImage) {
    // images that opencv work on
	cv::Mat input( toOcv( Channel8u( theDepthImage ) ) );
	cv::Mat blurred;
	cv::Mat thresholded;
	cv::Mat output;
	
	cv::blur(input, blurred, cv::Size(blur,blur));
	cv::threshold( blurred, thresholded, threshold, 255, CV_THRESH_BINARY);
	
	// store thresholded image in a texture
	cv::cvtColor( thresholded, output, CV_GRAY2RGB );
	mPreviewTexture = gl::Texture( fromOcv(output) );
	
	// find and store contours
	mContours.clear();
	cv::findContours(thresholded, mContours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);	
}

CINDER_APP_BASIC( ScheinrieseApp, RendererGl )
