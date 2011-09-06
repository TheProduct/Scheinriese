#include "Resources.h"

#include "cinder/app/AppBasic.h"
#include "cinder/gl/gl.h"
#include "cinder/gl/Texture.h"
#include "CinderOpenCv.h"
#include "Kinect.h"
#include "libfreenect.h"
#include "SimpleGUI.h"
#include <GLUT/glut.h>

#include "Tessellator.c"

using namespace ci;
using namespace ci::app;
using namespace std;
using namespace mowa::sgui;


class ScheinrieseApp : public AppBasic {
    
public:
	void setup();
	void mouseDown( MouseEvent event );
    void keyDown( KeyEvent event );
	void update();
	void draw();
    
private:
    void drawDebug();
    void findContours(gl::Texture theDepthImage);
    void findBlobs();
    void handleKinect();
    bool showDebug( MouseEvent event );
    
    SimpleGUI* mGui;
	float mThreshold;
	float mBlur;
    float mKinectTilt;
    
    Kinect mKinect;
	bool hasKinect;
    
	gl::Texture mDepthTexture;
    gl::Texture mColorTexture;
    gl::Texture mPreviewTexture;
    
	vector<vector<cv::Point> > mContours;
    
    bool mShowDebug;
    TextureVarControl* mDebugViewColor;
    TextureVarControl* mDebugViewDepth;
};

void ScheinrieseApp::setup()
{   
    // GUI
	mGui = new SimpleGUI(this);
    mGui->addColumn();
    mGui->addLabel("CONTROLS");
	mGui->addParam("Threshold", &mThreshold, 0, 255, 127);
	mGui->addParam("Blur", &mBlur, 1, 20, 1);
	mGui->addParam("Tilt", &mKinectTilt, -30, 30, 0);
    mGui->addColumn();
    mGui->addLabel("DEBUG VIEW");
    mGui->addParam("Show Debug", &mShowDebug, true);
//    mGui->addButton("Show Debug")->registerClick(this, &ScheinrieseApp::showDebug);
    
    mGui->load(getResourcePath(RES_SETTINGS));
    mGui->setEnabled(false);
    
    mBlur = 1;
	mThreshold = 127;
    
    mShowDebug = true;
    
	// KINECT
	hasKinect = false;
	console() << "### INFO: There are " << Kinect::getNumDevices() << " Kinects connected." << endl;
	if (Kinect::getNumDevices() >= 1) {
		mKinect = Kinect( Kinect::Device() );
		mKinect.setTilt(mKinectTilt);
		hasKinect = true;
	}
}


void ScheinrieseApp::update()
{
    handleKinect();
	
	if (mDepthTexture) {
		findContours(mDepthTexture);
	}
}

void ScheinrieseApp::handleKinect() 
{
    if (!hasKinect) {
		return;
	}
    
    if( mKinectTilt != mKinect.getTilt() ) {
		mKinect.setTilt( mKinectTilt );
    }  
    
	if( mKinect.checkNewDepthFrame() ) {
		mDepthTexture = mKinect.getDepthImage();
    }
	
	if( mKinect.checkNewVideoFrame() ) {
		mColorTexture = mKinect.getVideoImage();
    }
    
    /* debug view */
    if (mColorTexture && !mDebugViewColor) {
        mGui->addLabel("COLOR");
        mDebugViewColor = mGui->addParam("COLOR", &mColorTexture);
        mDebugViewColor->var = &mColorTexture;
        console() << "color" << endl;
    }
    
    if (mDepthTexture && !mDebugViewDepth) {
        mGui->addLabel("DEPTH");
        mDebugViewDepth = mGui->addParam("DEPTH", &mDepthTexture);
        mDebugViewDepth->var = &mDepthTexture;
        console() << "depth" << endl;
    }
}

void ScheinrieseApp::draw()
{
	gl::clear( Color( 0, 0, 0 ) ); 
	gl::setMatricesWindow( getWindowWidth(), getWindowHeight() );
	glColor3f( 1.0f, 1.0f, 1.0f );
	
    /* debug view */
    drawDebug();
    
    //	glPushMatrix();
    //	for (int i = 0; i < mContours.size(); i++) {
    //		glColor3f(0.0f,1.0f,1.0f);
    //		glBegin( GL_LINE_STRIP );
    //		for (int j = 0; j < mContours[i].size(); j+=10) {
    //			glVertex2f( mContours[i][j].x, mContours[i][j].y);
    //		}
    //		glEnd();
    //	}
    //	glPopMatrix();
    
    /*
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
     tessellate(quad1, 12);
     glPopMatrix();
     
     for (int i = 0; i < mContours.size(); i++) {
     gl::setMatricesWindow( getWindowWidth(), getWindowHeight() );
     glPushMatrix();
     for (int j = 0; j < mContours[i].size(); j+=1) {
     
     }
     glPopMatrix();
     }
     */	
	// GUI
	mGui->draw();
}

void ScheinrieseApp::drawDebug()
{
    if ( mShowDebug ) {
        if ( mDepthTexture ) {
            gl::draw( mDepthTexture );
        }
        
        if ( mColorTexture ) {
            gl::draw( mColorTexture );
        }
        
        if ( mPreviewTexture ) {
            gl::draw( mPreviewTexture );
        }
    }
}


void ScheinrieseApp::mouseDown( MouseEvent event )
{
}

bool ScheinrieseApp::showDebug( MouseEvent event )
{ 
    return false;
}


void ScheinrieseApp::keyDown( KeyEvent event )
{
    switch(event.getChar()) {				
        case 'd': mGui->dump(); break;
        case 'l': mGui->load(getResourcePath(RES_SETTINGS)); break;
        case 's': mGui->save(getResourcePath(RES_SETTINGS)); break;
    }
    switch(event.getCode()) {
        case KeyEvent::KEY_ESCAPE: quit(); break;
        case KeyEvent::KEY_SPACE: mGui->setEnabled(!mGui->isEnabled());
    }
}


void ScheinrieseApp::findContours(gl::Texture theDepthImage)
{    
//ImageSourceRef theDepthImage) {
    // images that opencv work on
    cv::Mat input( toOcv( Channel8u( theDepthImage ) ) );
    cv::Mat blurred;
    cv::Mat thresholded;
    cv::Mat output;
    
    cv::blur(input, blurred, cv::Size(mBlur, mBlur));
    cv::threshold( blurred, thresholded, mThreshold, 255, CV_THRESH_BINARY);
    
    // store thresholded image in a texture
    cv::cvtColor( thresholded, output, CV_GRAY2RGB );
    mPreviewTexture = gl::Texture( fromOcv(output) );
    
    // find and store contours
    mContours.clear();
    cv::findContours(thresholded, mContours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);	
}

void ScheinrieseApp::findBlobs() {
    CBlobResult blobs;
    int i;
    CBlob *currentBlob;
    IplImage *original, *originalThr;
    
    // load an image and threshold it
    original = cvLoadImage("pic1.png", 0);
    cvThreshold( original, originalThr, 100, 0, 255, CV_THRESH_BINARY );
    
    // find non-white blobs in thresholded image
    blobs = CBlobResult( originalThr, NULL, 255 );
    // exclude the ones smaller than param2 value
    blobs.Filter( blobs, B_EXCLUDE, CBlobGetArea(), B_LESS, param2 );
    
    // get mean gray color of biggest blob
    CBlob biggestBlob;
    CBlobGetMean getMeanColor( original );
    double meanGray;
    
    blobs.GetNth( CBlobGetArea(), 0, biggestBlob );
    meanGray = getMeanColor( biggestBlob );
    
    // display filtered blobs
    cvMerge( originalThr, originalThr, originalThr, NULL, displayedImage );
    
    for (i = 0; i < blobs.GetNumBlobs(); i++ )
    {
        currentBlob = blobs.GetBlob(i);
        currentBlob->FillBlob( displayedImage, CV_RGB(255,0,0));
    }
}

CINDER_APP_BASIC( ScheinrieseApp, RendererGl )
