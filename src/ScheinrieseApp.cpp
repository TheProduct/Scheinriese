// TODO: 'Mac OS X Lion – Disable app resume and window restore functionality' System Preferences -> General -> Remove the check from “Restore windows when quitting and re-opening apps”.
// TODO: just for the fun of it: implement image based blob detection

#include "Resources.h"

#include "cinder/app/AppBasic.h"
#include "cinder/gl/gl.h"
#include "cinder/gl/Texture.h"
#include "cinder/Capture.h"
#include "cinder/gl/GlslProg.h"

#include "CinderOpenCv.h"
#include "Kinect.h"
#include "cvblob.h"
#include "SimpleGUI.h"
#include "cinder/TriMesh.h"
#include "cinder/Triangulate.h"
#include "OscSender.h"

#include <ApplicationServices/ApplicationServices.h>

using namespace ci;
using namespace ci::app;
using namespace std;
using namespace mowa::sgui;
using namespace cvb;


const std::string       RES_SETTINGS = "settings.txt";
const int               CAMERA_WIDTH = 640;
const int               CAMERA_HEIGHT = 480;

float edge_fade(float x, float n);

class Sampler {
private:
    static const int    SCALE_SAMPLER_SIZE = 10;
    float               mScaleSamplerAverageTotal;
    float               mScaleSampler[SCALE_SAMPLER_SIZE];
    float               mScaleSamplerAverage;
    int                 mScaleSamplerCounter;

public:
    Sampler() {
        mScaleSamplerAverageTotal = 0.0;
        mScaleSamplerAverage = 0.0;
        mScaleSamplerCounter = 0;
        for (int i=0; i<SCALE_SAMPLER_SIZE; ++i) {
            mScaleSampler[i] = 0.0;
        }
    }

    void advance(const float mValue) {
        mScaleSamplerAverageTotal -= mScaleSampler[mScaleSamplerCounter];
        mScaleSampler[mScaleSamplerCounter] = mValue;
        mScaleSamplerAverageTotal += mValue;
        mScaleSamplerCounter++;
        mScaleSamplerCounter %= SCALE_SAMPLER_SIZE;
        mScaleSamplerAverage = mScaleSamplerAverageTotal / float(SCALE_SAMPLER_SIZE);    
    }
    
    float getAverage() {
        return mScaleSamplerAverage;
    }
};


class ScheinrieseApp : public AppBasic {
public:
	void setup();
	void update();
	void draw();
    void keyDown( KeyEvent pEvent );
    void quit();
    void mouseMove( MouseEvent event ) {};
    void mouseDrag( MouseEvent event ) {};
    
private:
    void        drawBlob(const CvBlob & pBlob);
    void        drawBlobAsMesh(const CvBlob & pBlob);
    void        drawBlobAsRect(const CvBlob & pBlob);
    void        drawBlobAsOutlinedMesh(const CvBlob & pBlob, const int pPolygonStyle);
    void        DEBUGdrawBlobsAndTracks( const CvBlobs& pBlobs, const CvTracks& pTracks );
    void        DEBUGdrawCameraImages();
    void        drawMesh( const TriMesh2d &mesh );
    TriMesh2d       triangulateShape( const Shape2d & mShape );
    Shape2d         convertPolygonToShape2d( const CvContourPolygon & polygon );
    Surface         getDepthImage();
    Surface         getVideoImage();
    void            updateDepthImageBuffer();
    void            estimateDistanceFromBlobHeight(const CvBlob & pBlob);
    void            estimateDistanceFromBlobDepth(const CvBlob & pBlob);
    void            estimatePosition(const CvBlob & pBlob);
    float           getAlphaByDistance();
    void            heartbeat(double mDeltaTime);

    /* properties */
    SimpleGUI *             mGui;
    
    int                     WINDOW_WIDTH;
    int                     WINDOW_HEIGHT;    
    
    int                     FRAME_RATE;   
    bool                    FULLSCREEN;
    int                     KINECT_ANGLE;  
    int                     BLOB_THRESHOLD;
    int                     BLOB_THRESHOLD_METHOD;
    int                     BLOB_SRC_IMAGE_TYPE;
    int                     BLOB_MIN_AREA;
    int                     BLOB_MAX_AREA;
    int                     BLOB_BLUR;
    
    float                   POLYGON_REDUCTION_MIN_DISTANCE;
    float                   POLYGON_OUTLINE_WIDTH;
    float                   DEPTH_TEX_ALIGN_X;
    float                   DEPTH_TEX_ALIGN_Y;
    float                   RGB_DEPTH_TEXTURE_ALIGN_SCALE;
    float                   BGR_IMG_GREY_SCALE_TOP;
    float                   BGR_IMG_GREY_SCALE_MIDDLE;
    float                   BGR_IMG_GREY_SCALE_BOTTOM;
    int                     TRACK_MAX_NUMBER_OF_FRAMES_INACTIVE;
    float                   TRACK_MATCHING_DISTANCE;
    float                   MASK_LEFT_TOP;
    float                   MASK_LEFT_BOTTOM;
    float                   MASK_RIGHT_TOP;
    float                   MASK_RIGHT_BOTTOM;
    float                   MASK_CIRCLE_RADIUS;
    int                     MASK_CIRCLE_X_POS;
    int                     MASK_CIRCLE_Y_POS;
    
    int                     DRAW_BLOB_AS;
    int                     DRAW_BIGGEST_ONLY;
    int                     DISTANCE_ESTIMATION_STRATEGY;
    
    float                   ROI_LEFT_TOP;
    float                   ROI_LEFT_BOTTOM;
    float                   ROI_RIGHT_TOP;
    float                   ROI_RIGHT_BOTTOM;
    float                   VIDEO_IMAGE_HUE;
    float                   VIDEO_IMAGE_SATURATION;
    float                   VIDEO_IMAGE_BRIGHTNESS;
    
    float                   BLOB_DEPTH_RESCALED_FAR;
    float                   BLOB_DEPTH_RESCALED_NEAR;
    float                   BLOB_DEPTH_CLAMP_FAR;
    float                   BLOB_DEPTH_CLAMP_NEAR;
    float                   BLOB_DEPTH_RESCALE_EXPONENT;

    float                   BLOB_HEIGHT_RESCALED_FAR;
    float                   BLOB_HEIGHT_RESCALED_NEAR;
    float                   BLOB_HEIGHT_CLAMP_FAR;
    float                   BLOB_HEIGHT_CLAMP_NEAR;
    float                   BLOB_HEIGHT_RESCALE_EXPONENT;
    
    float                   DEPTH_RAMP_MIN;
    float                   DEPTH_RAMP_MAX;
    
    float                   DISTANCE_FADE_EXP;
    float                   DISTANCE_FADE_EDGE_NEAR;
    float                   DISTANCE_FADE_EDGE_FAR;


    /* output */
    LabelControl *          mFPSOut;
    LabelControl *          mBlobsOut;
        
    /* kinect */
    Kinect *                    mKinect;
	gl::Texture                 mColorTexture;
	gl::Texture                 DEBUGmDepthTexture;
	gl::Texture                 DEBUGmBlobTexture;
    static const unsigned int   DEPTH_BUFFER_IMAGE_SIZE = 6;
    Surface                     mDepthImages[DEPTH_BUFFER_IMAGE_SIZE];
    int                         mDepthImageCounter;
    Surface                     mProcessedImage;
    Surface                     mAveragedSurface;
    
    /* blobs */
    CvBlobs                 mBlobs;
    CvTracks                mTracks;
    
    float       mFixedYPosition;
    float       mScaleFromHeight; // put this in a sampler
    float       mScaleFromDepth; // put this in a sampler

    Sampler         mSamplerScaleFromHeight;
    Sampler         mSamplerScaleFromDepth;
    Sampler         mSamplerDistanceFromDepth;
    Sampler         mSamplerXPosition;
    
    /* watchdog */
    osc::Sender     mWatchdogSender;
	std::string     mWatchdogHost;
	int             mWatchdogPort;
    float           mWatchdogCounter;
    float           mWatchdogInterval;
};

void ScheinrieseApp::setup() { 
    
    console() << "+++ Scheinriese (PID " << getpid() << ")." << endl;

    mScaleFromHeight = 0;
    mScaleFromDepth = 0;
    mFixedYPosition = CAMERA_HEIGHT; // we assume that all blobs start at the bottom

    /* watchdog */
    mWatchdogHost = "localhost";
	mWatchdogPort = 8080;
	mWatchdogSender.setup(mWatchdogHost, mWatchdogPort);
    mWatchdogCounter = 0.0;
    mWatchdogInterval = 2.5;

    /* settings */
    mGui = new SimpleGUI(this);
    
    mGui->addColumn(20, 20);
    
    mGui->addParam("WINDOW_WIDTH", &WINDOW_WIDTH, 0, 2048, 640)->active=false;
    mGui->addParam("WINDOW_HEIGHT", &WINDOW_HEIGHT, 0, 2048, 480)->active=false;
    mGui->addParam("FULLSCREEN", &FULLSCREEN, false, 0)->active=false;
    mGui->addParam("FRAME_RATE", &FRAME_RATE, 1, 120, 30);

    mGui->addParam("BGR_IMG_GREY_SCALE_TOP", &BGR_IMG_GREY_SCALE_TOP, 0, 1, 0.0);
    mGui->addParam("BGR_IMG_GREY_SCALE_MIDDLE", &BGR_IMG_GREY_SCALE_MIDDLE, 0, 1, 0.2);
    mGui->addParam("BGR_IMG_GREY_SCALE_BOTTOM", &BGR_IMG_GREY_SCALE_BOTTOM, 0, 1, 0.7);
    mGui->addParam("DEPTH_TEX_ALIGN_X", &DEPTH_TEX_ALIGN_X, 0, 50, 20);
    mGui->addParam("DEPTH_TEX_ALIGN_Y", &DEPTH_TEX_ALIGN_Y, 0, 50, 20);
    mGui->addParam("DEPTH_TEX_ALIGN_SCALE", &RGB_DEPTH_TEXTURE_ALIGN_SCALE, 0.9, 1.0, 0.925);
    mGui->addParam("POLYGON_REDUCTION_MIN_DISTANCE", &POLYGON_REDUCTION_MIN_DISTANCE, 0, 20, 10);
    mGui->addParam("POLYGON_OUTLINE_WIDTH", &POLYGON_OUTLINE_WIDTH, 0, 50, 10);
    mGui->addParam("TRACK_MAX_NUMBER_OF_FRAMES_INACTIVE", &TRACK_MAX_NUMBER_OF_FRAMES_INACTIVE, 0, 240, 30);
    mGui->addParam("TRACK_MATCHING_DISTANCE", &TRACK_MATCHING_DISTANCE, 0, 50, 5);

    mGui->addParam("KINECT_ANGLE", &KINECT_ANGLE, -31, 30, 20);
    mGui->addParam("BLOB_THRESHOLD", &BLOB_THRESHOLD, 1, 255, 30);
    mGui->addParam("BLOB_THRESHOLD_METHOD", &BLOB_THRESHOLD_METHOD, 0, 2, 0);
    mGui->addParam("BLOB_SRC_IMAGE_TYPE", &BLOB_SRC_IMAGE_TYPE, 0, 2, 0);
    
    mGui->addParam("BLOB_MIN_AREA", &BLOB_MIN_AREA, 1, 100000, 100);
    mGui->addParam("BLOB_MAX_AREA", &BLOB_MAX_AREA, 1, 500000, 500000);
    mGui->addParam("BLOB_BLUR", &BLOB_BLUR, 0, 50, 7  );

    mGui->addParam("DRAW_BLOB_AS", &DRAW_BLOB_AS, 0, 4, 3);
    mGui->addParam("DRAW_BIGGEST_ONLY", &DRAW_BIGGEST_ONLY, 0, 1, 0);
    mGui->addParam("DISTANCE_ESTIMATION_STRATEGY", &DISTANCE_ESTIMATION_STRATEGY, 0, 2, 1);

    mGui->addColumn(200, 20);

    mGui->addParam("MASK_LEFT_TOP", &MASK_LEFT_TOP, 115, 135, 127)->active=false;
    mGui->addParam("MASK_LEFT_BOTTOM", &MASK_LEFT_BOTTOM, 120, 140, 131)->active=false;
    mGui->addParam("MASK_RIGHT_TOP", &MASK_RIGHT_TOP, 120, 140, 131)->active=false;
    mGui->addParam("MASK_RIGHT_BOTTOM", &MASK_RIGHT_BOTTOM, 120, 140, 131)->active=false;
    mGui->addParam("MASK_CIRCLE_RADIUS", &MASK_CIRCLE_RADIUS, 0, 200, 50)->active=false;
    mGui->addParam("MASK_CIRCLE_X_POS", &MASK_CIRCLE_X_POS, -50, 50, 0)->active=false;
    mGui->addParam("MASK_CIRCLE_Y_POS", &MASK_CIRCLE_Y_POS, -100, 100, 0)->active=false;

    mGui->addParam("ROI_LEFT_TOP", &ROI_LEFT_TOP, 0, 400, 145);
    mGui->addParam("ROI_LEFT_BOTTOM", &ROI_LEFT_BOTTOM, 0, 400, 145);
    mGui->addParam("ROI_RIGHT_TOP", &ROI_RIGHT_TOP, 0, 400, 160);
    mGui->addParam("ROI_RIGHT_BOTTOM", &ROI_RIGHT_BOTTOM, 0, 400, 160);

    mGui->addParam("VIDEO_IMAGE_HUE", &VIDEO_IMAGE_HUE, 0, 3, 1);
    mGui->addParam("VIDEO_IMAGE_SATURATION", &VIDEO_IMAGE_SATURATION, 0, 3, 0.5);
    mGui->addParam("VIDEO_IMAGE_BRIGHTNESS", &VIDEO_IMAGE_BRIGHTNESS, 0, 3, 1);

    mGui->addParam("BLOB_DEPTH_RESCALED_FAR", &BLOB_DEPTH_RESCALED_FAR, 0, 3, 1.6);
    mGui->addParam("BLOB_DEPTH_RESCALED_NEAR", &BLOB_DEPTH_RESCALED_NEAR, 0, 1, 0.1);
    mGui->addParam("BLOB_DEPTH_CLAMP_FAR", &BLOB_DEPTH_CLAMP_FAR, 0, 255, 32);
    mGui->addParam("BLOB_DEPTH_CLAMP_NEAR", &BLOB_DEPTH_CLAMP_NEAR, 0, 255, 142);
    mGui->addParam("BLOB_DEPTH_RESCALE_EXPONENT", &BLOB_DEPTH_RESCALE_EXPONENT, 0, 5, 1);

    mGui->addParam("BLOB_HEIGHT_RESCALED_FAR", &BLOB_HEIGHT_RESCALED_FAR, 0, 600, 480);
    mGui->addParam("BLOB_HEIGHT_RESCALED_NEAR", &BLOB_HEIGHT_RESCALED_NEAR, 0, 300, 48);
    mGui->addParam("BLOB_HEIGHT_CLAMP_FAR", &BLOB_HEIGHT_CLAMP_FAR, 0, 480, 200);
    mGui->addParam("BLOB_HEIGHT_CLAMP_NEAR", &BLOB_HEIGHT_CLAMP_NEAR, 0, 480, 480);
    mGui->addParam("BLOB_HEIGHT_RESCALE_EXPONENT", &BLOB_HEIGHT_RESCALE_EXPONENT, 0, 5, 2);
    
    mGui->addParam("DEPTH_RAMP_MIN", &DEPTH_RAMP_MIN, 0, 1, 1);
    mGui->addParam("DEPTH_RAMP_MAX", &DEPTH_RAMP_MAX, 0, 1, 0.725);

    mGui->addParam("DISTANCE_FADE_EXP", &DISTANCE_FADE_EXP, 0, 8, 1);
    mGui->addParam("DISTANCE_FADE_EDGE_NEAR", &DISTANCE_FADE_EDGE_NEAR, 0.0, 0.2, 0.1);
    mGui->addParam("DISTANCE_FADE_EDGE_FAR", &DISTANCE_FADE_EDGE_FAR, 0.8, 1.0, 0.9);
//    mGui->addParam("DISTANCE_CLAMP_EDGE_NEAR", &DISTANCE_CLAMP_EDGE_NEAR, 0.0, 0.2, 0.05);
//    mGui->addParam("DISTANCE_CLAMP_EDGE_FAR", &DISTANCE_CLAMP_EDGE_FAR, 0.8, 1.0, 0.95);
    
    mGui->load(getResourcePath(RES_SETTINGS));
    mGui->setEnabled(false);
    mGui->dump(); 

    // output
    mGui->addSeparator();
    mFPSOut = mGui->addLabel("");
    mBlobsOut = mGui->addLabel("");
    
    /* kinect */
	console() << "+++ found " << Kinect::getNumDevices() << " kinect(s)." << std::endl;
	if (Kinect::getNumDevices() >= 1) {
		mKinect = new Kinect( Kinect::Device(0) );
        mKinect->setLedColor( Kinect::LED_YELLOW ); // LED_OFF
        console() << "+++ waiting for kinect ..." << endl;
        while(!mKinect->checkNewDepthFrame()) {}
        /* fill depth buffer */
        mAveragedSurface = Surface( CAMERA_WIDTH, CAMERA_HEIGHT, false, SurfaceChannelOrder::RGB );
        for (int i=0; i<DEPTH_BUFFER_IMAGE_SIZE; ++i) {
            mDepthImages[i] = mKinect->getDepthImage(); 
        }
        mDepthImageCounter = 0;
        mProcessedImage = getDepthImage();
        DEBUGmDepthTexture = getDepthImage();
        DEBUGmBlobTexture = getDepthImage();

        /* - */
        while(!mKinect->checkNewVideoFrame()) {}
        mColorTexture = getVideoImage();
        console() << "depth: " << DEBUGmDepthTexture.getWidth() << ", " << DEBUGmDepthTexture.getHeight() << endl;
        console() << "color: " << mColorTexture.getWidth() << ", " << mColorTexture.getHeight() << endl;
	} else {
        exit (1);
    }
    
    /* app */
    if (FULLSCREEN) {
        hideCursor();
    }
    setWindowSize( WINDOW_WIDTH, WINDOW_HEIGHT );
    setFrameRate( FRAME_RATE );
    setFullScreen( FULLSCREEN );
    
    console() << "+++ done setting up." << endl;
}



Surface ScheinrieseApp::getDepthImage() {
    return mAveragedSurface;
}

Surface ScheinrieseApp::getVideoImage() {
    return mKinect->getVideoImage();
}

void ScheinrieseApp::updateDepthImageBuffer() {   
    mDepthImages[mDepthImageCounter] = mKinect->getDepthImage();
    mDepthImageCounter++;
    mDepthImageCounter %= DEPTH_BUFFER_IMAGE_SIZE;
    
    /* --- */
    mAveragedSurface = Surface( CAMERA_WIDTH, CAMERA_HEIGHT, false, SurfaceChannelOrder::RGB );
    
    uint8_t * mData = mAveragedSurface.getData();
    const int mLength = CAMERA_WIDTH * CAMERA_HEIGHT * 3;
    
    for (int j=0; j<mLength; ++j) {
        int mAveragedPixel = 0;
        for (int i=0; i<DEPTH_BUFFER_IMAGE_SIZE; ++i) {
            const uint8_t * mSurfaceData = mDepthImages[i].getData(); 
            mAveragedPixel += mSurfaceData[j];
        }
        mAveragedPixel /= DEPTH_BUFFER_IMAGE_SIZE;
        mData[j] = mAveragedPixel;
    }
    
    Surface::Iter iter = mAveragedSurface.getIter( );
    while( iter.line() ) {
        const float mRatio = float(iter.y()) / float(CAMERA_HEIGHT);
        const float mRange = DEPTH_RAMP_MAX - DEPTH_RAMP_MIN;
        const float mValue = mRange * mRatio + DEPTH_RAMP_MIN;
        while( iter.pixel() ) {
            iter.r() *= mValue;
            iter.g() *= mValue;
            iter.b() *= mValue;
        }
    }
}

void ScheinrieseApp::update() {

    /* --- */
    setFrameRate( FRAME_RATE );
    
    /* info */
    if (mGui->isEnabled()) {
        {
            stringstream mStr;
            mStr << "FPS: " << getAverageFps();
            mFPSOut->setText(mStr.str());
        }
        {
            stringstream mStr;
            mStr << "BLOBS: " << mBlobs.size();
            mBlobsOut->setText(mStr.str());
        }
    }

    /* kinect */
    if ( mKinect ) {
        bool mNewVideoFrame = mKinect->checkNewVideoFrame();
        bool mNewDepthFrame = mKinect->checkNewDepthFrame();
        if (mKinect->getTilt() != KINECT_ANGLE) {
            mKinect->setTilt(KINECT_ANGLE);
        }
        /* video image */
        if ( mNewVideoFrame ) {
            Surface mSurface = getVideoImage();
            Surface::Iter iter = mSurface.getIter( );
            while( iter.line() ) {
                while( iter.pixel() ) {
                    Vec3f mHSV = rgbToHSV( Color(iter.r(), iter.g(), iter.b()) );
                    mHSV.x *= VIDEO_IMAGE_HUE;
                    mHSV.y *= VIDEO_IMAGE_SATURATION;
                    mHSV.z *= VIDEO_IMAGE_BRIGHTNESS;
                    Color mColor = hsvToRGB( mHSV );
                    iter.r() = min(255.0f, max(0.0f, mColor.r));
                    iter.g() = min(255.0f, max(0.0f, mColor.g));
                    iter.b() = min(255.0f, max(0.0f, mColor.b));
                }
            }
            mColorTexture.update(mSurface);
        }
        /* depth image */
        if ( mNewDepthFrame ) {
            Surface mSurface;
            switch (BLOB_SRC_IMAGE_TYPE) {
                case 0:
                    updateDepthImageBuffer();
                    mSurface = getDepthImage();
                    break;
                case 1:
                    mSurface = getVideoImage();
                    break;
            }
            /* copy to grey scale manually */
            Surface::Iter iter = mSurface.getIter();
            IplImage* mGreyImage = cvCreateImage(cvSize(mSurface.getWidth(), mSurface.getHeight()), IPL_DEPTH_8U, 1);
            int i = 0;
            while( iter.line() ) {
                while( iter.pixel() ) {
                    mGreyImage->imageData[i] = (iter.r() + iter.g() + iter.b()) / 3;
                    i++;
                }
            }
            
            /* threshold */
            if (BLOB_BLUR >= 1) {
                cvSmooth(mGreyImage, mGreyImage, CV_BLUR, BLOB_BLUR, BLOB_BLUR);
            }
            switch (BLOB_THRESHOLD_METHOD) {
                case 0:
                    cvThreshold(mGreyImage, mGreyImage, BLOB_THRESHOLD, 255, CV_THRESH_BINARY);
                    break;
                case 1:
                    cvThreshold(mGreyImage, mGreyImage, BLOB_THRESHOLD, 255, CV_THRESH_OTSU);
                    break;
            }
            
            /* mask !ROI */
            { // left
                CvPoint mROIPoints[] = {
                    cvPoint(0, 0),
                    cvPoint(ROI_RIGHT_TOP, 0),
                    cvPoint(ROI_RIGHT_BOTTOM, CAMERA_HEIGHT),
                    cvPoint(0, CAMERA_HEIGHT)
                };
                cvFillConvexPoly( mGreyImage, mROIPoints, 4, cvScalar(0) );
            }
            { // right
                CvPoint mROIPoints[] = {
                    cvPoint(CAMERA_WIDTH, 0),
                    cvPoint(CAMERA_WIDTH - ROI_LEFT_TOP, 0),
                    cvPoint(CAMERA_WIDTH - ROI_LEFT_BOTTOM, CAMERA_HEIGHT),
                    cvPoint(CAMERA_WIDTH, CAMERA_HEIGHT)
                };
                cvFillConvexPoly( mGreyImage, mROIPoints, 4, cvScalar(0) );
            }
            
            /* track blobs */
            cvReleaseBlobs(mBlobs);
            IplImage* mLabelImg = cvCreateImage(cvGetSize(mGreyImage), IPL_DEPTH_LABEL, 1);
            cvLabel(mGreyImage, mLabelImg, mBlobs);
            
            /* write image to texture */
            cv::Mat mResultTexCV = mGreyImage;
            mProcessedImage = fromOcv(mResultTexCV);
            if (mGui->isEnabled()) {
                DEBUGmBlobTexture.update(mProcessedImage);
                DEBUGmDepthTexture.update(mSurface);
            }
            
            cvFilterByArea(mBlobs, BLOB_MIN_AREA, BLOB_MAX_AREA);            
            if (DRAW_BIGGEST_ONLY) {
                cvFilterByLabel(mBlobs, cvGreaterBlob(mBlobs));
            }
            
            // TODO do something with tracks
            cvUpdateTracks(mBlobs, mTracks, 
                           TRACK_MATCHING_DISTANCE, 
                           TRACK_MAX_NUMBER_OF_FRAMES_INACTIVE);            
            
            /* clean up */
            cvReleaseImage(&mGreyImage);
            cvReleaseImage(&mLabelImg);
        }
    }
}

void ScheinrieseApp::heartbeat(double mDeltaTime) {
    mWatchdogCounter += mDeltaTime;
    if (mWatchdogCounter > mWatchdogInterval) {
        mWatchdogCounter = 0.0;
        console() << "+++ heartbeat" << std::endl;
        osc::Message mMessage;
        mMessage.addIntArg(getpid());
        mMessage.setAddress("/watchdog/register");
        mMessage.setRemoteEndpoint(mWatchdogHost, mWatchdogPort);
        mWatchdogSender.sendMessage(mMessage);
    }    
}

double mTime = 0;

void ScheinrieseApp::draw() {
    /* delta time */
    double mDeltaTime = getElapsedSeconds() - mTime;
    mTime = getElapsedSeconds();

    /* heartbeat */
    heartbeat(mDeltaTime);

    /* -- */
	gl::clear( Color( 0, 0, 0 ) ); 
    gl::setMatricesWindow(WINDOW_WIDTH, WINDOW_HEIGHT);
    
    /* draw background */
    const float mMiddleScaleOffset = BGR_IMG_GREY_SCALE_MIDDLE;// abs(sin(mTime * 0.1)) * 0.3 + BGR_IMG_GREY_SCALE_MIDDLE;
    glBegin(GL_QUADS);
    {
        glColor3f(BGR_IMG_GREY_SCALE_TOP, BGR_IMG_GREY_SCALE_TOP, BGR_IMG_GREY_SCALE_TOP);
        glVertex2f(0, 0);
        glVertex2f(WINDOW_WIDTH, 0);
        glColor3f(mMiddleScaleOffset, mMiddleScaleOffset, mMiddleScaleOffset);
        glVertex2f(WINDOW_WIDTH, WINDOW_HEIGHT/2);
        glVertex2f(0, WINDOW_HEIGHT/2);
    }
    {
        glColor3f(mMiddleScaleOffset, mMiddleScaleOffset, mMiddleScaleOffset);
        glVertex2f(0, WINDOW_HEIGHT/2);
        glVertex2f(WINDOW_WIDTH, WINDOW_HEIGHT/2);
        glColor3f(BGR_IMG_GREY_SCALE_BOTTOM, BGR_IMG_GREY_SCALE_BOTTOM, BGR_IMG_GREY_SCALE_BOTTOM);
        glVertex2f(WINDOW_WIDTH, WINDOW_HEIGHT);
        glVertex2f(0, WINDOW_HEIGHT);
    }
    glEnd();

    /* flip */
    glPushMatrix();
    glScalef(-1.0, 1.0, 1.0);
    glTranslatef(-WINDOW_WIDTH, 0.0, 0.0);
        
    /* normalize texture coordinates */
    Vec2f mNormalizeScale = Vec2f(1.0 / float(CAMERA_WIDTH), 1.0 / float(CAMERA_HEIGHT));
    glMatrixMode(GL_TEXTURE);
    glPushMatrix();
    glScalef(mNormalizeScale.x, mNormalizeScale.y, 1.0);
    glTranslatef(DEPTH_TEX_ALIGN_X, DEPTH_TEX_ALIGN_Y, 0.0);
    glScalef(RGB_DEPTH_TEXTURE_ALIGN_SCALE, RGB_DEPTH_TEXTURE_ALIGN_SCALE, 1.0);
    glMatrixMode(GL_MODELVIEW);
   
    /* adjust transform matrix */
    const float mWindowScaleX = float(WINDOW_WIDTH + 14.0) / float(CAMERA_WIDTH);
    const float mWindowScaleY = float(WINDOW_HEIGHT) / float(CAMERA_HEIGHT);
    glPushMatrix();

    glScalef(mWindowScaleX, mWindowScaleY, 1);

    bool DRAW_DEBUG = false;
    if (DRAW_DEBUG) {
        gl::enableWireframe();
    } else {
        mColorTexture.enableAndBind();
    }

    gl::color(1, 1, 1, 1);
    gl::enableAlphaBlending();
    
    /* handle blobs */
    if (mBlobs.size() > 0) {
        CvBlobs mBlobsCopy = mBlobs;
        cvGreaterBlob(mBlobsCopy);
        
        CvBlob mBlob = *(mBlobsCopy.begin()->second);
        estimateDistanceFromBlobDepth(mBlob);
        estimateDistanceFromBlobHeight(mBlob);
        estimatePosition(mBlob);
    }
    for (CvBlobs::const_iterator it=mBlobs.begin(); it!=mBlobs.end(); ++it) {
        drawBlob(*(it->second));
    }

    if (mGui->isEnabled()) {
        DEBUGdrawBlobsAndTracks(mBlobs, mTracks);  
    }
    gl::disableAlphaBlending();
    
    if (DRAW_DEBUG) {
        gl::disableWireframe();        
    } else {
        mColorTexture.disable();
    }

    /* restore transport matrices */
    glPopMatrix();
    glMatrixMode(GL_TEXTURE);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    
    
    /* mask */
    gl::color(0, 0, 0, 1);
    
    Path2d mPathLeft;
    mPathLeft.moveTo(0, 0);
    mPathLeft.lineTo(MASK_LEFT_TOP, 0);
    mPathLeft.lineTo(MASK_LEFT_BOTTOM, WINDOW_HEIGHT);
    mPathLeft.lineTo(0, WINDOW_HEIGHT);
    mPathLeft.close();    
    gl::drawSolid(mPathLeft);
    
    Path2d mPathRight;
    mPathRight.moveTo(WINDOW_WIDTH, 0);
    mPathRight.lineTo(WINDOW_WIDTH-MASK_RIGHT_TOP, 0);
    mPathRight.lineTo(WINDOW_WIDTH-MASK_RIGHT_BOTTOM, WINDOW_HEIGHT);
    mPathRight.lineTo(WINDOW_WIDTH, WINDOW_HEIGHT);
    mPathRight.close();    
    gl::drawSolid(mPathRight);
    
    gl::drawSolidCircle(Vec2f(WINDOW_WIDTH/2 + MASK_CIRCLE_X_POS, 
                              WINDOW_HEIGHT + MASK_CIRCLE_Y_POS), MASK_CIRCLE_RADIUS);
                
    /* flip */
    glPopMatrix();
    
    /* gui */
    if (mGui->isEnabled()) {
        gl::enableAlphaBlending();
        gl::color(1, 1, 1, 1);
        DEBUGdrawCameraImages();
        gl::color(1, 1, 1, 1);
        mGui->draw();
        gl::disableAlphaBlending();
    }
}


void ScheinrieseApp::DEBUGdrawCameraImages() {
    const int IMAGE_WIDTH = 240;
    const int IMAGE_HEIGHT = 180;
    const int IMAGE_POS_Y_OFFSET = 10;
    int mCounter = 0;
    
    if (mColorTexture) {
        gl::draw(mColorTexture, Rectf(WINDOW_WIDTH - IMAGE_POS_Y_OFFSET, 
                                      IMAGE_HEIGHT * (mCounter + 0), 
                                      WINDOW_WIDTH - IMAGE_WIDTH - IMAGE_POS_Y_OFFSET, 
                                      IMAGE_HEIGHT * (mCounter + 1)));
        mCounter++;
    }
    if (DEBUGmDepthTexture) {
        gl::draw(DEBUGmDepthTexture, Rectf(WINDOW_WIDTH - IMAGE_POS_Y_OFFSET, 
                                      IMAGE_HEIGHT * (mCounter + 0), 
                                      WINDOW_WIDTH - IMAGE_WIDTH - IMAGE_POS_Y_OFFSET, 
                                      IMAGE_HEIGHT * (mCounter + 1)));
        mCounter++;
        gl::draw(DEBUGmBlobTexture, Rectf(WINDOW_WIDTH - IMAGE_POS_Y_OFFSET, 
                                      IMAGE_HEIGHT * (mCounter + 0), 
                                      WINDOW_WIDTH - IMAGE_WIDTH - IMAGE_POS_Y_OFFSET, 
                                      IMAGE_HEIGHT * (mCounter + 1)));
        mCounter++;
    }
}

void ScheinrieseApp::estimatePosition(const CvBlob & pBlob) {
    /* origin is center bottom */
    mSamplerXPosition.advance(pBlob.minx + ( pBlob.maxx - pBlob.minx ) / 2);
}

void ScheinrieseApp::estimateDistanceFromBlobDepth(const CvBlob & pBlob) {
    const Surface mImage = getDepthImage();
    unsigned int mCounter = 1;
    unsigned int mTotalBrightness = 0;

    /* collect pixels from depthmap */
    for (int x=pBlob.minx; x < pBlob.maxx; ++x) {
        for (int y=pBlob.miny; y < pBlob.maxy; ++y) {
            const Color8u mPixel = mImage.getPixel(Vec2i(x, y));
            const int mBrightness = (mPixel.r + mPixel.g + mPixel.b) / 3; // a single component should suffice ...
            if (mProcessedImage.getPixel(Vec2i(x, y)).r > 0) {
                mCounter++;
                mTotalBrightness += mBrightness;
            }
        }
    }    
    mScaleFromDepth = float(mTotalBrightness) / float(mCounter);  
    mScaleFromDepth = max(BLOB_DEPTH_CLAMP_FAR, min(BLOB_DEPTH_CLAMP_NEAR, mScaleFromDepth));
    
    const float mRange = BLOB_DEPTH_CLAMP_NEAR - BLOB_DEPTH_CLAMP_FAR;
    mScaleFromDepth -= BLOB_DEPTH_CLAMP_FAR;
    mScaleFromDepth /= mRange;
    mScaleFromDepth = 1.0 - mScaleFromDepth;
    float mDistanceFromDepth = mScaleFromDepth;
    mScaleFromDepth = pow(mScaleFromDepth, BLOB_DEPTH_RESCALE_EXPONENT);
    
    const float mScaleRange = BLOB_DEPTH_RESCALED_FAR - BLOB_DEPTH_RESCALED_NEAR;
    mScaleFromDepth *= mScaleRange;
    mScaleFromDepth += BLOB_DEPTH_RESCALED_NEAR;
    
    // write scale to sampler
    mSamplerScaleFromDepth.advance(mScaleFromDepth);
    mSamplerDistanceFromDepth.advance(mDistanceFromDepth);
}

void ScheinrieseApp::estimateDistanceFromBlobHeight(const CvBlob & pBlob) {      
    /* handle scale */
    const float mMax = CAMERA_HEIGHT; // we assume that all blobs start at the bottom
    float mHeight = mMax - pBlob.miny;
    mHeight = max(BLOB_HEIGHT_CLAMP_FAR, min(mHeight, BLOB_HEIGHT_CLAMP_NEAR)); // clamp
    mHeight = mHeight - BLOB_HEIGHT_CLAMP_FAR;
    const float mHeightRange = BLOB_HEIGHT_CLAMP_NEAR - BLOB_HEIGHT_CLAMP_FAR;
    float mDistance = 1 - ( mHeight / mHeightRange ); // normalize // gross == 0, klein == 1
    mDistance = pow(mDistance, BLOB_HEIGHT_RESCALE_EXPONENT);
        
    // scale from ratio
    const float mMinScale = BLOB_HEIGHT_RESCALED_FAR / BLOB_HEIGHT_CLAMP_FAR;
    const float mMaxScale = BLOB_HEIGHT_RESCALED_NEAR / BLOB_HEIGHT_CLAMP_NEAR;
    mScaleFromHeight = mDistance * (mMinScale - mMaxScale) + mMaxScale;
    
    // write scale to sampler
    mSamplerScaleFromHeight.advance(mScaleFromHeight);
}

void ScheinrieseApp::drawBlobAsMesh(const CvBlob & pBlob) {
    const CvContourPolygon* mComplexPolygon = cvConvertChainCodesToPolygon(&pBlob.contour);
    const CvContourPolygon* mSimplePolygon = cvSimplifyPolygon(mComplexPolygon, POLYGON_REDUCTION_MIN_DISTANCE);
    const Shape2d mShape = convertPolygonToShape2d(*mSimplePolygon);
    Triangulator mTriangulator = Triangulator( mShape);
    
    const bool DRAW_INSIDE_POLYGONS = false;
    if (DRAW_INSIDE_POLYGONS) {
        const CvContoursChainCode mInternalContours = pBlob.internalContours;
        for (CvContoursChainCode::const_iterator mIterator = mInternalContours.begin(); mIterator != mInternalContours.end(); ++mIterator) {
            const CvContourChainCode* mInteralContour = *mIterator;
            const CvContourPolygon* mInternalPolygon = cvConvertChainCodesToPolygon(mInteralContour);
            Shape2d mShape = convertPolygonToShape2d(*mInternalPolygon);
            mTriangulator.addShape(mShape);
            delete mInternalPolygon;
        }
    }
    TriMesh2d mMesh = mTriangulator.calcMesh( Triangulator::WINDING_ODD );
    for (int j=0; j < mMesh.getVertices().size(); ++j) {
        mMesh.appendTexCoord(Vec2f(mMesh.getVertices()[j]));
    }

    delete mComplexPolygon;
    delete mSimplePolygon;

    glColor4f(1, 1, 1, getAlphaByDistance());
    drawMesh(mMesh);
}

void ScheinrieseApp::drawBlobAsRect(const CvBlob & pBlob) {
    glColor4f(1, 1, 1, getAlphaByDistance());
    const Vec2f mPadding = Vec2f(POLYGON_OUTLINE_WIDTH, POLYGON_OUTLINE_WIDTH);
    const Vec2f mMin = Vec2f(pBlob.minx, pBlob.miny) - mPadding;
    const Vec2f mMax = Vec2f(pBlob.maxx, pBlob.maxy) + mPadding;
    
    glBegin(GL_QUADS);
    {
        glTexCoord2f(mMin);
        glVertex2f(mMin);
        glTexCoord2f(mMax.x, mMin.y);
        glVertex2f(mMax.x, mMin.y);
        glTexCoord2f(mMax);
        glVertex2f(mMax);
        glTexCoord2f(mMin.x, mMax.y);
        glVertex2f(mMin.x, mMax.y);
    }
    glEnd();
}

void ScheinrieseApp::drawBlobAsOutlinedMesh(const CvBlob & pBlob, const int pPolygonStyle) {
    CvContourPolygon* mComplexPolygon = cvConvertChainCodesToPolygon(&pBlob.contour);
    CvContourPolygon* mSimplePolygon = cvSimplifyPolygon(mComplexPolygon, POLYGON_REDUCTION_MIN_DISTANCE);
    CvContourPolygon* mConvexHullPolygon = cvPolygonContourConvexHull(mComplexPolygon);
    CvContourPolygon* mPolygon;
    switch(pPolygonStyle) {
        case 1:
            mPolygon = mComplexPolygon;
            break;
        case 2:
            mPolygon = mSimplePolygon;
            break;
        case 3:
            mPolygon = mConvexHullPolygon;
            mPolygon-> pop_back();
            break;
    }
    
    const Vec2f mCenter = Vec2f(pBlob.centroid.x, pBlob.centroid.y);
    const float mContourWidth = POLYGON_OUTLINE_WIDTH;
    const float mAlpha = getAlphaByDistance();
    
    glBegin(GL_TRIANGLES);
    for (int ii = 0; ii<mPolygon->size(); ii++) {
        const int h = ( ii - 1 + mPolygon->size()) % mPolygon->size();
        const int i = ii % mPolygon->size();
        const int j = ( ii + 1 ) % mPolygon->size();
        const int k = ( ii + 2 ) % mPolygon->size();
        
        Vec2f pA1 = Vec2f((*mPolygon)[h].x, (*mPolygon)[h].y);
        Vec2f p1 = Vec2f((*mPolygon)[i].x, (*mPolygon)[i].y);
        Vec2f p2 = Vec2f((*mPolygon)[j].x, (*mPolygon)[j].y);
        Vec2f pA2 = Vec2f((*mPolygon)[k].x, (*mPolygon)[k].y);
        Vec2f pC1 = p1 - pA1;
        Vec2f pC2 = p2 - p1;
        Vec2f pC3 = pA2 - p2;
        
        pC1 = Vec2f(pC1.y, -pC1.x);
        pC2 = Vec2f(pC2.y, -pC2.x);
        pC3 = Vec2f(pC3.y, -pC3.x);
        pC1 = pC1.normalized();
        pC2 = pC2.normalized();
        pC3 = pC3.normalized();
        
        Vec2f p3 = ( pC1 + pC2 ) * 0.5;            
        p3 *= mContourWidth;
        p3 += p1;

        Vec2f p4 = ( pC2 + pC3 ) * 0.5;            
        p4 *= mContourWidth;
        p4 += p2;
        
        /* move original outline outwards */
        p1 = ( p1 + p3 ) * 0.5;
        p2 = ( p2 + p4 ) * 0.5;
        
        /* draw outline */
        glColor4f(1, 1, 1, mAlpha);
        glTexCoord2f(p1);
        glVertex2f(p1);
        
        glColor4f(1, 1, 1, mAlpha);
        glTexCoord2f(p2);
        glVertex2f(p2);
        
        glColor4f(1, 1, 1, 0);
        glTexCoord2f(p3);
        glVertex2f(p3);

        glColor4f(1, 1, 1, mAlpha);
        glTexCoord2f(p2);
        glVertex2f(p2);

        glColor4f(1, 1, 1, 0);
        glTexCoord2f(p4);
        glVertex2f(p4);

        glColor4f(1, 1, 1, 0);
        glTexCoord2f(p3);
        glVertex2f(p3);
        
        /* draw inside */
        glColor4f(1, 1, 1, mAlpha);
        glTexCoord2f(mCenter);
        glVertex2f(mCenter);

        glColor4f(1, 1, 1, mAlpha);
        glTexCoord2f(p2);
        glVertex2f(p2);

        glColor4f(1, 1, 1, mAlpha);
        glTexCoord2f(p1);
        glVertex2f(p1);
    }  
    glEnd();

    delete mComplexPolygon;
    delete mSimplePolygon;
    delete mConvexHullPolygon;
}

void ScheinrieseApp::drawBlob(const CvBlob & pBlob) {   
    /* draw mesh */
    glPushMatrix();
    glTranslatef(mSamplerXPosition.getAverage(), mFixedYPosition, 0.0);
    switch(DISTANCE_ESTIMATION_STRATEGY) {
        case 0:
            glScalef(mSamplerScaleFromDepth.getAverage(), mSamplerScaleFromDepth.getAverage(), 1);
            break;
        case 1:
            glScalef(mSamplerScaleFromHeight.getAverage(), mSamplerScaleFromHeight.getAverage(), 1.0);
            break;
    }
    glTranslatef(-mSamplerXPosition.getAverage(), -mFixedYPosition, 0.0);

    switch(DRAW_BLOB_AS) {
        case 0:
            drawBlobAsMesh(pBlob);
            break;
        case 1:
            drawBlobAsRect(pBlob);
            break;
        case 2:
            drawBlobAsOutlinedMesh(pBlob, 1);
            break;
        case 3:
            drawBlobAsOutlinedMesh(pBlob, 2);
            break;
        case 4:
            drawBlobAsOutlinedMesh(pBlob, 3);
            break;
    }

    glPopMatrix();
    
    /* draw debug */
    if (mGui->isEnabled()) {
        gl::color(1,0,0,0.5);
        gl::drawSolidCircle(Vec2f(pBlob.centroid.x, pBlob.centroid.y), 10);
        gl::color(0,1,0,0.5);
    }        
}

float ScheinrieseApp::getAlphaByDistance() { 
    const float mValue = mSamplerDistanceFromDepth.getAverage();
    if (mValue < DISTANCE_FADE_EDGE_NEAR) {
        return mValue / DISTANCE_FADE_EDGE_NEAR;
    } else if (mValue > DISTANCE_FADE_EDGE_FAR) {
        const float mRange = 1.0 - DISTANCE_FADE_EDGE_FAR;
        const float mInvers = 1.0 - mValue;
        return mInvers / mRange;        
    } else {
        return 1.0;
    }
}


void ScheinrieseApp::DEBUGdrawBlobsAndTracks(const CvBlobs& pBlobs, const CvTracks& pTracks) {
    /* iterate results */
    for (CvBlobs::const_iterator it=pBlobs.begin(); it!=pBlobs.end(); ++it) {           

        /* draw bounding box */
        const CvBlob mBlob = *((*it).second);
        gl::color(1, 1, 1, 1);
        gl::drawStrokedRect(Rectf(mBlob.minx-1, mBlob.miny-1, mBlob.maxx+1, mBlob.maxy+1));
        
        /* draw polygons */
        const CvContourPolygon* polygon = cvConvertChainCodesToPolygon(&(*it).second->contour);
        gl::color(1, 0, 0, 1);
        for (int i=0; i<polygon->size(); i++) {
            const CvPoint pointA = (*polygon)[i];
            const CvPoint pointB = (*polygon)[(i + 1) % polygon->size()];
            gl::drawLine(Vec2f(pointA.x, pointA.y), Vec2f(pointB.x, pointB.y));
        }
        
        /* draw simplified polygons */
        const CvContourPolygon* sPolygon = cvSimplifyPolygon(polygon, POLYGON_REDUCTION_MIN_DISTANCE);
        gl::color(0, 1, 0, 1);
        for (int i=0; i<sPolygon->size(); i++) {
            const CvPoint pointA = (*sPolygon)[i];
            const CvPoint pointB = (*sPolygon)[(i + 1) % sPolygon->size()];
            gl::drawLine(Vec2f(pointA.x, pointA.y), Vec2f(pointB.x, pointB.y));
        }
        
        /* draw contours */
        const CvContourPolygon* cPolygon = cvPolygonContourConvexHull(sPolygon);
        gl::color(0, 0, 1, 1);
        for (int i=0; i<cPolygon->size(); i++) {
            const CvPoint pointA = (*cPolygon)[i];
            const CvPoint pointB = (*cPolygon)[(i + 1) % cPolygon->size()];
            gl::drawLine(Vec2f(pointA.x, pointA.y), Vec2f(pointB.x, pointB.y));
        }
        
        delete polygon;
        delete sPolygon;
        delete cPolygon;
        
        /* draw internal contours */
        CvContoursChainCode mInternalContours = (*it).second->internalContours;
        for (CvContoursChainCode::iterator mIterator = mInternalContours.begin(); mIterator != mInternalContours.end(); ++mIterator) {
            const CvContourChainCode* mInteralContour = *mIterator;
            const CvContourPolygon* mInternalPolygon = cvConvertChainCodesToPolygon(mInteralContour);
            gl::color(1, 0, 1, 1);
            for (int i=0; i<mInternalPolygon->size(); i++) {
                CvPoint pointA = (*mInternalPolygon)[i];
                CvPoint pointB = (*mInternalPolygon)[(i + 1) % mInternalPolygon->size()];
                gl::drawLine(Vec2f(pointA.x, pointA.y), Vec2f(pointB.x, pointB.y));
            }
            delete mInternalPolygon;
        }
        
        /* draw tracks */
        gl::color(1, 0.5, 0, 1);
        for (CvTracks::const_iterator it=pTracks.begin(); it!=pTracks.end(); ++it) {
            const CvTrack* mTrack = it->second;
            if (mTrack && !mTrack->inactive) {
                const Rectf& mRect = Rectf(mTrack->minx, mTrack->miny, mTrack->maxx, mTrack->maxy);
                gl::drawStrokedRect(mRect);
            }
        }    
    }
}

/* this should be in cinder  */
void ScheinrieseApp::drawMesh( const TriMesh2d & mesh ) {
	glVertexPointer( 2, GL_FLOAT, 0, &(mesh.getVertices()[0]) );
	glEnableClientState( GL_VERTEX_ARRAY );

	if( mesh.hasColorsRgb() ) {
		glColorPointer( 3, GL_FLOAT, 0, &(mesh.getColorsRGB()[0]) );
		glEnableClientState( GL_COLOR_ARRAY );
	}
	else if( mesh.hasColorsRgba() ) {
		glColorPointer( 4, GL_FLOAT, 0, &(mesh.getColorsRGBA()[0]) );
		glEnableClientState( GL_COLOR_ARRAY );
	}
	else 
		glDisableClientState( GL_COLOR_ARRAY );	
    
	if( mesh.hasTexCoords() ) {
		glTexCoordPointer( 2, GL_FLOAT, 0, &(mesh.getTexCoords()[0]) );
		glEnableClientState( GL_TEXTURE_COORD_ARRAY );
	}
	else
		glDisableClientState( GL_TEXTURE_COORD_ARRAY );
	glDrawElements( GL_TRIANGLES, mesh.getNumIndices(), GL_UNSIGNED_INT, &(mesh.getIndices()[0]) );
    
	glDisableClientState( GL_VERTEX_ARRAY );
	glDisableClientState( GL_NORMAL_ARRAY );
	glDisableClientState( GL_COLOR_ARRAY );
	glDisableClientState( GL_TEXTURE_COORD_ARRAY );
}

Shape2d ScheinrieseApp::convertPolygonToShape2d(const CvContourPolygon & polygon) {
    Shape2d mShape;
    const Vec2f v = Vec2f((polygon)[0].x, (polygon)[0].y);
    mShape.moveTo(v);
    for (int i=1; i<polygon.size(); i++) {
        const Vec2f v = Vec2f((polygon)[i].x, (polygon)[i].y);
        mShape.lineTo(v);
    }  
    mShape.close();
    return mShape;
}

TriMesh2d ScheinrieseApp::triangulateShape(const Shape2d & mShape) {
    float mPrecision;
    mPrecision = 1.0f;
    TriMesh2d mesh = Triangulator( mShape, mPrecision ).calcMesh( Triangulator::WINDING_ODD );
    return mesh;
}

void ScheinrieseApp::keyDown( KeyEvent pEvent ) {
    switch(pEvent.getChar()) {				
        case 'd': mGui->dump(); break;
        case 'l': mGui->load(getResourcePath(RES_SETTINGS)); break;
        case 's': mGui->save(getResourcePath(RES_SETTINGS)); break;
        case 'f': setFullScreen( !isFullScreen() ); break;
    }
    switch(pEvent.getCode()) {
        case KeyEvent::KEY_ESCAPE: setFullScreen( false ); quit(); break;
        case KeyEvent::KEY_SPACE: mGui->setEnabled(!mGui->isEnabled());break;
    }
}


void ScheinrieseApp::quit() {
    console() << "### EXIT .";
    setFullScreen( false );
    try {
        // TODO find reliable way to quit
        console() << ".";
        kill(getpid(), 9);
        console() << ".";
        mKinect->stop();
        console() << ".";
        console() << ".";
        delete mKinect;
        delete mGui;
        delete mFPSOut;
        console() << ".";
        exit(0);
        // AppBasic::quit();
    }
    catch( ... ) {
        console() << "?" << endl;
    }
}

float edge_fade(float x, float n) {
    float y = x;
    y -= 0.5;
    y *= 2;
    y = abs(pow(y, n));
    y *= -1;
    y += 1;
    return y;
}

CINDER_APP_BASIC( ScheinrieseApp, RendererGl(0) )
