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


using namespace ci;
using namespace ci::app;
using namespace std;
using namespace mowa::sgui;
using namespace cvb;


const std::string       RES_SETTINGS = "settings.txt";
const int               CAMERA_WIDTH = 640;
const int               CAMERA_HEIGHT = 480;

struct BlobDistanceMap {
    float           average_distance;
    CvBlob          blob;
};

class ScheinrieseApp : public AppBasic {
public:
	void setup();
	void update();
	void draw();
    void keyDown( KeyEvent pEvent );
    void quit();
    void prepareSettings( Settings *settings );
    
private:
    void        DEBUGdrawBlobsAndTracks( const CvBlobs& pBlobs, const CvTracks& pTracks );
    void        drawDelaunay2D( const CvBlobs& pBlobs );
    void        drawTriangulatedBlobs(const CvBlobs & pBlobs);
    void        drawCameraImages();
    void        draw( const TriMesh2d &mesh );
    TriMesh2d   triangulateShape( const Shape2d & mShape );
    Shape2d     convertPolygonToShape2d( const CvContourPolygon & polygon );
    void        getAverageBlobDistanceMap(const CvBlobs & pBlobs, vector<BlobDistanceMap> & pAverageBlobDistanceMap);

    
    /* properties */
    SimpleGUI *             mGui;
    
    int                     WINDOW_WIDTH;
    int                     WINDOW_HEIGHT;    
    
    int                     FRAME_RATE;   
    bool                    FULLSCREEN;
    int                     KINECT_ANGLE;  
    int                     BLOB_THRESHOLD;
    int                     BLOB_MIN_AREA;
    int                     BLOB_MAX_AREA;
    int                     BLOB_BLUR;
    float                   BLOB_POLYGON_REDUCTION_MIN_DISTANCE;
    float                   BLOB_SCALE_MIN;
    float                   BLOB_SCALE_MAX;
    float                   BLOB_SCALE_DEPTH_CLAMP_MIN;
    float                   BLOB_SCALE_DEPTH_CLAMP_MAX;
    float                   BLOB_SCALE_EXPONENT;
    float                   BLOB_ALPHA_EDGE_BLEND;
    float                   RGB_DEPTH_TEXTURE_ALIGN_TRANSLATE_X;
    float                   RGB_DEPTH_TEXTURE_ALIGN_TRANSLATE_Y;
    float                   RGB_DEPTH_TEXTURE_ALIGN_SCALE;
    float                   BACKGROUND_SCALE_X;
    float                   BACKGROUND_SCALE_Y;
    float                   BACKGROUND_TRANSLATE_X;
    float                   BACKGROUND_TRANSLATE_Y;
    int                     ENABLE_SHADER;  

    /* output */
    LabelControl *          mFPSOut;
    LabelControl *          mFacesOut;
    
    double                  mTime;
    
    /* kinect */
    Kinect *                mKinect;
	gl::Texture             mDepthTexture;
	gl::Texture             mColorTexture;
	gl::Texture             mColorBackgroundTexture;
	gl::Texture             DEBUGmBlobTexture;
    
    /* blobs */
    CvBlobs                 mBlobs;
    CvTracks                mTracks;
    vector<BlobDistanceMap> mAverageBlobDistanceMap;

    /* shader */
    gl::GlslProg            mShader;
};

void ScheinrieseApp::setup() { 
    
    console() << "+++ Scheinriese (PID " << getpid() << ")." << endl;
    
    try {
    }
    catch( ... ) {
    }
    
    /* settings */
    mGui->addParam("BACKGROUND_SCALE_X", &BACKGROUND_SCALE_X, 1.0, 1.15, 1.1132);
    mGui->addParam("BACKGROUND_SCALE_Y", &BACKGROUND_SCALE_Y, 1.0, 1.1, 1.0764);
    mGui->addParam("BACKGROUND_TRANSLATE_X", &BACKGROUND_TRANSLATE_X, -30, 0, -24);
    mGui->addParam("BACKGROUND_TRANSLATE_Y", &BACKGROUND_TRANSLATE_Y, -70, 0, -64.32);
    mGui->addParam("RGB_DEPTH_TEXTURE_ALIGN_TRANSLATE_X", &RGB_DEPTH_TEXTURE_ALIGN_TRANSLATE_X, 0, 20, 7.2);
    mGui->addParam("RGB_DEPTH_TEXTURE_ALIGN_TRANSLATE_Y", &RGB_DEPTH_TEXTURE_ALIGN_TRANSLATE_Y, 0, 45, 37.6);
    mGui->addParam("RGB_DEPTH_TEXTURE_ALIGN_SCALE", &RGB_DEPTH_TEXTURE_ALIGN_SCALE, 0.9, 1.0, 0.925);

    // visible
    mGui->addParam("KINECT_ANGLE", &KINECT_ANGLE, -31, 30, 20);
    mGui->addParam("BLOB_THRESHOLD", &BLOB_THRESHOLD, 1, 255, 30);
    mGui->addParam("BLOB_MIN_AREA", &BLOB_MIN_AREA, 1, 100000, 5000);
    mGui->addParam("BLOB_MAX_AREA", &BLOB_MAX_AREA, 1, 500000, 500000);
    mGui->addParam("BLOB_BLUR", &BLOB_BLUR, 0, 20, 7);
    mGui->addParam("BLOB_POLYGON_REDUCTION_MIN_DISTANCE", &BLOB_POLYGON_REDUCTION_MIN_DISTANCE, 0, 20, 10);
    mGui->addParam("BLOB_SCALE_MIN", &BLOB_SCALE_MIN, 0, 3, 0.1);
    mGui->addParam("BLOB_SCALE_MAX", &BLOB_SCALE_MAX, 0, 2, 1.5);
    mGui->addParam("BLOB_SCALE_DEPTH_CLAMP_MIN", &BLOB_SCALE_DEPTH_CLAMP_MIN, 0, 255, 30);
    mGui->addParam("BLOB_SCALE_DEPTH_CLAMP_MAX", &BLOB_SCALE_DEPTH_CLAMP_MAX, 0, 255, 142);
    mGui->addParam("BLOB_SCALE_EXPONENT", &BLOB_SCALE_EXPONENT, 0, 10, 4);
    mGui->addParam("BLOB_ALPHA_EDGE_BLEND", &BLOB_ALPHA_EDGE_BLEND, 1, 100, 2);
    mGui->addParam("ENABLE_SHADER", &ENABLE_SHADER, 0, 1, 1);
    
    // output
    mGui->addSeparator();
    mFPSOut = mGui->addLabel("");
    mFacesOut = mGui->addLabel("");

    /* clean up controller window */
    mGui->getControlByName("WINDOW_WIDTH")->active=false;
    mGui->getControlByName("WINDOW_HEIGHT")->active=false;
    mGui->getControlByName("FULLSCREEN")->active=false;

    mGui->getControlByName("BACKGROUND_SCALE_X")->active=false;
    mGui->getControlByName("BACKGROUND_SCALE_Y")->active=false;
    mGui->getControlByName("BACKGROUND_TRANSLATE_X")->active=false;
    mGui->getControlByName("BACKGROUND_TRANSLATE_Y")->active=false;

    mGui->getControlByName("RGB_DEPTH_TEXTURE_ALIGN_TRANSLATE_X")->active=false;
    mGui->getControlByName("RGB_DEPTH_TEXTURE_ALIGN_TRANSLATE_Y")->active=false;
    mGui->getControlByName("RGB_DEPTH_TEXTURE_ALIGN_SCALE")->active=false;

    mGui->load(getResourcePath(RES_SETTINGS));
    mGui->setEnabled(true);
    mGui->dump(); 

    /* kinect */
	console() << "+++ found " << Kinect::getNumDevices() << " kinect(s)." << std::endl;
	if (Kinect::getNumDevices() >= 1) {
		mKinect = new Kinect( Kinect::Device(0) );
        mKinect->setLedColor( Kinect::LED_BLINK_RED_YELLOW );
//        mKinect->setVideoInfrared();
        console() << "+++ waiting for kinect ..." << endl;
        while(!mKinect->checkNewDepthFrame()) {}
        mDepthTexture = mKinect->getDepthImage();
        DEBUGmBlobTexture = mKinect->getDepthImage();
        while(!mKinect->checkNewVideoFrame()) {}
        mColorTexture = mKinect->getVideoImage();
        mColorBackgroundTexture = mKinect->getVideoImage();     
        console() << "depth: " << mDepthTexture.getWidth() << ", " << mDepthTexture.getHeight() << endl;
        console() << "color: " << mColorTexture.getWidth() << ", " << mColorTexture.getHeight() << endl;
	} else {
        exit (1);
    }
    
    /* app */
    if (FULLSCREEN) {
        hideCursor();
    }
    
    /* shader */
    try {
		mShader = gl::GlslProg( loadResource( RES_PASSTHRU_VERT ), loadResource( RES_BLUR_FRAG ) );
	}
	catch( gl::GlslProgCompileExc &exc ) {
		console() << "Shader compile error: " << std::endl;
		console() << exc.what();
	}
	catch( ... ) {
		console() << "Unable to load shader" << std::endl;
	}
}

void ScheinrieseApp::prepareSettings( Settings *settings ) {
    mGui = new SimpleGUI(this);

    mGui->addParam("WINDOW_WIDTH", &WINDOW_WIDTH, 0, 2048, 640);
    mGui->addParam("WINDOW_HEIGHT", &WINDOW_HEIGHT, 0, 2048, 480);
    mGui->addParam("FULLSCREEN", &FULLSCREEN, false, 0);
    mGui->addParam("FRAME_RATE", &FRAME_RATE, 1, 120, 30);
    mGui->load(getResourcePath(RES_SETTINGS));

    settings->setWindowSize( WINDOW_WIDTH, WINDOW_HEIGHT );
	settings->setFrameRate( FRAME_RATE );
    settings->setFullScreenSize( WINDOW_WIDTH, WINDOW_HEIGHT );
    settings->setFullScreen( FULLSCREEN );
}

void ScheinrieseApp::update() {
    double mDeltaTime = getElapsedSeconds() - mTime;
    mTime = getElapsedSeconds();
    {
        stringstream mStr;
        mStr << "FPS: " << getAverageFps();
        mFPSOut->setText(mStr.str());
    }
    {
        stringstream mStr;
        mStr << "BLOBS: " << mBlobs.size();
        mFacesOut->setText(mStr.str());
    }
    setFrameRate( FRAME_RATE );

    /* kinect */
    if ( mKinect ) {
        bool mNewVideoFrame = mKinect->checkNewVideoFrame();
        bool mNewDepthFrame = mKinect->checkNewDepthFrame();
        if (mKinect->getTilt() != KINECT_ANGLE) {
            mKinect->setTilt(KINECT_ANGLE);
        }
        if ( mNewVideoFrame ) {
            mColorTexture.update(mKinect->getVideoImage());
        }
        if ( mNewDepthFrame ) {
            Surface mSurface = mKinect->getDepthImage();

            /* get image from capture device */
            Surface::Iter iter = mSurface.getIter();
            
            /* convert to grey scale */
            // TODO maybe we need background substraction
            IplImage* mGreyImage = cvCreateImage(cvSize(mSurface.getWidth(), mSurface.getHeight()), IPL_DEPTH_8U, 1);
            int i = 0;
            while( iter.line() ) {
                while( iter.pixel() ) {
                    mGreyImage->imageData[i] = iter.r();
                    i++;
                }
            }
            
            /* threshold */
            if (BLOB_BLUR >= 1) {
                cvSmooth(mGreyImage, mGreyImage, CV_BLUR, BLOB_BLUR, BLOB_BLUR);
            }
            cvThreshold(mGreyImage, mGreyImage, BLOB_THRESHOLD, 255, CV_THRESH_BINARY);
            
            /* track blobs */
            cvReleaseBlobs(mBlobs);
            cvReleaseTracks(mTracks);
            IplImage* mLabelImg = cvCreateImage(cvGetSize(mGreyImage), IPL_DEPTH_LABEL, 1);
            cvLabel(mGreyImage, mLabelImg, mBlobs);
            
            /* write image to texture */
            mDepthTexture.update(mSurface);
            if (mGui->isEnabled()) {
                cv::Mat mResultTexCV = mGreyImage;
                DEBUGmBlobTexture.update(fromOcv(mResultTexCV));
            }
            
            cvFilterByArea(mBlobs, BLOB_MIN_AREA, BLOB_MAX_AREA);

            // TODO do something with tracks
            cvUpdateTracks(mBlobs, mTracks, 5., 10);            
            // CvTrack.lifetime
            
            /* clean up */
            cvReleaseImage(&mGreyImage);
            cvReleaseImage(&mLabelImg);
        }
        if ( mNewDepthFrame ) {
            // TODO maybe reject blobs with in human dimensions
            getAverageBlobDistanceMap(mBlobs, mAverageBlobDistanceMap);
        }
    }
}

void ScheinrieseApp::draw() {
    /* -- */
	gl::clear( Color( 1, 1, 1 ) ); 
     
    /* shader */
    // TODO make this more opt'd
    if (ENABLE_SHADER) {
        mShader.bind();
        const int STEPS = 8;
        float mThresholds[STEPS];// = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0};
        for (int i=0; i < STEPS; ++i) {
            mThresholds[i] = float(i) / float(STEPS - 1);
        }
        mShader.uniform("thresholds", mThresholds, 16);   
        mShader.uniform( "tex0", 0 );
    }
    
    /* draw image to as background */
    // TODO this should be a static background image
    gl::color(1, 1, 1, 1);
    gl::enableAlphaBlending();
    glPushMatrix();
    glTranslatef(BACKGROUND_TRANSLATE_X, BACKGROUND_TRANSLATE_Y, 0.0);
    glScalef(BACKGROUND_SCALE_X, BACKGROUND_SCALE_Y, 1.0);
    gl::draw(mColorBackgroundTexture, Rectf(0, 0, WINDOW_WIDTH, WINDOW_HEIGHT));
    glPopMatrix();
    gl::disableAlphaBlending();

    /* normalize texture coordinates */
    Vec2f mNormalizeScale = Vec2f(1.0 / float(CAMERA_WIDTH), 1.0 / float(CAMERA_HEIGHT));
    glMatrixMode(GL_TEXTURE);
    glPushMatrix();
    glScalef(mNormalizeScale.x, mNormalizeScale.y, 1.0);
    glTranslatef(RGB_DEPTH_TEXTURE_ALIGN_TRANSLATE_X, RGB_DEPTH_TEXTURE_ALIGN_TRANSLATE_Y, 0.0);
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
    drawTriangulatedBlobs(mBlobs);
//    DEBUGdrawBlobsAndTracks(mBlobs, mTracks);  
//    drawDelaunay2D(mBlobs);
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
    
    /* shader */
    if (ENABLE_SHADER) {
        mShader.unbind();
    }

    /* gui */
    if (mGui->isEnabled()) {
        gl::enableAlphaBlending();
        gl::color(1, 1, 1, 0.5);
        drawCameraImages();
        gl::disableAlphaBlending();
        gl::color(1, 1, 1, 1);
        mGui->draw();
    }
}

void ScheinrieseApp::drawCameraImages() {
    if (mColorTexture) {
        gl::draw(mColorTexture, Rectf(10 + 330 * 0, 
                                      10, 
                                      330 * 1, 
                                      250));
    }
    if (mDepthTexture) {
        gl::draw(mDepthTexture, Rectf(10 + 330 * 1, 
                                      10, 
                                      330 * 2, 
                                      250 ));
        
        gl::draw(DEBUGmBlobTexture, Rectf(10 + 330 * 2, 
                                     10, 
                                     330 * 3, 
                                     250 ));        
    }
}

void ScheinrieseApp::getAverageBlobDistanceMap(const CvBlobs & pBlobs, vector<BlobDistanceMap> & pAverageBlobDistanceMap) {
    pAverageBlobDistanceMap.clear();
    for (CvBlobs::const_iterator it=pBlobs.begin(); it!=pBlobs.end(); ++it) {
        const CvBlob mBlob = *(it->second);
        const Surface mImage = mKinect->getDepthImage();
        unsigned int mCounter = 1;
        unsigned int mTotalBrightness = 0;
        /* collect pixels from depthmap */
        for (int x=mBlob.minx; x < mBlob.maxx; ++x) {
            for (int y=mBlob.miny; y < mBlob.maxy; ++y) {
                const Color8u mPixel = mImage.getPixel(Vec2i(x, y));
                const int mBrightness = (mPixel.r + mPixel.g + mPixel.b) / 3; // a single component should suffice ...
                if (mBrightness > BLOB_THRESHOLD) {
                    mCounter++;
                    mTotalBrightness += mBrightness;
                }
            }
        }

        BlobDistanceMap mBlobDistanceMap;
        mBlobDistanceMap.blob = mBlob;
        mBlobDistanceMap.average_distance = float(mTotalBrightness) / float(mCounter);
        pAverageBlobDistanceMap.push_back(mBlobDistanceMap);
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
    
void ScheinrieseApp::drawTriangulatedBlobs(const CvBlobs & pBlobs) {
    /* iterate results */
    // TODO maybe use simplified polygons
    /* draw triangulated polygons with holes */
//    for (CvBlobs::const_iterator it=pBlobs.begin(); it!=pBlobs.end(); ++it) {
//        const CvBlob * mBlob = (*it).second; 
    for (int i=0; i<mAverageBlobDistanceMap.size(); ++i) {
        const CvBlob mBlob = mAverageBlobDistanceMap[i].blob;
        const CvContourPolygon* polygon = cvConvertChainCodesToPolygon(&mBlob.contour);
        const CvContoursChainCode mInternalContours = mBlob.internalContours;
        
        const Shape2d mShape = convertPolygonToShape2d(*polygon);
        Triangulator mTriangulator = Triangulator( mShape, 1.0 );
        for (CvContoursChainCode::const_iterator mIterator = mInternalContours.begin(); mIterator != mInternalContours.end(); ++mIterator) {
            const CvContourChainCode* mInteralContour = *mIterator;
            const CvContourPolygon* mInternalPolygon = cvConvertChainCodesToPolygon(mInteralContour);
            Shape2d mShape = convertPolygonToShape2d(*mInternalPolygon);
            mTriangulator.addShape(mShape);
            delete mInternalPolygon;
        }
        TriMesh2d mMesh = mTriangulator.calcMesh( Triangulator::WINDING_ODD );
        for (int j=0; j < mMesh.getVertices().size(); ++j) {
            mMesh.appendTexCoord(Vec2f(mMesh.getVertices()[j]));
        }
                
        float mScale = mAverageBlobDistanceMap[i].average_distance;
        mScale = min(max(mScale, BLOB_SCALE_DEPTH_CLAMP_MIN), BLOB_SCALE_DEPTH_CLAMP_MAX); // clamp to range
        mScale -= BLOB_SCALE_DEPTH_CLAMP_MIN;
        mScale /= BLOB_SCALE_DEPTH_CLAMP_MAX - BLOB_SCALE_DEPTH_CLAMP_MIN;
        mScale = 1.0 - mScale;
        float mAlpha = mScale;
        mScale = pow(mScale, BLOB_SCALE_EXPONENT);
        mScale *= BLOB_SCALE_MAX - BLOB_SCALE_MIN;
        mScale += BLOB_SCALE_MIN;
        
//        console() << i << ": " << mBlob.centroid.y << endl;
//        console() << i << ": " << (mBlob.miny + (mBlob.maxy - mBlob.miny) / 2) << endl;

        const float mMiddle = (mBlob.miny + (mBlob.maxy - mBlob.miny) / 2);//mBlob.centroid.y;        
        
        /* adjust postion */
        float mOffset = 0.0;
        mOffset = abs(float(mBlob.maxy - mBlob.miny));
        if (true) {
            /* fix them to the bottom of the unscaled blob */
            mOffset = ( mOffset - ( mOffset * mScale ) ) / 2;
        } else {
            /* fix blob to bottom of screen */
            // DOESN T WORK
            mOffset = ( CAMERA_HEIGHT - mMiddle ) * mScale - mOffset / 2;
        }
        
        // TODO clamp alpha at beginning and end.
        mAlpha = edge_fade(1.0, BLOB_ALPHA_EDGE_BLEND);
        gl::color(1, 1, 1, mAlpha);
        
        glPushMatrix();
        glTranslatef(mBlob.centroid.x, mMiddle, 0.0);
        glTranslatef(0.0, mOffset, 0.0);
        glScalef(mScale, mScale, 1);
        glTranslatef(-mBlob.centroid.x, -mMiddle, 0.0);
        draw(mMesh);
        glPopMatrix();
        
        delete polygon;
    }
}

/* create and draw voronoi and delauny shapes */

void draw_subdiv_edge(CvSubdiv2DEdge edge) {
    CvSubdiv2DPoint* org_pt;
    CvSubdiv2DPoint* dst_pt;
    CvPoint2D32f org;
    CvPoint2D32f dst;
    CvPoint iorg, idst;
    
    org_pt = cvSubdiv2DEdgeOrg(edge);
    dst_pt = cvSubdiv2DEdgeDst(edge);
    
    if( org_pt && dst_pt ) {
        org = org_pt->pt;
        dst = dst_pt->pt;
        
        // how do i sort out the ugly edges
//        if (org.x > 0 && dst.x > 0 && org.x < CAMERA_WIDTH && dst.x < CAMERA_WIDTH) {
            iorg = cvPoint( cvRound( org.x ), cvRound( org.y ));
            idst = cvPoint( cvRound( dst.x ), cvRound( dst.y ));
//        }
        
        gl::drawLine(Vec2f(idst.x, idst.y), Vec2f(iorg.x, iorg.y));
    }
}


void draw_subdiv( CvSubdiv2D* subdiv ) {
    CvSeqReader  reader;
    int i, total = subdiv->edges->total;
    int elem_size = subdiv->edges->elem_size;
    
    cvStartReadSeq( (CvSeq*)(subdiv->edges), &reader, 0 );
    
    for( i = 0; i < total; i++ ) {
        CvQuadEdge2D* edge = (CvQuadEdge2D*)(reader.ptr);
        
        if( CV_IS_SET_ELEM( edge )) {
            gl::color(1, 0, 0, 1);
            draw_subdiv_edge((CvSubdiv2DEdge)edge + 1 );
            gl::color(0, 1, 0, 1);
            draw_subdiv_edge((CvSubdiv2DEdge)edge);
        }
        
        CV_NEXT_SEQ_ELEM( elem_size, reader );
    }
}

void ScheinrieseApp::drawDelaunay2D(const CvBlobs& pBlobs) {
    for (CvBlobs::const_iterator it=pBlobs.begin(); it!=pBlobs.end(); ++it) {                  
        /* draw simplified polygons */
        const CvContourPolygon* sPolygon = cvSimplifyPolygon(cvConvertChainCodesToPolygon(&(*it).second->contour), BLOB_POLYGON_REDUCTION_MIN_DISTANCE);
        
        CvSubdiv2D* subdiv;
        CvMemStorage* storage;
        CvRect rect = { 0, 0, CAMERA_WIDTH, CAMERA_HEIGHT };
        
        storage = cvCreateMemStorage(0);
        subdiv = cvCreateSubdiv2D( CV_SEQ_KIND_SUBDIV2D, 
                                  sizeof(*subdiv),
                                  sizeof(CvSubdiv2DPoint),
                                  sizeof(CvQuadEdge2D),
                                  storage );
        cvInitSubdivDelaunay2D( subdiv, rect );

        for (int i=0; i<sPolygon->size(); i++) {
            const CvPoint p = (*sPolygon)[i];
            CvPoint2D32f fp = cvPoint2D32f(p.x, p.y);
            cvSubdivDelaunay2DInsert( subdiv, fp );
        }
        cvCalcSubdivVoronoi2D( subdiv ); // ... or inside for-loop
        draw_subdiv( subdiv );

        delete sPolygon;
        cvReleaseMemStorage( &storage );
    }
}


void ScheinrieseApp::DEBUGdrawBlobsAndTracks(const CvBlobs& pBlobs, const CvTracks& pTracks) {
    /* iterate results */
    for (CvBlobs::const_iterator it=pBlobs.begin(); it!=pBlobs.end(); ++it) {           
        
        /* draw polygons */
        const CvContourPolygon* polygon = cvConvertChainCodesToPolygon(&(*it).second->contour);
        gl::color(1, 0, 0, 1);
        for (int i=0; i<polygon->size(); i++) {
            const CvPoint pointA = (*polygon)[i];
            const CvPoint pointB = (*polygon)[(i + 1) % polygon->size()];
            gl::drawLine(Vec2f(pointA.x, pointA.y), Vec2f(pointB.x, pointB.y));
        }
        
        /* draw simplified polygons */
        const CvContourPolygon* sPolygon = cvSimplifyPolygon(polygon, BLOB_POLYGON_REDUCTION_MIN_DISTANCE);
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
//        console() << "### tracks : " << pTracks.size() << endl; 
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
void ScheinrieseApp::draw( const TriMesh2d & mesh ) {
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
        case 'b':         
            if (mKinect) {
                mColorBackgroundTexture.update(mKinect->getVideoImage());
            }
            break;
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
    try {
        console() << "### EXIT .";
        mKinect->stop();
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

CINDER_APP_BASIC( ScheinrieseApp, RendererGl )
