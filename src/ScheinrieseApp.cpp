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


class KinectData {
    
    // from: http://social.msdn.microsoft.com/Forums/en-US/kinectsdknuiapi/thread/250b5e2d-5e0e-4e7e-b222-4c85278a02f2
    
public:
    //    DWORD unk1;
    //    DWORD unk2;
    short* TDepth; // 2048 depth correction [as 1/16 pixels] for xC
    float kx[10]; // coefs for x^3, y^3, x^2*y, x*y^2, x^2, y^2, x*y, x, y, 1
    float ky[10]; // coefs for x^3, y^3, x^2*y, x*y^2, x^2, y^2, x*y, x, y, 1
    
    //    KinectData(const KinectData& kd) {
    //        memcpy(this,&kd,sizeof(*this));
    //        TDepth=new SHORT[2048];
    //        memcpy(TDepth,kd.TDepth,2048*sizeof(SHORT));
    //    }
    KinectData() {}
    
    ~KinectData() {
        delete[] TDepth; 
    }
    
    //    static KinectData* GetFromSDK() // works only for SDK Beta (MSRKINECTNUI.DLL 1.0.0.11) ! {
    //        return *(KinectData**)(*(DWORD*)(((BYTE*)NuiImageGetColorPixelCoordinatesFromDepthPixel)+0x22)+0x154C);
    //    }
    
    // GetColor640CoordFromDepth320 gives same output as NuiImageGetColorPixelCoordinatesFromDepthPixel
    void GetColor640CoordFromDepth320(long xD, long yD, int zD, long *pxC, long *pyC) const {
        GetColor640CoordFromDepth640(xD<<1, yD<<1, zD, pxC, pyC);
    }
    
    
    void GetColor640CoordFromDepth640(long xD, long yD, int zD, long *pxC, long *pyC) const {
        const int NUI_IMAGE_PLAYER_INDEX_SHIFT = 3;
        zD >>= NUI_IMAGE_PLAYER_INDEX_SHIFT;
        float dxDf=float(639-320-xD), dyDf=float(yD-256);   // why 256 ? ask Kinect/MS ...
        float Sx=kx[9]+dyDf*kx[8]+dxDf*kx[7], Sy=ky[9]+dyDf*ky[8]+dxDf*ky[7];
        float v=dyDf*dyDf;
        Sx+=kx[5]*v; Sy+=ky[5]*v;
        v*=dyDf;
        Sx+=kx[1]*v; Sy+=ky[1]*v;
        v=dxDf*dyDf;
        Sx+=kx[6]*v; Sy+=ky[6]*v;
        v*=dyDf;
        Sx+=kx[3]*v; Sy+=ky[3]*v;
        v=dxDf*dxDf;
        Sx+=kx[4]*v; Sy+=ky[4]*v;
        dyDf*=v;
        Sx+=kx[2]*dyDf; Sy+=ky[2]*dyDf;
        v*=dxDf;
        Sx+=kx[0]*v; Sy+=ky[0]*v;
        min(10, 20);
        *pxC=xD-long(Sx)-(TDepth[min(zD,2047)]>>4); // round before sub : loosing precision !
        *pyC=long(Sy)+yD;
    }
};

class ScheinrieseApp : public AppBasic {
public:
	void setup();
	void mouseDown( MouseEvent event );	
	void update();
	void draw();
    void keyDown( KeyEvent pEvent );
    void quit();
    
private:
    void drawBlobsAndTracks( const CvBlobs& pBlobs, const CvTracks& pTracks );
    void drawDelaunay2D( const CvBlobs& pBlobs );
    void drawTriangulatedBlobs(const CvBlobs & pBlobs);
    void drawCameraImages();
    void        draw( const TriMesh2d &mesh );
    TriMesh2d   triangulateShape( const Shape2d & mShape );
    Shape2d     convertPolygonToShape2d( const CvContourPolygon & polygon );
    
    /* properties */
    SimpleGUI *             mGui;
    
    int                     WINDOW_WIDTH;
    int                     WINDOW_HEIGHT;    
    
    int                     FRAME_RATE;   
    bool                    FULLSCREEN;
    int                     KINECT_ANGLE;  
    int                     BLOB_THRESHOLD;
    int                     BLOB_THRESHOLD_A;
    int                     BLOB_THRESHOLD_B;
    int                     BLOB_MIN_AREA;
    int                     BLOB_MAX_AREA;
    int                     BLOB_BLUR;
    float                   BLOB_REDUCTION_MIN_DISTANCE;

    /* output */
    LabelControl *          mFPSOut;
    LabelControl *          mFacesOut;
    
    double                  mTime;
    
    /* kinect */
    Kinect *                mKinect;
	gl::Texture             mDepthTexture;
	gl::Texture             mColorTexture;
	gl::Texture             mBlobTexture;
    
    /* blobs */
    CvBlobs                 mBlobs;
    CvTracks                mTracks;
};

void ScheinrieseApp::setup() {    
    /* settings */
    mGui = new SimpleGUI(this);
    mGui->addParam("WINDOW_WIDTH", &WINDOW_WIDTH, 0, 2048, 640);
    mGui->addParam("WINDOW_HEIGHT", &WINDOW_HEIGHT, 0, 2048, 480);
    mGui->addParam("FULLSCREEN", &FULLSCREEN, false, 0);
    // visible
    mGui->addParam("FRAME_RATE", &FRAME_RATE, 1, 120, 30);
    mGui->addParam("KINECT_ANGLE", &KINECT_ANGLE, -31, 30, 20);
    mGui->addParam("BLOB_THRESHOLD", &BLOB_THRESHOLD, 1, 255, 127);
    mGui->addParam("BLOB_MIN_AREA", &BLOB_MIN_AREA, 1, 100000, 5000);
    mGui->addParam("BLOB_MAX_AREA", &BLOB_MAX_AREA, 1, 500000, 100000);
    mGui->addParam("BLOB_BLUR", &BLOB_BLUR, 0, 20, 7);
    mGui->addParam("BLOB_REDUCTION_MIN_DISTANCE", &BLOB_REDUCTION_MIN_DISTANCE, 0, 20, 10);
    
    // output
    mGui->addSeparator();
    mFPSOut = mGui->addLabel("");
    mFacesOut = mGui->addLabel("");

    /* clean up controller window */
    mGui->getControlByName("WINDOW_WIDTH")->active=false;
    mGui->getControlByName("WINDOW_HEIGHT")->active=false;
    mGui->getControlByName("FULLSCREEN")->active=false;

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
        mBlobTexture = mKinect->getDepthImage();
        while(!mKinect->checkNewVideoFrame()) {}
        mColorTexture = mKinect->getVideoImage();
        console() << "depth: " << mDepthTexture.getWidth() << ", " << mDepthTexture.getHeight() << endl;
        console() << "color: " << mColorTexture.getWidth() << ", " << mColorTexture.getHeight() << endl;
	}
    
    /* app */
    setFullScreen( FULLSCREEN );
    if (FULLSCREEN) {
        hideCursor();
    }
    setWindowSize( WINDOW_WIDTH, WINDOW_HEIGHT );
}

void ScheinrieseApp::mouseDown( MouseEvent event )
{
    Vec2f v;
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
        if( mKinect->checkNewVideoFrame() ) {
            if (mGui->isEnabled()) {
                mColorTexture.update(mKinect->getVideoImage());
//                cv::Mat mMat = toOcv(mKinect->getVideoImage());
//                cvEqualizeHist(&mMat, &mMat);
//                mColorTexture.update(fromOcv(mMat));
            }
        }
        if (mKinect->getTilt() != KINECT_ANGLE) {
            mKinect->setTilt(KINECT_ANGLE);
        }
        if (mKinect->checkNewDepthFrame()) {
            Surface mSurface = mKinect->getDepthImage();

            /* get image from capture device */
            Surface::Iter iter = mSurface.getIter();
            
            /* convert to grey scale */
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
            if (mGui->isEnabled()) {
                mDepthTexture.update(mSurface);
                cv::Mat mResultTexCV = mGreyImage;
                mBlobTexture.update(fromOcv(mResultTexCV));
            }
            
            cvFilterByArea(mBlobs, BLOB_MIN_AREA, BLOB_MAX_AREA);
            cvUpdateTracks(mBlobs, mTracks, 5., 10);
            
            /* clean up */
            cvReleaseImage(&mGreyImage);
            cvReleaseImage(&mLabelImg);
        }
//        
//        long mX, mY;
//        KinectData mData;
//        mData.GetColor640CoordFromDepth640(0, 0, 0, &mX, &mY);
//        console() << "out: " << mX << ", " << mY;
    }
}

void ScheinrieseApp::draw() {
    /* -- */
	gl::clear( Color( 1, 1, 1) ); 
    
    /* gui */
    gl::color(1, 1, 1, 1);
    if (mGui->isEnabled()) {
        drawCameraImages();
    }

    /* normalize texture coordinates */
    Vec2f mNormalizeScale = Vec2f(1.0 / float(640), 1.0 / float(480));
//    Vec2f mNormalizeScale = Vec2f(1.0 / float(WINDOW_WIDTH), 1.0 / float(WINDOW_HEIGHT));
    glMatrixMode(GL_TEXTURE);
    glPushMatrix();
    glScalef(mNormalizeScale.x, mNormalizeScale.y, 1.0);
    glMatrixMode(GL_MODELVIEW);

    bool DRAW_DEBUG = false;
    if (DRAW_DEBUG) {
        gl::enableWireframe();
    } else {
        mColorTexture.enableAndBind();
    }

    drawTriangulatedBlobs(mBlobs);
//    drawBlobsAndTracks(mBlobs, mTracks);  
//    drawDelaunay2D(mBlobs);

    if (DRAW_DEBUG) {
        gl::disableWireframe();        
    } else {
        mColorTexture.disable();
    }

    /* restore texture coordinates */
    glMatrixMode(GL_TEXTURE);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);

    /* gui */
    mGui->draw();
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
        
        gl::draw(mBlobTexture, Rectf(10 + 330 * 2, 
                                     10, 
                                     330 * 3, 
                                     250 ));        
    }
}

    
void ScheinrieseApp::drawTriangulatedBlobs(const CvBlobs & pBlobs) {
    /* iterate results */
    for (CvBlobs::const_iterator it=pBlobs.begin(); it!=pBlobs.end(); ++it) {           
        
        /* draw polygons */
        const CvContourPolygon* polygon = cvConvertChainCodesToPolygon(&(*it).second->contour);
        gl::color(1, 0, 0, 1);

        /* draw triangulated polygons */
        Shape2d mShape = convertPolygonToShape2d(*polygon);
        gl::color(1, 1, 1, 1);
        TriMesh2d mMesh = triangulateShape(mShape);
        for (int i=0; i < mMesh.getVertices().size(); ++i) {
            mMesh.appendTexCoord(Vec2f(mMesh.getVertices()[i]));
        }
        draw(mMesh);
    }
}


void ScheinrieseApp::keyDown( KeyEvent pEvent ) {
    switch(pEvent.getChar()) {				
        case 'd': mGui->dump(); break;
        case 'l': mGui->load(getResourcePath(RES_SETTINGS)); break;
        case 's': mGui->save(getResourcePath(RES_SETTINGS)); break;
    }
    switch(pEvent.getCode()) {
        case KeyEvent::KEY_ESCAPE:  setFullScreen( false ); quit(); break;
        case KeyEvent::KEY_SPACE: mGui->setEnabled(!mGui->isEnabled());break;
    }
}

void ScheinrieseApp::quit() {
    delete mKinect;
    delete mGui;
    delete mFPSOut;
    AppBasic::quit();
}


void draw_subdiv_edge(CvSubdiv2DEdge edge) {
    CvSubdiv2DPoint* org_pt;
    CvSubdiv2DPoint* dst_pt;
    CvPoint2D32f org;
    CvPoint2D32f dst;
    CvPoint iorg, idst;
    
    org_pt = cvSubdiv2DEdgeOrg(edge);
    dst_pt = cvSubdiv2DEdgeDst(edge);
    
    if( org_pt && dst_pt )
    {
        org = org_pt->pt;
        dst = dst_pt->pt;
        
        iorg = cvPoint( cvRound( org.x ), cvRound( org.y ));
        idst = cvPoint( cvRound( dst.x ), cvRound( dst.y ));
        
        gl::drawLine(Vec2f(idst.x, idst.y), Vec2f(iorg.x, iorg.y));
    }
}

/* create and draw voronoi and delauny shapes */

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
        const CvContourPolygon* sPolygon = cvSimplifyPolygon(cvConvertChainCodesToPolygon(&(*it).second->contour), BLOB_REDUCTION_MIN_DISTANCE);
        
        CvSubdiv2D* subdiv;
        CvMemStorage* storage;
        CvRect rect = { 0, 0, 640, 480 };
        
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
            cvCalcSubdivVoronoi2D( subdiv );
        }
        draw_subdiv( subdiv );

        cvReleaseMemStorage( &storage );
    }
}


void ScheinrieseApp::drawBlobsAndTracks(const CvBlobs& pBlobs, const CvTracks& pTracks) {
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
        const CvContourPolygon* sPolygon = cvSimplifyPolygon(polygon, BLOB_REDUCTION_MIN_DISTANCE);
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


CINDER_APP_BASIC( ScheinrieseApp, RendererGl )
