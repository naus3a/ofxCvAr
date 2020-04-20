//
//  ofxCvAR.cpp
//
//  Created by nausea on 2/11/15.
//
//

#include "ofxCvAR.h"

namespace ofxCv{
    AR::AR(){
        projFrustum.reserve(4);
        setNearFar(10., 1000.);
        setViewport(ofRectangle(0,0,640,480));
        createMarker(10,10);
        //tVec = Mat_<double>::zeros(3, 1);
        //rVec = Mat_<double>::zeros(3, 1);
    }
    
    void AR::setup(float _near, float _far, ofRectangle _vp, Calibration _calib){
        setNearFar(_near, _far);
        setViewport(_vp);
        setCalibration(_calib);
        init();
    }
    
    void AR::setNearFar(float _near, float _far, bool bUpdate){
        nearDist = _near;
        farDist = _far;
        if(bUpdate)init();
    }
    
    void AR::setViewport(ofRectangle _vp, bool bUpdate){
        viewPort = _vp;
        if(bUpdate)init();
    }
    
    void AR::setCalibration(ofxCv::Calibration _calib, bool bUpdate){
        calib = _calib;
        if(bUpdate)init();
    }
    
    void AR::init(){
        float w = viewPort.width;
        float h = viewPort.height;
        float fx = calib.getDistortedIntrinsics().getCameraMatrix().at<double>(0,0);
        float fy = calib.getDistortedIntrinsics().getCameraMatrix().at<double>(1,1);
        float cx = calib.getDistortedIntrinsics().getPrincipalPoint().x;;
        float cy = calib.getDistortedIntrinsics().getPrincipalPoint().y;
        projFrustum[0] = nearDist * (-cx) / fx;
        projFrustum[1] = nearDist * (w - cx) / fx;
        projFrustum[2] = nearDist * (cy - h) / fy;
        projFrustum[3] = nearDist * (cy) / fy;
    }
    
    void AR::createMarker(float w, float h){
        markerSize.width = w;
        markerSize.height= h;
        worldPts.clear();
        worldPts.push_back(cv::Point3f(-markerSize.width/2, -markerSize.height/2, 0));
        worldPts.push_back(cv::Point3f( markerSize.width/2, -markerSize.height/2, 0));
        worldPts.push_back(cv::Point3f( markerSize.width/2,  markerSize.height/2, 0));
        worldPts.push_back(cv::Point3f(-markerSize.width/2,  markerSize.height/2, 0));
    }
    
    ofMatrix4x4 & AR::getModelMatrix(){
        return matModel;
    }
    
    ofVec3f AR::getModelRotation(){
        return ofVec3f(filter.oldRvec.at<double>(0),filter.oldRvec.at<double>(1),filter.oldRvec.at<double>(2));
    }
    
    ofVec3f AR::getModelTranslation(){
        return ofVec3f(filter.oldTvec.at<double>(0),filter.oldTvec.at<double>(1),filter.oldTvec.at<double>(2));
    }
    
    void AR::loadProjectionMatrix(){
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glFrustum(projFrustum[0], projFrustum[1], projFrustum[2], projFrustum[3], nearDist, farDist);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        gluLookAt(0, 0, 0,
                  0, 0, 1,
                  0,-1, 0);
    }
    
    void AR::update(cv::Mat screenPts){
        cv::Mat rVec, tVec;
        cv::solvePnP(cv::Mat(worldPts), cv::Mat(screenPts), calib.getDistortedIntrinsics().getCameraMatrix(), calib.getDistCoeffs(), rVec, tVec);
        
        filter.update(&rVec, &tVec);
        
        matModel = makeMatrix(rVec, tVec);
    }
    
    void AR::beginAR(){
        ofPushView();
        ofViewport(viewPort);
        loadProjectionMatrix();
        applyMatrix(matModel);
    }
    
    void AR::endAR(){
        ofPopView();
    }
    
}