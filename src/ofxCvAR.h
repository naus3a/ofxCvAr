//
//  ofxCvAR.h
//
//  Created by nausea on 2/11/15.
//
//

#pragma once
#include "ofMain.h"
#include "ofxCv.h"

namespace ofxCv {
    class ARMatrixFilter{
    public:
        ARMatrixFilter(){
            oldTvec = cv::Mat_<double>::zeros(3, 1);
            oldRvec = cv::Mat_<double>::zeros(3, 1);
            smt = 0.9;
            use(true);
        }
        
        void use(bool b){
            bActive = b;
            needsFiltering = false;
        }
        
        void checkFilteringNeed(bool bFound, bool bOld){
            if(!bOld){
                needsFiltering = false;
                return;
            }else{
                needsFiltering = bFound;
            }
        }
        
        void update(cv::Mat * newRvec, cv::Mat * newTvec){
            if(bActive){
                if(needsFiltering){
                    smoothCvVec(newRvec, &oldRvec);
                    smoothCvVec(newTvec, &oldTvec);
                }
                oldRvec = *newRvec;
                oldTvec = *newTvec;
            }
        }
        
        void smoothCvVec(cv::Mat * vNew, cv::Mat * vOld){
            double dx = vOld->at<double>(0) - vNew->at<double>(0);
            double dy = vOld->at<double>(1) - vNew->at<double>(1);
            double dz = vOld->at<double>(2) - vNew->at<double>(2);
            
            vNew->at<double>(0) += dx * smt;
            vNew->at<double>(1) += dy * smt;
            vNew->at<double>(2) += dz * smt;
        }
        
        cv::Mat oldTvec;
        cv::Mat oldRvec;
        float smt;
        bool bActive;
        bool needsFiltering;
    };
    
    class AR{
    public:
        AR();
        void init();
        void setup(float _near, float _far, ofRectangle _vp, Calibration _calib);
        void setNearFar(float _near, float _far, bool bUpdate=false);
        void setViewport(ofRectangle _vp, bool bUpdate=false);
        void setCalibration(Calibration _calib, bool bUpdate=false);
        void createMarker(float w, float h);
        ofMatrix4x4 & getModelMatrix();
        ofVec3f getModelRotation();
        ofVec3f getModelTranslation();
        void loadProjectionMatrix();
        void beginAR();
        void endAR();
        void update(cv::Mat screenPts);
        
        ARMatrixFilter filter;
        
    private:
        Calibration calib;
        //cv::Mat tVec;
        //cv::Mat rVec;
        cv::Size markerSize;
        vector<cv::Point3f> worldPts;
        ofMatrix4x4 matModel;
        ofRectangle viewPort;
        vector<float> projFrustum;
        float nearDist;
        float farDist;
    };
}
