//
// Created by masoud on 6/17/24.
//

#include <thread>
#include <glog/logging.h>

#include "MapViewer.hpp"
#include "Point3D.hpp"


using namespace std;


namespace NAV24 {

    MapViewer::MapViewer(const ChannelPtr& pChannel) : Output(pChannel),
        mbDisabled(false), mFrameSize(0.05f), mFrameLineWidth(1.f),
        mGraphLineWidth(0.9), mPointSize(2.f), mCameraSize(0.08f),
        mCameraLineWidth(3.f), mViewpointX(0.f), mViewpointY(-0.7f), mViewpointZ(-3.5f),
        mViewpointF(420.f), mMtxPoseQueue(), mMtxWoQueue(), mLastPose(),
        mSetFirstPoseState(0) {}

    void MapViewer::drawPose(const PosePtr& pPose) const {

        const float &w = mCameraSize;
        const float h = w*0.75f;
        const float z = w*0.6f;

        Eigen::Matrix4f Twc = pPose->getPose().cast<float>();
        //unsigned int index_color = pKF->mnOriginMapId;

#ifdef LIB_PANGOLIN_FOUND
//        pangolin::glDrawColouredCube();
        glPushMatrix();
        glMultMatrixf((GLfloat*)Twc.data());

        glLineWidth(mCameraLineWidth);
        glColor3f(1.0f,0.0f,0.0f);
        glBegin(GL_LINES);

        glVertex3f(0,0,0);
        glVertex3f(w,h, z);
        glVertex3f(0,0,0);
        glVertex3f(w,-h, z);
        glVertex3f(0,0,0);
        glVertex3f(-w,-h, z);
        glVertex3f(0,0,0);
        glVertex3f(-w,h, z);

        glVertex3f(w,h,z);
        glVertex3f(w,-h,z);

        glVertex3f(-w,h,z);
        glVertex3f(-w,-h,z);

        glVertex3f(-w,h,z);
        glVertex3f(w,h,z);

        glVertex3f(-w,-h,z);
        glVertex3f(w,-h,z);
        glEnd();

        glPopMatrix();
#endif
    }

    void MapViewer::drawPoseFrame(const PosePtr &pPose) const {

        const float &x = mCameraSize;
        const float y = x*1.0f;
        const float z = x*1.0f;

        Eigen::Matrix4f Twc = pPose->getPose().cast<float>();

#ifdef LIB_PANGOLIN_FOUND
//        pangolin::glDrawColouredCube();
        glPushMatrix();
        glMultMatrixf((GLfloat*)Twc.data());

        glLineWidth(mCameraLineWidth);
        glBegin(GL_LINES);

        // x-axis
        glColor3f(1.0f,0.0f,0.0f);
        glVertex3f(0,0,0);
        glVertex3f(x,0,0);

        // y-axis
        glColor3f(0.0f,1.0f,0.0f);
        glVertex3f(0,0,0);
        glVertex3f(0,y,0);

        // z-axis
        glColor3f(0.0f,0.0f,1.0f);
        glVertex3f(0,0,0);
        glVertex3f(0,0,z);

        glEnd();

        glPopMatrix();
#endif
    }

    void MapViewer::drawWorldObject(const WO::WoPtr &pWo) const {
#ifdef LIB_PANGOLIN_FOUND
//        pangolin::glDrawColouredCube();
        glPointSize(mPointSize);
        glBegin(GL_POINTS);
        glColor3f(0.0, 0.0, 0.0);

        if (pWo && dynamic_pointer_cast<WO::Point3D>(pWo)) {
            auto pt3d = dynamic_pointer_cast<WO::Point3D>(pWo);
            auto pos = pt3d->getPoint();
            glVertex3f((float)pos.x, (float)pos.y, (float)pos.z);
        }

        glEnd();
#endif
    }

    void MapViewer::drawTrajectory(const vector <FramePtr> &) {

    }

    void MapViewer::receive(const MsgPtr &msg) {
        Output::receive(msg);

        if (msg) {
            if (dynamic_pointer_cast<MsgType<PosePtr>>(msg)) {
                mMtxPoseQueue.lock();
                auto pPose = dynamic_pointer_cast<MsgType<PosePtr>>(msg)->getData();
                mspPose.insert(pPose);
                mMtxPoseQueue.unlock();
            }
            if (dynamic_pointer_cast<MsgType<vector<FramePtr>>>(msg)) {
                auto vpFrames = dynamic_pointer_cast<MsgType<vector<FramePtr>>>(msg)->getData();
                mMtxPoseQueue.lock();
                for (const auto& pFrame : vpFrames) {
                    if (pFrame) {
                        mspPose.insert(pFrame->getPose());
                    }
                }
                mMtxPoseQueue.unlock();
            }
            if (dynamic_pointer_cast<MsgType<vector<WO::WoPtr>>>(msg)) {
                auto vpWo = dynamic_pointer_cast<MsgType<vector<WO::WoPtr>>>(msg)->getData();
                mMtxWoQueue.lock();
                for (const auto& pWo : vpWo) {
                    mspWorldObjects.insert(pWo);
                }
                mMtxWoQueue.unlock();
            }
            if (dynamic_pointer_cast<MsgType<WO::WoPtr>>(msg)) {
                auto pWo = dynamic_pointer_cast<MsgType<WO::WoPtr>>(msg)->getData();
                mMtxWoQueue.lock();
                mspWorldObjects.insert(pWo);
                mMtxWoQueue.unlock();
            }

        }
    }

    void MapViewer::run() {

        if (mbDisabled) {
            DLOG(INFO) << "MapViewer::run, Map Viewer is disabled, abort\n";
            return;
        }

        set<PosePtr> spPoseCopy;
        set<WO::WoPtr> spWoCopy;

#ifdef LIB_PANGOLIN_FOUND

        pangolin::OpenGlMatrix Twc;

        pangolin::CreateWindowAndBind(mName, 640, 480);
        glEnable(GL_DEPTH_TEST);

        // Define Projection and initial ModelView matrix
        auto defProjMat = pangolin::ProjectionMatrix(640,480,mViewpointF,mViewpointF,
                                                     320,240,0.2,100);
        auto defModelView = pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ,
                                                      0,0,0, 0.0, -1.0, 0.0);
        pangolin::OpenGlRenderState s_cam(defProjMat, defModelView);

        // Create Interactive View in window
        pangolin::Handler3D handler(s_cam);
        pangolin::View& d_cam = pangolin::CreateDisplay()
                .SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f/480.0f)
                .SetHandler(&handler);

        while(!pangolin::ShouldQuit()) {

            mMtxPoseQueue.lock();
            spPoseCopy = mspPose;
            mMtxPoseQueue.unlock();
            mMtxWoQueue.lock();
            spWoCopy = mspWorldObjects;
            mMtxWoQueue.unlock();

            // Clear screen and activate view to render into
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            d_cam.Activate(s_cam);
            glClearColor(0.7f,0.7f,0.7f,1.0f);

            // Render OpenGL Cube
//            pangolin::glDrawColouredCube();
//            drawTrajectory(vector<FramePtr>());

            // Draw visible poses
            for (const auto& pose : spPoseCopy) {
                this->drawPoseFrame(pose);
                if (mLastPose == nullptr) {
                    mLastPose = pose;
                }
            }

            // set first camera view
            if (mSetFirstPoseState < 2 && mLastPose != nullptr) {
                getLastOpenGlCamera(Twc);
                s_cam.SetProjectionMatrix(defProjMat);
                s_cam.SetModelViewMatrix(defModelView);
                s_cam.Follow(Twc);
                mSetFirstPoseState++;
            }

            // Draw world objects
            for (const auto& pWo : spWoCopy) {
                this->drawWorldObject(pWo);
            }

            // Swap frames and Process Events
            pangolin::FinishFrame();

            if (this->isStopped()) {
                //cout << vpPose.size() << "\n";
                break;
            }
        }

        // unset the current context from the main thread
//        pangolin::GetBoundWindow()->RemoveCurrent();
#endif
    }

    void MapViewer::setup(const MsgPtr &msg) {
        Output::setup(msg);

        // retrieve MapViewer params
        if (msg && dynamic_pointer_cast<MsgConfig>(msg)) {
            auto msgConfig = dynamic_pointer_cast<MsgConfig>(msg);
            auto pParam = msgConfig->getConfig();
            if (pParam) {
                auto pDisabled = find_param<ParamType<int>>("disabled", pParam);
                mbDisabled = (pDisabled) ? pDisabled->getValue() : mbDisabled;

                auto pFrameSize = find_param<ParamType<double>>("frameSize", pParam);
                mFrameSize = static_cast<float>((pFrameSize) ? pFrameSize->getValue() : mFrameSize);

                auto pFrameLineWidth = find_param<ParamType<double>>("frameLineWidth", pParam);
                mFrameLineWidth = static_cast<float>((pFrameLineWidth) ? pFrameLineWidth->getValue() : mFrameLineWidth);

                auto pGraphLineWidth = find_param<ParamType<double>>("graphLineWidth", pParam);
                mGraphLineWidth = static_cast<float>((pGraphLineWidth) ? pGraphLineWidth->getValue() : mGraphLineWidth);

                auto pPointSize = find_param<ParamType<double>>("pointSize", pParam);
                mPointSize = static_cast<float>((pPointSize) ? pPointSize->getValue() : mPointSize);

                auto pCameraSize = find_param<ParamType<double>>("cameraSize", pParam);
                mCameraSize = static_cast<float>((pCameraSize) ? pCameraSize->getValue() : mCameraSize);

                auto pCameraLineWidth = find_param<ParamType<double>>("cameraLineWidth", pParam);
                mCameraLineWidth = static_cast<float>((pCameraLineWidth) ? pCameraLineWidth->getValue() : mCameraLineWidth);

                auto pViewpointX = find_param<ParamType<double>>("viewpointX", pParam);
                mViewpointX = static_cast<float>((pViewpointX) ? pViewpointX->getValue() : mViewpointX);

                auto pViewpointY = find_param<ParamType<double>>("viewpointY", pParam);
                mViewpointY = static_cast<float>((pViewpointY) ? pViewpointY->getValue() : mViewpointY);

                auto pViewpointZ = find_param<ParamType<double>>("viewpointZ", pParam);
                mViewpointZ = static_cast<float>((pViewpointZ) ? pViewpointZ->getValue() : mViewpointZ);

                auto pViewpointF = find_param<ParamType<double>>("viewpointF", pParam);
                mViewpointF = static_cast<float>((pViewpointF) ? pViewpointF->getValue() : mViewpointF);

            }
        }
    }

    void MapViewer::handleRequest(const MsgPtr &msg) {
        Output::handleRequest(msg);
    }

    void MapViewer::requestStop(const string &) {

    }

    void MapViewer::stop() {
        MsgCallback::stop();
#ifdef LIB_PANGOLIN_FOUND
        pangolin::QuitAll();
#endif
    }

    bool MapViewer::isStopped() {
        return MsgCallback::isStopped();
    }

#ifdef LIB_PANGOLIN_FOUND
    void MapViewer::getLastOpenGlCamera(pangolin::OpenGlMatrix &M) {

        Eigen::Matrix4f Twc;

        {
            //unique_lock<mutex> lock(mMutexCamera);
            Twc = mLastPose->getPose().cast<float>();
        }

        for (int i = 0; i<4; i++) {
            M.m[4*i] = Twc(0,i);
            M.m[4*i+1] = Twc(1,i);
            M.m[4*i+2] = Twc(2,i);
            M.m[4*i+3] = Twc(3,i);
        }
    }



#endif

} // NAV24
