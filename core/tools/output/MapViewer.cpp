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
        mbDisabled(false), mCameraLineWidth(3.f), mCameraSize(0.08f),
        mFrameLineWidth(1.f), mFrameSize(0.05f),
        mGraphLineWidth(0.9), mPointSize(2.f), mViewpointF(420.f),
        mViewpointX(0.f), mViewpointY(-0.7f), mViewpointZ(-3.5f),
        mMtxPoseQueue(), mMtxWoQueue() {}

    void MapViewer::drawPose(const PosePtr& pPose) const {

        const float &w = mCameraSize;
        const float h = w*0.75f;
        const float z = w*0.6f;

        Eigen::Matrix4f Twc = pPose->getPose().cast<float>();
        //unsigned int index_color = pKF->mnOriginMapId;

        glPushMatrix();
        glMultMatrixf((GLfloat*)Twc.data());

        glLineWidth(mCameraLineWidth);
        glColor3f(1.0f,0.0f,0.0f);
        glBegin(GL_LINES);

        glVertex3f(0,0,0);
        glVertex3f(w,h,z);
        glVertex3f(0,0,0);
        glVertex3f(w,-h,z);
        glVertex3f(0,0,0);
        glVertex3f(-w,-h,z);
        glVertex3f(0,0,0);
        glVertex3f(-w,h,z);

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
    }

    void MapViewer::drawWorldObject(const WO::woPtr &pWo) const {

        glPointSize(mPointSize);
        glBegin(GL_POINTS);
        glColor3f(0.0, 0.0, 0.0);

        if (pWo && static_pointer_cast<WO::Point3D>(pWo)) {
            auto pt3d = static_pointer_cast<WO::Point3D>(pWo);
            auto pos = pt3d->getPoint();
            glVertex3f((float)pos.x, (float)pos.y, (float)pos.z);
        }

        glEnd();
    }

    void MapViewer::drawTrajectory(const vector <FramePtr> &vpFrame) {


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
            if (dynamic_pointer_cast<MsgType<vector<WO::woPtr>>>(msg)) {
                auto vpWo = dynamic_pointer_cast<MsgType<vector<WO::woPtr>>>(msg)->getData();
                mMtxWoQueue.lock();
                for (const auto& pWo : vpWo) {
                    mspWorldObjects.insert(pWo);
                }
                mMtxWoQueue.unlock();
            }
            if (dynamic_pointer_cast<MsgType<WO::woPtr>>(msg)) {
                auto pWo = dynamic_pointer_cast<MsgType<WO::woPtr>>(msg)->getData();
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
        set<WO::woPtr> spWoCopy;

        pangolin::CreateWindowAndBind("Main",640,480);
        glEnable(GL_DEPTH_TEST);

        // Define Projection and initial ModelView matrix
        pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(640,480,mViewpointF,mViewpointF,
                                           320,240,0.2,100),
                pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ,
                                          0,0,0, 0.0, -1.0, 0.0)
        );

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
            glClearColor(1.0f,1.0f,1.0f,1.0f);
            // Render OpenGL Cube
            //pangolin::glDrawColouredCube();

            // Draw visible poses
            for (const auto& pose : spPoseCopy) {
                this->drawPose(pose);
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

    void MapViewer::requestStop(const string &channel) {

    }

    void MapViewer::stop() {
        MsgCallback::stop();
        pangolin::QuitAll();
    }

    bool MapViewer::isStopped() {
        return MsgCallback::isStopped();
    }

} // NAV24
