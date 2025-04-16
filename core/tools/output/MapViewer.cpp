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
        mSetFirstPoseState(0), mmTrajColors() {

        srand (static_cast <unsigned> (time(0)));
    }

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

    void MapViewer::drawTrajectory(const std::vector<PosePtr>& spPose, const std::vector<float> &color) const {

        glLineWidth(mGraphLineWidth);
        if (color.size() == 4) {
            glColor4f(color[0], color[1], color[2], color[3]);
        }
        else {
            glColor4f(1.0f, 0.6f, 0.0f, 0.6f);
        }
        glBegin(GL_LINES);

        //Draw inertial links
        PosePtr prevPose = nullptr;
        for(const auto& pPose : spPose) {
            if (prevPose) {

                Eigen::Vector3f Ow = prevPose->getPose().cast<float>().block<3, 1>(0, 3);
                Eigen::Vector3f Owp = pPose->getPose().cast<float>().block<3, 1>(0, 3);
                glVertex3f(Ow(0), Ow(1), Ow(2));
                glVertex3f(Owp(0), Owp(1), Owp(2));
            }
            prevPose = pPose;
        }

        glEnd();
    }

    void MapViewer::drawTrajectories(const MapNamedPose &poseTable) const {

        for(const auto& poseInfo : poseTable) {

            string trajName = poseInfo.first;
            vector<float> color{};
            if (mmTrajColors.contains(trajName)) {
                color = mmTrajColors.at(trajName);
            }
            this->drawTrajectory(poseInfo.second, color);
        }
    }

    void MapViewer::receive(const MsgPtr &msg) {
        Output::receive(msg);

        if (msg) {
            if (dynamic_pointer_cast<MsgType<PosePtr>>(msg)) {
                auto pPose = dynamic_pointer_cast<MsgType<PosePtr>>(msg)->getData();
                this->insertPoses({pPose});
            }
            if (dynamic_pointer_cast<MsgType<vector<PosePtr>>>(msg)) {
                auto vpPose = dynamic_pointer_cast<MsgType<vector<PosePtr>>>(msg)->getData();
                this->insertPoses(vpPose);
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

        MapNamedPose poseTable;
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

        DLOG(INFO) << "MapViewer::run, started\n";

        while(!pangolin::ShouldQuit()) {

            this->retrievePoses(poseTable);
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
            for (const auto& poseInfo : poseTable) {
                string trajName = poseInfo.first;
                bool loop1 = true;
                for (const auto& pose : poseInfo.second) {
                    if (pose->getLevel() >= 1) {
                        // draw only keyframes
                        this->drawPoseFrame(pose);
                    }
                    if (mLastPose == nullptr && loop1) {
                        mLastPose = pose;
                        loop1 = false;
                    }
                }
            }

            // draw trajectory
            this->drawTrajectories(poseTable);

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

            if (this->isStopped()) {
                //cout << vpPose.size() << "\n";
                break;
            }

            // Swap frames and Process Events
            pangolin::FinishFrame();
        }

        DLOG(INFO) << "MapViewer::run, stopped\n";

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
        this_thread::sleep_for(chrono::microseconds(100));
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
//            Twc = Twc.inverse().eval();
        }

        for (int i = 0; i<4; i++) {
            M.m[4*i] = Twc(0,i);
            M.m[4*i+1] = Twc(1,i);
            M.m[4*i+2] = Twc(2,i);
            M.m[4*i+3] = Twc(3,i);
        }
    }

    void MapViewer::insertPoses(const vector <PosePtr> &vpPose) {

        mMtxPoseQueue.lock();
        for (const auto& pPose : vpPose) {
            if (pPose) {
                string poseName = pPose->getName();
                if (!poseName.empty()) {
                    if (!mPoseTable.contains(poseName)) {
                        mPoseTable[poseName] = vector<PosePtr>();
                    }
                    mPoseTable[poseName].push_back(pPose);
                }
                if (!mmTrajColors.contains(poseName)) {
                    vector<float> color{1.f, 0.f, 0.f, 0.f};
                    for (size_t i = 1; i < color.size(); i++)
                        color[i] = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
                    mmTrajColors[poseName] = color;
                }
            }
        }
        mMtxPoseQueue.unlock();
    }

    void MapViewer::retrievePoses(MapNamedPose &poseTable) {

        mMtxPoseQueue.lock();
        for (const auto& poseInfo : mPoseTable) {
            string poseName = poseInfo.first;
            if (!poseTable.contains(poseName)) {
                poseTable[poseName] = vector<PosePtr>();
            }
            poseTable[poseName].clear();
            poseTable[poseName].reserve((poseInfo.second.size()));
            for (const auto& pPose : poseInfo.second) {
                if (pPose && pPose->isValid()) {
                    poseTable[poseName].push_back(pPose);
                }
            }
        }
        mPoseTable = poseTable;
        mMtxPoseQueue.unlock();
    }




#endif

} // NAV24
