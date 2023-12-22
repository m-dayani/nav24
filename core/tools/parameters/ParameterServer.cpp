//
// Created by root on 5/10/21.
//

#include "ParameterServer.hpp"

using namespace std;


namespace NAV24 {

    ParameterServer::ParameterServer(const ChannelPtr &server) : mpChannel(server) {

        // Never create shared pointers from raw pointers!
        //shared_ptr<MsgCallback> callback(this);
        //mpChannel->registerChannel(callback, ParameterServer::TOPIC);
    }

    ParameterServer::ParameterServer(const ChannelPtr& server, const MsgPtr& configMsg) : ParameterServer(server) {

        this->receive(configMsg);
    }

    std::string ParameterServer::getFullStat() {

        ostringstream oss;

        oss << "# Dataset Info:\n";
        oss << mvpDS_Params->printStr("\t");

        oss << "# Camera Parameters:\n";
        oss << CamParams::printStr(mpCamParams.get(), "\t");
        if (mpCamParams->mpLinkedCam) {
            oss << "# Right Camera Parameters:\n";
            oss << CamParams::printStr(mpCamParams->mpLinkedCam, "\t");
        }

        oss << "# IMU Parameters:\n";
        oss << mpImuParams->printStr("\t");

        return oss.str();
    }

    void ParameterServer::load(const string &settingsFile) {

        cv::FileStorage fsSettings(settingsFile, cv::FileStorage::READ);
        if(!fsSettings.isOpened()) {
            cerr << "** ERROR: Failed to open settings file: " << settingsFile << endl;
            return;
        }

        mvpDS_Params = vector<DS_ParamsPtr>();
        mvpDS_Params.push_back(make_shared<DS_Params>(fsSettings));

        mpCamParams = make_shared<CamParams>(fsSettings);

        mpImuParams = make_shared<IMU_Params>(fsSettings);

        mpSensorConfig = make_shared<SensorConfig>(fsSettings);

        fsSettings.release();
    }

    void ParameterServer::save(const string &pathParams) {

        cv::FileStorage paramsFile(pathParams, cv::FileStorage::WRITE);
        if(!paramsFile.isOpened()) {

            cerr << "** ERROR: Failed to open settings file: " << pathParams << endl;
            return;
        }

        mvpDS_Params->write(paramsFile);
        mpCamParams->write(paramsFile);
        mpImuParams->write(paramsFile);

        paramsFile.release();
    }

    void ParameterServer::receive(const MsgPtr &msg) {

        // check the topic
        string topic = msg->getTopic();
        if (topic != ParameterServer::TOPIC) {
            // log warning
            return;
        }

        // check function
        int action = msg->getTargetId();
        switch (action) {
            case FCN_PS_LOAD:
                this->load(msg->getMessage());
                break;
            case FCN_PS_SAVE:
                this->save(msg->getMessage());
                break;
            case FCN_PS_REQ:
                this->handleRequest(msg);
                break;
            default:
                //log warning
                break;
        }
    }

    void ParameterServer::handleRequest(const MsgPtr &msg) {

        MsgReqPtr request = static_pointer_cast<MsgRequest>(msg);
        if (!request) {
            // log warning
            return;
        }

        MsgCbPtr sender = request->getCallback();

        // check message to find which parameter type it requests
        string tag = request->getMessage();
        ParamPtr param{};
        string res = "";
        if (tag == DS_Params::TAG) {
            param = mvpDS_Params;
            res = mvpDS_Params->printStr();
        }
        else if (tag == CamParams::TAG) {
            param = mpCamParams;
            res = mpCamParams->printStr(mpCamParams.get());
        }
        else if (tag == IMU_Params::TAG) {
            param = mpImuParams;
            res = mpImuParams->printStr();
        }
        else if (tag == TAG_PS_GET_STAT) {
            sender->receive(make_shared<Message>(msg->getTopic(), this->getFullStat()));
            return;
        }

        // create a response message and send back to the caller
        MsgPtr response = make_shared<MsgConfig>(msg->getTopic(), param);
        response->setMessage(res);

        sender->receive(response);
    }

} // NAV24
