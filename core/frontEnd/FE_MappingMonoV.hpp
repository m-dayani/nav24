//
// Created by masoud on 4/7/25.
//

#ifndef NAV24_FE_MAPPINGMONOV_HPP
#define NAV24_FE_MAPPINGMONOV_HPP


#include <map>

#include "FrontEnd.hpp"
#include "Image.hpp"
#include "OP_FtDtOrbSlam.hpp"
#include "OP_FtAssocOrbSlam.hpp"


namespace NAV24::FE {

    class MappingMonoV : public FrontEnd {
    public:
        explicit MappingMonoV(const ChannelPtr& pChannel);

        void receive(const MsgPtr &msg) override;

    protected:
        void setup(const MsgPtr &configMsg) override;
        void run() override;

    private:
        void createNewFrame();

    private:
        std::map<double, PosePtr> mmPose;
        std::mutex mPoseMapLock;

        std::map<double, ImagePtr> mmImage;
        std::mutex mImageMapLock;

        OP::FtDtPtr mpOrbDetector;
        std::shared_ptr<OP::FtAssocOrbSlam> mpOrbMatcher;

//        ParamPtr mpTempParam;;
    };

} // NAV24::FE

#endif //NAV24_FE_MAPPINGMONOV_HPP
