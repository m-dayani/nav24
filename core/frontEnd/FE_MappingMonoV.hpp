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

#define FE_DEF_WORLD_NAME "world0"

    class MappingMonoV : public FrontEnd {
    public:
        explicit MappingMonoV(const ChannelPtr& pChannel);

        void receive(const MsgPtr &msg) override;

    protected:
        void setup(const MsgPtr &configMsg) override;
        void run() override;

    private:
        void processImage();
        void createNewFrame();

    private:
        std::map<double, PosePtr> mmPose;
        std::mutex mPoseMapLock;

        std::map<double, ImagePtr> mmImage;
        std::mutex mImageMapLock;

        OP::FtDtPtr mpOrbDetector;
        std::shared_ptr<OP::FtAssocOrbSlam> mpOrbMatcher;

        PosePtrConst T_bc0;

        FramePtr mpLastFrame;

        std::vector<FramePtr> mvpAllFrames;

        CalibPtrRO mpCalib;

//        ParamPtr mpTempParam;;
    };

} // NAV24::FE

#endif //NAV24_FE_MAPPINGMONOV_HPP
