//
// Created by masoud on 2/22/25.
//

#ifndef NAV24_OP_OBJDETAPRILTAG_HPP
#define NAV24_OP_OBJDETAPRILTAG_HPP


#ifdef LIB_APRILTAG_FOUND
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#endif

#include "OP_ObjDet.hpp"


namespace NAV24::OP  {
    class ObjDetApriltag : public ObjDet {
    public:
        explicit ObjDetApriltag(const ChannelPtr& pChannel) : ObjDet(pChannel) {}
        explicit ObjDetApriltag(const std::string& family);
        ~ObjDetApriltag();

        void detect(const ImagePtr& pImage, std::vector<OB::ObsPtr> &vpObs) override;

    protected:
        void setup(const MsgPtr &configMsg) override;

    private:
        std::string mTagFamily;
#ifdef LIB_APRILTAG_FOUND
        apriltag_family_t* mpFamily{};
        apriltag_detector_t* mpDetector{};
#endif

    };
}   // NAV24::OP


#endif //NAV24_OP_OBJDETAPRILTAG_HPP
