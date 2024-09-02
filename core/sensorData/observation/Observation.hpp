//
// Created by masoud on 2/11/24.
//

#ifndef NAV24_OBSERVATION_HPP
#define NAV24_OBSERVATION_HPP

#include <memory>

#include "SmartObject.hpp"


namespace NAV24::WO {
    class WorldObject;
    typedef std::shared_ptr<WorldObject> WoPtr;
    typedef std::weak_ptr<WorldObject> WoPtrW;
}

namespace NAV24::OB {

    class Observation : public SmartObject {
    public:

        void setWorldObject(const WO::WoPtr& pWo) { mpWo = pWo; }
        WO::WoPtr getWorldObject() { return mpWo.lock(); }

        [[nodiscard]] long getFrameId() const { return mFrameId; }
        void setFrameId(const long& frameId) { mFrameId = frameId; }

        virtual std::string toString() { return "Observation, Base Class\n"; }
    protected:
        WO::WoPtrW mpWo{};
        long mFrameId;
    };
    typedef std::shared_ptr<Observation> ObsPtr;
    typedef std::weak_ptr<Observation> ObsPtrW;

    typedef std::pair<long, ObsPtr> ObsTimed;

} // NAV24::OB

#endif //NAV24_OBSERVATION_HPP