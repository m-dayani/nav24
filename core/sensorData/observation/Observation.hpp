//
// Created by masoud on 2/11/24.
//

#ifndef NAV24_OBSERVATION_HPP
#define NAV24_OBSERVATION_HPP

#include <memory>

#include "SmartObject.hpp"


namespace NAV24::WO {
    class WorldObject;
    typedef std::shared_ptr<WorldObject> woPtr;
    typedef std::weak_ptr<WorldObject> woPtrW;
}

namespace NAV24::OB {

    class Observation : public SmartObject {
    public:

        void setWorldObject(const WO::woPtr& pWo) { mpWo = pWo; }
        WO::woPtr getWorldObject() { return mpWo.lock(); }

        virtual std::string toString() { return "Observation, Base Class\n"; }
    protected:
        WO::woPtrW mpWo{};
    };
    typedef std::shared_ptr<Observation> ObsPtr;
    typedef std::weak_ptr<Observation> ObsPtrW;

    typedef std::pair<long, ObsPtr> ObsTimed;

} // NAV24::OB

#endif //NAV24_OBSERVATION_HPP
