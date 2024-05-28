//
// Created by masoud on 2/11/24.
//

#ifndef NAV24_OBSERVATION_HPP
#define NAV24_OBSERVATION_HPP

#include <memory>

namespace NAV24::WO {
    class WorldObject;
    typedef std::shared_ptr<WorldObject> woPtr;
    typedef std::weak_ptr<WorldObject> woPtrW;
}

namespace NAV24::OB {

    class Observation {
    public:

        void setWorldObject(const WO::woPtr& pWo) { mpWo = pWo; }
        WO::woPtr getWorldObject() { return mpWo.lock(); }
    protected:
        WO::woPtrW mpWo{};
    };
    typedef std::shared_ptr<Observation> ObsPtr;
    typedef std::weak_ptr<Observation> ObsPtrW;

    typedef std::pair<long, ObsPtr> ObsTimed;

} // NAV24::OB

#endif //NAV24_OBSERVATION_HPP
