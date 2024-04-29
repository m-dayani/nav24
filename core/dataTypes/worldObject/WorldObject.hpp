//
// Created by masoud on 2/11/24.
//

#ifndef NAV24_WORLDOBJECT_HPP
#define NAV24_WORLDOBJECT_HPP

#include <memory>


namespace NAV24::WO {

    class WorldObject {
    public:
    };
    typedef std::shared_ptr<WorldObject> woPtr;
    typedef std::weak_ptr<WorldObject> woPtrW;

} // NAV24::WO

#endif //NAV24_WORLDOBJECT_HPP
