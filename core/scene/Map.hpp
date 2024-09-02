//
// Created by masoud on 2/11/24.
//

#ifndef NAV24_MAP_HPP
#define NAV24_MAP_HPP

#include <memory>
#include <string>
#include <vector>

#include "WorldObject.hpp"

namespace NAV24 {

    class Map {
    public:
        explicit Map(std::string  mapName);

        void addWorldObject(const WO::WoPtr& pWorldObj);

    protected:
        std::string mName;
        std::vector<WO::WoPtr> mvpWorldObjects;
    };
    typedef std::shared_ptr<Map> MapPtr;

} // NAV24

#endif //NAV24_MAP_HPP
