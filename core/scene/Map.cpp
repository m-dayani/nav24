//
// Created by masoud on 2/11/24.
//

#include "Map.hpp"

#include <utility>

namespace NAV24 {
    Map::Map(std::string mapName) : mName(std::move(mapName)), mvpWorldObjects() {}

    void Map::addWorldObject(const WO::WoPtr &pWorldObj) {
        mvpWorldObjects.push_back(pWorldObj);
    }
} // NAV24