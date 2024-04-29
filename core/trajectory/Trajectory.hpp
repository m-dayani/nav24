//
// Created by masoud on 2/11/24.
//

#ifndef NAV24_TRAJECTORY_HPP
#define NAV24_TRAJECTORY_HPP

#include <memory>


namespace NAV24 {

    class Trajectory {
    public:
        explicit Trajectory(const std::string& traj) : mName() {}

    protected:
        std::string mName;
    };
    typedef std::shared_ptr<Trajectory> TrajPtr;

} // NAV24

#endif //NAV24_TRAJECTORY_HPP
