//
// Created by masoud on 2/11/24.
//

#ifndef NAV24_BE_CALIBCAMCV_HPP
#define NAV24_BE_CALIBCAMCV_HPP

#include "BackEnd.hpp"

namespace NAV24::BE {

    class CalibCamCv : public BackEnd {
    public:
        void solve(const ProblemPtr &problem) override;
    };

} // NAV24::BE

#endif //NAV24_BE_CALIBCAMCV_HPP
