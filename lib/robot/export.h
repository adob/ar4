#pragma once

#include "env.h"

#include <highfive/highfive.hpp>
#define H5_USE_OPENCV 1
#include <highfive/H5Easy.hpp>

namespace robot {
    using namespace lib;
    
    void export_episode(str path, std::list<Observation> const& obs, error&);
}