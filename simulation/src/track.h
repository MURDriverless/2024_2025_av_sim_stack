#include "cone.h"
#ifndef TRACK_H
#define TRACK_H


#include <iostream>
#include <vector>
#include <string>


//at this stage, we only assume the straight track enviornment given

class Track {
public:
    std::vector<Cone> leftCones;  // yellow-leftside
    std::vector<Cone> rightCones; // blue-rightside

    std::vector<Cone> getAllCones() const;

    void reportTrackStatus();
};
#endif