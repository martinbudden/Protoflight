#pragma once

#include "FlightController.h"
#include <PIDF.h>


constexpr FlightController::pidf_array_t gDefaultPIDs = {
    {
        { 0.65F,    0.0F,   0.010F, 0.00F, 0.00F }, // roll rate
        { 0.95F,    0.0F,   0.025F, 0.00F, 0.00F }, // pitch rate
        { 0.50F,    0.0F,   0.010F, 0.00F, 0.00F }, // yaw rate
        { 5.00F,    0.0F,   0.040F, 0.00F, 0.00F }, // roll angle
        { 5.00F,    0.0F,   0.040F, 0.00F, 0.00F }, // pitch angle
        { 1.00F,    0.0F,   0.010F, 0.00F, 0.00F }, // roll sin angle
        { 1.00F,    0.0F,   0.010F, 0.00F, 0.00F }  // pitch sin angle
    }
};
constexpr FlightController::pidf_array_t gScaleFactors = {
    {
        { 0.01F,    0.01F,   0.001F, 0.1F, 0.1F }, // roll rate
        { 0.01F,    0.01F,   0.001F, 0.1F, 0.1F }, // pitch rate
        { 0.01F,    0.01F,   0.001F, 0.1F, 0.1F }, // yaw rate
        { 0.01F,    0.01F,   0.001F, 0.1F, 0.1F }, // roll angle
        { 0.01F,    0.01F,   0.001F, 0.1F, 0.1F }, // pitch angle
        { 0.01F,    0.01F,   0.001F, 0.1F, 0.1F }, // roll sin angle
        { 0.01F,    0.01F,   0.001F, 0.1F, 0.1F }  // pitch sin angle
    }
};
