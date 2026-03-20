/**
 * ============================================================================
 * COPYRIGHT AND ATTRIBUTION NOTICE
 * ============================================================================
 * This module is part of a real-time Flight Management Toolkit (FMT).
 * * It utilizes proprietary algorithms and data structures derived from the 
 * MathWorks UAV Toolbox (specifically 'uavDubinsConnection'). The core 
 * kinematic calculations for UAV Dubins paths are executed by pre-compiled 
 * object files (.o) generated via MATLAB Coder.
 * * All rights to the underlying mathematical logic and kinematic libraries 
 * remain the property of MathWorks.
 * ============================================================================
 */

#ifndef UAV_DUBINS_HPP
#define UAV_DUBINS_HPP

#include <cmath>
#include <cstddef>
#include <cstdlib>


// Code created by coder:
namespace coder {

    // Structure to define the motion time and the pathSeg:
    struct cell_wrap_0 {
        struct {
            char data[2];
            int size[2];
        } f1;
    };

    // Define the class of the UAVPathSegment funciton:
    class uavDubinsPathSegment{
        public:
            // Initilization function:
            void init(const double startPose[4], const double goalPose[4], double flightPathAngle, 
                double airSpeed, double minTurningRadius, double helixRadius, const char motionType1Data[],
                const int motionType1Size[2], char motionType2, char motionType3, const char motionType4Data[], 
                const int motionType4Size[2], const double motionLengths[4]);

            // Constructore:
            uavDubinsPathSegment();
            // Destructore:
            ~uavDubinsPathSegment();
            
            // Variables used during the code:
            double StartPose[4];
            double GoalPose[4];
            double FlightPathAngle;
            double AirSpeed;
            double MinTurningRadius;
            double HelixRadius;
            cell_wrap_0 MotionTypes[4];
            double MotionLengths[4];
            double Length;
    };

    // Function used for detemining the path segmen, and the length of the Dubins COnnection:
    void uav_dubins_paths(const double state1[4], const double state2[4], double roll_max, double Vair, const double fpa_lim[2],
        double lengths[4], coder::uavDubinsPathSegment *pathSeg);
};



// Class used for the FMT specifically and use the dubins connector as the object
class FMTDubinsConnector {
    private:
        // UAV characteristics used for the ceration of the dubins connector:
        double roll_max;
        double Vair;
        double fpa_lim[2];
    
    public:
        // Initilization to start the connector with the flight parameters:
        FMTDubinsConnector(double maxRollAngle, double airSpeed, double fpaMin, double fpaMax) {
            roll_max = maxRollAngle;
            Vair = airSpeed;
            fpa_lim[0] = fpaMin;
            fpa_lim[1] = fpaMax;
        }

        // Call them to get your segment and the geometry:
        coder::uavDubinsPathSegment computePath(const double state1[4], const double state2[4], double outLengths[4]){
            // Create teh segment:
            coder::uavDubinsPathSegment pathResult;

            // Call the fucntion to obtiant he path:
            coder::uav_dubins_paths(state1, state2, roll_max, Vair, fpa_lim, outLengths, &pathResult);

            // Retrun the path results: 
            return pathResult;
        }
        
};

#endif