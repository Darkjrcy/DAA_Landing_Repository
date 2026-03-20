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

// include the header file:
#define _USE_MATH_DEFINES
#include <cmath>
#include "uav_dynamics/uav_dubins_paths.hpp"



// Include the external objects geenrated by matlab to use the UAV dubins connector:
extern void* uavDubinsConnectionObj(double, double, const double*, void*, unsigned int);
extern void autonomousDubinsDistanceCodegen_real64(void*, const double*, unsigned int, const double*, unsigned int, bool, double, double*, double*, double*, double*, double*, double*, double*, double*);
extern void autonomousDubinsSegmentsCodegen_real64(void*, const double*, unsigned int, const double*, unsigned int, bool, double, double*, double*, double*, double*, double*, double*, double*, double*);
extern void uavDubinsDistanceCodegen_real64(void*, const double*, unsigned int, const double*, unsigned int, bool, double, double*, double*, double*, double*, double*, double*, double*, double*);

// Start the connnection using the coder:
namespace coder {
    // Start the UAVDubins connector:
    class uavDubinsConnection{
        public:
            // Define the init function:
            uavDubinsConnection *init(double maxRollAngle, double airSpeed, const double fpaLimits[2]);
            
            // Deifne the connection function between two points:
            void connect(const double startPoses[4], const double goalPoses[4], uavDubinsPathSegment *pathSegObjs) const;

            // Define the characteristic of the UAV used during the UAV segmentation creation:
            double MinTurningRadius;
            double AirSpeed;
            double MaxRollAngle;
            double FlightPathAngleLimit[2];
    };


    // Define the uav namespace used in the MATLAB objects:
    namespace uav{
        namespace internal {
            namespace coder {
                // Create a UAV object from MATLAB:
                class uavDubinsBuildable {
                    public:
                        void init(double airSpeed, double maxRollAngle, const double flightPathAngle[2]);
                        void *UAVDubinsBuildableObj;
                };
            }
        }
    }


    // Cast functions to standarize the type of maneuvers teh UAV should do in order to create the dubins path:
    // In case teh motion ha two characters:
    static char cast(const char t2_f1[2], char t2_f2, char t2_f3, char t2_f4,
                 char t3_f1_data[], int t3_f1_size[2], char &t3_f3,
                 char t3_f4_data[], int t3_f4_size[2]) {
        char t3_f2;
        t3_f1_size[0] = 1; t3_f1_size[1] = 2;
        t3_f1_data[0] = t2_f1[0]; t3_f1_data[1] = t2_f1[1];
        t3_f2 = t2_f2; t3_f3 = t2_f3;
        t3_f4_size[0] = 1; t3_f4_size[1] = 1;
        t3_f4_data[0] = t2_f4;
        return t3_f2;
    }

    // In case the motrion is defined by one single character:
    static char cast(char t0_f1, char t0_f2, char t0_f3, const char t0_f4[2],
                 char t1_f1_data[], int t1_f1_size[2], char &t1_f3,
                 char t1_f4_data[], int t1_f4_size[2]) {
        char t1_f2;
        t1_f1_size[0] = 1; t1_f1_size[1] = 1;
        t1_f1_data[0] = t0_f1;
        t1_f2 = t0_f2; t1_f3 = t0_f3;
        t1_f4_size[0] = 1; t1_f4_size[1] = 2;
        t1_f4_data[0] = t0_f4[0]; t1_f4_data[1] = t0_f4[1];
        return t1_f2;
    }


    // Create teh robotcs functions used during Dubins path generationn:
    namespace robotics{
        namespace internal {
            // Function to normalize angles between 0 to 2pi.
            static double wrapTo2Pi(double theta){
                double thetaWrap = std::fmod(theta, 2.0 * M_PI);
                if (thetaWrap < 0.0) thetaWrap += 2.0 * M_PI;
                return thetaWrap;
            }
        }
    }


    // Function to create the create the connect DUbins object:
    void uavDubinsConnection::connect(const double startPoses[4], const double goalPoses[4], uavDubinsPathSegment *pathSegObjs) const {
        // Constants to define the type of movement:
        // Helix left:
        static const char motionTypes_f7_f1[2]{'H', 'l'};
        // Helix rigth:
        static const char motionTypes_f9_f1[2]{'H', 'r'};

        // Start the buildable object:
        uav::internal::coder::uavDubinsBuildable buildableObj;

        // Strat the variables used in the segmentation of teh path:
        double g[16], ml[16], s[16], b_ml[4], goalPose[4], startPose[4];
        double a, fpa, h, mt, mtr;

        // Start the characters to find the type of movemnt:
        int m_f1_size[2], m_f4_size[2];
        char m_f1_data[2], m_f4_data[2], m_f2, m_f3;

        // Create the buildable dubins object:
        buildableObj.init(AirSpeed, MaxRollAngle, FlightPathAngleLimit);

        // Generate the gioa position and starting position:
        for(int i=0; i<4; i++) {
            startPose[i] = startPoses[i];
            goalPose[i] = goalPoses[i];
        }

        // Make teh MATLAB object to obtain the distance and the segments:
        uavDubinsDistanceCodegen_real64(buildableObj.UAVDubinsBuildableObj, &startPose[0], 1U, &goalPose[0], 1U,
            true, MinTurningRadius, &s[0], &g[0], &fpa, &a, &mtr, &h, &mt, &ml[0]);

        // Define the values for the path chatracteristics:
        m_f1_size[0] = 1; m_f1_size[1] = 1; m_f1_data[0] = 'L';
        m_f2 = 'S'; m_f3 = 'L';
        m_f4_size[0] = 1; m_f4_size[1] = 1; m_f4_data[0] = 'N';

        // State machine logic for path selection from the 27 different cases (left untouched to match .o files)
        int pathCase = static_cast<int>(mt + 1.0);
        switch (pathCase) {
            // Basic 2D dubins cases
            case 1:  m_f1_data[0] = 'L'; m_f3 = 'L'; m_f4_data[0] = 'N'; break;
            case 2:  m_f1_data[0] = 'L'; m_f3 = 'R'; m_f4_data[0] = 'N'; break;
            case 3:  m_f1_data[0] = 'R'; m_f3 = 'L'; m_f4_data[0] = 'N'; break;
            case 4:  m_f1_data[0] = 'R'; m_f3 = 'R'; m_f4_data[0] = 'N'; break;
            // Three turns cases:
            case 5:  m_f1_data[0] = 'R'; m_f2 = 'L'; m_f3 = 'R'; m_f4_data[0] = 'N'; break;
            case 6:  m_f1_data[0] = 'L'; m_f2 = 'R'; m_f3 = 'L'; m_f4_data[0] = 'N'; break;
            // Helix + Turn case:
            case 7:  m_f2 = cast(motionTypes_f7_f1, 'L', 'S', 'L', m_f1_data, m_f1_size, m_f3, m_f4_data, m_f4_size); break;
            case 8:  m_f2 = cast(motionTypes_f7_f1, 'L', 'S', 'R', m_f1_data, m_f1_size, m_f3, m_f4_data, m_f4_size); break;
            case 9:  m_f2 = cast(motionTypes_f9_f1, 'R', 'S', 'L', m_f1_data, m_f1_size, m_f3, m_f4_data, m_f4_size); break;
            case 10: m_f2 = cast(motionTypes_f9_f1, 'R', 'S', 'R', m_f1_data, m_f1_size, m_f3, m_f4_data, m_f4_size); break;
            case 11: m_f2 = cast(motionTypes_f9_f1, 'R', 'L', 'R', m_f1_data, m_f1_size, m_f3, m_f4_data, m_f4_size); break;
            case 12: m_f2 = cast(motionTypes_f7_f1, 'L', 'R', 'L', m_f1_data, m_f1_size, m_f3, m_f4_data, m_f4_size); break;
            // Turn + helix case
            case 13: m_f2 = cast('L', 'S', 'L', motionTypes_f7_f1, m_f1_data, m_f1_size, m_f3, m_f4_data, m_f4_size); break;
            case 14: m_f2 = cast('L', 'S', 'R', motionTypes_f9_f1, m_f1_data, m_f1_size, m_f3, m_f4_data, m_f4_size); break;
            case 15: m_f2 = cast('R', 'S', 'L', motionTypes_f7_f1, m_f1_data, m_f1_size, m_f3, m_f4_data, m_f4_size); break;
            case 16: m_f2 = cast('R', 'S', 'R', motionTypes_f9_f1, m_f1_data, m_f1_size, m_f3, m_f4_data, m_f4_size); break;
            case 17: m_f2 = cast('R', 'L', 'R', motionTypes_f9_f1, m_f1_data, m_f1_size, m_f3, m_f4_data, m_f4_size); break;
            case 18: m_f2 = cast('L', 'R', 'L', motionTypes_f7_f1, m_f1_data, m_f1_size, m_f3, m_f4_data, m_f4_size); break;
            // COmplex multi turn case:
            case 19: m_f1_data[0] = 'L'; m_f2 = 'R'; m_f3 = 'S'; m_f4_data[0] = 'L'; break;
            case 20: m_f1_data[0] = 'L'; m_f2 = 'R'; m_f3 = 'S'; m_f4_data[0] = 'R'; break;
            case 21: m_f1_data[0] = 'L'; m_f2 = 'R'; m_f3 = 'L'; m_f4_data[0] = 'R'; break;
            case 22: m_f1_data[0] = 'R'; m_f2 = 'L'; m_f3 = 'S'; m_f4_data[0] = 'R'; break;
            case 23: m_f1_data[0] = 'R'; m_f2 = 'L'; m_f3 = 'R'; m_f4_data[0] = 'L'; break;
            case 24: m_f1_data[0] = 'R'; m_f2 = 'L'; m_f3 = 'S'; m_f4_data[0] = 'L'; break;
            case 25: m_f1_data[0] = 'L'; m_f2 = 'S'; m_f3 = 'R'; m_f4_data[0] = 'L'; break;
            case 26: m_f1_data[0] = 'R'; m_f2 = 'S'; m_f3 = 'R'; m_f4_data[0] = 'L'; break;
            case 27: m_f1_data[0] = 'L'; m_f2 = 'S'; m_f3 = 'L'; m_f4_data[0] = 'R'; break;
            case 28: m_f1_data[0] = 'R'; m_f2 = 'S'; m_f3 = 'L'; m_f4_data[0] = 'R'; break;
            // In case it doesn't find a defined case:
            default: break;
        }

        // REdefine the goal and strat positions with the new trajectories obtained:
        startPose[0] = s[0]; goalPose[0] = g[0]; b_ml[0] = ml[0];
        startPose[1] = s[1]; goalPose[1] = g[1]; b_ml[1] = ml[1];
        startPose[2] = s[2]; goalPose[2] = g[2]; b_ml[2] = ml[2];
        startPose[3] = s[3]; goalPose[3] = g[3]; b_ml[3] = ml[3];

        // Initialize the result object with the calculated maneuvers
        pathSegObjs->init(startPose, goalPose, fpa, a, mtr, h, m_f1_data, m_f1_size, m_f2, m_f3, m_f4_data, m_f4_size, b_ml);
    }


    // Calcualte the moveemtn limits of the UAV:
    uavDubinsConnection *uavDubinsConnection::init(double maxRollAngle, double airSpeed, const double fpaLimits[2]) {
        // Start the connection:
        uavDubinsConnection *this_ = this;
        // Add hte airplane characteristics:
        this_->AirSpeed = airSpeed;
        this_->MaxRollAngle = maxRollAngle;
        this_->FlightPathAngleLimit[0] = fpaLimits[0];
        this_->FlightPathAngleLimit[1] = fpaLimits[1];

        // Calcualte the min radius:
        double x = std::tan(this->MaxRollAngle);
        this->MinTurningRadius = (this->AirSpeed * this->AirSpeed) / (9.8 * x);

        // return teh UavCOnnection witht eh UAV apramters:
        return this_;
    }


    // Fucntion to start the segmentation of the dubins apth:
    void uavDubinsPathSegment::init(const double startPose[4], const double goalPose[4], double flightPathAngle,
        double airSpeed, double minTurningRadius, double helixRadius, const char motionType1Data[], const int motionType1Size[2],
        char motionType2, char motionType3, const char motionType4Data[], const int motionType4Size[2], const double motionLengths[4]){
        // NOrmlaize look up table:
        static const char cv[128] {
            '\x00', '\x01', '\x02', '\x03', '\x04', '\x05', '\x06', '\a',   '\b', '\t',   '\n',   '\v',   '\f',   '\r',   '\x0e', '\x0f', 
            '\x10', '\x11', '\x12', '\x13', '\x14', '\x15', '\x16', '\x17', '\x18', '\x19', '\x1a', '\x1b', '\x1c', '\x1d', '\x1e', '\x1f', 
            ' ',    '!',    '\"',   '#',    '$',    '%',    '&',    '\'',   '(',    ')',    '*',    '+',    ',',    '-',    '.',    '/',    
            '0',    '1',    '2',    '3',    '4',    '5',    '6',    '7',    '8',    '9',    ':',    ';',    '<',    '=',    '>',    '?',    
            '@',    'A',    'B',    'C',    'D',    'E',    'F',    'G',    'H',    'I',    'J',    'K',    'L',    'M',    'N',    'O',    
            'P',    'Q',    'R',    'S',    'T',    'U',    'V',    'W',    'X',    'Y',    'Z',    '[',    '\\',   ']',    '^',    '_',    
            '`',    'A',    'B',    'C',    'D',    'E',    'F',    'G',    'H',    'I',    'J',    'K',    'L',    'M',    'N',    'O',    
            'P',    'Q',    'R',    'S',    'T',    'U',    'V',    'W',    'X',    'Y',    'Z',    '{',    '|',    '}',    '~',    '\x7f'
        };
        
        // Get the minimum tturn radius adn deifne teh start and goal positions:
        MinTurningRadius = minTurningRadius;
        AirSpeed = airSpeed;
        HelixRadius = helixRadius;
        FlightPathAngle = flightPathAngle;
        for(int i=0; i<3; i++) {
            StartPose[i] = startPose[i];
            GoalPose[i] = goalPose[i];
        }
        // Wrap the heading to be between 0 and 2pi
        StartPose[3] = robotics::internal::wrapTo2Pi(startPose[3]);
        GoalPose[3] = robotics::internal::wrapTo2Pi(goalPose[3]);

        // define motion lenghts:
        for(int i=0; i<4; i++) MotionLengths[i] = motionLengths[i];

        // Map motion types cleanly
        MotionTypes[0].f1.size[0] = 1; 
        MotionTypes[0].f1.size[1] = motionType1Size[1];
        for (int k = 0; k < motionType1Size[1]; k++) {
            MotionTypes[0].f1.data[k] = cv[static_cast<int>(motionType1Data[k])]; 
        }

        MotionTypes[3].f1.size[0] = 1; 
        MotionTypes[3].f1.size[1] = motionType4Size[1];
        for (int k = 0; k < motionType4Size[1]; k++) {
            MotionTypes[3].f1.data[k] = cv[static_cast<int>(motionType4Data[k])]; 
        }

        // Single character segments also use the table
        MotionTypes[1].f1.size[0] = 1; MotionTypes[1].f1.size[1] = 1; 
        MotionTypes[1].f1.data[0] = cv[static_cast<int>(motionType2)];

        MotionTypes[2].f1.size[0] = 1; MotionTypes[2].f1.size[1] = 1; 
        MotionTypes[2].f1.data[0] = cv[static_cast<int>(motionType3)];

        Length = motionLengths[0] + motionLengths[1] + motionLengths[2] + motionLengths[3];
    }


    // Function to start the building table
    void uav::internal::coder::uavDubinsBuildable::init(double airSpeed, double maxRollAngle, const double flightPathAngle[2]) {
        double b_flightPathAngle[2] = {flightPathAngle[0], flightPathAngle[1]};
        UAVDubinsBuildableObj = uavDubinsConnectionObj(airSpeed, maxRollAngle, &b_flightPathAngle[0], nullptr, 0U);
    }

    // Constructore of the UavPthSegment object:
    uavDubinsPathSegment::uavDubinsPathSegment() = default;
    // Destrrctore:
    uavDubinsPathSegment::~uavDubinsPathSegment() = default;

    // Function to get the segment lenght adn the pathsegments that are going to be used for the FMT guidance system:
    void uav_dubins_paths(const double state1[4], const double state2[4], double roll_max, double Vair, const double fpa_lim[2], double lengths[4], coder::uavDubinsPathSegment *pathSeg) {
        // Start the connector
        coder::uavDubinsConnection conn;
        // Set up the cosntant of the Uav
        conn.init(roll_max, Vair, fpa_lim);
        // cONNECT THE SEGMENTS
        conn.connect(state1, state2, pathSeg);
        // sAVE THE PATH SEGMENTS WITH LENGHT ETC...:
        for(int i=0; i<4; i++) lengths[i] = pathSeg->MotionLengths[i];
    }

} // namespace coder

