#pragma once

#define _USE_MATH_DEFINES

#include <math.h>

//---------------------------- Program Constants ------------------------------------------------------------
const double PI = M_PI; // the one and only

const double TOL = 1.0e-12;                // crazy precision!!!
const double POSITION_TOLERANCE = 1.0e-4;  // check if same point
const double ANGLE_TOLERANCE = 1.0e-4;     // check if arm angle

const char *strHack = "2ck3Bkj_XT102u3";  // string flag to fix issue echoing console to file using scanf_s

#define L1 350.0                    // length of the inner arm
#define L2 250.0                    // length of the outer arm
#define ABS_THETA1_DEG_MAX 150.0    // maximum magnitude of shoulder angle in degrees
#define ABS_THETA2_DEG_MAX 170.0    // maximum magnitude of elbow angle in degrees
#define LMAX (L1 + L2)              // max L -> maximum reach of robot
#define LMIN (L1 - L2)              // min L -> minimum reach of robot

const int PRECISION = 4;         // for printing values to console
const int LIMIT_PRECISION = 1;   // for printing limit values to console
const int FIELD_WIDTH = 9;       // for printing values to console

const int TABLE_WIDTH = 90;   // Table width for points data (width includes left/right borders)
const int LEFT_MARGIN = 2;    // number of spaces between left border and the first character printed

// TABLE BORDER SYMBOLS
const wchar_t HL = L'\u2500';  // horizontal border line
const wchar_t VL = L'\u2502';  // vertical border line
const wchar_t TL = L'\u250C';  // top left border symbol
const wchar_t TC = L'\u252C';  // top center border symbol
const wchar_t TR = L'\u2510';  // top right border symbol
const wchar_t CL = L'\u251C';  // left center border symbol
const wchar_t CC = L'\u253C';  // center center border symbol (cross)
const wchar_t CR = L'\u2524';  // right center border symbol
const wchar_t BL = L'\u2514';  // bottom left border symbol
const wchar_t BC = L'\u2534';  // bottom center border symbol
const wchar_t BR = L'\u2518';  // bottom right border symbol

const wchar_t DEGREE_SYMBOL = L'\u00B0';        // the degree symbol
const wchar_t THETA_SYMBOL  = L'\u03B8';        // theta symbol
const wchar_t ERROR_SYMBOL_LEFT = L'\u25BA';    // error symbol (left side)
const wchar_t ERROR_SYMBOL_RIGHT = L'\u25C4';   // error symbol (right side)
const int NUM_ERROR_SYMBOLS = 7;          // number of leading/trailing error symbols to print

// constants to indicate reach errors
const int L_EXCEEDS_MIN = 1 << 0;         // (1) L < LMIN
const int L_EXCEEDS_MAX = 1 << 1;         // (2) L > LMAX
const int THETA1L_EXCEEDS_MAX = 1 << 2;   // (4) |theta1LDeg| > ABS_THETA1_DEG_MAX
const int THETA2L_EXCEEDS_MAX = 1 << 3;   // (8) |theta2LDeg| > ABS_THETA2_DEG_MAX
const int THETA1R_EXCEEDS_MAX = 1 << 4;   // (16) |theta1RDeg| > ABS_THETA1_DEG_MAX
const int THETA2R_EXCEEDS_MAX = 1 << 5;   // (32) |theta2RDeg| > ABS_THETA2_DEG_MAX

enum ARM { LEFT, RIGHT };      // left arm or right arm configuration
enum TRACE_THICKNESS { THICK_TRACE, MEDIUM_TRACE, THIN_TRACE}; // trace thickness (related to motor speed)
enum MOTOR_SPEED { MOTOR_SPEED_LOW, MOTOR_SPEED_MEDIUM, MOTOR_SPEED_HIGH }; // motor speed

enum COMMAND_INDEX  // list of all command indexes
{
   ROTATE_JOINTS, MOTOR_SPEED, CYCLE_PEN_COLORS, PEN_UP, PEN_DOWN, PEN_COLOR, CLEAR_TRACE,
   CLEAR_REMOTE_COMMAND_LOG, CLEAR_POSITION_LOG, MESSAGE, SHUTDOWN_SIMULATION, END, HOME,
   MOVE_TO, LINE, ARC, QUADRATIC_BEZIER, TRIANGLE, RECTANGLE, CUBIC_BEZIER, TRANSLATE, SCALE, ROTATE, RESET_TRANSFORMATION_MATRIX
};

const int BLANK_LINE = -1;          // blank line encountered in input file
const int UNKNOWN_COMMAND = -2;     // unknown command keyword encountered in input file
const int THICKNESS_ERROR = -1;     // unknown thickness parameter
const int NP_ERROR = 0;             // bad NP value

// various static array sizes
enum ARRAY_SIZES { MAX_COMMAND = 256, MAX_TITLE = 86, MAX_BUFF = 1024, MAX_FILENAME = 512, MAX_LINE = 1024 };

//-------------------------------- Structure Definitions ----------------------------------------------------
// structure to map command keyword string to a command index
typedef struct COMMAND
{
   const int index;
   const char *strCommand;
}
COMMAND;

// RGB color
typedef struct RGB
{
   int r, g, b;  // ranges are 0-255
}
RGB;

// SCARA tooltip coordinates
typedef struct TOOL_POSITION
{
   double x, y;
}
TOOL_POSITION, POINT2D;

// SCARA joint angles (degrees)
typedef struct JOINT_ANGLES
{
   double theta1Deg, theta2Deg;
}
JOINT_ANGLES;

// color and thickness of line/curve/arc etc to be drawn
typedef struct SHAPE_ATTRIBUTES
{
   RGB penColor;    // shape color
   int thickness;   // related to shape thickness
}
SHAPE_ATTRIBUTES;

// data for the current state of the robot
typedef struct SCARA_STATE
{
   RGB penColor;
   int penPos;
   int motorSpeed;
   JOINT_ANGLES jointAngles;
}
SCARA_STATE;

// inverse kinematics solution data 
typedef struct INVERSE_SOLUTION
{
   JOINT_ANGLES leftArm, rightArm;       // left and right arm joint angles (in degrees)
   int reachState;                       // compound value for limit checks
   bool bLeftCanReach, bRightCanReach;   // true if robot arm configuration can reach, false if not
}
INVERSE_SOLUTION;

// combination of x,y position and inverse solution values
typedef struct POINT_DATA
{
   POINT2D pt;             // x,y position
   INVERSE_SOLUTION isol;  // inverse solution values (angles, reachability)
}
POINT_DATA;

// ALL info needed to draw a shape with the robot
typedef struct SHAPE_DATA
{
   SHAPE_ATTRIBUTES traceAttributes;            // shape color, thickness
   POINT_DATA *pointsData;                      // data for all points on the shape
   size_t NP;                                   // number of points on the shape
   bool bLeftCanDraw, bRightCanDraw;            // arm drawability for shape
   double leftArmDeltaDeg, rightArmDeltaDeg;    // arm angle deltas (to judge drawing time for each arm)
}
SHAPE_DATA;

// input data needed to draw a line
typedef struct LINE_INPUT_DATA
{
   POINT2D P[2];                       // start and end point coordinates
   size_t NP;                          // number of points on line (including endpoints)
   SHAPE_ATTRIBUTES traceAttributes;   // thickness and color of line
}
LINE_INPUT_DATA;

// input data needed to draw an arc
typedef struct ARC_INPUT_DATA
{
   POINT2D pc;                         // center of arc
   double r;                           // arc radius
   double thetaStartDeg, thetaEndDeg;  // start and end point angles (relative to arc center)
   size_t NP;                          // number of points on arc (including endpoints)
   SHAPE_ATTRIBUTES traceAttributes;   // thickness and color of line
}
ARC_INPUT_DATA;

// input data needed to draw an quadratic bezier curve
typedef struct BEZIER_INPUT_DATA
{
   POINT2D *P;                       // start, mid, end control point coordinates
   size_t NP;                          // number of points on bezier (including endpoints)
   SHAPE_ATTRIBUTES traceAttributes;   // thickness and color of line
}
BEZIER_INPUT_DATA;

typedef struct TRIANGLE_INPUT_DATA
{
   POINT2D P[3];                       // start and end point coordinates
   size_t NP[3];                          // number of points on line (including endpoints)
   size_t totalNP;
   SHAPE_ATTRIBUTES traceAttributes;   // thickness and color of line
}
TRIANGLE_INPUT_DATA;

typedef struct RECTANGLE_INPUT_DATA
{
   POINT2D P[4];                       // start and end point coordinates
   size_t NP[4];                          // number of points on line (including endpoints)
   size_t totalNP;
   SHAPE_ATTRIBUTES traceAttributes;   // thickness and color of line
}
RECTANGLE_INPUT_DATA;

   //---------------------------- Structure Constants ---------------------------------------------------------------
const RGB AQUA = {0,255,255};
const RGB BLACK = {0,0,0};
const RGB GREEN = {0,255,0};
const RGB HOTPINK = {255,20,147};
const RGB NAVY = {0,0,255};
const RGB ORANGE = {255,165,0};
const RGB PURPLE = {155,0,155};
const RGB RED = {255,0,0};
const RGB WHITE = {255,255,255};
const RGB YELLOW = {255,255,0};
const RGB PEN_COLOR_ERROR = {-1,-1,-1};

