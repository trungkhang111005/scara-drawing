/**********************************************************************************************************************
Course: ROBT1270 - C Programming

Program: Lab5: SCARA robot intermediate control

Details: Control the SCARA robot using various commands.  Draw shapes with various colors and line thicknesses.
         A shape will be drawn its entirety using either the left or right arm without switching arms at any point.
         If both arms are able to draw the shape, then the arm which draws the shape faster will be chosen.

         Components of the C program include variables, formatted console output, branches, loops, functions,
         bitwise operations, structures, arrays and file I/O.

Author(s): Trung NGuyen

Declaration: I, TrungA, declare that the following program was written by me/us.

Date Created: March 28 2024

**********************************************************************************************************************/
#undef __cplusplus

#define TEST

//-------------------------- Standard library prototypes --------------------------------------------------------------
#include <stdlib.h>     // standard functions and constant
#include <stdio.h>      // i/o functions
#include <math.h>       // math functions
#include <string.h>     // string functions
#include <ctype.h>      // character functions
#include <stdbool.h>    // bool definitions
#include <limits.h>     // various constants like MAX_PATH
#include <stdarg.h>     // for variable argument functions
#include <locale.h>     // for printing wide characters
#include <float.h>      // float and double limits
#include "constants.h"  // program constants and structure definitions

#define printf dsprintf // make all printf calls use dsprintf

//-------------------------------- Robot Definitions and Function Prototypes ------------------------------------------
int initializeRobot();              // creates a TCP/IP connection between this program and the robot.
int sendRobotCommand(const char *); // sends a command remotely to the SCARA robot
void closeRobot();                  // closes the TCP/IP connection to the robot
SCARA_STATE getRobotState();        // gets the current state of the SCARA robot

//----------------------------- Function Prototypes -------------------------------------------------------------------
void waitForEnterKey();          // waits for the Enter key to be pressed
void endProgram();               // ends the program from anywhere in the code
double degToRad(double);         // returns angle in radians from input angle in degrees
double radToDeg(double);         // returns angle in degrees from input angle in radians
double mapAngle(double angRad);  // make sure inverseKinematic angled are mapped in range robot understands
void printHLine(int len);        // prints a horizontal line
void printRepeatedChar(wchar_t ch, int numRepeats);               // prints an ascii character repeatedly
void printTableHeader(int tableWidth, const char *strTableTitle); // prints table header with centered title
void printTableHBorder(wchar_t chL, wchar_t chR, int tableWidth); // prints table horizontal border 
int dsprintf(char *fmt, ...);                                     // mimics printf but also prints to a log file

bool areSameAngles(JOINT_ANGLES currentJA, JOINT_ANGLES JA);      // checks if two sets of joint angles are the same
bool areSameColor(RGB color1, RGB color2);                         // checks if two RGB color values are the same
void getInputString(char *strInput);                              // gets a string from user and reflects to log file

// prints joint angle and reachability data into table for a given [x,y] coordinate.  
void printPointsData(const SHAPE_DATA *pSD, const char *strTitle);

void processFileCommands(); // reads commands from a file and executes them using the robot

// get command index matching command string keyword
int getCommandIndex(const char *strLine, const COMMAND *command, size_t NC);

void processCommand(int commandIndex, char *strCommand, int nLine, double TM[3][3]);     // send commands to robot
void makeStringUpperCase(char *str);                                    // makes a string upper case

// converts a string token into a double with custom error messages for NULL token and garbage
double tokenToDouble(char *tok, const char *msg1, const char *msg2);
void rotateJoints(JOINT_ANGLES ja);  // send command to robot to rotate the joints based on joint angles
bool rotateJointsFromCommandString(char *strCommand, int nLine); // send command to robot to rotate the joints based on file command
size_t tokenToNP(char *tok, int nLine);  // converts token to NP value
bool getPenColor(const char *strColor, RGB *pColor);  // gets a pen RGB color from list of valid pen colors


//------------ YOUR FUNCTION PROTOTYPES
bool setPenColorFromCommandString(char *strCommand, int nLine); // send command to robot to set pen color based on file command
void setPenColor(RGB penColor);
void setMotorSpeed(int motorSpeed);
bool getMotorSpeed(const char *strSpeed, int *motorSpeed, int nLine);
int tokenToInt(char *tok, const char *msg1, const char *msg2); // converts tokent to int
bool setCyclePenColorFromCommandString(char *strCommand, int nLine);
bool moveToFromCommandString(char *strCommand, int nLine);
bool setMotorSpeedFromCommandString(char *strCommand, int nLine);
bool drawShape(int shapeType, char *strCommand, int nLine, double TM[3][3]);
void getLineInputData(LINE_INPUT_DATA *lineData, char *strCommand, int nLine);
void getArcInputData(ARC_INPUT_DATA *arcDatachar, char *strCommand, int nLine);
void getQuadraticBezierData(BEZIER_INPUT_DATA *quadData, char *strCommand, int nLine);
void getTriangleInputData(TRIANGLE_INPUT_DATA *triangleData, char *strCommand, int nLine);
void getRectangleInputData(RECTANGLE_INPUT_DATA *rectangleData, char *strCommand, int nLine);
void getCubicBezierInputData(BEZIER_INPUT_DATA *cubicData, char *strCommand, int nLine);

void getShapePointsData(int shapeType, const void *pShapeInputData, SHAPE_DATA *shapeData, double TM[3][3]);
void drawSegment(SHAPE_ATTRIBUTES attributes, JOINT_ANGLES ja, size_t i);
void inverseKinematics(POINT2D P, INVERSE_SOLUTION *solution);
void getArmAngleDeltas(SHAPE_DATA *shapeData);
void setPenPos(int penPos);
void resetTransformMatrix(double TM[][3]);                     // resets the transform matrix to the identity matrix
void transformMatrixMultiply(double TM[][3], double M[][3]);   // premultiplies the transform matrix TM by matrix M
POINT2D transform(double TM[][3], POINT2D pt);     // tranform tool position coordinates

bool addScaling(char *strCommand, int nLine, double TM[3][3]);
bool addRotation(char *strCommand, int nLine, double TM[3][3]);
bool addTranslation(char *strCommand, int nLine, double TM[3][3]);
//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  C program to draw shapes with the SCARA robot using commands from a file
// ARGUMENTS:    none
// RETURN VALUE: an int that tells the O/S how the program ended.  0 = EXIT_SUCCESS = normal termination
int main()
{
   double TM[3][3] = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
   SCARA_STATE currentState = getRobotState();

   // set up code pages for mirroring screen to log file
   system("chcp 65001");
   setlocale(LC_ALL, ".UTF-8");

   if(!initializeRobot()) exit(0);  // connect to the robot
   printf("current color :  %d %d %d ", currentState.penColor.r, currentState.penColor.g, currentState.penColor.b);
   processFileCommands();  // executes all commands stored in a file

   printf("\nPress ENTER to end program..."); // don't use endProgram because don't want error beeps
   waitForEnterKey();
   return EXIT_SUCCESS;
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  processes robot commands stored in a file and uses them to control the SCARA robot
// ARGUMENTS:    none
// RETURN VALUE: none
void processFileCommands()
{
   char strFileName[MAX_FILENAME];              // stores input file name
   char strCommandLine[MAX_LINE];               // stores one line out of input file
   char strCommandKeyword[MAX_LINE];            // for getting the command keyword from strCommandLine
   FILE *fi = NULL;                             // input file handle
   errno_t err = -1;                            // stores fopen_s error value
   int numChars = -1;                           // used to draw dividing line
   int nLine = -1;                              // file line number
   int commandIndex = UNKNOWN_COMMAND;          // command index
   const char *strEXT = ".in";                  // input file extension
   char *tok = NULL, *nextTok = NULL;           // for strtok_s
   size_t NC;                                   // number of file command keywords
   double TM[3][3] = {{1.0,0.0,0.0},{0.0,1.0,0.0},{0.0,0.0,1.0}};
   // array of command keyword string to command index associations
   const COMMAND command[] = {{ROTATE_JOINTS, "ROTATE_JOINTS"}, {MOTOR_SPEED, "MOTOR_SPEED"},
                               {PEN_UP, "PEN_UP"}, {PEN_DOWN, "PEN_DOWN"}, {CYCLE_PEN_COLORS, "CYCLE_PEN_COLORS"},
                               {PEN_COLOR, "PEN_COLOR"}, {CLEAR_TRACE, "CLEAR_TRACE"},
                               {CLEAR_REMOTE_COMMAND_LOG, "CLEAR_REMOTE_COMMAND_LOG"},
                               {CLEAR_POSITION_LOG, "CLEAR_POSITION_LOG"}, {MESSAGE, "MESSAGE"},
                               {SHUTDOWN_SIMULATION, "SHUTDOWN_SIMULATION"}, {END, "END"}, {HOME, "HOME"},
                               {LINE, "LINE"}, {ARC, "ARC"}, {QUADRATIC_BEZIER, "QUADRATIC_BEZIER"},
                               {MOVE_TO, "MOVE_TO"}, {TRIANGLE, "TRIANGLE"}, {RECTANGLE, "RECTANGLE"},
                               {CUBIC_BEZIER, "CUBIC_BEZIER"}, {TRANSLATE, "TRANSLATE"}, {SCALE,"SCALE"}, {ROTATE, "ROTATE"}, {RESET_TRANSFORMATION_MATRIX, "RESET_TRANSFORMATION_MATRIX"}
   };
   NC = sizeof(command) / sizeof(command[0]);

   // open the input file
   while(true)
   {
#ifdef TEST
      strcpy_s(strFileName, MAX_FILENAME, "test.in");
#else
      printf("Please enter the name of the commands file: ");
      getInputString(strFileName);
#endif
      if(strstr(strFileName, strEXT) == NULL) strcat_s(strFileName, MAX_FILENAME, strEXT); // add extension if not found

      err = fopen_s(&fi, strFileName, "r");  // try to open

      if(err == 0 && fi != NULL) break;      // opened ok

      // process errors
      printf("Failed to open %s!\nError code = %d", strFileName, err);
      if(err == ENOENT)
         printf(" (File not found!  Check name/path)\n");
      else if(err == EACCES)
         printf(" (Permission Denied! Is the file opened in another program?)\n");
      else
         printf("\n");

#ifdef TEST
      if(err != 0 || fi == NULL)
      {
         printf("Press ENTER to end program...");
         waitForEnterKey();
         exit(0);
      }
#endif
   }
   numChars = printf("Processing %s\n", strFileName);  // echo the filename
   printHLine(numChars - 1); // underline

   // get each line from the input file and process the command
   nLine = 0;
   while(fgets(strCommandLine, MAX_LINE, fi) != NULL)
   {
      // remove '\n'
      if(strCommandLine[strlen(strCommandLine) - 1] == '\n') strCommandLine[strlen(strCommandLine) - 1] = '\0';

      // copy strCommandLine and extract the keyword
      strcpy_s(strCommandKeyword, MAX_LINE, strCommandLine);         // make a copy
      tok = strtok_s(strCommandKeyword, " \t,", &nextTok);           // extract the keyword
      if(tok != NULL) strcpy_s(strCommandKeyword, MAX_LINE, tok);    // overwrite with just keyword

      //--- get the command index and process it
      makeStringUpperCase(strCommandKeyword);  // make string all upper case (to match command keywords)
      commandIndex = getCommandIndex(strCommandKeyword, command, NC); // get the command index from the first token

      nLine++;
      if(commandIndex != BLANK_LINE) // process the command
      {
         printf("Line %02d: %s\n", nLine, strCommandLine);  // echo the line
         if(commandIndex != UNKNOWN_COMMAND)  // in the list
         {
            printf("Command found: \"%s\".  Processing...\n", strtok_s(strCommandKeyword, " \t,", &nextTok)); // echo
            processCommand(commandIndex, strCommandLine, nLine, TM); // process command
         }
         else // process the command
            printf("Unknown Command!\n\n");
      }
      else
         printf("Line %02d: BLANK LINE!\n\n", nLine);  // indicate blank line

   }

   fclose(fi); // close the file
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Gets user input as a string
// ARGUMENTS:    strInput:  string used to store the input
// RETURN VALUE: none
void getInputString(char *strInput)
{
   static char buff[MAX_BUFF]; // temp char array

   fgets(strInput, MAX_BUFF, stdin);      // get all user input character as a string (including the '\n'!)
   strcpy_s(buff, MAX_BUFF, strInput);    // copy the input string
   strcat_s(buff, MAX_BUFF, strHack);     // append the hack string onto the end of the user input string
   printf(buff);                          // echo the user input to the file (hack prevents console echo)
   strInput[strlen(strInput) - 1] = '\0'; // remove the '\n' character
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Gets the index of the inputted command keyword string
// ARGUMENTS:    strCommandKeyword:    The command keyword string
//               command:              array of valid command keywords
//               NC:                   number of valid command keywords
// RETURN VALUE: the command keyword index or BLANK_LINE if the line is blank or
//               UNKNOWN_COMMAND if the command keyword doesn't exist
int getCommandIndex(const char *strCommandKeyword, const COMMAND *command, size_t NC)
{
   int ic;   // index to cycle through the list of known commands

   if(strCommandKeyword[0] == '\0') return BLANK_LINE;

   for(ic = 0; ic < NC; ic++)
   {
      if(strcmp(strCommandKeyword, command[ic].strCommand) == 0) // exact check
      {
         return command[ic].index;
      }
   }
   return UNKNOWN_COMMAND;
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  processes a command referenced by the commandIndex.  Parses the command string from the file and 
//               packages up the command to be sent to the robot if no errors found.  
// ARGUMENTS:    commandIndex:  index of the command keyword string
//               strCommand: command line from the file in the form of a string
// RETURN VALUE: none
void processCommand(int commandIndex, char *strCommand, int nLine, double TM[3][3])
{
   bool bTranformCommand = false;
   bool bSuccess = true;           // if certain commands are processed or not
   char strMessage[MAX_COMMAND];   // for sending a message to the simulator remote command log

   switch(commandIndex)
   {
      case MESSAGE: // echo the command string to the robot
         sprintf_s(strMessage, MAX_COMMAND, "%s\n", strCommand);
         sendRobotCommand(strMessage);
         break;
      case CLEAR_TRACE: // clear the traces on the robot
         sendRobotCommand("CLEAR_TRACE\n");
         break;
      case CLEAR_REMOTE_COMMAND_LOG: // clear remote command log
         sendRobotCommand("CLEAR_REMOTE_COMMAND_LOG\n");
         break;
      case CLEAR_POSITION_LOG: // clear the position log
         sendRobotCommand("CLEAR_POSITION_LOG\n");
         break;
      case SHUTDOWN_SIMULATION: // shutdown the robot (closes it)
         sendRobotCommand("SHUTDOWN_SIMULATION\n");
         break;
      case END: // end the connection to the robot
         sendRobotCommand("END\n");
         break;
      case HOME: // send the robot home (note: robot automatically lifts pen it down)
         sendRobotCommand("HOME\n");
         break;
      case PEN_UP: // set the robot pen position to up
         sendRobotCommand("PEN_UP\n");
         break;
      case PEN_DOWN: // set the robot pen position to down
         sendRobotCommand("PEN_DOWN\n");
         break;
      case PEN_COLOR: // set the robot pen color
         bSuccess = setPenColorFromCommandString(strCommand, nLine);
         break;
      case CYCLE_PEN_COLORS:  // set the robot CYCLE_PEN_COLOR mode (on or off)
         setCyclePenColorFromCommandString(strCommand, nLine);
         break;
      case ROTATE_JOINTS:  // rotate the robot joints to specified angles
         bSuccess = rotateJointsFromCommandString(strCommand, nLine);
         break;
      case MOVE_TO: // move the robot to a specified x,y position
         bSuccess = moveToFromCommandString(strCommand, nLine);
         break;
      case MOTOR_SPEED: // set the robot motor speed
         bSuccess = setMotorSpeedFromCommandString(strCommand, nLine);
         break;
      case LINE: // draw a specified line
         bSuccess = drawShape(LINE, strCommand, nLine, TM);
         break;
      case ARC: // draw a specified arc
         bSuccess = drawShape(ARC, strCommand, nLine, TM);
         break;
      case QUADRATIC_BEZIER: // draw a specified quadratic bezier curve
         bSuccess = drawShape(QUADRATIC_BEZIER, strCommand, nLine, TM);
         break;
      case TRIANGLE: // draw a specified quadratic bezier curve
      case RECTANGLE: // draw a specified quadratic bezier curve
      case CUBIC_BEZIER: // draw a specified quadratic bezier curve
         bSuccess = drawShape(commandIndex, strCommand, nLine, TM);
         break;
      case ROTATE:
         bSuccess = addRotation(strCommand, nLine, TM);
         bTranformCommand = bSuccess;
         break;
      case TRANSLATE:
         bSuccess = addTranslation(strCommand, nLine, TM);
         bTranformCommand = bSuccess;
         break;
      case SCALE:
         bSuccess = addScaling(strCommand, nLine, TM);
         bTranformCommand = bSuccess;
         break;
      case RESET_TRANSFORMATION_MATRIX:
         resetTransformMatrix(TM);
         bTranformCommand = true;
         break;
      default:  // command keyword not found
         printf("unknown command!\n"); 
   }

   // certain commands may not be successful due to missing/errant data.  If so, shut down the program.
   if(bTranformCommand)
      dsprintf("Transform matrix modified\n\n");
   else if(bSuccess)
      dsprintf("Command sent to robot!\n\n");
   else
      endProgram(); // includes "Command _NOT_ sent" message
}


bool drawShape(int shapeType, char *strCommand, int nLine, double TM[3][3])
{
   char strTitle[MAX_TITLE]; // title for points data table title
   size_t iPoint = 0;
   LINE_INPUT_DATA lineInputData = {0}; // inputs for line geometry
   ARC_INPUT_DATA arcInputData = {0}; // inputs for arc geometry
   BEZIER_INPUT_DATA quadraticBezierInputData = {0}; // inputs for quad Bezier geometry
   TRIANGLE_INPUT_DATA triangleInputData = {0};
   RECTANGLE_INPUT_DATA rectangleInputData = {0};
   BEZIER_INPUT_DATA cubicBezierInputData = {0};
   SHAPE_DATA shapeData = {0}; // store all data needed to draw a shape with optimal a

   switch(shapeType)
   {
      case LINE:
         getLineInputData(&lineInputData, strCommand, nLine);
         shapeData.pointsData = calloc(lineInputData.NP, sizeof(POINT_DATA));
         if(shapeData.pointsData == NULL)
         {
            fprintf(stderr, "Failed to allocate memory for points data.\n");
            return false; // or handle error appropriately
         }
         shapeData.NP = lineInputData.NP;
         getShapePointsData(LINE, &lineInputData, &shapeData, TM);
         shapeData.traceAttributes.penColor = lineInputData.traceAttributes.penColor;
         shapeData.traceAttributes.thickness = lineInputData.traceAttributes.thickness;
         snprintf(strTitle, MAX_TITLE, "Line: P0 = [%lf, %lf], P1 = [%lf, %lf], NP = %zu",
            lineInputData.P[0].x, lineInputData.P[0].y, lineInputData.P[1].x, lineInputData.P[1].y, shapeData.NP);
         break;

      case ARC:
         getArcInputData(&arcInputData, strCommand, nLine);
         shapeData.pointsData = calloc(arcInputData.NP, sizeof(POINT_DATA));
         if(shapeData.pointsData == NULL)
         {
            fprintf(stderr, "Failed to allocate memory for points data.\n");
            return false; // or handle error appropriately
         }
         shapeData.NP = arcInputData.NP;
         shapeData.traceAttributes.penColor = arcInputData.traceAttributes.penColor;
         shapeData.traceAttributes.thickness = arcInputData.traceAttributes.thickness;
         getShapePointsData(ARC, &arcInputData, &shapeData, TM);
         snprintf(strTitle, MAX_TITLE, "ARC: P0 = [%.lf, %.lf], r = %.lf, thetaStart = %.lf, thetaEnd = %.lf, NP = %zu",
            arcInputData.pc.x, arcInputData.pc.y, arcInputData.r, arcInputData.thetaStartDeg, arcInputData.thetaEndDeg, shapeData.NP);
         break;

      case QUADRATIC_BEZIER:
         quadraticBezierInputData.P = calloc(3, sizeof(POINT2D));
         getQuadraticBezierData(&quadraticBezierInputData, strCommand, nLine);

         shapeData.pointsData = calloc(quadraticBezierInputData.NP, sizeof(POINT_DATA));
         if(shapeData.pointsData == NULL)
         {
            fprintf(stderr, "Failed to allocate memory for points data.\n");
            return false; // or handle error appropriately
         }
         shapeData.NP = quadraticBezierInputData.NP;
         shapeData.traceAttributes.penColor = quadraticBezierInputData.traceAttributes.penColor;
         shapeData.traceAttributes.thickness = quadraticBezierInputData.traceAttributes.thickness;
         getShapePointsData(QUADRATIC_BEZIER, &quadraticBezierInputData, &shapeData, TM);
         snprintf(strTitle, MAX_TITLE, "Quad: P0 = [%.lf, %.lf], P1 = [%.lf, %.lf], P2 = [%.lf, %.lf], NP = %.zu",
            quadraticBezierInputData.P[0].x, quadraticBezierInputData.P[0].y, quadraticBezierInputData.P[1].x, quadraticBezierInputData.P[1].y,
            quadraticBezierInputData.P[2].x, quadraticBezierInputData.P[2].y, quadraticBezierInputData.NP);
         break;

      case  TRIANGLE:
            getTriangleInputData(&triangleInputData, strCommand, nLine);
       shapeData.NP = triangleInputData.totalNP;
            shapeData.pointsData = calloc(triangleInputData.totalNP, sizeof(POINT_DATA));
            if(shapeData.pointsData == NULL)
            {
               fprintf(stderr, "Failed to allocate memory for points data.\n");
               return false; // or handle error appropriately
            }
            getShapePointsData(TRIANGLE, &triangleInputData, &shapeData, TM);
            
        
            shapeData.traceAttributes.penColor = triangleInputData.traceAttributes.penColor;
            shapeData.traceAttributes.thickness = triangleInputData.traceAttributes.thickness;
            snprintf(strTitle, MAX_TITLE, "Triangle: P0 = [%lf, %lf], P1 = [%lf, %lf], P2 = [%lf, %lf], NP = %zu",
               triangleInputData.P[0].x, triangleInputData.P[0].y, triangleInputData.P[1].x, triangleInputData.P[1].x,
               triangleInputData.P[2].x, triangleInputData.P[2].y, triangleInputData.totalNP);
            break;

      case RECTANGLE:
         getRectangleInputData(&rectangleInputData, strCommand, nLine);
            shapeData.NP = rectangleInputData.totalNP;
            shapeData.pointsData = calloc(rectangleInputData.totalNP, sizeof(POINT_DATA));
            if(shapeData.pointsData==NULL)
            {
               fprintf(stderr, "Failed to allocate memory for points data.\n");
               return false; // or handle error appropriately
            }
            getShapePointsData(RECTANGLE, &rectangleInputData, &shapeData, TM);


            shapeData.traceAttributes.penColor = rectangleInputData.traceAttributes.penColor;
            shapeData.traceAttributes.thickness = rectangleInputData.traceAttributes.thickness;
            snprintf(strTitle, MAX_TITLE, "rectangle: P0 = [%.lf, %.lf], P1 = [%.lf, %.lf], NP = %zu",
               rectangleInputData.P[0].x, rectangleInputData.P[0].y, rectangleInputData.P[2].x, rectangleInputData.P[2].y,
              rectangleInputData.totalNP);
            break;
      case CUBIC_BEZIER:
         cubicBezierInputData.P = calloc(4, sizeof(POINT2D));
         getCubicBezierInputData(&cubicBezierInputData, strCommand, nLine);
         shapeData.pointsData = calloc(cubicBezierInputData.NP, sizeof(POINT_DATA));
         if(shapeData.pointsData == NULL)
         {
            fprintf(stderr, "Failed to allocate memory for points data.\n");
            return false; // or handle error appropriately
         }
         shapeData.NP = cubicBezierInputData.NP;
         shapeData.traceAttributes.penColor = cubicBezierInputData.traceAttributes.penColor;
         shapeData.traceAttributes.thickness = cubicBezierInputData.traceAttributes.thickness;
         getShapePointsData(CUBIC_BEZIER, &cubicBezierInputData, &shapeData, TM);
         snprintf(strTitle, MAX_TITLE, "Cubic: P0 = [%.lf, %.lf], P1 = [%.lf, %.lf], P2 = [%.lf, %.lf], P3 = [%.lf, %.lf] NP = %.zu",
           cubicBezierInputData.P[0].x, cubicBezierInputData.P[0].y, cubicBezierInputData.P[1].x, cubicBezierInputData.P[1].y,
           cubicBezierInputData.P[2].x, cubicBezierInputData.P[2].y, cubicBezierInputData.P[3].x, cubicBezierInputData.P[3].y, cubicBezierInputData.NP);
         break;


   }
   printPointsData(&shapeData, strTitle);
 
   for(iPoint = 0; iPoint < shapeData.NP; iPoint++)
   {
      if(shapeData.bLeftCanDraw && !shapeData.bRightCanDraw)
      {
         drawSegment(shapeData.traceAttributes, shapeData.pointsData[iPoint].isol.leftArm, iPoint);
      }
      else if(!shapeData.bLeftCanDraw && shapeData.bRightCanDraw)
      {
         drawSegment(shapeData.traceAttributes, shapeData.pointsData[iPoint].isol.rightArm, iPoint);
      }
      else if(shapeData.bLeftCanDraw && shapeData.bRightCanDraw)
      {
         if(shapeData.leftArmDeltaDeg > shapeData.rightArmDeltaDeg)
         {
            drawSegment(shapeData.traceAttributes, shapeData.pointsData[iPoint].isol.rightArm, iPoint);

         }
         else
         {
            drawSegment(shapeData.traceAttributes, shapeData.pointsData[iPoint].isol.leftArm, iPoint);
         }
      }
      else printf("shape out of limit cannot be drawn drawn on line %d", nLine);
   }
   free(shapeData.pointsData);
   free(quadraticBezierInputData.P);
   free(cubicBezierInputData.P);
   return true;
}


//
//
//
void getShapePointsData(int shapeType, const void *pShapeInputData, SHAPE_DATA *shapeData, double TM[3][3])
{
   size_t iPoint;
   double t = NAN, theta = NAN, thetaStartRad = NAN, thetaEndRad = NAN;
   const LINE_INPUT_DATA *pLineInputData = (const LINE_INPUT_DATA *)pShapeInputData; // line inputs
   const ARC_INPUT_DATA *pArcInputData = (const ARC_INPUT_DATA *)pShapeInputData; // arc inputs
   const BEZIER_INPUT_DATA *pQuadBezierInputData = (const BEZIER_INPUT_DATA *)pShapeInputData; // bezier inputs
   const TRIANGLE_INPUT_DATA *pTriangleInputData = (const TRIANGLE_INPUT_DATA *)pShapeInputData;
   const RECTANGLE_INPUT_DATA *pRectangleInputData = (const RECTANGLE_INPUT_DATA *)pShapeInputData;
   const BEZIER_INPUT_DATA *pCubicInputData = (const BEZIER_INPUT_DATA *)pShapeInputData;
   shapeData->bLeftCanDraw = true;
   shapeData->bRightCanDraw = true;
   switch(shapeType)
   {
      case LINE:
        
         for(iPoint = 0; iPoint < pLineInputData->NP; iPoint++)
         {
            t = (double)iPoint / ((double)pLineInputData->NP - 1.);
            shapeData->pointsData[iPoint].pt.x = (1. - t) * pLineInputData->P[0].x + t * pLineInputData->P[1].x; // parametric equation
            shapeData->pointsData[iPoint].pt.y = (1. - t) * pLineInputData->P[0].y + t * pLineInputData->P[1].y; // parametric equation
    
         }
    
         break;
      case ARC:
         
         for(iPoint = 0; iPoint < pArcInputData->NP; iPoint++)
         {
            t = (double)iPoint / ((double)pArcInputData->NP - 1.);
            thetaEndRad = degToRad(pArcInputData->thetaEndDeg);
            thetaStartRad = degToRad(pArcInputData->thetaStartDeg);
            theta = thetaStartRad * (1 - t) + thetaEndRad * t;
            shapeData->pointsData[iPoint].pt.x = pArcInputData->pc.x + pArcInputData->r * cos(theta);
            shapeData->pointsData[iPoint].pt.y = pArcInputData->pc.y + pArcInputData->r * sin(theta);
          
         }
         
         break;
      case QUADRATIC_BEZIER:
        
         for(iPoint = 0; iPoint < pQuadBezierInputData->NP; iPoint++)
         {
            t = (double)iPoint / ((double)pQuadBezierInputData->NP - 1);
            shapeData->pointsData[iPoint].pt.x = pow(1. - t, 2.) * pQuadBezierInputData->P[0].x + 2. * (1. - t) * t * pQuadBezierInputData->P[1].x + pow(t, 2.) * pQuadBezierInputData->P[2].x;
            shapeData->pointsData[iPoint].pt.y = pow(1. - t, 2.) * pQuadBezierInputData->P[0].y + 2. * (1. - t) * t * pQuadBezierInputData->P[1].y + pow(t, 2.) * pQuadBezierInputData->P[2].y;
          
         }
         
         break;
        
      case TRIANGLE:
            for(iPoint = 0; iPoint < pTriangleInputData->NP[0]; iPoint++)
            {
               t = (double)iPoint / ((double)pTriangleInputData->NP[0] - 1.);
                  shapeData->pointsData[iPoint].pt.x = (1. - t) * pTriangleInputData->P[0].x + t * pTriangleInputData->P[1].x; // parametric equation
                  shapeData->pointsData[iPoint].pt.y = (1. - t) * pTriangleInputData->P[0].y + t * pTriangleInputData->P[1].y; // parametric equation
            }
            for(iPoint = 1; iPoint < pTriangleInputData->NP[1]; iPoint++)
            {
               t = (double)iPoint / ((double)pTriangleInputData->NP[0] - 1.);
               shapeData->pointsData[iPoint+ pTriangleInputData->NP[0]-1].pt.x = (1. - t) * pTriangleInputData->P[1].x + t * pTriangleInputData->P[2].x; // parametric equation
               shapeData->pointsData[iPoint+ pTriangleInputData->NP[0]-1].pt.y = (1. - t) * pTriangleInputData->P[1].y + t * pTriangleInputData->P[2].y; // parametric equation
            }
            for(iPoint = 1 ; iPoint < pTriangleInputData->NP[2]; iPoint++)
            {
               t = (double)iPoint / ((double)pTriangleInputData->NP[2] - 1.);
               shapeData->pointsData[iPoint + pTriangleInputData->NP[0]+ pTriangleInputData->NP[1]-2].pt.x = (1. - t) * pTriangleInputData->P[2].x + t * pTriangleInputData->P[0].x; // parametric equation
               shapeData->pointsData[iPoint + pTriangleInputData->NP[0]+ pTriangleInputData->NP[1] - 2].pt.y = (1. - t) * pTriangleInputData->P[2].y + t * pTriangleInputData->P[0].y; // parametric equation
            }

        
         break;
      case RECTANGLE:
         for(iPoint = 0; iPoint < pRectangleInputData->NP[0]; iPoint++)
         {
            t = (double)iPoint / ((double)pRectangleInputData->NP[0] - 1.);
            shapeData->pointsData[iPoint].pt.x = (1. - t) * pRectangleInputData->P[0].x + t * pRectangleInputData->P[1].x; // parametric equation
            shapeData->pointsData[iPoint].pt.y = (1. - t) * pRectangleInputData->P[0].y + t * pRectangleInputData->P[1].y; // parametric equation
         }
         for(iPoint = 1; iPoint < pRectangleInputData->NP[1]; iPoint++)
         {
            t = (double)iPoint / ((double)pRectangleInputData->NP[0] - 1.);
            shapeData->pointsData[iPoint + pRectangleInputData->NP[0] - 1].pt.x = (1. - t) * pRectangleInputData->P[1].x + t * pRectangleInputData->P[2].x; // parametric equation
            shapeData->pointsData[iPoint + pRectangleInputData->NP[0] - 1].pt.y = (1. - t) * pRectangleInputData->P[1].y + t * pRectangleInputData->P[2].y; // parametric equation
         }
         for(iPoint = 1; iPoint < pRectangleInputData->NP[2]; iPoint++)
         {
            t = (double)iPoint / ((double)pRectangleInputData->NP[2] - 1.);
            shapeData->pointsData[iPoint + pRectangleInputData->NP[0] + pRectangleInputData->NP[1] - 2].pt.x = (1. - t) * pRectangleInputData->P[2].x + t * pRectangleInputData->P[3].x; // parametric equation
            shapeData->pointsData[iPoint + pRectangleInputData->NP[0] + pRectangleInputData->NP[1] - 2].pt.y = (1. - t) * pRectangleInputData->P[2].y + t * pRectangleInputData->P[3].y; // parametric equation
         }
         for(iPoint = 1; iPoint < pRectangleInputData->NP[3]; iPoint++)
         {
            t = (double)iPoint / ((double)pRectangleInputData->NP[3] - 1.);
            shapeData->pointsData[iPoint + pRectangleInputData->NP[0] + pRectangleInputData->NP[1]+ pRectangleInputData->NP[2] - 3].pt.x = (1. - t) * pRectangleInputData->P[3].x + t * pRectangleInputData->P[0].x; // parametric equation
            shapeData->pointsData[iPoint + pRectangleInputData->NP[0] + pRectangleInputData->NP[1]+ pRectangleInputData->NP[2] - 3].pt.y = (1. - t) * pRectangleInputData->P[3].y + t * pRectangleInputData->P[0].y; // parametric equation
         }
         break;

      case CUBIC_BEZIER:
         for(iPoint = 0; iPoint < pCubicInputData->NP; iPoint++)
         {
            t = (double)iPoint / ((double)pCubicInputData->NP - 1);
            shapeData->pointsData[iPoint].pt.x = pow(1. - t, 3.) * pCubicInputData->P[0].x + 3. * pow(1. - t,2.) * t * pCubicInputData->P[1].x
               + 3. * (1. - t) * pow(t,2.) * pCubicInputData->P[2].x + pow(t, 3.) * pCubicInputData->P[3].x;

            shapeData->pointsData[iPoint].pt.y = pow(1. - t, 3.) * pCubicInputData->P[0].y + 3. * pow(1. - t, 2.) * t * pCubicInputData->P[1].y 
               + 3. * (1. - t) * pow(t, 2.) * pCubicInputData->P[2].y + pow(t, 3.) * pCubicInputData->P[3].y;

         }
         break;
   }
   
   
   for(int i = 0; i < shapeData->NP; i++)
   {
      shapeData->pointsData[i].pt = transform(TM, shapeData->pointsData[i].pt);
      inverseKinematics(shapeData->pointsData[i].pt, &shapeData->pointsData[i].isol);
      getArmAngleDeltas(shapeData);
      shapeData->bLeftCanDraw = shapeData->bLeftCanDraw && shapeData->pointsData[i].isol.bLeftCanReach;
      shapeData->bRightCanDraw = shapeData->bRightCanDraw && shapeData->pointsData[i].isol.bRightCanReach;
   }

}

void drawSegment(SHAPE_ATTRIBUTES attributes, JOINT_ANGLES ja, size_t i)
{
   JOINT_ANGLES homeJoints = {0.,0.};
   bool sameColor, sameAngle, sameHomeAngle;
   SCARA_STATE currentState;
   currentState = getRobotState();
   sameColor = areSameColor(attributes.penColor, currentState.penColor);
   sameAngle = areSameAngles(ja, currentState.jointAngles);
   sameHomeAngle = areSameAngles(homeJoints, currentState.jointAngles);
  // printf("current state: %d %d %d\n", currentState.penColor.r, currentState.penColor.g, currentState.penColor.b);
   if(i == 0)
   {
      if(!sameColor)
      {
         if(!sameHomeAngle)
         {
            setMotorSpeed(MOTOR_SPEED_HIGH);
            sendRobotCommand("HOME\n");
            setPenPos(PEN_UP);
         }
         else
         {
            setPenPos(PEN_UP);
         }
         setPenColor(attributes.penColor);
         rotateJoints(ja);
         setPenPos(PEN_DOWN);
         setMotorSpeed(attributes.thickness);
      }
      else if(!sameAngle && sameColor)
      {
        // printf("current: %d %d %d, target %d %d %d\n", currentState.penColor.r, currentState.penColor.g, currentState.penColor.b, attributes.penColor.r, attributes.penColor.g, attributes.penColor.b);
         setPenPos(PEN_UP);
         setMotorSpeed(MOTOR_SPEED_HIGH);
         rotateJoints(ja);
         setPenPos(PEN_DOWN);
         setMotorSpeed(attributes.thickness);
      }

   }
   else
   {
      if(currentState.motorSpeed != attributes.thickness)
      {
         setMotorSpeed(attributes.thickness);
      }

      rotateJoints(ja);

   }
}

void inverseKinematics(POINT2D P, INVERSE_SOLUTION *solution)
{
   int reachState = 0;
   double x = P.x, y = P.y, L = NAN;
   double theta1Ldeg, theta2Ldeg, theta1Rdeg, theta2Rdeg;

   reachState = 0;
   solution->bLeftCanReach = true;
   solution->bRightCanReach = true;
   L = sqrt(pow(x, 2.) + pow(y, 2.));

   theta1Ldeg = atan2(y, x) + acos((pow(L2, 2.) - pow(L, 2.) - pow(L1, 2.)) / (-2 * L * L1));
   theta2Ldeg = atan2((y - L1 * sin(theta1Ldeg)), (x - L1 * cos(theta1Ldeg))) - theta1Ldeg;
   theta1Rdeg = atan2(y, x) - acos((pow(L2, 2.) - pow(L, 2.) - pow(L1, 2.)) / (-2 * L * L1));
   theta2Rdeg = atan2((y - L1 * sin(theta1Rdeg)), (x - L1 * cos(theta1Rdeg))) - theta1Rdeg;

   theta1Ldeg = mapAngle(theta1Ldeg);
   theta2Ldeg = mapAngle(theta2Ldeg);
   theta1Rdeg = mapAngle(theta1Rdeg);
   theta2Rdeg = mapAngle(theta2Rdeg);

   if(L > LMAX) reachState |= L_EXCEEDS_MAX;
   if(L < LMIN) reachState |= L_EXCEEDS_MIN;
   if(fabs(theta1Ldeg) > ABS_THETA1_DEG_MAX) reachState |= THETA1L_EXCEEDS_MAX;
   if(fabs(theta1Rdeg) > ABS_THETA1_DEG_MAX) reachState |= THETA1R_EXCEEDS_MAX;
   if(fabs(theta2Ldeg) > ABS_THETA2_DEG_MAX) reachState |= THETA2L_EXCEEDS_MAX;
   if(fabs(theta2Rdeg) > ABS_THETA2_DEG_MAX) reachState |= THETA2R_EXCEEDS_MAX;

   solution->bLeftCanReach = solution->bLeftCanReach && !(reachState & (THETA1L_EXCEEDS_MAX | THETA2L_EXCEEDS_MAX));
   solution->bRightCanReach = solution->bRightCanReach && !(reachState & (THETA1R_EXCEEDS_MAX | THETA2R_EXCEEDS_MAX));

   solution->leftArm.theta1Deg = theta1Ldeg;
   solution->leftArm.theta2Deg = theta2Ldeg;
   solution->rightArm.theta1Deg = theta1Rdeg;
   solution->rightArm.theta2Deg = theta2Rdeg;
   solution->reachState = reachState;
}

void getArmAngleDeltas(SHAPE_DATA *shapeData)
{
   size_t iPoint = 999;
   SCARA_STATE currentState = getRobotState();
   shapeData->leftArmDeltaDeg = 0.;
   shapeData->rightArmDeltaDeg = 0.;
   for(iPoint = 0; iPoint < shapeData->NP; iPoint++)
   {
      if(iPoint == 0)
      {
         shapeData->leftArmDeltaDeg = max(fabs(shapeData->pointsData[iPoint].isol.leftArm.theta1Deg - currentState.jointAngles.theta1Deg),
            fabs(shapeData->pointsData[iPoint].isol.leftArm.theta2Deg - currentState.jointAngles.theta2Deg));
         shapeData->rightArmDeltaDeg = max(fabs(shapeData->pointsData[iPoint].isol.rightArm.theta1Deg - currentState.jointAngles.theta1Deg),
            fabs(shapeData->pointsData[iPoint].isol.rightArm.theta2Deg - currentState.jointAngles.theta2Deg));
      }
      else
      {
         shapeData->leftArmDeltaDeg = shapeData->leftArmDeltaDeg + max(fabs(shapeData->pointsData[iPoint].isol.leftArm.theta1Deg - shapeData->pointsData[iPoint - 1].isol.leftArm.theta1Deg),
            fabs(shapeData->pointsData[iPoint].isol.leftArm.theta2Deg - shapeData->pointsData[iPoint - 1].isol.leftArm.theta2Deg));
         shapeData->rightArmDeltaDeg = shapeData->rightArmDeltaDeg + max(fabs(shapeData->pointsData[iPoint].isol.rightArm.theta1Deg - shapeData->pointsData[iPoint - 1].isol.rightArm.theta1Deg),
            fabs(shapeData->pointsData[iPoint].isol.rightArm.theta2Deg - shapeData->pointsData[iPoint - 1].isol.rightArm.theta2Deg));
      }
   }
}

//
//
//
void getLineInputData(LINE_INPUT_DATA *lineData, char *strCommand, int nLine)
{
   bool bSuccess = true;
   char *tok = NULL, *nextTok = NULL;                    // for tokenizing strCommand
   const char *seps = " \t\n,;";                         // tokenizing delimiters
   size_t i;                                             // index
   char msg1[MAX_BUFF], msg2[MAX_BUFF];                  // for composing error message strings
   const char *strLine[4] = {"P0x", "P0y", "P1x", "P1y"};        // descriptions

   tok = strtok_s(strCommand, seps, &nextTok); // discard PEN_COLOR keyword


   for(i = 0; i < 2; i++)  // get both joint angles from the command string
   {
      sprintf_s(msg1, MAX_BUFF, "Error! Missing data for %s on line %d\n", strLine[2 * i], nLine);
      sprintf_s(msg2, MAX_BUFF, "Error! Garbage found in %s on line %d\n", strLine[2 * i], nLine);
      tok = strtok_s(NULL, seps, &nextTok); // get the token
      lineData->P[i].x = tokenToDouble(tok, msg1, msg2);
      if(isnan(lineData->P[i].x)) endProgram();
      sprintf_s(msg1, MAX_BUFF, "Error! Missing data for %s on line %d\n", strLine[2 * i + 1], nLine);
      sprintf_s(msg2, MAX_BUFF, "Error! Garbage found in %s on line %d\n", strLine[2 * i + 1], nLine);
      tok = strtok_s(NULL, seps, &nextTok); // get the token
      lineData->P[i].y = tokenToDouble(tok, msg1, msg2);
      if(isnan(lineData->P[i].y)) endProgram();
   }
   tok = strtok_s(NULL, seps, &nextTok); // get the token
   lineData->NP = tokenToNP(tok, nLine);
   if(lineData->NP == NP_ERROR) endProgram();


   tok = strtok_s(NULL, seps, &nextTok); // get the token
   bSuccess = getMotorSpeed(tok, &lineData->traceAttributes.thickness, nLine);
   if(!bSuccess) endProgram();

   tok = strtok_s(NULL, seps, &nextTok); // get the token
   bSuccess = getPenColor(tok, &lineData->traceAttributes.penColor);
   if(!bSuccess) endProgram();
}

void getArcInputData(ARC_INPUT_DATA *arcData, char *strCommand, int nLine)
{
   bool bSuccess = true;
   char *tok = NULL, *nextTok = NULL;                    // for tokenizing strCommand
   const char *seps = " \t\n,;";                         // tokenizing delimiters
   size_t i;                                             // index
   char msg1[MAX_BUFF], msg2[MAX_BUFF];                  // for composing error message strings
   const char *strLine[8] = {"P0x", "P0y", "radius", "thetaStart","thetaEnd", "NP", "Color", "Thickness"};        // descriptions

   tok = strtok_s(strCommand, seps, &nextTok); // discard PEN_COLOR keyword

   for(i = 0; i < 5; i++)
   {
      tok = strtok_s(NULL, seps, &nextTok);// get the token
      sprintf_s(msg1, MAX_BUFF, "Error! Missing data for %s on line %d\n", strLine[i], nLine);
      sprintf_s(msg2, MAX_BUFF, "Error! Garbage found in %s on line %d\n", strLine[i], nLine);
      switch(i)
      {
         case 0:
            arcData->pc.x = tokenToDouble(tok, msg1, msg2);
            if(isnan(arcData->pc.x)) endProgram();
            break;
         case 1:
            arcData->pc.y = tokenToDouble(tok, msg1, msg2);
            if(isnan(arcData->pc.y)) endProgram();
         case 2:
            arcData->r = tokenToDouble(tok, msg1, msg2);
            if(isnan(arcData->r)) endProgram();
            break;
         case 3:
            arcData->thetaStartDeg = tokenToDouble(tok, msg1, msg2);
            if(isnan(arcData->thetaStartDeg)) endProgram();
            break;
         case 4:
            arcData->thetaEndDeg = tokenToDouble(tok, msg1, msg2);
            if(isnan(arcData->thetaEndDeg)) endProgram();
            break;
      }
   }
   tok = strtok_s(NULL, seps, &nextTok); // get the token
   arcData->NP = tokenToNP(tok, nLine);
   if(arcData->NP == NP_ERROR) endProgram();


   tok = strtok_s(NULL, seps, &nextTok); // get the token
   bSuccess = getMotorSpeed(tok, &arcData->traceAttributes.thickness, nLine);
   if(!bSuccess) endProgram();

   tok = strtok_s(NULL, seps, &nextTok); // get the token
   bSuccess = getPenColor(tok, &arcData->traceAttributes.penColor);
   if(!bSuccess) endProgram();
}

void getQuadraticBezierData(BEZIER_INPUT_DATA *quadData, char *strCommand, int nLine)
{
   bool bSuccess = true;
   char *tok = NULL, *nextTok = NULL;                    // for tokenizing strCommand
   const char *seps = " \t\n,;";                         // tokenizing delimiters
   size_t i;                                             // index
   char msg1[MAX_BUFF], msg2[MAX_BUFF];                  // for composing error message strings
   const char *strLine[9] = {"P0x", "P0y", "P1x", "P1y","P2x", "P2y", "NP", "Color", "Thickness"};        // descriptions

   tok = strtok_s(strCommand, seps, &nextTok); // discard PEN_COLOR keyword

   for(i = 0; i < 3; i++)  // get both joint angles from the command string
   {
      sprintf_s(msg1, MAX_BUFF, "Error! Missing data for %s on line %d\n", strLine[2 * i], nLine);
      sprintf_s(msg2, MAX_BUFF, "Error! Garbage found in %s on line %d\n", strLine[2 * i], nLine);
      tok = strtok_s(NULL, seps, &nextTok); // get the token
      quadData->P[i].x = tokenToDouble(tok, msg1, msg2);
      if(isnan(quadData->P[i].x)) endProgram();
      sprintf_s(msg1, MAX_BUFF, "Error! Missing data for %s on line %d\n", strLine[2 * i + 1], nLine);
      sprintf_s(msg2, MAX_BUFF, "Error! Garbage found in %s on line %d\n", strLine[2 * i + 1], nLine);
      tok = strtok_s(NULL, seps, &nextTok); // get the token
      quadData->P[i].y = tokenToDouble(tok, msg1, msg2);
      if(isnan(quadData->P[i].y)) endProgram();
   }
   tok = strtok_s(NULL, seps, &nextTok); // get the token
   quadData->NP = tokenToNP(tok, nLine);
   if(quadData->NP == NP_ERROR) endProgram();


   tok = strtok_s(NULL, seps, &nextTok); // get the token
   bSuccess = getMotorSpeed(tok, &quadData->traceAttributes.thickness, nLine);
   if(!bSuccess) endProgram();

   tok = strtok_s(NULL, seps, &nextTok); // get the token
   bSuccess = getPenColor(tok, &quadData->traceAttributes.penColor);
   if(!bSuccess) endProgram();
}

void getTriangleInputData(TRIANGLE_INPUT_DATA *triangleData, char *strCommand, int nLine)
{
   bool bSuccess = true;
   char *tok = NULL, *nextTok = NULL;                    // for tokenizing strCommand
   const char *seps = " \t\n,;";                         // tokenizing delimiters
   size_t i;                                             // index
   char msg1[MAX_BUFF], msg2[MAX_BUFF];                  // for composing error message strings
   const char *strTriangle[6] = {"P0x", "P0y", "P1x", "P1y", "P2x", "P2y"};        // descriptions
   triangleData->totalNP = 0;
   tok = strtok_s(strCommand, seps, &nextTok); // discard PEN_COLOR keyword


   for(i = 0; i < 3; i++)  // get both joint angles from the command string
   {
      sprintf_s(msg1, MAX_BUFF, "Error! Missing data for %s on line %d\n", strTriangle[2 * i], nLine);
      sprintf_s(msg2, MAX_BUFF, "Error! Garbage found in %s on line %d\n", strTriangle[2 * i], nLine);
      tok = strtok_s(NULL, seps, &nextTok); // get the token
      triangleData->P[i].x = tokenToDouble(tok, msg1, msg2);
      if(isnan(triangleData->P[i].x)) endProgram();
      sprintf_s(msg1, MAX_BUFF, "Error! Missing data for %s on line %d\n", strTriangle[2 * i + 1], nLine);
      sprintf_s(msg2, MAX_BUFF, "Error! Garbage found in %s on line %d\n", strTriangle[2 * i + 1], nLine);
      tok = strtok_s(NULL, seps, &nextTok); // get the token
      triangleData->P[i].y = tokenToDouble(tok, msg1, msg2);
      if(isnan(triangleData->P[i].y)) endProgram();
   }
   for(i = 0; i < 3; i++)  // get both joint angles from the command string
   {
      tok = strtok_s(NULL, seps, &nextTok); // get the token
      triangleData->NP[i] = tokenToNP(tok, nLine);

      if(triangleData->NP[i] == NP_ERROR) endProgram();
      triangleData->totalNP += triangleData->NP[i];
   }
   triangleData->totalNP -= 2;
   tok = strtok_s(NULL, seps, &nextTok); // get the token
   bSuccess = getMotorSpeed(tok, &triangleData->traceAttributes.thickness, nLine);
   if(!bSuccess) endProgram();

   tok = strtok_s(NULL, seps, &nextTok); // get the token
   bSuccess = getPenColor(tok, &triangleData->traceAttributes.penColor);
   if(!bSuccess) endProgram();

}

void getRectangleInputData(RECTANGLE_INPUT_DATA *rectangleData, char *strCommand, int nLine)
{
   bool bSuccess = true;
   char *tok = NULL, *nextTok = NULL;                    // for tokenizing strCommand
   const char *seps = " \t\n,;";                         // tokenizing delimiters
   size_t i;                                             // index
   char msg1[MAX_BUFF], msg2[MAX_BUFF];                  // for composing error message strings
   const char *strRectangle[8] = {"P0x", "P0y", "P1x", "P1y"};        // descriptions
   rectangleData->totalNP = 0;
   tok = strtok_s(strCommand, seps, &nextTok); // discard PEN_COLOR keyword


   for(i = 0; i < 2; i++)  // get both joint angles from the command string
   {
      sprintf_s(msg1, MAX_BUFF, "Error! Missing data for %s on line %d\n", strRectangle[2 * i], nLine);
      sprintf_s(msg2, MAX_BUFF, "Error! Garbage found in %s on line %d\n", strRectangle[2 * i], nLine);
      tok = strtok_s(NULL, seps, &nextTok); // get the token
      rectangleData->P[2*i].x = tokenToDouble(tok, msg1, msg2);
      if(isnan(rectangleData->P[2*i].x)) endProgram();
      sprintf_s(msg1, MAX_BUFF, "Error! Missing data for %s on line %d\n", strRectangle[2 * i + 1], nLine);
      sprintf_s(msg2, MAX_BUFF, "Error! Garbage found in %s on line %d\n", strRectangle[2 * i + 1], nLine);
      tok = strtok_s(NULL, seps, &nextTok); // get the token
      rectangleData->P[2*i].y = tokenToDouble(tok, msg1, msg2);
      if(isnan(rectangleData->P[2*i].y)) endProgram();
   }
   for(i = 0; i < 2; i++)
   {
      rectangleData->P[2*i + 1].x = rectangleData->P[2*i].x; // P[1].x = P[0].x;  P[3].x = P[2].x
      rectangleData->P[2 * i + 1].y = rectangleData->P[-2*i + 2].y; // P[1].y = P[2].y;   P[3].y = P[0].x
   }
    for(i = 0; i < 2; i++)
   {
      tok = strtok_s(NULL, seps, &nextTok); // get the token
      rectangleData->NP[i] = tokenToNP(tok, nLine);
      if(rectangleData->NP[i] == NP_ERROR) endProgram();
      rectangleData->NP[i + 2] = rectangleData->NP[i];
      rectangleData->totalNP += 2*rectangleData->NP[i];
   }

   rectangleData->totalNP -= 3;
   tok = strtok_s(NULL, seps, &nextTok); // get the token
   bSuccess = getMotorSpeed(tok, &rectangleData->traceAttributes.thickness, nLine);
   if(!bSuccess) endProgram();

   tok = strtok_s(NULL, seps, &nextTok); // get the token
   bSuccess = getPenColor(tok, &rectangleData->traceAttributes.penColor);
   if(!bSuccess) endProgram();
   printf("%lf %lf %lf %lf %zu", rectangleData->P[0].x, rectangleData->P[1].y, rectangleData->P[2].x, rectangleData->P[3].y, rectangleData->totalNP);
}

void getCubicBezierInputData(BEZIER_INPUT_DATA *cubicData, char *strCommand, int nLine)
{
   bool bSuccess = true;
   char *tok = NULL, *nextTok = NULL;                    // for tokenizing strCommand
   const char *seps = " \t\n,;";                         // tokenizing delimiters
   size_t i;                                             // index
   char msg1[MAX_BUFF], msg2[MAX_BUFF];                  // for composing error message strings
   const char *strLine[11] = {"P0x", "P0y", "P1x", "P1y","P2x", "P2y","P3x", "P3y", "NP", "Color", "Thickness"};        // descriptions

   tok = strtok_s(strCommand, seps, &nextTok); // discard PEN_COLOR keyword

   for(i = 0; i < 4; i++)  // get both joint angles from the command string
   {
      sprintf_s(msg1, MAX_BUFF, "Error! Missing data for %s on line %d\n", strLine[2 * i], nLine);
      sprintf_s(msg2, MAX_BUFF, "Error! Garbage found in %s on line %d\n", strLine[2 * i], nLine);
      tok = strtok_s(NULL, seps, &nextTok); // get the token
      cubicData->P[i].x = tokenToDouble(tok, msg1, msg2);
      if(isnan(cubicData->P[i].x)) endProgram();
      sprintf_s(msg1, MAX_BUFF, "Error! Missing data for %s on line %d\n", strLine[2 * i + 1], nLine);
      sprintf_s(msg2, MAX_BUFF, "Error! Garbage found in %s on line %d\n", strLine[2 * i + 1], nLine);
      tok = strtok_s(NULL, seps, &nextTok); // get the token
      cubicData->P[i].y = tokenToDouble(tok, msg1, msg2);
      if(isnan(cubicData->P[i].y)) endProgram();
   }
   tok = strtok_s(NULL, seps, &nextTok); // get the token
   cubicData->NP = tokenToNP(tok, nLine);
   if(cubicData->NP == NP_ERROR) endProgram();


   tok = strtok_s(NULL, seps, &nextTok); // get the token
   bSuccess = getMotorSpeed(tok, &cubicData->traceAttributes.thickness, nLine);
   if(!bSuccess) endProgram();

   tok = strtok_s(NULL, seps, &nextTok); // get the token
   bSuccess = getPenColor(tok, &cubicData->traceAttributes.penColor);
   if(!bSuccess) endProgram();
}


//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Rotates the robot joints based on data in a command string from the input file
// ARGUMENTS:    strCommand:  the command string
//               nLine: the file line number where the command string was read
// RETURN VALUE: true if the command was valid, false if not
bool rotateJointsFromCommandString(char *strCommand, int nLine)
{
   char *tok = NULL, *nextTok = NULL;                    // for tokenizing strCommand
   const char *seps = " \t\n,;";                         // tokenizing delimiters
   double angDeg[2] = {NAN, NAN};                        // joint angles (in degrees)
   const char *strAng[2] = {"shoulder", "elbow"};        // descriptions
   size_t i;                                             // index
   char msg1[MAX_BUFF], msg2[MAX_BUFF];                  // for composing error message strings
   JOINT_ANGLES ja = {NAN, NAN};                         // specified joint angles

   tok = strtok_s(strCommand, seps, &nextTok); // discard ROTATE_JOINTS keyword

   for(i = 0; i < 2; i++)  // get both joint angles from the command string
   {
      tok = strtok_s(NULL, seps, &nextTok); // get the token

      // prepare the error message strings
      sprintf_s(msg1, MAX_BUFF, "Error! Missing data for %s angle on line %d\n", strAng[i], nLine);
      sprintf_s(msg2, MAX_BUFF, "Error! Garbage found in %s angle on line %d\n", strAng[i], nLine);
      angDeg[i] = tokenToDouble(tok, msg1, msg2);
      if(isnan(angDeg[i])) return false;
   }
   if(fabs(angDeg[0]) > ABS_THETA1_DEG_MAX || fabs(angDeg[1]) > ABS_THETA2_DEG_MAX) // one or both angles exceed limits!
   {
      if(fabs(angDeg[0]) > ABS_THETA1_DEG_MAX)
         printf("Magnitude of %s angle exceeds maximum (%lg) on line %d\n", strAng[0], ABS_THETA1_DEG_MAX, nLine);
      if(fabs(angDeg[1]) > ABS_THETA2_DEG_MAX)
         printf("Magnitude of %s angle exceeds maximum (%lg) on line %d\n", strAng[1], ABS_THETA2_DEG_MAX, nLine);

      return false;
   }

   // package the angles and send them to the robot via a ROTATE_JOINT command
   ja.theta1Deg = angDeg[0];
   ja.theta2Deg = angDeg[1];
   rotateJoints(ja);

   return true;
}

//
// 
// 
bool setPenColorFromCommandString(char *strCommand, int nLine)
{
   char *tok = NULL, *nextTok = NULL;                    // for tokenizing strCommand
   const char *seps = " \t\n,;";                         // tokenizing delimiters
   int color[3] = {-1, -1, -1};     // descriptions
   size_t i;                                             // index
   char msg1[MAX_BUFF], msg2[MAX_BUFF];                  // for composing error message strings
   const char *strColor[3] = {"red/1st value", "green/2nd value", "blue/3rd value"};        // descriptions
   RGB pColor = {-1,-1,-1};
   tok = strtok_s(strCommand, seps, &nextTok); // discard PEN_COLOR keyword


   for(i = 0; i < 3; i++)  // get both joint angles from the command string
   {
      tok = strtok_s(NULL, seps, &nextTok); // get the token
      // prepare the error message strings
      sprintf_s(msg1, MAX_BUFF, "Error! Missing data for pen color %s on line %d\n", strColor[i], nLine);
      sprintf_s(msg2, MAX_BUFF, "Error! Garbage found in pen color %s on line %d\n", strColor[i], nLine);
      color[i] = tokenToInt(tok, msg1, msg2);
      if(color[i] == 1110) return false;

      if(color[i] > 255)
      {
         printf("Pen color of %s beyond range 0 - 255 of on line %d\n", strColor[i], nLine);
         return false;
      }
      else if(color[i] < 0)
      {
         printf("Pen color of %s below range 0 - 255 on line %d\n", strColor[i], nLine);
         return false;
      }
   }

   pColor.r = color[0];
   pColor.g = color[1];
   pColor.b = color[2];
   setPenColor(pColor);
   return true;
}
//---------------------------------------------------------------------------------------------------------------------
//
//

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Checks if two arm configurations are the same (within a tolerance)
// ARGUMENTS:    JA1:  first set of joint angles
//               JA2:  second set of joint angles
// RETURN VALUE: true if JA1 is the same as JA2, false if not
bool areSameAngles(JOINT_ANGLES JA1, JOINT_ANGLES JA2)
{
   // BOTH configuration shoulder angles and BOTH configuration elbow angles must be same within a tolerance
   if(fabs(JA1.theta1Deg - JA2.theta1Deg) > ANGLE_TOLERANCE ||
      fabs(JA1.theta2Deg - JA2.theta2Deg) > ANGLE_TOLERANCE)
      return false;
   else
      return true;
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Converts a command string token into a double.  Issues error messages if NULL token or has garbage
// ARGUMENTS:    tok:  the string token
//               msg1: the error message string if the token is NULL
//               msg2: the error message string if the token has non-numerical garbage
// RETURN VALUE: the converted double or NAN if an error occurred
double tokenToDouble(char *tok, const char *msg1, const char *msg2)
{
   double d = NAN;         // the converted double
   char *pGarbage = NULL;  // for garbage check

   if(tok == NULL)  // NULL token error
   {
      printf("%s", msg1);
      return NAN;
   }

   d = strtod(tok, &pGarbage); // convert token to double

   if(pGarbage[0] != '\0') // conversion detected garbage
   {
      printf("%s", msg2);
      return NAN;
   }

   return d;  // conversion successful
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Converts a token for NP into a size_t.  Issues error messages if NULL token or has garbage
// ARGUMENTS:    tok:  the string token
//               nLine: the line from the input file where the token was taken
// RETURN VALUE: the converted size_t value or 0 if an error occurred
size_t tokenToNP(char *tok, int nLine)
{
   size_t NP = 0;           // NP value
   long lNP = -1;           // to manage negative numbers
   char *pGarbage = NULL;   // stores garbage

   if(tok == NULL)  // missing token!
   {
      printf("Error! Missing data for NP on line %d\n", nLine);
      return NP_ERROR;
   }

   lNP = strtol(tok, &pGarbage, 10);
   if(pGarbage[0] != '\0')
   {
      printf("Error! Garbage found for NP on line %d\n", nLine);
      return NP_ERROR;
   }
   else if(lNP < 2)
   {
      printf("Error! NP value <2 on line %d\n", nLine);
      return NP_ERROR;
   }

   NP = (size_t)lNP;  // ok to store in size_t variable
   return NP;
}

//---------------------------------------------------------------------------------------------------------------------
//
//
int tokenToInt(char *tok, const char *msg1, const char *msg2)
{
   int color = -1;         // the converted double
   char *pGarbage = NULL;  // for garbage check

   if(tok == NULL)  // NULL token error
   {
      printf("%s", msg1);
      return 1110;  // 1110 used for error checking
   }

   color = strtol(tok, &pGarbage, 10); // convert token to double

   if(pGarbage[0] != '\0') // conversion detected garbage
   {
      printf("%s", msg2);
      return 1110;// 1110 used for error checking
   }

   return color;  // conversion successful
}

bool setCyclePenColorFromCommandString(char *strCommand, int nLine)
{
   char *tok = NULL, *nextTok = NULL;                    // for tokenizing strCommand
   const char *seps = " \t\n,;";                         // tokenizing delimiters                
   tok = strtok_s(strCommand, seps, &nextTok); // discard CYCLE_PEN_COLOR keyword

   tok = strtok_s(NULL, seps, &nextTok);
   printf("next tok %s", tok);
   if(strcmp(tok, "ON") == 0)
   {
      sendRobotCommand("CYCLE_PEN_COLORS ON\n");

   }
   else if(strcmp(tok, "OFF") == 0)
   {
      sendRobotCommand("CYCLE_PEN_COLORS OFF\n");
   }
   else
   {
      printf("Invalid state of CYCLE_PEN_COLORS on line %d", nLine);
      endProgram();
      return false;
   }
   return true;

}

bool moveToFromCommandString(char *strCommand, int nLine)
{
   char *tok = NULL, *nextTok = NULL;                    // for tokenizing strCommand
   const char *seps = " \t\n,;";                         // tokenizing delimiters
   POINT2D point = {0};                                            // index
   INVERSE_SOLUTION isol = {0};
   char msg1[MAX_BUFF], msg2[MAX_BUFF];                  // for composing error message strings
   const char *strLine[2] = {"x", "y"};        // descriptions
   SCARA_STATE currentState = getRobotState();

   tok = strtok_s(strCommand, seps, &nextTok); // discard MOVE_TO keyword


   sprintf_s(msg1, MAX_BUFF, "Error! Missing data for %s on line %d\n", strLine[0], nLine);
   sprintf_s(msg2, MAX_BUFF, "Error! Garbage found in %s on line %d\n", strLine[0], nLine);
   tok = strtok_s(NULL, seps, &nextTok); // get the token
   point.x = tokenToDouble(tok, msg1, msg2);
   if(isnan(point.x)) endProgram();
   sprintf_s(msg1, MAX_BUFF, "Error! Missing data for %s on line %d\n", strLine[1], nLine);
   sprintf_s(msg2, MAX_BUFF, "Error! Garbage found in %s on line %d\n", strLine[1], nLine);
   tok = strtok_s(NULL, seps, &nextTok); // get the token
   point.y = tokenToDouble(tok, msg1, msg2);
   if(isnan(point.y)) endProgram();

   inverseKinematics(point, &isol);

   if(isol.bLeftCanReach && !isol.bRightCanReach)
   {
      rotateJoints(isol.leftArm);

   }
   else if(!isol.bLeftCanReach && isol.bRightCanReach)
   {
      rotateJoints(isol.rightArm);
   }
   else if(max(fabs(isol.leftArm.theta1Deg - currentState.jointAngles.theta1Deg), fabs(isol.leftArm.theta2Deg - currentState.jointAngles.theta2Deg)) >
      max(fabs(isol.rightArm.theta1Deg - currentState.jointAngles.theta1Deg), fabs(isol.rightArm.theta2Deg - currentState.jointAngles.theta2Deg)))
      rotateJoints(isol.rightArm);
   else
      printf("Point on line %d is out of reach\n", nLine);
   return true;
}

bool setMotorSpeedFromCommandString(char *strCommand, int nLine)
{
   char *tok = NULL, *nextTok = NULL;                    // for tokenizing strCommand
   const char *seps = " \t\n,;";                         // tokenizing delimiters
   int speed;     // descriptions
   bool bSuccess = true;
   tok = strtok_s(strCommand, seps, &nextTok); // discard PEN_COLOR keyword


   tok = strtok_s(NULL, seps, &nextTok); // get the token

   bSuccess = getMotorSpeed(tok, &speed, nLine);
   if(!bSuccess) return false;
   setMotorSpeed(speed);
   return true;

}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  sets an RGB color structure to a listed color based on a string with the colors name
// ARGUMENTS:    strColor:  Name of the color
//               pColor:    pointer to RGB structure
// RETURN VALUE: true if color found in list, false if not
bool getPenColor(const char *strColor, RGB *pColor)
{
   if(strcmp(strColor, "AQUA") == 0)
      *pColor = AQUA;
   else if(strcmp(strColor, "BLACK") == 0)
      *pColor = BLACK;
   else if(strcmp(strColor, "GREEN") == 0)
      *pColor = GREEN;
   else if(strcmp(strColor, "HOTPINK") == 0)
      *pColor = HOTPINK;
   else if(strcmp(strColor, "NAVY") == 0)
      *pColor = NAVY;
   else if(strcmp(strColor, "ORANGE") == 0)
      *pColor = ORANGE;
   else if(strcmp(strColor, "PURPLE") == 0)
      *pColor = PURPLE;
   else if(strcmp(strColor, "RED") == 0)
      *pColor = RED;
   else if(strcmp(strColor, "WHITE") == 0)
      *pColor = WHITE;
   else if(strcmp(strColor, "YELLOW") == 0)
      *pColor = YELLOW;
   else
   {
      printf("Invalid Pen Color!!! (%s)\n", strColor);
      *pColor = PEN_COLOR_ERROR;
      return false;
   }

   return true;
}

bool getMotorSpeed(const char *strSpeed, int *motorSpeed, int nLine)
{
   if(strcmp(strSpeed, "THICK") == 0)
      *motorSpeed = MOTOR_SPEED_LOW;
   else if(strcmp(strSpeed, "THIN") == 0)
      *motorSpeed = MOTOR_SPEED_HIGH;
   else if(strcmp(strSpeed, "MEDIUM") == 0)
      *motorSpeed = MOTOR_SPEED_MEDIUM;
   else
   {
      printf("Invalid Line Thickness!!! (%s) on line %d\n", strSpeed, nLine);
      *motorSpeed = THICKNESS_ERROR;
      return false;
   }
   return true;

}

void setPenColor(RGB penColor)
{
   char strCommand[MAX_COMMAND];  // for storing robot command strings
   int r, g, b;

   r = penColor.r;
   g = penColor.g;
   b = penColor.b;

   // compose the command string using sprintf_s
   sprintf_s(strCommand, MAX_COMMAND, "PEN_COLOR %d %d %d\n", r, g, b);
   // send the command to the robot
   sendRobotCommand(strCommand);
}
void setMotorSpeed(int motorSpeed)
{
   if(motorSpeed == MOTOR_SPEED_HIGH)
   {
      sendRobotCommand("MOTOR_SPEED HIGH\n");
   }
   else if(motorSpeed == MOTOR_SPEED_MEDIUM)
   {
      sendRobotCommand("MOTOR_SPEED MEDIUM\n");
   }
   else if(motorSpeed == MOTOR_SPEED_LOW)
   {
      sendRobotCommand("MOTOR_SPEED LOW\n");
   }
}
//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  prints a table header with centered title (includes top/bottom borders)
// ARGUMENTS:    tableWdith:     width of table included left/right borders
//               strTableTitle:  title string
// RETURN VALUE: none
void printTableHeader(int tableWidth, const char *strTableTitle)
{
   int numChars = -1;   // number of characters printed by printf
   int titleWidth = -1; // number of characters in the table title
   int leftSpaces = -1; // spaces to left of title
   int exVL = snprintf(NULL, 0, "%lc", VL) - 1;  // extra bytes need to adjust printf for VL character :(

   titleWidth = (int)strlen(strTableTitle);  // title width

   leftSpaces = (tableWidth - 2 - titleWidth) / 2;  // -2 for the left/right borders

   printTableHBorder(TL, TR, tableWidth); // top border

   // header title
   numChars = printf("%lc%*c%s", VL, leftSpaces, ' ', strTableTitle) - exVL; // printf returns more than 1 to print VL!
   printf("%*lc\n", tableWidth - numChars + exVL, VL); // +exVL for right border

   printTableHBorder(CL, CR, tableWidth);  // mid border (excepts table data row below the header)
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  prints details of point data for a shape into a table
// ARGUMENTS:    pSD:      pointer to SHAPE_DATA structure containing all information about the shape
//               strTitle: header title
// RETURN VALUE: none
void printPointsData(const SHAPE_DATA *pSD, const char *strTitle)
{
   size_t NP = pSD->NP;                                       // number of points on shape
   int numChars = -1;                                         // number of characters returned by printf
   size_t ip = 0;                                             // point index
   wchar_t chL = 0, chR = 0;                                  // left/right border characters
   double L = NAN;                                            // distance to pen tool from robot base
   int exVL = snprintf(NULL, 0, "%lc", VL) - 1;               // extra bytes needed to adjust printf for VL character :(
   int exDeg = snprintf(NULL, 0, "%lc", DEGREE_SYMBOL) - 1;   // extra bytes for DEGREE_SYMBOL :(
   int exTheta = snprintf(NULL, 0, "%lc", THETA_SYMBOL) - 1;  // extra bytes for THETA_SYMBOL :(
   const POINT2D *ppos = NULL;
   const INVERSE_SOLUTION *psol = NULL;

   printTableHeader(TABLE_WIDTH, strTitle);  // print the title

   for(ip = 0; ip < NP; ip++)  // print data for each point in the shape
   {
      ppos = &pSD->pointsData[ip].pt;   // pointer to the point x,y values
      psol = &pSD->pointsData[ip].isol; // pointer to the point inverse solution

      L = sqrt(pow(ppos->x, 2) + pow(ppos->y, 2));  // compute the distance to the point

      // coordinates and L (print all text up to right spaces)
      numChars = printf("%lc%*cPOINT %02zu:   x = %+*.*lf,   y = %+*.*lf.   L = %+*.*lf", VL, LEFT_MARGIN, ' ', ip + 1,
         FIELD_WIDTH, PRECISION, ppos->x, FIELD_WIDTH, PRECISION, ppos->y, FIELD_WIDTH, PRECISION, L) - exVL;
      printf("%*lc\n", TABLE_WIDTH - numChars + exVL, VL); // right spaces and border

      if(psol->reachState == L_EXCEEDS_MIN || psol->reachState == L_EXCEEDS_MAX)  // unreachable because of L limits
      {
         numChars = printf("%lc%*c", VL, LEFT_MARGIN, ' ') - exVL;  // left margin

         printRepeatedChar(ERROR_SYMBOL_LEFT, NUM_ERROR_SYMBOLS); // left error symbols
         numChars += NUM_ERROR_SYMBOLS;

         // error message
         if(psol->reachState == L_EXCEEDS_MAX)
            numChars += printf(" POINT IS OUTSIDE MAXIMUM REACH OF ROBOT (L_MAX = %.*lf) ", LIMIT_PRECISION, LMAX);
         else
            numChars += printf(" POINT IS INSIDE MINIMUM REACH OF ROBOT (L_MIN = %.*lf) ", LIMIT_PRECISION, LMIN);

         printRepeatedChar(ERROR_SYMBOL_RIGHT, NUM_ERROR_SYMBOLS); // right error symbols
         numChars += NUM_ERROR_SYMBOLS;

         printf("%*lc\n", TABLE_WIDTH - numChars + exVL, VL);  // right spaces and border
      }
      else // reachable in terms of L
      {
         //------------- LEFT ARM DATA -------------
         // left arm theta values
         numChars = printf("%lc%*cLEFT ARM:  %lc1 = %+*.*lf%lc, %lc2 = %+*.*lf%lc.  ", VL, LEFT_MARGIN, ' ',
            THETA_SYMBOL, FIELD_WIDTH, PRECISION, psol->leftArm.theta1Deg, DEGREE_SYMBOL,
            THETA_SYMBOL, FIELD_WIDTH, PRECISION, psol->leftArm.theta2Deg, DEGREE_SYMBOL)
            - exVL - 2 * exTheta - 2 * exDeg;

         // left arm theta errors
         if(psol->reachState & THETA1L_EXCEEDS_MAX || psol->reachState & THETA2L_EXCEEDS_MAX)
         {
            if(psol->reachState & THETA1L_EXCEEDS_MAX && psol->reachState & THETA2L_EXCEEDS_MAX)
               numChars += printf("%lc1 and %lc2 exceed max angle!", THETA_SYMBOL, THETA_SYMBOL) - 2 * exTheta;
            else if(psol->reachState & THETA1L_EXCEEDS_MAX)
               numChars += printf("%lc1 exceeds max angle!", THETA_SYMBOL) - exTheta;
            else
               numChars += printf("%lc2 exceeds max angle!", THETA_SYMBOL) - exTheta;
         }
         printf("%*lc\n", TABLE_WIDTH - numChars + exVL, VL);  // right spaces and border

         //------------- RIGHT ARM DATA -------------
         // right arm theta values
         numChars = printf("%lc%*cRIGHT ARM: %lc1 = %+*.*lf%lc, %lc2 = %+*.*lf%lc.  ", VL, LEFT_MARGIN, ' ',
            THETA_SYMBOL, FIELD_WIDTH, PRECISION, psol->rightArm.theta1Deg, DEGREE_SYMBOL,
            THETA_SYMBOL, FIELD_WIDTH, PRECISION, psol->rightArm.theta2Deg, DEGREE_SYMBOL)
            - exVL - 2 * exTheta - 2 * exDeg;

         // right arm theta errors
         if(psol->reachState & THETA1R_EXCEEDS_MAX || psol->reachState & THETA2R_EXCEEDS_MAX)
         {
            if(psol->reachState & THETA1R_EXCEEDS_MAX && psol->reachState & THETA2R_EXCEEDS_MAX)
               numChars += printf("%lc1 and %lc2 exceed max angle!", THETA_SYMBOL, THETA_SYMBOL) - 2 * exTheta;
            else if(psol->reachState & THETA1R_EXCEEDS_MAX)
               numChars += printf("%lc1 exceeds max angle!", THETA_SYMBOL) - exTheta;
            else
               numChars += printf("%lc2 exceeds max angle!", THETA_SYMBOL) - exTheta;
         }
         printf("%*lc\n", TABLE_WIDTH - numChars + exVL, VL); // right spaces and border
      }

      // bottom border (use bottom left/right symbols if last point, othewize mid left/right symbols)
      chL = ip == NP - 1 ? BL : CL;
      chR = ip == NP - 1 ? BR : CR;
      printTableHBorder(chL, chR, TABLE_WIDTH);
   }

   // print the angle deltas
   printf(" Left arm sum of max angle changes: ");
   if(!pSD->bLeftCanDraw)
      printf("N/A (undrawable)\n");
   else
      printf("%.2lf%lc\n", pSD->leftArmDeltaDeg, DEGREE_SYMBOL);

   printf("Right arm sum of max angle changes: ");
   if(!pSD->bRightCanDraw)
      printf("N/A (undrawable)\n");
   else
      printf("%.2lf%lc\n", pSD->rightArmDeltaDeg, DEGREE_SYMBOL);
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Prints a horiztonal border for a table
// ARGUMENTS:    chL/chR: left/right border characters
//               tableWidth: width of table including borders
// RETURN VALUE: none
void printTableHBorder(wchar_t chL, wchar_t chR, int tableWidth)
{
   printf("%lc", chL);
   printRepeatedChar(HL, tableWidth - 2);
   printf("%lc\n", chR);
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Prints a horiztonal border for a table
// ARGUMENTS:    len: length of line
// RETURN VALUE: none
void printHLine(int len)
{
   printRepeatedChar(HL, len);
   printf("\n");
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Prints a character to the console repeatedly
// ARGUMENTS:    ch: character to repeat
//               numRepeats: number of repeats
// RETURN VALUE: none
void printRepeatedChar(wchar_t ch, int numRepeats)
{
   int i = -1; // loop counter

   for(i = 0; i < numRepeats; i++)
   {
      printf("%lc", ch);
   }
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Maps an angle in radians into a an equivalent angle in degrees that follows the angle ranges 
//               defined in the robot (-180 <= theta <= +180)
// ARGUMENTS:    ang: the angle in radians 
// RETURN VALUE: the mapped angle in radians
double mapAngle(double theta)
{
   theta = fmod(theta, 2.0 * PI);  // put in range -2*PI <= ang <= +2*PI

   // map into range -PI <= ang <= +PI
   if(theta > PI)
      theta -= 2.0 * PI;
   else if(theta < -PI)
      theta += 2.0 * PI;

   return radToDeg(theta);
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Returns angle in degrees from input angle in radian
// ARGUMENTS:    angDeg:  angle in degrees
// RETURN VALUE: angle in radians
double degToRad(double angDeg)
{
   return (PI / 180.0) * angDeg;
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Returns angle in radians from input angle in degrees
// ARGUMENTS:    angRad:  angle in radians
// RETURN VALUE: angle in degrees
double radToDeg(double angRad)
{
   return (180.0 / PI) * angRad;
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Waits user to press enter.  flushes stdin if keystrokes precede enter
// ARGUMENTS:    none
// RETURN VALUE: none
void waitForEnterKey()
{
   char buff[MAX_BUFF];
   fgets(buff, MAX_BUFF, stdin);
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Ends program from anywhere in code.
// ARGUMENTS:    none
// RETURN VALUE: none
void endProgram()
{
   printf("\a\a\a\a");  // error beeps
   printf("Command _NOT_ sent to robot\nPress ENTER to end the program...\n"); // termination message
   waitForEnterKey();
   exit(0);
}

//-----------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Move the SCARA Robot to specified joint angles
// ARGUMENTS:    theta1Deg, theta2Deg:  shoulder and elbow angles in degrees
// RETURN VALUE: none
void rotateJoints(JOINT_ANGLES ja)
{
   char strCommand[MAX_COMMAND];  // for storing robot command strings

   // compose the command string using sprintf_s
   sprintf_s(strCommand, MAX_COMMAND, "ROTATE_JOINT ANG1 %lf ANG2 %lf\n", ja.theta1Deg, ja.theta2Deg);

   // send the command to the robot
   sendRobotCommand(strCommand);
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  prints to both a file and to the console
// ARGUMENTS:    f:  the file handle
//               fmt, ...: for variable number of parameters
// RETURN VALUE: the number of characters printed
int dsprintf(char *fmt, ...)
{
   static bool bIsFirst = true;  // to determine if first entry to function
   va_list args;                 // list of arguments to function
   int n1 = -1, n2 = -1;         // fix a glitch in number of characters printed:  file vs screen
   FILE *flog = NULL;            // for writing to a log file
   errno_t err = 0;              // detect error opening log file
   char fmt2[1024];              // hack!

   // if first time in, open in write mode to clear the previous file contents else open in append mode
   if(bIsFirst)
   {
      err = fopen_s(&flog, "log.txt", "w");
      bIsFirst = false;
   }
   else
   {
      err = fopen_s(&flog, "log.txt", "a");
   }

   // can't open!
   if(flog == NULL || err != 0)
   {
      printf("Cannot open log.txt for writing!!\n");
      printf("Please check that log.txt is not opened by another program.\n");
      printf("Press ENTER to end this program");
      waitForEnterKey();
      exit(0);
   }

   // write to console
   if(strstr(fmt, strHack) == NULL)
   {
      va_start(args, fmt);
      n2 = vfprintf(stdout, fmt, args);
      va_end(args);
   }

   // write to log file
   strcpy_s(fmt2, 1024, fmt);
   if(strstr(fmt2, strHack) != NULL)
   {
      char *p = strstr(fmt2, strHack); // cut off the appended hack string
      if(p) *p = 0;
   }
   fmt = fmt2;  // must be const pointer!

   va_start(args, fmt);
   n1 = vfprintf(flog, fmt, args);
   va_end(args);

   fclose(flog); // close the file

   // fix glitch in number of characters returned
   if(n2 < n1) n1 = n2;
   return n1;
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  checks if two RGB colors are identical
// ARGUMENTS:    color1, color2:  the two RGB colors
// RETURN VALUE: true if identical, false if not
bool areSameColor(RGB color1, RGB color2)
{
   // must have identical components
   return (color1.r == color2.r && color1.g == color2.g && color1.b == color2.b);
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  makes a string all upper case characters
// ARGUMENTS:    str:  the string memory address
// RETURN VALUE: none
void makeStringUpperCase(char *str)
{
   if(str == NULL) return; // safety!

   for(size_t i = 0; i < strlen(str); i++) str[i] = (char)toupper(str[i]);
}


//-----------------------------------------------------------------------------------------------------------
// DESCRIPTION:  sets the pen position of the robot
// ARGUMENTS:    penPos:  the pen position index (PEN_UP or PEN_DOWN)
// RETURN VALUE: none
void setPenPos(int penPos)
{
   if(penPos == PEN_UP)  // set the pen to the up position
      sendRobotCommand("PEN_UP\n");
   else if(penPos == PEN_DOWN)  // set the pen to the down position
      sendRobotCommand("PEN_DOWN\n");
   else // unkown position
      printf("Bad value (%d) for pen positiion\n", penPos);
}

//---------------------------------------------------------------------------------------------------------------------
// Tranforms a 2D coordinate based on the current tranformation matrix
// INPUTS:  TM: the 3x3 transform matrix
//          pt: the untransformed point
// RETURN:  The transformed point
POINT2D transform(double TM[][3], POINT2D pt)
{
   POINT2D tpt = {0.0};  // transformed tool position coordinates

   // matrix multiply transformation
   tpt.x = pt.x * TM[0][0] + pt.y * TM[0][1] + TM[0][2];
   tpt.y = pt.x * TM[1][0] + pt.y * TM[1][1] + TM[1][2];

   return tpt;
}

//---------------------------------------------------------------------------------------------------------------------
// Resets the tranform matrix to the unit matrix.  x, y points will no longer be transformed in inverseKinematics
// INPUTS:  the 3x3 transform matrix
// RETURN:  nothing
void resetTransformMatrix(double TM[][3])  // resets to unit matrix
{
   int r, c;  // matrix row, column indexes

   for(r = 0; r < 3; r++)
   {
      for(c = 0; c < 3; c++)
      {
         TM[r][c] = (r == c ? 1.0 : 0.0);
      }
   }
}

//---------------------------------------------------------------------------------------------------------------------
// Premultiplies the transform matrix by matrix M.  M is the rotation matrix, translation matrix, or the scaling matrix
// INPUTS:  TM: the 3x3 transform matrix, M the premultiplier matrix
// RETURN:  nothing
void transformMatrixMultiply(double TM[][3], double M[][3])
{
   int r, c, cc;     // row, column indexes
   double TMM[3][3] = {0}; // temp matrix

   for(r = 0; r < 3; r++)
   {
      for(c = 0; c < 3; c++)
      {
         TMM[r][c] = 0.0;  // set element to zero

         for(cc = 0; cc < 3; cc++) // accumulate the multiples
         {
            TMM[r][c] += M[r][cc] * TM[cc][c];
         }
      }
   }

   for(r = 0; r < 3; r++)  // copy temp matrix to TM
   {
      for(c = 0; c < 3; c++)
      {
         TM[r][c] = TMM[r][c];
      }
   }
}

bool addScaling(char *strCommand, int nLine, double TM[3][3])
{
   bool bSuccess = true;
   char *tok = NULL, *nextTok = NULL;                    // for tokenizing strCommand
   const char *seps = " \t\n,;";                         // tokenizing delimiters
   size_t i=0;                                             // index
   char msg1[MAX_BUFF], msg2[MAX_BUFF];                  // for composing error message strings
   const char *strScale[2] = {"sx","sy"};        // descriptions
   double s[2] = {0.}, scaleMatrix[3][3] = {NAN};

   tok = strtok_s(strCommand, seps, &nextTok); // discard SCALE keyword


   for(i = 0; i < 2; i++)
   {
      sprintf_s(msg1, MAX_BUFF, "Error! Missing data for %s on line %d\n", strScale[i], nLine);
      sprintf_s(msg2, MAX_BUFF, "Error! Garbage found in %s on line %d\n", strScale[i], nLine);
      tok = strtok_s(NULL, seps, &nextTok); // get the token
      s[i] = tokenToDouble(tok, msg1, msg2);
      if(isnan(s[i])) endProgram();
   }


      memcpy(scaleMatrix, (double[3][3]) { { s[0], 0., 0. }, {0., s[1], 0.}, {0., 0., 1.} }, sizeof(scaleMatrix));
      transformMatrixMultiply(TM, scaleMatrix);
  
   return true;
}

bool addRotation(char *strCommand, int nLine, double TM[3][3])
{
   bool bSuccess = true;
   char *tok = NULL, *nextTok = NULL;                    // for tokenizing strCommand
   const char *seps = " \t\n,;";                         // tokenizing delimiters
   size_t i = 0;                                             // index
   char msg1[MAX_BUFF], msg2[MAX_BUFF];                  // for composing error message strings
   const char *strRotate = "rDeg";        // descriptions
   double rDeg = {0.}, rotateMatrix[3][3] = {NAN};
      
   tok = strtok_s(strCommand, seps, &nextTok); // discard rotate keyword


      sprintf_s(msg1, MAX_BUFF, "Error! Missing data for %s on line %d\n", strRotate, nLine);
      sprintf_s(msg2, MAX_BUFF, "Error! Garbage found in %s on line %d\n", strRotate, nLine);
      tok = strtok_s(NULL, seps, &nextTok); // get the token
      rDeg = tokenToDouble(tok, msg1, msg2);
      if(isnan(rDeg)) endProgram();
  


   memcpy(rotateMatrix, (double[3][3]) { { cos(degToRad(rDeg)), -sin(degToRad(rDeg)),0. },
      {sin(degToRad(rDeg)), cos(degToRad(rDeg)), 0.}, {0., 0., 1.} }, sizeof(rotateMatrix));
   transformMatrixMultiply(TM, rotateMatrix);
  

      return true;
   }

  
bool addTranslation(char *strCommand, int nLine, double TM[3][3])
{
   bool bSuccess = true;
   char *tok = NULL, *nextTok = NULL;                    // for tokenizing strCommand
   const char *seps = " \t\n,;";                         // tokenizing delimiters
   size_t i = 0;                                             // index
   char msg1[MAX_BUFF], msg2[MAX_BUFF];                  // for composing error message strings
   const char *strtranslate[2] = {"tx","ty"};        // descriptions
   double t[2] = {0.}, translateMatrix[3][3] = {NAN};

   tok = strtok_s(strCommand, seps, &nextTok); // discard translate keyword


   for(i = 0; i < 2; i++)
   {
      sprintf_s(msg1, MAX_BUFF, "Error! Missing data for %s on line %d\n", strtranslate[i], nLine);
      sprintf_s(msg2, MAX_BUFF, "Error! Garbage found in %s on line %d\n", strtranslate[i], nLine);
      tok = strtok_s(NULL, seps, &nextTok); // get the token
      t[i] = tokenToDouble(tok, msg1, msg2);
      if(isnan(t[i])) endProgram();
   }


   memcpy(translateMatrix, (double[3][3]) { { 1., 0., t[0] }, {0., 1., t[1]}, {0., 0., 1.} }, sizeof(translateMatrix));
   transformMatrixMultiply(TM, translateMatrix);

   return true;
  
}