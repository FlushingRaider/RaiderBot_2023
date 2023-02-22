/*
  Lookup.cpp

  Created on: Jan 3, 2020
  Author: 5561

  This file contains functions related to lookup and interpolation.

 */
#include "Const.hpp"
#include "Pathloader.hpp"
#include <math.h>

AutonPath path;

/******************************************************************************
 * Function:     LookUp1D_Table
 *
 * Description:  Single dimension lookup table.
 ******************************************************************************/
double LookUp1D_Table(const double *LKeLU_Cmd_XAxis,
                      const double *LKLU_Cmd_TableData1D,
                      int LeLU_Int_AxisSize,
                      int LaLU_CalArraySize,
                      double LeLU_Cmd_Input)
{
  int LeLU_Int_Index = 0;
  double LeLU_Int_LookupX1 = 0.0;
  double LeLU_Int_LookupX2 = 0.0;
  double LeLU_Int_LookupXDiff = 0.0;
  double LeLU_Int_LookupY1 = 0.0;
  double LeLU_Int_LookupY2 = 0.0;
  double LeLU_Int_LookupYDiff = 0.0;
  double LeLU_Int_LookupDiv = 0.0;
  bool LeLU_b_LookUpPt1Found = false;
  double LeLU_Int_Output = 0.0;

  /* Table length MUST equal axis length. */
  if (LaLU_CalArraySize == LeLU_Int_AxisSize)
  {
    if (LeLU_Cmd_Input >= (LKeLU_Cmd_XAxis[LeLU_Int_AxisSize - 1]))
    {
      // We have gone off or are at the end of the axis
      return (LKLU_Cmd_TableData1D[LeLU_Int_AxisSize - 1]);
    }
    else if (LeLU_Cmd_Input <= (LKeLU_Cmd_XAxis[0]))
    {
      // We have gone off or are at the beginning of the axis
      return (LKLU_Cmd_TableData1D[0]);
    }
    else
    {
      for (LeLU_Int_Index = 0; ((LeLU_Int_Index < (LeLU_Int_AxisSize - 1)) && (LeLU_b_LookUpPt1Found == false)); LeLU_Int_Index++)
      {
        if ((LeLU_Cmd_Input >= LKeLU_Cmd_XAxis[LeLU_Int_Index]) &&
            (LeLU_Cmd_Input < LKeLU_Cmd_XAxis[LeLU_Int_Index + 1]) &&
            (LeLU_b_LookUpPt1Found == false))
        {
          LeLU_Int_LookupX1 = LKeLU_Cmd_XAxis[LeLU_Int_Index];
          LeLU_Int_LookupY1 = LKLU_Cmd_TableData1D[LeLU_Int_Index];
          LeLU_Int_LookupX2 = LKeLU_Cmd_XAxis[LeLU_Int_Index + 1];
          LeLU_Int_LookupY2 = LKLU_Cmd_TableData1D[LeLU_Int_Index + 1];
          LeLU_b_LookUpPt1Found = true;

          LeLU_Int_Index = LeLU_Int_AxisSize;
        }
      }

      if ((LeLU_b_LookUpPt1Found == true))
      {
        LeLU_Int_LookupXDiff = LeLU_Int_LookupX2 - LeLU_Int_LookupX1;
        LeLU_Int_LookupYDiff = LeLU_Int_LookupY2 - LeLU_Int_LookupY1;
        if (LeLU_Int_LookupXDiff != 0.0)
        {
          /* Protect for zero division */
          LeLU_Int_LookupDiv = LeLU_Int_LookupYDiff / LeLU_Int_LookupXDiff;
        }
        else
        {
          LeLU_Int_LookupDiv = 0.0;
        }
        LeLU_Int_Output = LeLU_Int_LookupY1 + (LeLU_Cmd_Input - LeLU_Int_LookupX1) * LeLU_Int_LookupDiv;

        return LeLU_Int_Output;
      }
    }
  }

  // Not in range...
  return 0;
}

/******************************************************************************
 * Function:     RampTo
 *
 * Description:  Function to ramp from one value to another.
 *****************************************************************************/
double RampTo(double LeLU_Cmd_Final,
              double LeLU_Cmd_Current,
              double LeLU_Cmd_Slope)
{
  if (LeLU_Cmd_Final - LeLU_Cmd_Current > 0)
  {
    LeLU_Cmd_Current += LeLU_Cmd_Slope;

    if (LeLU_Cmd_Current >= LeLU_Cmd_Final)
    {
      LeLU_Cmd_Current = LeLU_Cmd_Final;
    }
  }
  else if (LeLU_Cmd_Final - LeLU_Cmd_Current < 0)
  {
    LeLU_Cmd_Current -= LeLU_Cmd_Slope;
    if (LeLU_Cmd_Current <= LeLU_Cmd_Final)
    {
      LeLU_Cmd_Current = LeLU_Cmd_Final;
    }
  }
  else
  {
    LeLU_Cmd_Current = LeLU_Cmd_Final;
  }
  return (LeLU_Cmd_Current);
}

/******************************************************************************
 * Function:     LookUp1D_Axis
 *
 * Description:  Single axis lookup.
 ******************************************************************************/
void LookUp1D_Axis(const double *LKLU_Cmd_Axis,
                   int LeLU_Int_AxisSize,
                   int *LeLU_Int_Index_i,
                   int *LeLU_Int_Index_j,
                   double LeLU_Cmd_Input,
                   double *LeLU_Cmd_InputScalar,
                   double *LeLU_Cmd_InputScalar1Minus)
{
  int LeLU_Int_Index = 0;
  bool LeLU_b_LookUpPt1Found = false;
  double LeLU_Int_Denomenator = 0.0;

  if (LeLU_Cmd_Input >= (LKLU_Cmd_Axis[LeLU_Int_AxisSize - 1]))
  {
    // We have gone off or are at the end of the axis
    *LeLU_Int_Index_i = LeLU_Int_AxisSize - 2;
    *LeLU_Int_Index_j = LeLU_Int_AxisSize - 1;
    *LeLU_Cmd_InputScalar = 1;
    *LeLU_Cmd_InputScalar1Minus = 0;
  }
  else if (LeLU_Cmd_Input <= (LKLU_Cmd_Axis[0]))
  {
    // We have gone off or are at the beginning of the axis
    *LeLU_Int_Index_i = 0;
    *LeLU_Int_Index_j = 1;
    *LeLU_Cmd_InputScalar = 0;
    *LeLU_Cmd_InputScalar1Minus = 1;
  }
  else
  {
    for (LeLU_Int_Index = 0; ((LeLU_Int_Index < (LeLU_Int_AxisSize - 1)) && (LeLU_b_LookUpPt1Found == false)); LeLU_Int_Index++)
    {
      if ((LeLU_Cmd_Input >= LKLU_Cmd_Axis[LeLU_Int_Index]) &&
          (LeLU_Cmd_Input < LKLU_Cmd_Axis[LeLU_Int_Index + 1]) &&
          (LeLU_b_LookUpPt1Found == false))
      {
        LeLU_b_LookUpPt1Found = true;
        *LeLU_Int_Index_i = LeLU_Int_Index;
        *LeLU_Int_Index_j = LeLU_Int_Index + 1;
        LeLU_Int_Denomenator = LKLU_Cmd_Axis[LeLU_Int_Index + 1] - LKLU_Cmd_Axis[LeLU_Int_Index];
        if (LeLU_Int_Denomenator != 0.0)
        {
          /* Protect for zero division */
          *LeLU_Cmd_InputScalar1Minus = (LKLU_Cmd_Axis[LeLU_Int_Index + 1] - LeLU_Cmd_Input) / (LeLU_Int_Denomenator);
        }
        else
        {
          *LeLU_Cmd_InputScalar1Minus = 1;
        }
        *LeLU_Cmd_InputScalar = 1 - *LeLU_Cmd_InputScalar1Minus;
      }
    }

    if (LeLU_b_LookUpPt1Found == false)
    {
      /* Defensive programming.  We really shouldn't reach here... */
      *LeLU_Int_Index_i = 0;
      *LeLU_Int_Index_j = 1;
      *LeLU_Cmd_InputScalar = 0;
      *LeLU_Cmd_InputScalar1Minus = 1 - *LeLU_Cmd_InputScalar;
    }
  }
}

/******************************************************************************
 * Function:     LookUp2D_Table
 *
 * Description:  Two dimension lookup table.  Based on the linear interpolation
 *               at the link below:
 *               https://en.wikipedia.org/wiki/Bilinear_interpolation
 ******************************************************************************/
double LookUp2D_Table(double const *LKeLU_Cmd_XAxis,
                      int LeLU_Cmd_XAxisSize,
                      double LeLU_Cmd_X_Input,
                      double const *LKeLU_Cmd_Y_Axis,
                      int LeLU_Cmd_Y_AxisSize,
                      double LeLU_Cmd_Y_Input,
                      double **LeLU_Cmd_TableData2D)
{
  int LeLU_Int_X_Index_i = 0;
  int LeLU_Int_X_Index_j = 0;
  double LeLU_Int_X_IndexScalar = 0.0;
  double LeLU_Int_X_IndexScalar1Minus = 0.0;
  int LeLU_Int_Y_Index_i = 0;
  int LeLU_Int_Y_Index_j = 0;
  double LeLU_Int_Y_IndexScalar = 0.0;
  double LeLU_Int_Y_IndexScalar1Minus = 0.0;
  double LeLU_Int_F_XiYi = 0.0;
  double LeLU_Int_F_XiYj = 0.0;
  double LeLU_Int_F_XjYi = 0.0;
  double LeLU_Int_F_XjYj = 0.0;
  double LeLU_Int_Output = 0.0;

  LookUp1D_Axis(&LKeLU_Cmd_XAxis[0],
                LeLU_Cmd_XAxisSize,
                &LeLU_Int_X_Index_i,
                &LeLU_Int_X_Index_j,
                LeLU_Cmd_X_Input,
                &LeLU_Int_X_IndexScalar,
                &LeLU_Int_X_IndexScalar1Minus);

  LookUp1D_Axis(&LKeLU_Cmd_Y_Axis[0],
                LeLU_Cmd_Y_AxisSize,
                &LeLU_Int_Y_Index_i,
                &LeLU_Int_Y_Index_j,
                LeLU_Cmd_Y_Input,
                &LeLU_Int_Y_IndexScalar,
                &LeLU_Int_Y_IndexScalar1Minus);

  LeLU_Int_F_XiYi = LeLU_Cmd_TableData2D[(int)LeLU_Int_X_Index_i][(int)LeLU_Int_Y_Index_i];
  LeLU_Int_F_XiYj = LeLU_Cmd_TableData2D[(int)LeLU_Int_X_Index_i][(int)LeLU_Int_Y_Index_j];
  LeLU_Int_F_XjYi = LeLU_Cmd_TableData2D[(int)LeLU_Int_X_Index_j][(int)LeLU_Int_Y_Index_i];
  LeLU_Int_F_XjYj = LeLU_Cmd_TableData2D[(int)LeLU_Int_X_Index_j][(int)LeLU_Int_Y_Index_j];

  LeLU_Int_Output = LeLU_Int_F_XiYi * LeLU_Int_X_IndexScalar1Minus * LeLU_Int_Y_IndexScalar1Minus +
                    LeLU_Int_F_XjYi * LeLU_Int_X_IndexScalar * LeLU_Int_Y_IndexScalar1Minus +
                    LeLU_Int_F_XiYj * LeLU_Int_X_IndexScalar1Minus * LeLU_Int_Y_IndexScalar +
                    LeLU_Int_F_XjYj * LeLU_Int_X_IndexScalar * LeLU_Int_Y_IndexScalar;

  return (LeLU_Int_Output);
}

/******************************************************************************
 * Function:     DesiredAutonLocation2
 *
 * Description:  Determine the desired X/Y location based on the current time.
 ******************************************************************************/
bool DesiredAutonLocation2(double LeLU_s_AutonTime,
                           int LeLU_Int_AutonSelection,
                           std::string V_ADAS_Auto_PathName,
                           double *LeLU_Cmd_L_X_Location,
                           double *LeLU_Cmd_L_Y_Location,
                           double *LeLU_Cmd_Deg_Angle)
{
  double LeLU_Int_L_X_Loc = 0.0;
  double LeLU_Cmd_L_Y_Loc = 0.0;
  double LeLU_Deg_Ang = 0.0;
  int LeLU_Int_X_AxisSize = 0;
  int LeLU_Int_X_CalArraySize = 0;
  int LeLU_Int_Y_AxisSize = 0;
  int LeLU_Inr_Y_CalArraySize = 0;
  int LeLU_Int_Ang_AxisSize = 0;
  int LeLU_Int_Ang_CalArraySize = 0;
  bool LeLU_b_timeTableDONE = false;

  switch (LeLU_Int_AutonSelection)
  {
  case 99: // Auto load path from json files BROKEN!!!!
    path = PathLoader(V_ADAS_Auto_PathName);

    // Path is a struct that contains vectors. Vectors will not return the correct size with sizeof() but .size() will.
    LeLU_Int_X_AxisSize = (int)(sizeof(path.time.size()) / sizeof(path.x[0]));
    LeLU_Int_X_CalArraySize = (int)(sizeof(path.x.size()) / sizeof(path.x[0]));
    LeLU_Int_Y_AxisSize = (int)(sizeof(path.time.size()) / sizeof(path.y[0]));
    LeLU_Inr_Y_CalArraySize = (int)(sizeof(path.y.size()) / sizeof(path.y[0]));
    LeLU_Int_Ang_AxisSize = (int)(sizeof(path.time.size()) / sizeof(path.rot[0]));
    LeLU_Int_Ang_CalArraySize = (int)(sizeof(path.rot.size()) / sizeof(path.rot[0]));

    LeLU_Int_L_X_Loc = LookUp1D_Table(&path.time[0],
                                      &path.x[0],
                                      LeLU_Int_X_AxisSize,
                                      LeLU_Int_X_CalArraySize,
                                      LeLU_s_AutonTime);

    LeLU_Cmd_L_Y_Loc = LookUp1D_Table(&path.time[0],
                                      &path.y[0],
                                      LeLU_Int_Y_AxisSize,
                                      LeLU_Inr_Y_CalArraySize,
                                      LeLU_s_AutonTime);

    LeLU_Deg_Ang = LookUp1D_Table(&path.time[0],
                                  &path.rot[0],
                                  LeLU_Int_Ang_AxisSize,
                                  LeLU_Int_Ang_CalArraySize,
                                  LeLU_s_AutonTime);

    if (LeLU_s_AutonTime >= path.time[LeLU_Int_X_AxisSize - 1])
    {
      LeLU_b_timeTableDONE = true;
    }

    break;

  case 1:
    LeLU_Int_X_AxisSize = (int)(sizeof(K_t_ADAS_DM_Red_1T) / sizeof(K_l_ADAS_DM_Red_1X[0]));
    LeLU_Int_X_CalArraySize = (int)(sizeof(K_l_ADAS_DM_Red_1X) / sizeof(K_l_ADAS_DM_Red_1X[0]));
    LeLU_Int_Y_AxisSize = (int)(sizeof(K_t_ADAS_DM_Red_1T) / sizeof(K_l_ADAS_DM_Red_1Y[0]));
    LeLU_Inr_Y_CalArraySize = (int)(sizeof(K_l_ADAS_DM_Red_1Y) / sizeof(K_l_ADAS_DM_Red_1Y[0]));
    LeLU_Int_Ang_AxisSize = (int)(sizeof(K_t_ADAS_DM_Red_1T) / sizeof(K_rad_ADAS_DM_Red_1Ang[0]));
    LeLU_Int_Ang_CalArraySize = (int)(sizeof(K_rad_ADAS_DM_Red_1Ang) / sizeof(K_rad_ADAS_DM_Red_1Ang[0]));

    LeLU_Int_L_X_Loc = LookUp1D_Table(&K_t_ADAS_DM_Red_1T[0],
                                      &K_l_ADAS_DM_Red_1X[0],
                                      LeLU_Int_X_AxisSize,
                                      LeLU_Int_X_CalArraySize,
                                      LeLU_s_AutonTime);

    LeLU_Cmd_L_Y_Loc = LookUp1D_Table(&K_t_ADAS_DM_Red_1T[0],
                                      &K_l_ADAS_DM_Red_1Y[0],
                                      LeLU_Int_Y_AxisSize,
                                      LeLU_Inr_Y_CalArraySize,
                                      LeLU_s_AutonTime);

    LeLU_Deg_Ang = LookUp1D_Table(&K_t_ADAS_DM_Red_1T[0],
                                  &K_rad_ADAS_DM_Red_1Ang[0],
                                  LeLU_Int_Ang_AxisSize,
                                  LeLU_Int_Ang_CalArraySize,
                                  LeLU_s_AutonTime);

    if (LeLU_s_AutonTime >= K_t_ADAS_DM_Red_1T[LeLU_Int_X_AxisSize - 1])
    {
      LeLU_b_timeTableDONE = true;
    }

    break;

  case 2:
    LeLU_Int_X_AxisSize = (int)(sizeof(K_t_ADAS_DM_Red_2T) / sizeof(K_l_ADAS_DM_Red_2X[0]));
    LeLU_Int_X_CalArraySize = (int)(sizeof(K_l_ADAS_DM_Red_2X) / sizeof(K_l_ADAS_DM_Red_2X[0]));
    LeLU_Int_Y_AxisSize = (int)(sizeof(K_t_ADAS_DM_Red_2T) / sizeof(K_l_ADAS_DM_Red_2Y[0]));
    LeLU_Inr_Y_CalArraySize = (int)(sizeof(K_l_ADAS_DM_Red_2Y) / sizeof(K_l_ADAS_DM_Red_2Y[0]));
    LeLU_Int_Ang_AxisSize = (int)(sizeof(K_t_ADAS_DM_Red_2T) / sizeof(K_rad_ADAS_DM_Red_2Ang[0]));
    LeLU_Int_Ang_CalArraySize = (int)(sizeof(K_rad_ADAS_DM_Red_2Ang) / sizeof(K_rad_ADAS_DM_Red_2Ang[0]));

    LeLU_Int_L_X_Loc = LookUp1D_Table(&K_t_ADAS_DM_Red_2T[0],
                                      &K_l_ADAS_DM_Red_2X[0],
                                      LeLU_Int_X_AxisSize,
                                      LeLU_Int_X_CalArraySize,
                                      LeLU_s_AutonTime);

    LeLU_Cmd_L_Y_Loc = LookUp1D_Table(&K_t_ADAS_DM_Red_2T[0],
                                      &K_l_ADAS_DM_Red_2Y[0],
                                      LeLU_Int_Y_AxisSize,
                                      LeLU_Inr_Y_CalArraySize,
                                      LeLU_s_AutonTime);

    LeLU_Deg_Ang = LookUp1D_Table(&K_t_ADAS_DM_Red_2T[0],
                                  &K_rad_ADAS_DM_Red_2Ang[0],
                                  LeLU_Int_Ang_AxisSize,
                                  LeLU_Int_Ang_CalArraySize,
                                  LeLU_s_AutonTime);

    if (LeLU_s_AutonTime >= K_t_ADAS_DM_Red_2T[LeLU_Int_X_AxisSize - 1])
    {
      LeLU_b_timeTableDONE = true;
    }

    break;

  case 3:

    LeLU_Int_X_AxisSize = (int)(sizeof(K_t_ADAS_DM_Red_3T) / sizeof(K_l_ADAS_DM_Red_3X[0]));
    LeLU_Int_X_CalArraySize = (int)(sizeof(K_l_ADAS_DM_Red_3X) / sizeof(K_l_ADAS_DM_Red_3X[0]));
    LeLU_Int_Y_AxisSize = (int)(sizeof(K_t_ADAS_DM_Red_3T) / sizeof(K_l_ADAS_DM_Red_3Y[0]));
    LeLU_Inr_Y_CalArraySize = (int)(sizeof(K_l_ADAS_DM_Red_3Y) / sizeof(K_l_ADAS_DM_Red_3Y[0]));
    LeLU_Int_Ang_AxisSize = (int)(sizeof(K_t_ADAS_DM_Red_3T) / sizeof(K_rad_ADAS_DM_Red_3Ang[0]));
    LeLU_Int_Ang_CalArraySize = (int)(sizeof(K_rad_ADAS_DM_Red_3Ang) / sizeof(K_rad_ADAS_DM_Red_3Ang[0]));

    LeLU_Int_L_X_Loc = LookUp1D_Table(&K_t_ADAS_DM_Red_3T[0],
                                      &K_l_ADAS_DM_Red_3X[0],
                                      LeLU_Int_X_AxisSize,
                                      LeLU_Int_X_CalArraySize,
                                      LeLU_s_AutonTime);

    LeLU_Cmd_L_Y_Loc = LookUp1D_Table(&K_t_ADAS_DM_Red_3T[0],
                                      &K_l_ADAS_DM_Red_3Y[0],
                                      LeLU_Int_Y_AxisSize,
                                      LeLU_Inr_Y_CalArraySize,
                                      LeLU_s_AutonTime);

    LeLU_Deg_Ang = LookUp1D_Table(&K_t_ADAS_DM_Red_3T[0],
                                  &K_rad_ADAS_DM_Red_3Ang[0],
                                  LeLU_Int_Ang_AxisSize,
                                  LeLU_Int_Ang_CalArraySize,
                                  LeLU_s_AutonTime);

    if (LeLU_s_AutonTime >= K_t_ADAS_DM_Red_3T[LeLU_Int_X_AxisSize - 1])
    {
      LeLU_b_timeTableDONE = true;
    }
    break;
  case 4:
    LeLU_Int_X_AxisSize = (int)(sizeof(TestPath1_T) / sizeof(TestPath1_X[0]));
    LeLU_Int_X_CalArraySize = (int)(sizeof(TestPath1_X) / sizeof(TestPath1_X[0]));

    LeLU_Int_Y_AxisSize = (int)(sizeof(TestPath1_T) / sizeof(TestPath1_Y[0]));
    LeLU_Inr_Y_CalArraySize = (int)(sizeof(TestPath1_Y) / sizeof(TestPath1_Y[0]));
    
    LeLU_Int_Ang_AxisSize = (int)(sizeof(TestPath1_T) / sizeof(TestPath1_ROT[0]));
    LeLU_Int_Ang_CalArraySize = (int)(sizeof(TestPath1_ROT) / sizeof(TestPath1_ROT[0]));

    LeLU_Int_L_X_Loc = LookUp1D_Table(&TestPath1_T[0],
                                      &TestPath1_X[0],
                                      LeLU_Int_X_AxisSize,
                                      LeLU_Int_X_CalArraySize,
                                      LeLU_s_AutonTime);

    LeLU_Cmd_L_Y_Loc = LookUp1D_Table(&TestPath1_T[0],
                                      &TestPath1_Y[0],
                                      LeLU_Int_Y_AxisSize,
                                      LeLU_Inr_Y_CalArraySize,
                                      LeLU_s_AutonTime);

    LeLU_Deg_Ang = LookUp1D_Table(&TestPath1_T[0],
                                  &TestPath1_ROT[0],
                                  LeLU_Int_Ang_AxisSize,
                                  LeLU_Int_Ang_CalArraySize,
                                  LeLU_s_AutonTime);

    if (LeLU_s_AutonTime >= TestPath1_T[LeLU_Int_X_AxisSize - 1])
    {
      LeLU_b_timeTableDONE = true;
    }
    break;
  default:  
    break;
  }

  *LeLU_Cmd_L_X_Location = LeLU_Cmd_L_Y_Loc;
  *LeLU_Cmd_L_Y_Location = LeLU_Int_L_X_Loc;
  *LeLU_Cmd_Deg_Angle = LeLU_Deg_Ang;

  return (LeLU_b_timeTableDONE);
}

/******************************************************************************
 * Function:     DesiredAutonLocation
 *
 * Description:  Determine the desired X/Y location based on the current time.
 ******************************************************************************/
void DesiredAutonLocation(double LeLU_s_AutonTime,
                          int LeLU_Int_AutonSelection,
                          double *LeLU_Cmd_L_X_Location,
                          double *LeLU_Cmd_L_Y_Location)
{
  // double LeLU_Int_L_X_Loc = 0.0;
  // double LeLU_Cmd_L_Y_Loc = 0.0;
  // int LeLU_Int_X_AxisSize             = (int)(sizeof(K_t_Slalom_V125A50_T) / sizeof(K_l_Slalom_V125A50_X[0]));
  // int LeLU_Int_X_CalArraySize         = (int)(sizeof(K_l_Slalom_V125A50_X) / sizeof(K_l_Slalom_V125A50_X[0]));
  // int LeLU_Int_Y_AxisSize             = (int)(sizeof(K_t_Slalom_V125A50_T) / sizeof(K_l_Slalom_V125A50_Y[0]));
  // int LeLU_Inr_Y_CalArraySize         = (int)(sizeof(K_l_Slalom_V125A50_Y) / sizeof(K_l_Slalom_V125A50_Y[0]));

  // LeLU_Int_L_X_Loc = LookUp1D_Table(&K_t_Slalom_V125A50_T[0],
  //                            &K_l_Slalom_V125A50_X[0],
  //                             LeLU_Int_X_AxisSize,
  //                             LeLU_Int_X_CalArraySize,
  //                             LeLU_s_AutonTime);

  // LeLU_Cmd_L_Y_Loc = LookUp1D_Table(&K_t_Slalom_V125A50_T[0],
  //                            &K_l_Slalom_V125A50_Y[0],
  //                             LeLU_Int_Y_AxisSize,
  //                             LeLU_Inr_Y_CalArraySize,
  //                             LeLU_s_AutonTime);

  // *LeLU_Cmd_L_X_Location = LeLU_Int_L_X_Loc;
  // *LeLU_Cmd_L_Y_Location = LeLU_Cmd_L_Y_Loc;
}

/******************************************************************************
 * Function:     ScaleJoystickAxis
 *
 * Description:  Function to scale the joystick input.
 *               Primarily used for smooth debouncing.
 ******************************************************************************/
double ScaleJoystickAxis(double LeLU_Cmd_JoystickAxis)
{
  double LeLU_RPM_DesiredDriveSpeed = 0.0; // may or may not be RPM
  int LeLU_Int_AxisSize = (int)(sizeof(K_SD_DesiredDriveSpeedAxis) / sizeof(K_SD_DesiredDriveSpeed[0]));
  int LaLU_CalArraySize = (int)(sizeof(K_SD_DesiredDriveSpeed) / sizeof(K_SD_DesiredDriveSpeed[0]));

  LeLU_RPM_DesiredDriveSpeed = LookUp1D_Table(&K_SD_DesiredDriveSpeedAxis[0],
                                              &K_SD_DesiredDriveSpeed[0],
                                              LeLU_Int_AxisSize,
                                              LaLU_CalArraySize,
                                              LeLU_Cmd_JoystickAxis);

  return LeLU_RPM_DesiredDriveSpeed;
}

/****************************************************************************
 * Function:     DtrmnAutoLauncherSpeed
 *
 * Description:  Function to determine the desired launcher speed based on
 *               estimated distance from taget from vision.
 ******************************************************************************/
double DtrmnAutoLauncherSpeed(double LeLU_Cmd_TargetDistance)
{
  double LeLU_RPM_DesiredLaunchSpeed = 0.0; // may or may not be RPM
  int LeLU_Int_AxisSize = (int)(sizeof(K_BH_LauncherSpeedAxis) / sizeof(K_BH_LauncherSpeed[0]));
  int LaLU_CalArraySize = (int)(sizeof(K_BH_LauncherSpeed) / sizeof(K_BH_LauncherSpeed[0]));

  LeLU_RPM_DesiredLaunchSpeed = LookUp1D_Table(&K_BH_LauncherSpeedAxis[0],
                                               &K_BH_LauncherSpeed[0],
                                               LeLU_Int_AxisSize,
                                               LaLU_CalArraySize,
                                               LeLU_Cmd_TargetDistance);

  return LeLU_RPM_DesiredLaunchSpeed;
}

/******************************************************************************
 * Function:     DtrmnTimeToDriveToCaptureBall
 *
 * Description:  Function to determine the amount of time to drive forward to
 *               capture the ball.
 ******************************************************************************/
double DtrmnTimeToDriveToCaptureBall(double LeLU_Cmd_EstTargetDistance)
{
  double LeLU_s_DesiredDriveTime = 0.0; // may or may not be seconds
  int LeLU_Int_AxisSize = (int)(sizeof(K_ADAS_BT_DriveTimeAxis) / sizeof(K_ADAS_BT_DriveTime[0]));
  int LaLU_CalArraySize = (int)(sizeof(K_ADAS_BT_DriveTime) / sizeof(K_ADAS_BT_DriveTime[0]));

  LeLU_s_DesiredDriveTime = LookUp1D_Table(&K_ADAS_BT_DriveTimeAxis[0],
                                           &K_ADAS_BT_DriveTime[0],
                                           LeLU_Int_AxisSize,
                                           LaLU_CalArraySize,
                                           LeLU_Cmd_EstTargetDistance);

  return LeLU_s_DesiredDriveTime;
}

/******************************************************************************
 * Function:     DesiredRotateSpeed
 *
 * Description:  Function to determine the speed at which to rotate the robot
 *               (for auto targeting and auto rotate).
 ******************************************************************************/
double DesiredRotateSpeed(double LeLU_Cmd_Error)
{
  double LeLU_RPM_DesiredRotateSpeed = 0.0; // may or may not be RPM
  int LeLU_Int_AxisSize = (int)(sizeof(K_DesiredRotateSpeedAxis) / sizeof(K_DesiredRotateSpeed[0]));
  int LaLU_CalArraySize = (int)(sizeof(K_DesiredRotateSpeed) / sizeof(K_DesiredRotateSpeed[0]));

  LeLU_RPM_DesiredRotateSpeed = LookUp1D_Table(&K_DesiredRotateSpeedAxis[0],
                                               &K_DesiredRotateSpeed[0],
                                               LeLU_Int_AxisSize,
                                               LaLU_CalArraySize,
                                               LeLU_Cmd_Error);

  return LeLU_RPM_DesiredRotateSpeed;
}

/******************************************************************************
 * Function:     DesiredAutoRotateSpeed
 *
 * Description:  Function to determine the speed at which to rotate the robot
 *               (for auto targeting and auto rotate).
 ******************************************************************************/
double DesiredAutoRotateSpeed(double LeLU_Cmd_Error)
{
  double LeLU_RPM_DesiredRotateSpeed = 0.0;
  int LeLU_Int_AxisSize = (int)(sizeof(K_DesiredAutoRotateSpeedAxis) / sizeof(K_DesiredAutoRotateSpeed[0]));
  int LaLU_CalArraySize = (int)(sizeof(K_DesiredAutoRotateSpeed) / sizeof(K_DesiredAutoRotateSpeed[0]));

  LeLU_RPM_DesiredRotateSpeed = LookUp1D_Table(&K_DesiredAutoRotateSpeedAxis[0],
                                               &K_DesiredAutoRotateSpeed[0],
                                               LeLU_Int_AxisSize,
                                               LaLU_CalArraySize,
                                               LeLU_Cmd_Error);

  return LeLU_RPM_DesiredRotateSpeed;
}

/******************************************************************************
 * Function:     DesiredRollerSpeed
 *
 * Description:  Function to determine the roller speed, aka the "special
 *               beam cannon".  This is a function of distance out from the
 *               target and the angle of the robot relative to the target.
 *               We also look up the ideal robot angle for targeting.
 *
 *               Not currently used
 ******************************************************************************/
void DesiredRollerSpeed(double LeLU_In_Distance, // may or may not be inches
                        double LeLU_Deg_Angle,   // may or may not be degrees
                        double *LeLU_Cmd_UpperCmnd,
                        double *LeLU_Cmd_LowerCmnd)
{
  // double  L_DesiredRobotAngle       = 0.0;
  // double  L_DesiredRollerSpeedUpper = 0.0;
  // double  L_DesiredRollerSpeedLower = 0.0;
  // double *L_RollerSpeedCalibration[K_BallLauncherDistanceSz];
  // double  L_Temp[K_BallLauncherDistanceSz][K_BallLauncherAngleSz];
  // int     i;
  // int     j;

  // for (i = 0; i < K_BallLauncherDistanceSz; i++)
  //   {
  //   for (j = 0; j < K_BallLauncherAngleSz; j++)
  //     {
  //     L_Temp[i][j] = K_BallLauncherUpperSpeed[i][j];
  //     }
  //   L_RollerSpeedCalibration[i] = L_Temp[i];
  //   }

  // L_DesiredRollerSpeedUpper = LookUp2D_Table(&K_BallLauncherDistanceAxis[0],
  //                                             K_BallLauncherDistanceSz,
  //                                             LeLU_In_Distance,
  //                                            &K_BallLauncherAngleAxis[0],
  //                                             K_BallLauncherAngleSz,
  //                                             LeLU_Deg_Angle,
  //                                             L_RollerSpeedCalibration);

  // for (i = 0; i < K_BallLauncherDistanceSz; i++)
  //   {
  //   for (j = 0; j < K_BallLauncherAngleSz; j++)
  //     {
  //     L_Temp[i][j] = K_BallLauncherLowerSpeed[i][j];
  //     }
  //   L_RollerSpeedCalibration[i] = L_Temp[i];
  //   }

  // L_DesiredRollerSpeedLower = LookUp2D_Table(&K_BallLauncherDistanceAxis[0],
  //                                             K_BallLauncherDistanceSz,
  //                                             LeLU_In_Distance,
  //                                            &K_BallLauncherAngleAxis[0],
  //                                             K_BallLauncherAngleSz,
  //                                             LeLU_Deg_Angle,
  //                                             L_RollerSpeedCalibration);

  // *LeLU_Cmd_UpperCmnd  = L_DesiredRollerSpeedUpper;
  // *LeLU_Cmd_LowerCmnd  = L_DesiredRollerSpeedLower;
}
