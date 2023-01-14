/*
  Filter.cpp

   Created on: March 10, 2022
   Author: Biggs

  Contains a number of filter functions. 
 */

// #include <math.h>


/******************************************************************************
 * Function:     Filter_FirstOrderLag
 *
 * Description:  First order lag filter.
 ******************************************************************************/
double Filter_FirstOrderLag(double L_RawValue,
                            double L_FilteredValue,
                            double L_FilterConst)
  {
    L_FilteredValue = L_FilterConst * L_RawValue + (1 - L_FilterConst) * L_FilteredValue;
  }