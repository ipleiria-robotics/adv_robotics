/*
Copyright (c) 2014, Hugo Costelha, based on Brad Kratochvil, Toby Collett, Brian
Gerkey and Andrew Howard example.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.
    * Neither the name of the Player Project nor the names of its contributors
      may be used to endorse or promote products derived from this software
      without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "utils.hpp"

#include <iostream>
#include <iomanip> // Manipulate output sream


void drawPostures(cv::Mat &image, double x, double y, double theta,
                  double resolution, cv::Scalar color)
{
  // Draw a circle in the robot position
  cv::circle(image,
             cv::Point(cvRound(image.size().width/2.0 + x/resolution),
                       cvRound(image.size().height/2.0 - y/resolution)),
             2, color, -1);
  // Draw a small line showing its orientation
  double delta_line = 5; // [pixels]
  cv::line(image,
           cv::Point(cvRound(image.size().width/2.0 + x/resolution),
                     cvRound(image.size().height/2.0 - y/resolution)),
           cv::Point(cvRound(image.size().width/2.0 + x/resolution + cos(theta)*delta_line),
                     cvRound(image.size().height/2.0 - y/resolution - sin(theta)*delta_line)),
           color,
           1);
}


void showMatValues(cv::Mat* matrix)
{
  switch(matrix->type())
  {
    case(CV_8UC1):
    {
      std::cout << std::right << std::fixed;
      //  For through each (i,j) position of the matrix and output its value.
      //  By using (int)matrix->at<uchar>(j, i) we have access to the i column
      // and j row of the matrix, as integer.
      std::cout << " ----> Image values \n\n";
      for( int j=0; j < matrix->size().height; j++)
      {
        for( int i=0; i < matrix->size().width; i++)
          std::cout << std::setw(5) << (int)matrix->at<uchar>(j, i) << " ";
        std::cout << "\n";
      }
      break;
    }

    case(CV_32FC1):
    {
      std::cout << std::setprecision( 3 ) << std::right << std::fixed;
      //  For through each (i,j) position of the matrix and output its value.
      //  By using (float)matrix->at<float>(j, i) we have access to the i column
      // and j row of the matrix, as float.
      std::cout << " ----> Image values \n\n";
      for( int j=0; j < matrix->size().height; j++)
      {
        for( int i=0; i < matrix->size().width; i++)
          std::cout << std::setw(5) << matrix->at<float>(j, i) << " ";
        std::cout << "\n";
      }
      break;
    }

    case(CV_64FC1):
    {
      std::cout << std::setprecision( 3 ) << std::right << std::fixed;
      //  For through each (i,j) position of the matrix and output its value.
      //  By using (double)matrix->at<double>(j, i) we have access to the i
      // column and j row of the matrix, as double.
      std::cout << " ----> Image values \n\n";
      for( int j=0; j < matrix->size().height; j++)
      {
        for( int i=0; i < matrix->size().width; i++)
          std::cout << std::setw(5) << matrix->at<double>(j, i) << " ";
        std::cout << "\n";
      }
      break;
    }

    default:
    {
      std::cerr << "Printing the requested type of matrix is not implemented!";
      break;
    }
  }
  std::cout << std::endl;
}

