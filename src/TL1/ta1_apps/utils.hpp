/*
Copyright (c) 2017, Hugo Costelha
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

#ifndef _UTILS_HPP_
#define _UTILS_HPP_

#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <iomanip>

/**
 * Show matrix values (for double values only)
 * @Argument mat - matrix whose values are to be shown
 */
void showMatValues(const cv::Mat &mat, std::string title = " ----> Mat values:")
{
  // For through each (i,j) position of the matrix and output its value.
  std::cout << title << "\n";
  for( int j=0; j < mat.size().height; j++)
  {
    for( int i=0; i < mat.size().width; i++ )
    {
      switch(mat.type())
      {
        case CV_8SC1:
          std::cout << std::setw(6) << (int)mat.at<char>(j, i) << " ";
          break;
        case CV_8UC1:
          std::cout << std::setw(6) << (int)mat.at<uchar>(j, i) << " ";
          break;
        case CV_16SC1:
          std::cout << std::setw(6) << mat.at<short>(j, i) << " ";
          break;
        case CV_16UC1:
          std::cout << std::setw(6) << mat.at<unsigned short>(j, i) << " ";
          break;
        case CV_32SC1:
          std::cout << std::setw(6) << mat.at<int>(j, i) << " ";
          break;
        case CV_64FC1:
          std::cout << std::setw(6) << mat.at<double>(j, i) << " ";
          break;
        default:
          std::cerr << "UNKNOWN MAT TYPE !!\n";
      }
    }
    std::cout << "\n";
  }
  std::cout << std::endl;
}

/**
 * Show matrix values (for any Mat_ type matrix)
 * @Argument mat - matrix whose values are to be shown
 */
template<typename _Tp>
void showMatValues(const cv::Mat_<_Tp> &mat, std::string title = " ----> Mat values:")
{
  // For through each (i,j) position of the matrix and output its value.
  typedef typename cv::DataType<_Tp>::work_type _wTp;
  std::cout << title << "\n";;
  for( int j=0; j < mat.size().height; j++)
  {
    for( int i=0; i < mat.size().width; i++ )
      std::cout << std::setw(6) << (_wTp) mat(j, i) << " ";
    std::cout << "\n";
  }
  std::cout << std::endl;
}

#endif //_UTILS_HPP_
