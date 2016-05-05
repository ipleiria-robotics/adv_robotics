/*
Copyright (c) 2013, Hugo Costelha, based on the example provided in
http://docs.opencv.org/doc/tutorials/features2d/trackingmotion/harris_detector/harris_detector.html
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

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

/// Global variables
cv::Mat src, src_gray;
int thresh = 200;
int max_thresh = 255;

#define CORNERS_WINDOW "Corners detected"
#define HARRIS_WINDOW "Harris result (scaled)"

/// Function header
void cornerHarris_demo(int, void * );

/** @function main */
int main( int argc, char** argv )
{
  /// Load source image and convert
  src = cv::imread( "squares.jpg", 1 );
//  src = cv::imread( "building.jpg", 1 );

  /// Convert original image to greyscale
  cv::cvtColor( src, src_gray, CV_BGR2GRAY );

  /// Create a window and a trackbar
  cv::namedWindow( CORNERS_WINDOW, 0 );
  cv::createTrackbar( "Threshold: ", CORNERS_WINDOW, &thresh, max_thresh, cornerHarris_demo );

  cornerHarris_demo( 0, 0 );

  cv::waitKey(0);
  return(0);
}

/** @function cornerHarris_demo */
void cornerHarris_demo( int, void* )
{

  cv::Mat org_copy, dst, dst_norm, dst_norm_scaled;
  dst = cv::Mat::zeros( src.size(), CV_32FC1 );

  /// Detector parameters
  int blockSize = 2;
  int apertureSize = 3;
  double k = 0.04;

  /// Detecting corners
  cv::cornerHarris( src_gray, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT );

  /// Normalizing
  cv::normalize( dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat() );
  cv::convertScaleAbs( dst_norm, dst_norm_scaled );

  /// Drawing a circle around corners
  org_copy = src.clone();
  for( int j = 0; j < dst_norm.rows ; j++ )
  {
    for( int i = 0; i < dst_norm.cols; i++ )
    {
      if( (int) dst_norm.at<float>(j,i) > thresh )
      {
        cv::circle( org_copy, cv::Point( i, j ), 5,  cv::Scalar(0, 0, 255), 2, 8, 0 );
      }
    }
  }
  /// Showing the result
  // Harris result (scaled)
  cv::namedWindow( HARRIS_WINDOW, 0 );
  cv::imshow( HARRIS_WINDOW, dst_norm_scaled );
  // Corners detected
  cv::imshow( CORNERS_WINDOW, org_copy );
}
