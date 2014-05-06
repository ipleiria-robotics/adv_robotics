/*
Copyright (c) 2010, Hugo Costelha.
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

/**
 * Main function
 */
int main(int argc, char** argv)
{
  // Load image
  cv::Mat image = cv::imread("baboon.jpg");

  // Show image
  cv::namedWindow("Exemplo 5", 0);
  cv::imshow("Exemplo 5", image);

  // Create image for the result
  cv::Mat image_res(image.size().height, image.size().width, CV_8UC3);

  ///
  /// Mean filter
  ///

  // Apply mean filter
  cv::blur(image, image_res, cv::Size(5,5));

  // Show image
  cv::namedWindow("Exemplo 5 - Mean filter", 0);
  cv::imshow("Exemplo 5 - Mean filter", image_res);

  ///
  /// Median filter
  ///

  // Apply mean filter
  cv::medianBlur(image, image_res, 5);

  // Show image
  cv::namedWindow("Exemplo 5 - Median filter", 0);
  cv::imshow("Exemplo 5 - Median filter", image_res);

  ///
  /// Gauss filter
  ///

  // Apply mean filter
  cv::GaussianBlur(image, image_res, cv::Size(5,5), 10);

  // Show image
  cv::namedWindow("Exemplo 5 - Gaussian filter", 0);
  cv::imshow("Exemplo 5 - Gaussian filter", image_res);


  // "Really" show the images and wait for the user input
  cv::waitKey(0);

  return 1;
}

