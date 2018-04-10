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
  cv::Mat image = cv::imread("pic1.png");

  // Create image for convertion result with the same size of the original image
  // but only with one channel
  cv::Mat image_gray(image.size().height, image.size().width, CV_8UC1);

  // Convert from RGB to gray
  cv::cvtColor(image, image_gray, CV_RGB2GRAY);

  // Show image
  cv::namedWindow("Exemplo 4", 0);
  cv::imshow("Exemplo 4", image_gray);

  ///
  /// Sobel
  ///

  // Create the image to hold the Sobel result
  cv::Mat image_sobel(image.size().height, image.size().width, CV_8UC1);

  // Apply Sobel operator
  cv::Sobel(image_gray, image_sobel, image_sobel.depth(), 1, 1);

  // Show sobel
  cv::namedWindow("Exemplo 4 - Sobel", 0);
  cv::imshow("Exemplo 4 - Sobel", image_sobel);

  // Create binary image by applying threshold to the Sobel result
  cv::Mat image_sobel_bin(image.size().height, image.size().width, CV_8UC1);
  cv::threshold(image_sobel, image_sobel_bin, 50, 255, cv::THRESH_BINARY);

  // Show binary image
  cv::namedWindow("Exemplo 4 - Sobel - binary", 0);
  cv::imshow("Exemplo 4 - Sobel - binary", image_sobel_bin);

  ///
  /// Canny
  ///

  // Create the image to hold the Canny result
  cv::Mat image_canny(image.size().height, image.size().width, CV_8UC1);

  // Run Canny algorithm, which already returns a binary image
  cv::Canny(image_gray, image_canny, 30, 100);

  // Show Canny
  cv::namedWindow("Exemplo 4 - Canny", 0);
  cv::imshow("Exemplo 4 - Canny", image_canny);

  // "Really" show the image and wait for the user input
  cv::waitKey(0);

  return 1;
}

