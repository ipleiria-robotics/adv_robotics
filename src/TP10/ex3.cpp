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

  // Create image for convertion result with the same size of the original image
  // but only with one channel
  cv::Mat image_gray(image.size().height, image.size().width, CV_8UC1);

  // Convert from RGB to gray
  cv::cvtColor(image, image_gray, CV_RGB2GRAY);

  // Create window for the camera image
  cv::namedWindow("Exemplo 3", 0);

  // Show image
  cv::imshow("Exemplo 3", image);

  // "Really" show the image and wait for the user input
  cv::waitKey(0);

  // Save image
  cv::imwrite("Exemplo_3.png", image_gray);

  // Create three images, one for each channel
  cv::Mat image_red(image.size().height, image.size().width, CV_8UC1);
  cv::Mat image_green(image.size().height, image.size().width, CV_8UC1);
  cv::Mat image_blue(image.size().height, image.size().width, CV_8UC1);

  // Separate the RGB color image into three separate images
  // Note that OpenCV stores RGB images in BGR order
  int fromTo[] = {0,2, 1,1, 2,0};
  cv::Mat out[] = {image_red, image_green, image_blue};
  cv::mixChannels(&image, 3, out, 3, fromTo, 3);

  // Show images
  cv::namedWindow("Exemplo 3 - Red", 0);
  cv::imshow("Exemplo 3 - Red", image_red);
  cv::namedWindow("Exemplo 3 - Green", 0);
  cv::imshow("Exemplo 3 - Green", image_green);
  cv::namedWindow("Exemplo 3 - Blue", 0);
  cv::imshow("Exemplo 3 - Blue", image_blue);

  // "Really" show the image and wait for the user input
  cv::waitKey(0);

  return 1;
}

