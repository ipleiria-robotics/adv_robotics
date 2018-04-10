/*
Copyright (c) 2014, Hugo Costelha.
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
  // Create all white image
  cv::Mat image(240, 320, CV_8UC3, cv::Scalar(255,255,255));

  // Draw line with color B=255, G=0 e R=0
  cv::line(image, cv::Point(10,50), cv::Point(50,200),
           cv::Scalar(255,0,0), 1, 8);

  // Draw rectangle with color B=0, G=255 e R=0
  cv::rectangle(image, cv::Point(10,10), cv::Point(20,200),
                cv::Scalar(0,255,0), 2, 8);

  // Draw circle with color B=0, G=0 e R=255
  cv::circle(image, cv::Point(120,160), 50, cv::Scalar(0,0,255), 3, 8);

  // Create window for the camera image
  cv::namedWindow("Exemplo 2", CV_LOAD_IMAGE_GRAYSCALE);

  // Show image
  cv::imshow("Exemplo 2", image);

  // "Really" show the image and wait for the user input
  cv::waitKey(0);

  // Save image
  cv::imwrite("Exemplo_2.png", image);

  return 1;
}

