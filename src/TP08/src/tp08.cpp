/*
Copyright (c) 2015, Hugo Costelha.
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
#include <opencv2/ml/ml.hpp>

#include <iostream>
#include <fstream>

#define NUM_DATA_VECTORS 5000
#define SIZE_TRAINING_SET 200

#define INPUT_LAYER_SIZE 400
#define LAYER_2_SIZE      25
#define OUTPUT_LAYER_SIZE 10

#define DEBUG_NN 0

/**
 * Main function
 */
int main(int argc, char** argv)
{
  // Initialize the random number generator
  time_t seconds = time(0);
  cv::RNG rng(seconds);

  // Create window for the camera image
  cv::namedWindow("Debug", CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO | CV_GUI_EXPANDED );

  // Generate a random set of numbers to select the samples for the training set
  cv::Mat training_set(SIZE_TRAINING_SET, 1, CV_32SC1);
  rng.fill(training_set, cv::RNG::UNIFORM, 0, NUM_DATA_VECTORS);

  // Vector with all the data
  cv::Mat X(0, 0, CV_32FC1); // Data vector for the validation set
  cv::Mat Y(0, 0, CV_32FC1); // Output vector for the validation set
  cv::Mat X_t(0, 0, CV_32FC1); // Data vector for the training set
  cv::Mat Y_t(0, 0, CV_32FC1); // Output vector for the training set

  /// Get data from folder
  std::cout << "Reading data from folder..." << std::endl;
  // Get images and their classification
  // We split the set as we read into the training set and the validation set
  std::stringstream tmpstream;
  std::ifstream yfile;
  yfile.open ("./data/y.txt", std::ifstream::in);
  uint num_training_samples = 0,
       num_validation_samples = 0;
  for(uint n=1; n <= NUM_DATA_VECTORS; n++)
  {
    tmpstream.str(std::string());
    tmpstream << "../data/" << n << ".png";
    // Read image and convert it to  a single row float vector
    cv::Mat x;
    cv::imread(tmpstream.str(), 0).reshape(0, 1).convertTo(x, CV_32FC1);
    // Read classification
    int value;
    yfile >> value;
    cv::Mat_<float> y = cv::Mat_<float>::zeros(1, OUTPUT_LAYER_SIZE);
    y(value) = 1.0;

    // If this is a training sample, add it to the training set, otherwise add
    // it to the validation set.
    if( countNonZero(training_set == n) > 0)
    {
      X_t.push_back(x);
      Y_t.push_back(y);
      num_training_samples++;
    } else
    {
      X.push_back(x);
      Y.push_back(y);
      num_validation_samples++;
    }
  }

  // DEBUG
#if DEBUG_NN
  for(uint n=0; n < NUM_DATA_VECTORS; n++)
  {
    cv::imshow("Debug", X.row(n).reshape(0,20));
    cv::waitKey(10);
  }
#endif
  /////////////////////////////////////////////////////////////////////////////
  /// Create artificial neural network
  std::cout << "Creating the artificial neural network..." << std::endl;
  cv::Mat num_layers = (cv::Mat_<int>(3,1) << INPUT_LAYER_SIZE,
                                              LAYER_2_SIZE,
                                              OUTPUT_LAYER_SIZE);
  CvANN_MLP ann(num_layers);

  /// Train the neural network
  std::cout << "Training the artificial neural network..." << std::endl;
  // We will use the same importance weight for all samples.
  cv::Mat sampleWeights = cv::Mat::ones(num_training_samples, 1, CV_32FC1);
  ann.train(X_t, Y_t, sampleWeights);

  /////////////////////////////////////////////////////////////////////////////
  // Test the neural network using the training set
  float correct_classifications = 0;
  for(uint n=0; n < num_training_samples; n++)
  {
    cv::Mat y;
    ann.predict(X_t.row(n), y);
#if DEBUG_NN
    std::cout << y << "\n"
              << Y.row(n) << "\n" << std::endl;
#endif
    int maxy[2], maxY[2];
    cv::minMaxIdx(y, 0, 0, 0, maxy);
    cv::minMaxIdx(Y_t.row(n), 0, 0, 0, maxY);
    if (maxy[1] == maxY[1])
      correct_classifications++;
  }

  std::cout << "Got " << correct_classifications/num_training_samples*100
            << "% of correct classification fom the training set." << std::endl;

  /////////////////////////////////////////////////////////////////////////////
  // Test the neural network using the validation set (not used to train the NN)
  correct_classifications = 0;
  for(uint n=0; n < num_validation_samples; n++)
  {
    cv::Mat y;
    ann.predict(X.row(n), y);
#if DEBUG_NN
    std::cout << y << "\n"
              << Y.row(n) << "\n" << std::endl;
#endif
    int maxy[2], maxY[2];
    cv::minMaxIdx(y, 0, 0, 0, maxy);
    cv::minMaxIdx(Y.row(n), 0, 0, 0, maxY);
    if (maxy[1] == maxY[1])
      correct_classifications++;
  }

  std::cout << "Got " << correct_classifications/num_validation_samples*100
            << "% of correct classification for the validation set." << std::endl;

  /////////////////////////////////////////////////////////////////////////////
  // Test the neural network again using the validation set, step by step,
  // showing the user the result.
  for(uint n=0; n < num_validation_samples; n++)
  {
    cv::Mat y;
    ann.predict(X.row(n), y);
    int maxy[2];
    cv::minMaxIdx(y, 0, 0, 0, maxy);
    cv::imshow("Debug", X.row(n).reshape(0,20));
    std::cout << "Digit " << maxy[1] << std::endl;
    if( cv::waitKey(1000) == 127 )
      break;
  }
  return 1;
}

