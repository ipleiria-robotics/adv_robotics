/**
 * @function moments_demo.cpp
 * @brief Demo code to calculate moments
 * @author OpenCV team
 */

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

cv::Mat src; cv::Mat src_gray;
int thresh = 100;
int max_thresh = 255;
cv::RNG rng(12345);

/// Function header
void thresh_callback(int, void* );

/**
 * @function main
 */
int main( int, char** argv )
{
  /// Load source image and convert it to gray
  std::string img_file_path(getenv("HOME"));
  img_file_path += "/ros/src/TP07/pic1.png";
  src_gray = cv::imread(img_file_path, CV_LOAD_IMAGE_GRAYSCALE);

  /// Create Window
  const char* source_window = "Source";
  cv::namedWindow( source_window, cv::WINDOW_AUTOSIZE );
  cv::imshow( source_window, src_gray );

  cv::createTrackbar( " Canny thresh:", "Source", &thresh, max_thresh, thresh_callback );
  thresh_callback( 0, 0 );

  cv::waitKey(0);
  return(0);
}

/**
 * @function thresh_callback
 */
void thresh_callback(int, void* )
{
  cv::Mat canny_output;
  std::vector<std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy;

  /// Detect edges using canny
  cv::Canny( src_gray, canny_output, thresh, thresh*2, 3 );
  /// Find contours
  cv::findContours( canny_output, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

  /// Get the moments
  std::vector<cv::Moments> mu(contours.size() );
  for( size_t i = 0; i < contours.size(); i++ )
  {
    mu[i] = cv::moments( contours[i], false );
  }

  ///  Get the mass centers:
  std::vector<cv::Point2f> mc( contours.size() );
  for( size_t i = 0; i < contours.size(); i++ )
  {
    mc[i] = cv::Point2f( static_cast<float>(mu[i].m10/mu[i].m00) , static_cast<float>(mu[i].m01/mu[i].m00) );
  }

  /// Draw contours
  cv::Mat drawing = cv::Mat::zeros( canny_output.size(), CV_8UC3 );
  for( size_t i = 0; i< contours.size(); i++ )
  {
    cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
    cv::drawContours( drawing, contours, (int)i, color, 2, 8, hierarchy, 0, cv::Point() );
    cv::circle( drawing, mc[i], 4, color, -1, 8, 0 );
  }

  /// Show in a window
  cv::namedWindow( "Contours", cv::WINDOW_AUTOSIZE );
  cv::imshow( "Contours", drawing );

  /// Calculate the area with the moments 00 and compare with the result of the OpenCV function
  std::cout << "\t Info: Area and Contour Length" << std::endl;
  for( size_t i = 0; i< contours.size(); i++ )
  {
    std::cout << " * Contour[" << (int)i << "] - Area (M_00) = " << mu[i].m00
              << " - Area OpenCV: " << cv::contourArea(contours[i])
              << " - Length: " << cv::arcLength( contours[i], true ) << std::endl;
    cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
    cv::drawContours( drawing, contours, (int)i, color, 2, 8, hierarchy, 0, cv::Point() );
    cv::circle( drawing, mc[i], 4, color, -1, 8, 0 );
  }
}
