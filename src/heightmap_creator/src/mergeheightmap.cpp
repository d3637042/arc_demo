#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <algorithm> 
#include <vector>
using namespace cv;
using namespace std;

Mat src1, src2, src1color, src2color; 
Mat heightmap, heightmapcolor;

/// Function header

/** @function main */
int main( int argc, char** argv )
{
  string camera1 = argv[1];
  string camera2 = argv[2];
  /// Load source image and convert it to gray
  src1 = imread( "/home/nctuece/catkin_ws/" + camera1 + "heightmap.jpg", 1 );
  src1color = imread( "/home/nctuece/catkin_ws/" + camera1 + "heightmapcolor.jpg", 1 );
  src2 = imread( "/home/nctuece/catkin_ws/" + camera2 + "heightmap.jpg", 1 );
  src2color = imread( "/home/nctuece/catkin_ws/" + camera2 + "heightmapcolor.jpg", 1 );
  heightmap = src1;
  heightmapcolor = src1color;

  int i, j;
  //cout << src1.rows << endl;
  //cout << src2.rows << endl;
  for(i=0; i<src1.rows; i++){
    for(j=0;j<src1.cols;j++){
      if (src1.at<float>(i,j)!=0)
        cout << src1.at<float>(i,j) << " ";
      if (src2.at<float>(i,j)!=0)
        cout << src2.at<float>(i,j) << " ";
      if (src1.at<float>(i,j) > src2.at<float>(i,j))
        heightmap.at<float>(i,j)=src1.at<float>(i,j);
      else if (src1.at<float>(i,j) < src2.at<float>(i,j))
        heightmap.at<float>(i,j)=src2.at<float>(i,j);
    }
  }
  heightmapcolor = max(src1color, src2color);
  /*
  char* heightmap_window = "heightmapcolor";
  namedWindow( heightmap_window, CV_WINDOW_AUTOSIZE );
  imshow( heightmap_window, heightmapcolor );
  */


  imwrite("/home/nctuece/catkin_ws/heightmapcolor.jpg", heightmapcolor);
  imwrite("/home/nctuece/catkin_ws/heightmap.jpg", heightmap);
  waitKey(0);
  return(0);
}
