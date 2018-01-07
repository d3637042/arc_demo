#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <algorithm> 
#include <vector>
using namespace cv;
using namespace std;

Mat src, src2; Mat src_gray;
int thresh = 70;
int max_thresh = 255;
RNG rng(12345);
Mat warp_mat( 2, 3, CV_32FC1 );
string name_str;
/// Function header
void thresh_callback(int, void* );

/** @function main */
int main( int argc, char** argv )
{
  
  

  name_str = argv[1];
  /// Load source image and convert it to gray
  src = imread( "/home/nctuece/catkin_ws/" + name_str + "colorimage.jpg", 1 );
  src2 = imread( "/home/nctuece/catkin_ws/" + name_str + "depthimage.jpg", 1 );
  /// Convert image to gray and blur it
  cvtColor( src, src_gray, CV_BGR2GRAY );
  blur( src_gray, src_gray, Size(3,3) );

  /// Create Window
  char* source_window = "Source";
  namedWindow( source_window, CV_WINDOW_AUTOSIZE );
  imshow( source_window, src );

  if(argc>2){
    string mat_file = "/home/nctuece/catkin_ws/" + name_str + "adjustment.xml";
    FileStorage fs(mat_file,FileStorage::READ);
    fs[name_str] >> warp_mat;
  }
  else if(argc==2){
    thresh_callback(0, 0);
    string mat_file = "/home/nctuece/catkin_ws/" + name_str + "adjustment.xml";
    FileStorage fs( mat_file, FileStorage::WRITE);
    fs << name_str << warp_mat;
    fs.release();
  }

  Mat warp_dst, warp_dst2;
  warpAffine( src, warp_dst, warp_mat, warp_dst.size() );
  warpAffine( src2, warp_dst2, warp_mat, warp_dst2.size() );
  char* warp_window = "Warped";
  cout << warp_dst.size() << endl;
  namedWindow( warp_window, CV_WINDOW_AUTOSIZE );
  imshow( warp_window, warp_dst );

  imwrite("/home/nctuece/catkin_ws/" + name_str + "heightmapcolor.jpg", warp_dst);
  imwrite("/home/nctuece/catkin_ws/" + name_str + "heightmap.jpg", warp_dst2);
  waitKey(0);
  return(0);
}

/** @function thresh_callback */
void thresh_callback(int, void* )
{
  Mat threshold_output;
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;

  /// Detect edges using Threshold
  threshold( src_gray, threshold_output, thresh, 255, THRESH_BINARY );
  char* thres_window = "thres";
  imshow( thres_window, threshold_output );
  /// Find contours
  findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

  /// Approximate contours to polygons + get bounding rects and circles
  vector<vector<Point> > contours_poly( contours.size() );
  vector<RotatedRect> boundRect( contours.size() );
  RotatedRect apriltagboundRect;
  vector<float>radius( contours.size() );
  Point2f Apriltagbox[4];
  for( int i = 0; i < contours.size(); i++ )
     { approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
       boundRect[i] = minAreaRect( Mat(contours_poly[i]) );
     }


  /// Draw polygonal contour + bonding rects + circles
  Mat drawing = Mat::zeros( threshold_output.size(), CV_8UC3 );
  for( int i = 0; i< contours.size(); i++ )
     {
       Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
       drawContours( drawing, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
       float size = boundRect[i].size.width*boundRect[i].size.height;
       //cout << size << endl;
       if (size>7000 && size<10000){
        for( int j = 0; j < 4; j++ ){
          boundRect[i].points(Apriltagbox);
          apriltagboundRect = boundRect[i];
          line( drawing, Apriltagbox[j], Apriltagbox[(j+1)%4], Scalar(255,0,0), 10, 8 );  
          //cout << Apriltagbox[j] << endl;
        }

       }
 
     }

  /// Show in a window
  namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
  imshow( "Contours", drawing );

  //transform to groundtruth
  //top_left: 160-65=95 112-65=47 130*130
  Point2f srcpnts[4];
  Point2f dstpnts[4];
  Point2f Apriltagbox_sorted[4];
  for (int i=0; i<4; i++){
    //cout << Apriltagbox[i].x << endl;
    if((Apriltagbox[i].x < apriltagboundRect.center.x)&&(Apriltagbox[i].y < apriltagboundRect.center.y)){
      //cout << "0" << endl;
      Apriltagbox_sorted[0] = Apriltagbox[i];
    }

    if((Apriltagbox[i].x < apriltagboundRect.center.x)&&(Apriltagbox[i].y > apriltagboundRect.center.y)){
      //cout << "1" << endl;
      Apriltagbox_sorted[1] = Apriltagbox[i];
    }

    if((Apriltagbox[i].x > apriltagboundRect.center.x)&&(Apriltagbox[i].y > apriltagboundRect.center.y)){
      //cout << "2" << endl;
      Apriltagbox_sorted[2] = Apriltagbox[i];
    }

    if((Apriltagbox[i].x> apriltagboundRect.center.x)&&(Apriltagbox[i].y < apriltagboundRect.center.y)){
      //cout << "3" << endl;
      Apriltagbox_sorted[3] = Apriltagbox[i];
    }
  }

  srcpnts[0] = Apriltagbox_sorted[0];
  srcpnts[1] = Apriltagbox_sorted[1];
  srcpnts[2] = Apriltagbox_sorted[2];
  srcpnts[3] = Apriltagbox_sorted[3];
  dstpnts[0] = Point2f(160-40, 112-40);
  dstpnts[1] = Point2f(160-40, 112+40);
  dstpnts[2] = Point2f(160+40, 112+40);
  dstpnts[3] = Point2f(160+40, 112-40);
  cout << apriltagboundRect.center.x << " " << apriltagboundRect.center.y << endl;
  cout << Apriltagbox[0] << Apriltagbox[1] << Apriltagbox[2] << Apriltagbox[3] << endl;
  cout << Apriltagbox_sorted[0] << Apriltagbox_sorted[1] << Apriltagbox_sorted[2] << Apriltagbox_sorted[3] << endl;
  
  warp_mat = getAffineTransform( srcpnts, dstpnts );


}
