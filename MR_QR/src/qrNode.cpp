// standard includes
#include <stdio.h>
#include <iostream>
#include <string>
#include <iomanip>
#include <sstream>

// include for OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// include Zbar for barcode and QR decoder
#include <zbar.h>

using namespace cv;
using namespace std;
using namespace zbar;

String read_code(Mat src)
{
	// set up the scanner 
	Mat gray;
	ImageScanner scanner;
	scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);
	
	// convort image to grayscale
	cvtColor(src,gray,CV_RGB2GRAY); // image should always be grayscale!!!
	
	// get size of image
	int width = src.cols;
	int height = src.rows;
	
	// ready the image for reading
	uchar *raw = (uchar *)gray.data;
	Image image( width, height, "Y800", raw , width * height);
	
	// scan the image for QR code
	int n = scanner.scan(image);

	// string used for telling what the QR or barcode says.
	string data_type, data;
	
	// read the image
	for( Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol)
	{
		vector<Point> vp;
		
		// write out the symbols and data
		// type name equals type of code QR or Barcode...
		// data equals the data that can be found in the QR or Barcode 
		//cout << "decode" << symbol->get_type_name() << " symbol \"" << symbol->get_data() << '"' << " " << endl;
		
		//data_type = symbol->get_type_name();
		data = symbol->get_data();
		
		// get the point for the QR code to show where they are. 
		for(int i = 0; i < n; i++)
		{
			vp.push_back(Point(symbol->get_location_x(i), symbol->get_location_y(i)));
		}
		
		RotatedRect r = minAreaRect(vp);
		Point2f pts[4];
		r.points(pts); 
		
		// draw the lines around the QR code. not working fully yet.
		for( int i = 0; i < 4; i++)
		{
			line(src,pts[i],pts[i+1],Scalar(255,0,0),3);
		}
	}
	// show the image with the QR code + lines around. 
	namedWindow("decoded image", WINDOW_AUTOSIZE);
	imshow("decoded image", src);
	
	return data; 
}


// video main
// used this main for capture an image from a webcam and use it for image processing.
/*int main(int argc, char* argv[] )
{	
    // VideoCapture(X), decide which camera to use. if -1, it will seach and take
    // the first one.
    VideoCapture cap(0);
    
    // set size of the captured frame
    cap.set(CV_CAP_PROP_FRAME_WIDTH,800);  
	cap.set(CV_CAP_PROP_FRAME_HEIGHT,640); 
    
    Mat Frame; 
    char exit;
    string data; 
    
    while(1)
    {
		// read the captured frame and save it.
		cap.read(Frame);
		
		// read the image for QR codes.
		data = read_code(Frame);
		
		cout << data << endl;
		
		// exit parameter for while loop
		cin>>exit;
		if( exit = 'q')
			break;
    }
    
    waitKey(0);
    return 0;
}
*/


// use this main for a single picture.
int main(int argc, char* argv[] )
{
	// load image
	Mat image;
	image = imread("/home/student/workspace/QR_reader/market-qr-code.png", 1);
	
	// check for image was found.
	if( !image.data)
	{
		cout << "no image found with that name" << endl;
	}
	
	// read and show the QR code.
	string test = read_code(image);
	cout << test << endl;
	
	// wait and exit. 
	waitKey(0);
    return 0;
}
