// Includes
#include <stdio.h>
#include <iostream>
#include <string>
#include <iomanip>
#include <sstream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <zbar.h>

std::string readCode(cv::Mat src)
{
	// set up the scanner 
    cv::Mat gray;
    zbar::ImageScanner scanner;
    scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);
	
	// convort image to grayscale
    cv::cvtColor(src, gray, CV_RGB2GRAY); // image should always be grayscale!!!
	
	// get size of image
	int width = src.cols;
	int height = src.rows;
	
	// ready the image for reading
    uchar *raw = (uchar*)gray.data;
    zbar::Image image( width, height, "Y800", raw , width * height);
	
	// scan the image for QR code
	int n = scanner.scan(image);

	// string used for telling what the QR or barcode says.
    std::string data_type, data;
	
	// read the image
    for(zbar::Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol)
	{
        std::vector<cv::Point> vp;
		
		// write out the symbols and data
		// type name equals type of code QR or Barcode...
		// data equals the data that can be found in the QR or Barcode 
		//cout << "decode" << symbol->get_type_name() << " symbol \"" << symbol->get_data() << '"' << " " << endl;
		
		//data_type = symbol->get_type_name();
		data = symbol->get_data();
		
		// get the point for the QR code to show where they are. 
        for(int i = 0; i < n; i++)
            vp.push_back(cv::Point(symbol->get_location_x(i), symbol->get_location_y(i)));
		
        cv::RotatedRect r = cv::minAreaRect(vp);
        cv::Point2f pts[4];
		r.points(pts); 
		
		// draw the lines around the QR code. not working fully yet.
        for(int i = 0; i < 4; i++)
            cv::line(src, pts[i], pts[i+1], cv::Scalar(255,0,0), 3);
	}
	// show the image with the QR code + lines around. 
    cv::namedWindow("decoded image", cv::WINDOW_AUTOSIZE);
    cv::imshow("decoded image", src);
	
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
    cv::Mat image = cv::imread("/home/student/workspace/QR_reader/market-qr-code.png", 1);
	
    // Check for image was found.
    if(!image.data)
	{
        std::cout << "No image found with that name!" << std::endl;
	}
	
	// read and show the QR code.
    std::string test = readCode(image);
    std::cout << test << std::endl;
	
	// wait and exit. 
    cv::waitKey(0);
    return 0;
}
