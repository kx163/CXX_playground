#include <iostream>
#include <chrono>
using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


int main(int argc, char const *argv[])
{
    // reading image from argv[1]
    cv::Mat image;
    image = cv::imread(argv[1]);

    // checking the existence of the image
    if (image.data == nullptr) {
        cerr << "The file " << argv[1] << " doesn't exist." << endl;
        return 0;
    }

    // image reading suceeded, outputing some basic information
    cout << "Image read, width: " << image.cols << ", height: " << image.rows << ", channels: " << image.channels() << endl;
    cv::imshow("image", image);  // show the image
    cv::waitKey(0);  // wait the user to press a key

    // checking the type of the image
    if (image.type() != CV_8UC1 && image.type() != CV_8UC3) {
        // the image type is not satisfied
        cout << "Please pass a color image or a greyscale image!" << endl;
        return 0;
    }
    
    // To iterate over the image
    // The same method could be used to randomly access the image
    // Using chrono to timing the algorithm
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    for(size_t y = 0; y < image.rows; y++)
    {
        for(size_t x = 0; x < image.cols; x++)
        {
            // accessing the pixel at x, y
            // using cv::Mat::ptr to obtain the line pointer
            unsigned char* row_ptr = image.ptr<unsigned char>(y);  // row_ptr is the header pointer of row y
            unsigned char* data_ptr = &row_ptr[x*image.channels()];  // data_ptr points to the pixel data waiting for access
            // outputing each channel of the pixel
            // greyscale image only have one channel
            for(int c = 0; c < image.channels(); c++)
            {
                unsigned char data = data_ptr[c];  // data is the value of channel c of pixel I(x,y)
            }
        }
    }
    
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "Timed used for iterating over the image: " << time_used.count() << " seconds." << endl;

    // cv::Mat for copying
    // direct assignment will not copy the data
    cv::Mat image_another = image;
    // any change in image_another will also change image
    image_another(cv::Rect(0, 0, 100, 100)).setTo(0);  // set the left top corner to 0
    cv::imshow("image", image);
    cv::waitKey(0);

    // using clone to copy data
    cv::Mat image_clone = image.clone();
    image_clone(cv::Rect(0, 0, 100, 100)).setTo(255);
    cv::imshow("image", image);
    cv::imshow("image_clone", image_clone);
    cv::waitKey(0);
    
    // destroying all windows
    cv::destroyAllWindows();
    return 0;
}
