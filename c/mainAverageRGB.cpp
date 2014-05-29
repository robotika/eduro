// simple RGB statistics from RobotChallenge 2009
#include <string> 
#include <iostream>
#include <cv.h> 
#include <highgui.h> 

const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 512; 

const char* ROBOTCHALLENGE_WND_INPUT = "SICK Robot Day"; 

struct MyColor {
  int r, g, b;
};

MyColor averageRGB( IplImage *image, CvRect* roi )
{
  int r, g, b;
  int x, y;
  
  r = g = b = 0;
  for( y = roi->y; y < roi->y + roi->height; y++ )
  {
    for( x = roi->x; x < roi->x + roi->width; x++ )
    {
      char* ptr = image->imageData+ 3*x+y*image->widthStep;
      r += (unsigned char)ptr[2];
      g += (unsigned char)ptr[1];
      b += (unsigned char)ptr[0];

      ptr[0] = (char)255;
      ptr[1] = (char)255;
      ptr[2] = (char)255;
    }
  }
  MyColor ret;
  int count = roi->width*roi->height;
  ret.r = r/count;
  ret.g = g/count;
  ret.b = b/count;
  return ret;
}

bool LoadImage(const std::string filename, IplImage* dest, CvRect* roi = NULL)
{
  IplImage* frame = cvLoadImage(filename.c_str());

  if(!frame)
  {
    std::cerr << "Failed to load an image: " << filename << std::endl;
    return false;
  }

  if(roi)
  {
    IplImage* roi_frame = cvCreateImage(cvSize(roi->width, roi->height), frame->depth, frame->nChannels);
    cvSetImageROI(frame, *roi);
    cvCopy(frame, roi_frame);
    cvResetImageROI(frame);
    cvReleaseImage(&frame);
    frame = roi_frame;
  }

  //Update the state using the current (resized) image.
  cvResize(frame, dest);
  cvReleaseImage(&frame);

  //flip upside-down, if neccesary
  if(dest->origin == IPL_ORIGIN_BL)
  {
    cvFlip(dest, NULL, 0);
  }

  return true;
} 

const char* COMMAND_QUIT = "quit";
const char* COMMAND_FILE = "file";

const char* COMMAND_SET = "set"; 
const char* COMMAND_GET = "get"; 
const char* COMMAND_SET_ROI = "roi"; 
const char* COMMAND_GET_ROI = "roi"; 

int main( int argc, char *argv[] )
{
  bool withWindows = (argc > 1);
  int pauseTimeMs = 100;
  bool paused = false;
  bool showOrig = false;
  bool useRoi1 = true;
  if( argc > 1 )
    sscanf( argv[1], "%d", &pauseTimeMs );

  IplImage* img = cvCreateImage(cvSize(FRAME_WIDTH, FRAME_HEIGHT), IPL_DEPTH_8U, 3); 
  IplImage* orig = cvCreateImage(cvSize(FRAME_WIDTH, FRAME_HEIGHT), IPL_DEPTH_8U, 3); 
  CvRect* roi = new CvRect();
//  *roi = cvRect(520,380,600-520,450-380);
//  *roi = cvRect(557,407,610-557,450-407);
  *roi = cvRect(557,407+20,610-557,450-407);

  if( withWindows )
    cvNamedWindow( ROBOTCHALLENGE_WND_INPUT, CV_WINDOW_AUTOSIZE ); 

  std::string command, subcommand;
  std::string filename;
  while(std::cin)
  {
    std::cin >> command;

    if(command == COMMAND_QUIT || !std::cin)
    {
      break;
    }
    else if(command == COMMAND_SET)
    {
      std::cin.ignore(64, ' '); //skip the spaces
      std::cin >> subcommand; 
      if(subcommand == COMMAND_SET_ROI)
      {
        if(roi == NULL) roi = new CvRect();
        std::cin >> roi->x >> roi->y >> roi->width >> roi->height;
      } 
    }
    else if(command == COMMAND_GET)
    {
      std::cin.ignore(64, ' '); //skip the spaces
      std::cin >> subcommand; 
      if(subcommand == COMMAND_GET_ROI)
      {
        if(roi == NULL)
        {
          std::cout << "unset" << std::endl;
        }
        else
        {
          std::cout << roi->x << ' ' << roi->y << ' ' << roi->width << ' ' << roi->height << std::endl;
        }
      }
    }
    else if(command == COMMAND_FILE)
    {
      std::cin.ignore(64, ' '); //skip the spaces
      std::getline(std::cin, filename);

      if(!LoadImage(filename, img, NULL))
      {
        continue;
      }
      MyColor c;
      c = averageRGB( img, roi );
      fprintf( stdout, "%d %d %d\n", c.r, c.g, c.b );
      fflush( stdout );
      if( withWindows )
      {
        int key;
        do
        {
          if( showOrig )
            cvShowImage( ROBOTCHALLENGE_WND_INPUT, orig );
          else
            cvShowImage( ROBOTCHALLENGE_WND_INPUT, img );
          key = cvWaitKey( paused ? 0 : pauseTimeMs );
          if( key == 'p' || key == 'P' )
            paused = !paused;
          if( key == ' ' )
            showOrig = !showOrig;
        }
        while( key == ' ' || key == '1' );
        if( key == 'q' || key == 'Q' )
          break;
      }
    }
    else
    {
      std::cerr << "Unknown command: " << command << std::endl;
    }
  }
  cvReleaseImage(&img);
  cvReleaseImage(&orig);
  delete roi;
  return 0;
}
