// simple RGB statistics for FRE2010 - ver0
#include <string> 
#include <iostream>
#include <cv.h> 
#include <highgui.h> 

const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 512; 

const char* ROBOTCHALLENGE_WND_INPUT = "FRE2010"; 

class CameraBlobs
{
public:
  enum BlobType
  {
    ENoBlob,
    ERedPuck,
    EBluePuck
  };
};

struct Color
{
  unsigned int r() { return (m_value >> 16) & 0xFF; }
  unsigned int g() { return (m_value >> 8) & 0xFF; }
  unsigned int b() { return m_value & 0xFF; }

  void setRGB( unsigned char red, unsigned char green, unsigned char blue );

  unsigned int m_value;

  // helper functions
  bool isRed();
  bool isBlue();
  bool isGreen();
};

// helper functions
void Color::setRGB( unsigned char red, unsigned char green, unsigned char blue )
{
  m_value = ((unsigned int)red << 16) | ((unsigned int)green << 8) | (unsigned int)blue;
}

bool Color::isRed()
{
  return (r() > 1.2 * g()) && (r() > 1.2 * b());
//  return r() > 50 && (r() > 1.2 * g()) && (r() > 1.2 * b()); // hack only for the line on the table
}

bool Color::isBlue()
{
  return (b() > 1.2* r()) && (b() > 1.2 * g());
//  return b()> 50 && (b() > 1.2* r()) && (b() > 1.2 * g()); // hack only for the line on the table
}

bool Color::isGreen()
{
//  return (g() > 1.0* r()) && (g() > 1.0 * b());
//  return (g() > 1.05* r()) && (g() > 1.0 * b());
  return (g() > 1.3* r()) && (g() > 1.3 * b());
}

//-----------------------------------------------------------------------------

struct BubbleStruct
{
  int count;
  int sumX, sumY;
  int sumXX, sumYY, sumXY;
};

const int MEDIAN_SIZE = 70;

struct RoadPoint
{
  int x, y, width;
};

//ver2
int findGreen( IplImage *image, RoadPoint *arr )
{
  int x, y, centerX;
  int Y;
  centerX = image->width/2;
//  const int limit = 12*MEDIAN_SIZE;
  const int limit = MEDIAN_SIZE/2;
  int count,leftX, rightX;
  int index = 0;
  for( Y = 361; Y >= 221; Y -= MEDIAN_SIZE )
  {
    count = 0;
    for( x = centerX; x < image->width; x++ )
    {
      for( y = Y; y < Y+MEDIAN_SIZE && y < image->height; y++ )
      {
        char* ptr = image->imageData+ 3*x+y*image->widthStep;
        Color c;
        c.setRGB( ptr[2], ptr[1], ptr[0] );
        if( c.isGreen() )
        {
          ptr[0] = 0;
          ptr[1] = (char)255;
          ptr[2] = 0;
          count++;
        }
        else
        {
          ptr[0] = (char)255;
          ptr[1] = (char)255;
          ptr[2] = (char)255;
        }
      }
      if( count > limit )
        break;
    }
    rightX = x;

    count = 0;
    for( x = centerX; x > 0; x-- )
    {
      for( y = Y; y < Y+MEDIAN_SIZE && y < image->height; y++ )
      {
        char* ptr = image->imageData+ 3*x+y*image->widthStep;
        Color c;
        c.setRGB( ptr[2], ptr[1], ptr[0] );
        if( c.isGreen() )
        {
          ptr[0] = 0;
          ptr[1] = (char)255;
          ptr[2] = 0;
          count++;
        }
        else
        {
          ptr[0] = (char)255;
          ptr[1] = (char)255;
          ptr[2] = (char)255;
        }
      }
      if( count > limit )
        break;
    }
    leftX = x;
    centerX = (leftX+rightX)/2;
    cvCircle(image, cvPoint(centerX,y-MEDIAN_SIZE/2), 7, CV_RGB(255,0,0), CV_FILLED );
    //arr[index].x = centerX;
    //arr[index].y = y-MEDIAN_SIZE/2;
    arr[index].x = leftX;
    arr[index].y = rightX;
    arr[index].width = rightX-leftX;
    index++;
  }

  return index;
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
  RoadPoint arr[10];
  int arrSize;
  bool withWindows = (argc > 1);
  int pauseTimeMs = 100;
  bool paused = false;
  bool showOrig = false;
  bool useRoi1 = true;
  bool displayResult = true;
  if( argc > 1 )
    sscanf( argv[1], "%d", &pauseTimeMs );

  IplImage* img = cvCreateImage(cvSize(FRAME_WIDTH, FRAME_HEIGHT), IPL_DEPTH_8U, 3); 
  IplImage* orig = cvCreateImage(cvSize(FRAME_WIDTH, FRAME_HEIGHT), IPL_DEPTH_8U, 3); 

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
    else if(command == COMMAND_FILE)
    {
      std::cin.ignore(64, ' '); //skip the spaces
      std::getline(std::cin, filename);

      if(!LoadImage(filename, img, NULL))
      {
        continue;
      }
      cvCopy( img, orig );
      arrSize = findGreen( img, arr );
      for( int i = 0; i < arrSize; i++ )
        fprintf( stdout, "%d %d %d ", arr[i].x, arr[i].y, arr[i].width );
      fprintf( stdout, "\n");
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
  return 0;
}
