// simple RGB statistics from RobotChallenge 2009
#include <string> 
#include <iostream>
#include <cv.h> 
#include <highgui.h> 

const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 512; 

const char* ROBOTCHALLENGE_WND_INPUT = "RobotChallenge"; 

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
//  return (r() > 1.2 * g()) && (r() > 1.2 * b()); // RC
//    return (r() > 2.0 * g()) && (r() > 2.0 * b() && r() > 200); // Magellan ver0
    return (r() > 2.0 * g()) && (r() > 2.0 * b() && r() > 150); // Magellan
//  return r() > 50 && (r() > 1.2 * g()) && (r() > 1.2 * b()); // hack only for the line on the table
}

bool Color::isBlue() // hack, yellow
{
  return r()>100 && g()>100 && (b() < 0.7* r()) && (b() < 0.7 * g());
//  return (b() < 0.8* r()) && (b() < 0.8 * g());
//  return (b() > 1.2* r()) && (b() > 1.2 * g());
//  return b()> 50 && (b() > 1.2* r()) && (b() > 1.2 * g()); // hack only for the line on the table
}

bool Color::isGreen()
{
  return (g() > 1.0* r()) && (g() > 1.0 * b());
//  return (g() > 1.3* r()) && (g() > 1.3 * b());
//  return true;
}

//-----------------------------------------------------------------------------

struct BubbleStruct
{
  int count;
  int sumX, sumY;
  int sumXX, sumYY;
};

void findBlobFrames( IplImage *image, CvRect* roi )
{
}

CameraBlobs::BlobType findPuckInFront( IplImage *image, CvRect* roi, BubbleStruct *pRed=0, BubbleStruct *pBlue=0 )
{
  int countRed = 0, countBlue = 0;
  int sumRedX = 0, sumBlueX = 0;
  int sumRedY = 0, sumBlueY = 0;
  int sumRedXX = 0, sumBlueXX = 0;
  int sumRedYY = 0, sumBlueYY = 0;
  int x, y;
  int minX, maxX;
  int middleY;
  double leftSlope = -(426.0/190.0), leftOffset = 426.0;
  double rightSlope = 354.0/190.0, rightOffset = 354.0;
  middleY = roi->y + roi->height/2;
  for( y = roi->y; y < roi->y + roi->height; y++ )
  {
    minX = std::max( roi->x, (int)(y*leftSlope + leftOffset));
    maxX = std::min( roi->x + roi->width, (int)(y*rightSlope + rightOffset));
    for( x = minX; x < maxX; x++ )
    {
      char* ptr = image->imageData+ 3*x+y*image->widthStep;
      Color c;
      c.setRGB( ptr[2], ptr[1], ptr[0] );
      if( c.isRed() )
      {
        countRed++;
        sumRedX += x;
        sumRedY += y;
        sumRedXX += (x*x);
        sumRedYY += (y*y);
      }
      if( c.isBlue() )
      {
        countBlue++;
        sumBlueX += x;
        sumBlueY += y;
        sumBlueXX += (x*x);
        sumBlueYY += (y*y);
      }

      if( c.isRed() )
      {
        ptr[0] = 0;
        ptr[1] = 0;
        ptr[2] = (char)255;
      }
      else if( c.isBlue() )
      {
        ptr[0] = (char)255;
        ptr[1] = 0;
        ptr[2] = 0;
      }
      else
      {
        ptr[0] = (char)255;
        ptr[1] = (char)255;
        ptr[2] = (char)255;
      }
    }
  }

  if( pRed )
  {
    pRed->count = countRed;
    pRed->sumX = sumRedX;
    pRed->sumY = sumRedY;
    pRed->sumXX = sumRedXX;
    pRed->sumYY = sumRedYY;
  }
  if( pBlue )
  {
    pBlue->count = countBlue;
    pBlue->sumX = sumBlueX;
    pBlue->sumY = sumBlueY;
    pBlue->sumXX = sumBlueXX;
    pBlue->sumYY = sumBlueYY;
  }

  if( countRed + countBlue < 20 )
    return CameraBlobs::ENoBlob;

  return countRed > countBlue ? CameraBlobs::ERedPuck : CameraBlobs::EBluePuck;
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
//  *roi = cvRect(262,321,409-262,421-321); // RC
//  *roi = cvRect(262,171,409-262,421-321); // used during RoboOrienteering
//  *roi = cvRect(100,171,550-100,421-321);
//  *roi = cvRect(0,101,640,348-101); // RobotsIntellect

//  *roi = cvRect(54, 332, 207- 54, 447-332); // FRE2013
//  *roi = cvRect(54, 332, 550, 447-332); // FRE2013
  *roi = cvRect(0, 332, 639, 447-332); // FRE2013
  CvRect* roi2 = new CvRect();
  *roi2 = cvRect(0,0,640,512);

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
      BubbleStruct r,b;
      cvCopy( img, orig );
      findPuckInFront( img, useRoi1 ? roi : roi2, &r, &b );
      fprintf( stdout, "%d %d %d %d %d %d ", r.count, b.count, r.sumX, r.sumY, b.sumX, b.sumY );
      fprintf( stdout, "%d %d %d %d\n", r.sumXX, r.sumYY, b.sumXX, b.sumYY );
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
          if( key == '1' )
          {
            useRoi1 = !useRoi1;
            cvCopy( orig, img );
            findPuckInFront( img, useRoi1 ? roi : roi2, &r, &b );
          }
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
  delete roi2;
  return 0;
}
