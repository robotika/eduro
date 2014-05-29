// simple RGB statistics from RobotChallenge 2009
#include <string> 
#include <iostream>
#include <vector>
#include <cv.h>
#include <highgui.h>

//const int THRESHOLD = 60; // parameter of the image thresholding
//const int THRESHOLD = 30; // parameter of the image thresholding
const int THRESHOLD = 40; // parameter of the image thresholding
const CvRect ROI = cvRect(150, 280, 340, 210); //region of interest
//const CvRect ROI = cvRect(120, 280, 400, 210); //region of interest
const int N = 20; // maximal number of points
//JI: const int NEARNESS = 30; // how many pixels appart can points of the road be to be still considered OK
const int NEARNESS = 50; // how many pixels appart can points of the road be to be still considered OK

const int ROW_STEP = ROI.height / (N + 1);

int findLine( const char *filename, char *outFilename = 0 )
{
  IplImage* img = cvLoadImage(filename);
  if(img == NULL)
  {
    std::cerr << "Failed to load the image: " << filename << std::endl;
    return 2;
  }

  IplImage* gray = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 1);
  cvCvtColor(img, gray, CV_BGR2GRAY);
  IplImage* bin = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 1);
  cvThreshold(gray, bin, THRESHOLD, 255, CV_THRESH_BINARY);

  std::vector<CvPoint> all;
  int x, y;
  for(y = ROI.y; y < ROI.y + ROI.height; y += ROW_STEP)
  {
    int longest_length = -1;
    int longest_from = 0;
    int alt_len = -1;
    int alt_from = 0;

    int current_length = 0;
    int current_from = ROI.x - 1;
    for(x = ROI.x; x < ROI.x + ROI.width; x++)
    {
      int val = (int)cvGet2D(bin, y, x).val[0];

      if(val == 0) //black
      {
        current_length++;
        if(current_length > longest_length)
        {
          longest_length = current_length;
          longest_from = current_from;
        }
      }
      else
      {
        current_length = 0;
        current_from = x;
      }
    }

    if(longest_length > 0)
    {
      int x1 = longest_from + 1 + longest_length / 2;
      all.push_back(cvPoint(x1, y));

      current_length = 0;;
      for(x = ROI.x; x < ROI.x + ROI.width; x++)
      {
        int val = (int)cvGet2D(bin, y, x).val[0];

        if(val == 0) //black
        {
          current_length++;
          if(current_length > alt_len && current_from != longest_from)
          {
            alt_len = current_length;
            alt_from = current_from;
          }
        }
        else
        {
          current_length = 0;
          current_from = x;
        }
      }
      if(alt_len > 0)
      {
        int x1 = alt_from + 1 + alt_len / 2;
        all.push_back(cvPoint(x1, y));
      }
    }
  }

  for(x = ROI.x; x < ROI.x + ROI.width; x += ROW_STEP)
  {
    int longest_length = -1;
    int longest_from = 0;
    int alt_len = -1;
    int alt_from = 0;

    int current_length = 0;
    int current_from = ROI.y - 1;
    for(y = ROI.y; y < ROI.y + ROI.height; y++ )
    {
      int val = (int)cvGet2D(bin, y, x).val[0];

      if(val == 0) //black
      {
        current_length++;
        if(current_length > longest_length)
        {
          longest_length = current_length;
          longest_from = current_from;
        }
      }
      else
      {
        current_length = 0;
        current_from = y;
      }
    }

    if(longest_length > 0)
    {
      int y1 = longest_from + 1 + longest_length / 2;
      all.push_back(cvPoint(x, y1));

      current_length = 0;;
      for(y = ROI.y; y < ROI.y + ROI.height; y++ )
      {
        int val = (int)cvGet2D(bin, y, x).val[0];

        if(val == 0) //black
        {
          current_length++;
          if(current_length > alt_len && current_from != longest_from)
          {
            alt_len = current_length;
            alt_from = current_from;
          }
        }
        else
        {
          current_length = 0;
          current_from = y;
        }
      }
      if(alt_len > 0)
      {
        int y1 = alt_from + 1 + alt_len / 2;
        all.push_back(cvPoint(x, y1));
      }
    }
  }


  std::vector<CvPoint> nice;
  for(unsigned int k = 0; k < all.size(); k++)
  {
    int good = 0;
    CvPoint curr = all[k];

    for(unsigned int j = 0; j < all.size(); j++)
    {
      CvPoint other = all[j];

      if( !( curr.x < other.x - NEARNESS
        || curr.x > other.x + NEARNESS
        || curr.y < other.y - NEARNESS
        || curr.y > other.y + NEARNESS) )
      {
        good++;
      }
    }

    if(1) //good >= 2 )
    {
      nice.push_back(curr);
    }
  }

  //std::cout << nice.size();
  //for(std::vector<CvPoint>::iterator it = nice.begin(); it != nice.end(); it++)
  //{
  //  std::cout  << ' ' << it->x << ' ' << it->y;
  //}
  //std::cout << std::endl;
  fprintf( stdout, "%d", (int)nice.size() );
  for(std::vector<CvPoint>::iterator it = nice.begin(); it != nice.end(); it++)
  {
    fprintf( stdout, " %d %d", it->x, it->y );
  }
  fprintf( stdout, "\n" );
  fflush( stdout );

  if( outFilename )
  {
    cvCvtColor(bin, img, CV_GRAY2BGR);

    for(std::vector<CvPoint>::iterator it = all.begin(); it != all.end(); it++)
    {
      cvCircle(img, *it, 3, CV_RGB(0, 255, 0), 1);
    }

    for(std::vector<CvPoint>::iterator it = nice.begin(); it != nice.end(); it++)
    {
      cvCircle(img, *it, 5, CV_RGB(255, 0, 0), 3);
    }

    cvSaveImage(outFilename, img);
  }

  cvReleaseImage(&bin);
  cvReleaseImage(&gray);
  cvReleaseImage(&img);
  return 0;
} 

const char* COMMAND_QUIT = "quit";
const char* COMMAND_FILE = "file";

int main( int argc, char *argv[] )
{
  if( argc > 2 )
  {
    // test single image
    findLine( argv[1], argv[2] );
    return 0;
  }

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

      findLine( filename.c_str() );
    }
    else
    {
      std::cerr << "Unknown command: " << command << std::endl;
    }
  }
  return 0;
}
