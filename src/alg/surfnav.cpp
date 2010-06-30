#include <stdio.h>
#include "surfnav.hpp"
#include "msg.hpp"
#include "surf/surflib.hpp"

namespace sim {
namespace alg {

static IplImage *iplImageFromOsg(const osg::Image *img);
static void drawPoint(IplImage *img, sim::alg::SurfLandmark &ipt);

SurfNav::SurfNav()
{
}

SurfNav::~SurfNav()
{
}

void SurfNav::update(const osg::Image *im, float posx, float posy)
{
    std::vector<SurfLandmark> landmarks;
    IplImage *image = 0;

    // create image
    image = iplImageFromOsg(im);
    if (image == 0)
        return;

    // obtain landmarks
    surf::surfDetDes(image, landmarks, false, 5, 4, 2, 0.0004);

    DBG("landmarks: " << landmarks.size());
    {
        for (size_t i = 0; i < landmarks.size(); i++){
            drawPoint(image, landmarks[i]);
        }
        char fn[100];
        static int count = 0;
        sprintf(fn, "surf/%06d.png", count++);
        cvSaveImage(fn, image);
    }

    // delete created image
    cvReleaseImage(&image);
}

static IplImage *iplImageFromOsg(const osg::Image *img)
{
    IplImage *image = 0;
    CvSize size;
    char *data;
    osg::Vec4 color;

    // set size of image
    size.width = img->s();
    size.height = img->t();
    if (size.width == 0 || size.height == 0)
        return 0;

    // create image
    image = cvCreateImage(size, IPL_DEPTH_8U, 1);

    // transfer image itself from osg to IplImage
    data = (char *)image->imageData;
    for (int i = 0; i < size.width; i++){
        for (int j = 0; j < size.height; j++){
            color = img->getColor(i, j);
            data[i + j * image->widthStep] = (color.x() * 255 + color.y() * 255 + color.z() * 255) / 3.;
        }
    }

    // flip IplImage
    cvConvertImage(image, image, CV_CVTIMG_FLIP);

    return image;
}

static void drawPoint(IplImage *img, sim::alg::SurfLandmark &ipt)
{
  float s, o;
  int r1, c1, r2, c2, lap;

  s = (2.5f * ipt.scale);
  o = ipt.orientation;
  lap = ipt.laplacian_sign;
  r1 = surf::fRound(ipt.y);
  c1 = surf::fRound(ipt.x);

  // Green line indicates orientation
  if (o) // Green line indicates orientation
  {
    c2 = surf::fRound(s * cos(o)) + c1;
    r2 = surf::fRound(s * sin(o)) + r1;
    cvLine(img, cvPoint(c1, r1), cvPoint(c2, r2), cvScalar(0, 255, 0));
  }
  else  // Green dot if using upright version
    cvCircle(img, cvPoint(c1,r1), 1, cvScalar(0, 255, 0),-1);

  if (lap >= 0)
  { // Blue circles indicate light blobs on dark backgrounds
    cvCircle(img, cvPoint(c1,r1), surf::fRound(s), cvScalar(255, 0, 0),1);
  }
  else
  { // Red circles indicate light blobs on dark backgrounds
    cvCircle(img, cvPoint(c1,r1), surf::fRound(s), cvScalar(0, 0, 255),1);
  }
}

}
}

