#include <stdio.h>
#include "surfnav.hpp"
#include "msg.hpp"
#include "surf/surflib.hpp"
#include "sim/common.hpp"

namespace sim {
namespace alg {

static IplImage *iplImageFromOsg(const osg::Image *img);
static void drawPoint(IplImage *img, sim::alg::SurfLandmark &ipt);
static void drawLearned(std::list<SurfLandmark> &learned);
static void sortLandmarksByVisibility(std::list<SurfLandmark> &lms);
static void rgb2hsv(const osg::Vec4 &rgb, float *h, float *s, float *v);


SurfSegment::~SurfSegment()
{
}

void SurfSegment::learnStart(float x, float y)
{
    _pos_x = x;
    _pos_y = y;
    _dist = 0;

    _tracked_lms.clear();
    _learned_lms.clear();

    _learning = true;
}

void SurfSegment::learnFinish()
{
    for_each(std::list<SurfLandmark>::iterator, _tracked_lms){
        _learned_lms.push_back(*it);
    }
    _tracked_lms.clear();

    _learning = false;

    drawLearned(_learned_lms);
}

void SurfSegment::learn(const osg::Image *im, float posx, float posy)
{
    std::vector<SurfLandmark> lms; // List of current landmarks waiting for processing
   
    // update distance
    _dist = _distFromInit(posx, posy);

    _obtainLandmarks(im, posx, posy, _dist, lms);
    _trackLandmarks(lms, _dist);
}

void SurfSegment::_obtainLandmarks(const osg::Image *im, float posx, float posy,
                                   float dist, std::vector<SurfLandmark> &lms)

{
    IplImage *image = 0;

    // create image
    image = iplImageFromOsg(im);
    if (image == 0)
        return;

    // obtain landmarks
    surf::surfDetDes(image, lms, false, 5, 4, 2, 0.0004);

    DBG("landmarks: " << lms.size());
    {
        for (size_t i = 0; i < lms.size(); i++){
            drawPoint(image, lms[i]);
        }
        char fn[100];
        static int count = 0;
        sprintf(fn, "surf/%06d.png", count++);
        cvSaveImage(fn, image);
    }

    // delete created image
    cvReleaseImage(&image);

    // initialize all Landmarks
    for_each(std::vector<SurfLandmark>::iterator, lms){
        it->last_x = it->x;
        it->last_y = it->y;

        it->distance = it->last_distance = dist;
    }
}

void SurfSegment::_trackLandmarks(std::vector<SurfLandmark> &lms, float dist)
{
    std::vector<SurfLandmark>::iterator best[2]; // two best matching landmarks from _lms
    std::list<SurfLandmark>::iterator it, it_end;
    bool found;

    // iterate over all tracked landmarks
    it = _tracked_lms.begin();
    it_end = _tracked_lms.end();
    while (it != it_end){
        found = _bestMatching(*it, lms, best + 0, best + 1);

        if (found && best[0]->dist(*it) * 5. < best[1]->dist(*it) /*TODO best[0] << best[1] */){
            // increase visibility
            it->visibility++;

            // set last position
            it->last_x = best[0]->x;
            it->last_y = best[0]->y;

            // set last distance
            it->last_distance = dist;

            // disable landmark from _lms
            best[0]->enabled = false;

            // step to next landmark
            ++it;
        }else{
            // add tracked landmark to list of learned ones
            _learned_lms.push_back(*it);

            // remove it from list of tracked landmarks
            it = _tracked_lms.erase(it);
        }
    }

    // put all enabled landmarks to tracked list
    for_each(std::vector<SurfLandmark>::iterator, lms){
        if (it->enabled)
            _tracked_lms.push_back(*it);
    }
    lms.clear();
}

bool SurfSegment::_bestMatching(const SurfLandmark &l, std::vector<SurfLandmark> &lms,
                            std::vector<SurfLandmark>::iterator *s1,
                            std::vector<SurfLandmark>::iterator *s2)
{
    bool found[2];
    double dist, best[2];

    found[0] = found[1] = false;
    best[0] = best[1] = DBL_MAX;

    for_each(std::vector<SurfLandmark>::iterator, lms){
        if (!it->enabled)
            continue;

        dist = l.dist(*it);

        if (dist < best[0]){
            // move first best to second best and save new first best
            found[1] = found[0];
            found[0] = true;
            *s2 = *s1;
            *s1 = it;
            best[1] = best[0];
            best[0] = dist;
        }else if (dist < best[1]){
            found[1] = true;
            best[1] = dist;
            *s2 = it;
        }
    }

    if (found[0] && found[1])
        return true;
    return false;
}

static IplImage *iplImageFromOsg(const osg::Image *img)
{
    IplImage *image = 0;
    CvSize size;
    char *data;
    osg::Vec4 color;
    float h, s, v;

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
            rgb2hsv(color, &h, &s, &v);
            //data[i + j * image->widthStep] = (color.x() * 255 + color.y() * 255 + color.z() * 255) / 3.;
            data[i + j * image->widthStep] = h * 255;
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

static void drawLearned(std::list<SurfLandmark> &learned)
{
    int i;

    sortLandmarksByVisibility(learned);

    i = 0;
    for_each(std::list<SurfLandmark>::iterator, learned){
        if (i++ >= 10)
            break;

#ifndef NDEBUG
        DBG("(" << it->x << " " << it->y << ")"
                << " -> "
                << "(" << it->last_x << " " << it->last_y << ")"
                << ", "
                << it->distance << " -> " << it->last_distance
                << " :: "
                << it->visibility);
#endif /* NDEBUG */

    }

}

static bool cmpVis(const SurfLandmark &a, const SurfLandmark &b)
{
    return a.visibility > b.visibility;
}

static void sortLandmarksByVisibility(std::list<SurfLandmark> &lms)
{
    lms.sort(cmpVis);
}

static void rgb2hsv(const osg::Vec4 &rgb, float *h, float *s, float *v)
{
    float r, g, b;
    float max, min;

    r = rgb.r();
    g = rgb.g();
    b = rgb.b();

    max = std::max(r, std::max(g, b));
    min = std::min(r, std::min(g, b));

    if (eq(max, min)){
        *h = 0;
    }else if (eq(max, r) && g >= b){
        *h = 60.f / 360.f * ((g - b) / (max - min));
    }else if (eq(max, r)){ // g < b is implicit
        *h = (60.f / 360.f * ((g - b) / (max - min))) + 1;
    }else if (eq(max, g)){
        *h = (60.f / 360.f * ((b - r) / (max - min))) + (120.f / 360.f);
    }else{ // eq(max, b)
        *h = (60.f / 360.f * ((r - g) / (max - min))) + (240.f / 360.f);
    }

    if (sim::isZero(max)){
        *s = 0.f;
    }else{
        *s = 1 - (min / max);
    }

    *v = max;
}


}
}

