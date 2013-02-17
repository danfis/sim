/***
 * sim
 * ---------------------------------
 * Copyright (c)2010 Daniel Fiser <danfis@danfis.cz>
 *
 *  This file is part of sim.
 *
 *  sim is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as
 *  published by the Free Software Foundation; either version 3 of
 *  the License, or (at your option) any later version.
 *
 *  sim is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include "surfnav.hpp"
#include "msg.hpp"
#include "surf/surflib.hpp"
#include "sim/common.hpp"

namespace sim {
namespace alg {

static IplImage *iplImageFromOsg(const osg::Image *img);
static void drawPoint(IplImage *img, sim::alg::SurfLandmark &ipt);
static void sortLandmarksByVisibility(std::list<SurfLandmark> &lms);
static void rgb2hsv(const osg::Vec4 &rgb, float *h, float *s, float *v);


SurfHist::SurfHist(float binsize, std::list<float> &nums)
    : _binsize(binsize), _nums(0), _nums_size(0),
      _bins_head(0), _bins_tail(0)
{
    size_t i;
    _bin_t *bin = 0;
    float first;
    size_t first_idx;

    // allocate memory for numbers
    _nums = new float[nums.size()];
    _nums_size = nums.size();

    // sort list
    nums.sort();

    // copy list to allocated array
    i = 0;
    for_each(std::list<float>::iterator, nums){
        _nums[i++] = *it;
    }

    if (_nums_size > 1){
        // create bins
        first_idx = 0;
        first = _nums[0];
        for (i = 1; i < _nums_size; i++){
            if (bin == 0){
                bin = new _bin_t;
                bin->nums = _nums + first_idx;

                // insert to list
                if (_bins_head){
                    _bins_tail->next = bin;
                    _bins_tail = bin;
                }else{
                    _bins_head = _bins_tail = bin;
                }
            }

            if (fabsf(_nums[i] - first) > binsize){
                bin->size = i - first_idx;

                first = _nums[i];
                first_idx = i;

                bin = 0;
            }
        }

        if (bin != 0){
            bin->size = _nums_size - first_idx;
        }
    }else if (_nums_size == 1){
        bin = new _bin_t;
        bin->nums = _nums;
        bin->size = 1;

        // insert to list
        if (_bins_head){
            _bins_tail->next = bin;
            _bins_tail = bin;
        }else{
            _bins_head = _bins_tail = bin;
        }
    }
}

SurfHist::~SurfHist()
{
    _bin_t *bin = _bins_head;
    _bin_t *bin_del;

    while (bin){
        bin_del = bin;
        bin = bin->next;
        delete bin_del;
    }

    delete _nums;
}

float SurfHist::hd() const
{
    _bin_t *bin, *bin_prev;
    _bin_t *best[3];
    size_t best_size = 0;
    float hd, size;

    best[0] = best[1] = best[2] = 0;

    // find highest bin and two surrounding ones
    bin_prev = 0;
    bin = _bins_head;
    while (bin){
        if (bin->size >= best_size){
            best[1] = bin;
            best[0] = bin_prev;
            best[2] = bin->next;
        }

        bin_prev = bin;
        bin = bin->next;
    }

    // compute average value
    size = 0.f;
    hd = 0.f;
    for (size_t i = 0; i < 3; i++){
        if (best[i] == 0)
            continue;

        for (size_t j = 0; j < best[i]->size; j++){
            hd += best[i]->nums[j];
            size += 1.f;
        }
    }

    if (!isZero(size))
        hd = hd / size;

    if (std::isnan(hd))
        return 0.f;
    return hd;
}


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
}

void SurfSegment::learn(const osg::Image *im, float posx, float posy)
{
    std::vector<SurfLandmark> lms; // List of current landmarks waiting for processing

    if (!_learning)
        return;
   
    // update distance
    _dist = _distFromInit(posx, posy);

    _obtainLandmarks(im, posx, posy, _dist, &lms);
    _trackLandmarks(lms, _dist);
}

void SurfSegment::learnSave(const char *fn)
{
    std::ofstream fout(fn);

    fout << _pos_x << " " << _pos_y << " " << _dist << std::endl;

    if (fout.good()){
        for_each(std::list<SurfLandmark>::iterator, _learned_lms){
            it->write(fout);
        }
        fout.close();
    }
}

void SurfSegment::learnLoad(const char *fn)
{
    std::ifstream fin(fn);
    SurfLandmark lm;

    fin >> _pos_x >> _pos_y >> _dist;

    _learned_lms.clear();
    while (fin.good() && !fin.eof() && lm.read(fin)){
        _learned_lms.push_back(lm);
    }
    fin.close();
}

void SurfSegment::traverseStart(float x, float y)
{
    _traversing = true;
}

void SurfSegment::traverseFinish()
{
    _traversing = false;
}

float SurfSegment::traverse(const osg::Image *image, float posx, float posy)
{
    float hd = 0.; // horizontal difference
    std::vector<SurfLandmark> lms;
    std::list<SurfLandmark> tracked;
    SurfHist *hist;
    float dist;

    if (!_traversing)
        return 0.;

    dist = _distFromInit(posx, posy);

    _obtainLandmarks(image, posx, posy, dist, &lms);
    _pickLearned(dist, &tracked);
    hist = _horizontalDiffsHist(dist, tracked, lms);

    hd = hist->hd();
    delete hist;

    return hd;
}

void SurfSegment::_obtainLandmarks(const osg::Image *im, float posx, float posy,
                                   float dist, std::vector<SurfLandmark> *lms)

{
    IplImage *image = 0;

    // create image
    image = iplImageFromOsg(im);
    if (image == 0)
        return;

    // obtain landmarks
    surf::surfDetDes(image, *lms, false, 5, 4, 2, 0.0004);

    //DBG("landmarks: " << lms->size());
    /*
    {
        for (size_t i = 0; i < lms->size(); i++){
            drawPoint(image, (*lms)[i]);
        }
        char fn[100];
        static int count = 0;
        sprintf(fn, "surf/%06d.png", count++);
        cvSaveImage(fn, image);
    }
    */

    // delete created image
    cvReleaseImage(&image);

    // initialize all Landmarks
    for_each(std::vector<SurfLandmark>::iterator, *lms){
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

void SurfSegment::_pickLearned(float dist, std::list<SurfLandmark> *tracked)
{
    for_each(std::list<SurfLandmark>::iterator, _learned_lms){
        if ((it->distance < dist || eq(it->distance, dist))
                && (it->last_distance > dist || eq(it->last_distance, dist))){
            tracked->push_back(*it);
        }
    }
}

SurfHist *SurfSegment::_horizontalDiffsHist(float dist,
                                            std::list<SurfLandmark> &tracked,
                                            std::vector<SurfLandmark> &lms)
{
    SurfLandmark lm;
    size_t max_count = 150; // TODO: parametrize this?
    bool found;
    float hd;
    std::vector<SurfLandmark>::iterator best[2];
    std::list<float> hds;

    // sort landmarks - those seen most often are first
    sortLandmarksByVisibility(tracked);

    for (size_t i = 0; tracked.size() > 0 && i < max_count; i++){
        // pick up best landmark
        lm = tracked.front();
        tracked.pop_front();

        // find two best matching landmarks from lms
        found = _bestMatching(lm, lms, best, best + 1);
        //DBG("found: " << found << " " << best[0]->dist(lm) << " " << best[1]->dist(lm));
        if (found && best[0]->dist(lm) * 2. < best[1]->dist(lm) /*TODO best[0] << best[1] */){
            // compute horizontal difference
            hd  = fabsf(lm.last_x - lm.x);
            hd /= fabsf(lm.last_distance - lm.distance);
            hd *= dist - lm.last_distance;
            hd += lm.x - best[0]->x;

            // store difference in list
            hds.push_back(hd);
        }
    }

    // TODO: parametrize binsize
    return new SurfHist(.1f, hds);
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

