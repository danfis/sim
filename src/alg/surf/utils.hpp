/*********************************************************** 
*  --- OpenSURF ---                                       *
*  This library is distributed under the GNU GPL. Please   *
*  use the contact form at http://www.chrisevansdev.com    *
*  for more information.                                   *
*                                                          *
*  C. Evans, Research Into Robust Visual Features,         *
*  MSc University of Bristol, 2008.                        *
*                                                          *
************************************************************/

#ifndef _SIM_ALG_SURF_UTILS_HPP_
#define _SIM_ALG_SURF_UTILS_HPP_

#include <vector>
#include <cv.h>
#include <sim/alg/surfnav.hpp>


namespace sim {
namespace alg {
namespace surf {

//! Display error message and terminate program
void error(const char *msg);

//! Show the provided image and wait for keypress
void showImage(const IplImage *img);

//! Show the provided image in titled window and wait for keypress
void showImage(char *title,const IplImage *img);

// Convert image to single channel 32F
IplImage* getGray(const IplImage *img);

//! Draw a single feature on the image
void drawIpoint(IplImage *img, sim::alg::SurfLandmark &ipt, int tailSize = 0);

//! Draw all the Ipoints in the provided vector
void drawIpoints(IplImage *img, std::vector<sim::alg::SurfLandmark> &ipts, int tailSize = 0);

//! Draw descriptor windows around Ipoints in the provided vector
void drawWindows(IplImage *img, std::vector<sim::alg::SurfLandmark> &ipts);

// Draw the FPS figure on the image (requires at least 2 calls)
void drawFPS(IplImage *img);

//! Draw a Point at feature location
void drawPoint(IplImage *img, sim::alg::SurfLandmark &ipt);

//! Draw a Point at all features
void drawPoints(IplImage *img, std::vector<sim::alg::SurfLandmark> &ipts);

//! Save the SURF features to file
void saveSurf(char *filename, std::vector<sim::alg::SurfLandmark> &ipts);

//! Load the SURF features from file
void loadSurf(char *filename, std::vector<sim::alg::SurfLandmark> &ipts);

//! Round float to nearest integer
inline int fRound(float flt)
{
  return (int) floor(flt+0.5f);
}

} /* namespace surf */
} /* namespace alg */
} /* namespace sim */

#endif
