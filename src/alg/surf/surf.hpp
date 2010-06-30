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

#ifndef _SIM_ALG_SURF_SURF_HPP_
#define _SIM_ALG_SURF_SURF_HPP_

#include <vector>
#include <cv.h>
#include <sim/alg/surfnav.hpp>
#include "integral.hpp"

namespace sim {
namespace alg {
namespace surf {

class Surf {
  
  public:
    
    //! Standard Constructor (img is an integral image)
    Surf(IplImage *img, std::vector<sim::alg::SurfLandmark> &ipts);

    //! Describe all features in the supplied vector
    void getDescriptors(bool bUpright = false);
  
  private:
    
    //---------------- Private Functions -----------------//

    //! Assign the current sim::alg::SurfLandmark an orientation
    void getOrientation();
    
    //! Get the descriptor. See Agrawal ECCV 08
    void getDescriptor(bool bUpright = false);

    //! Calculate the value of the 2d gaussian at x,y
    inline float gaussian(int x, int y, float sig);
    inline float gaussian(float x, float y, float sig);

    //! Calculate Haar wavelet responses in x and y directions
    inline float haarX(int row, int column, int size);
    inline float haarY(int row, int column, int size);

    //! Get the angle from the +ve x-axis of the vector given by [X Y]
    float getAngle(float X, float Y);


    //---------------- Private Variables -----------------//

    //! Integral image where sim::alg::SurfLandmarks have been detected
    IplImage *img;

    //! sim::alg::SurfLandmarks vector
    std::vector<sim::alg::SurfLandmark> &ipts;

    //! Index of current sim::alg::SurfLandmark in the vector
    int index;
};

} /* namespace surf */
} /* namespace alg */
} /* namespace sim */

#endif
