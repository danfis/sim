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

#ifndef _SIM_VIS_WORLD_HPP_
#define _SIM_VIS_WORLD_HPP_

#include <osgViewer/CompositeViewer>
#include <osgGA/TrackballManipulator>

#include "visbody.hpp"

namespace sim {

/**
 * Visual representation of world.
 * Based on openscenegraph.
 */
class VisWorld {
    osg::ref_ptr<osgViewer::CompositeViewer> _viewer; /*!< Main viewer */
    osg::ref_ptr<osgViewer::View> _view_main;

    osg::ref_ptr<osg::StateSet> _state_set;
    osg::ref_ptr<osg::Group> _root; /*!< Root of scene graph */

    osg::ref_ptr<osg::Group> _cams; /*!< Offline cameras */

    /**
     * Root of visible environment. This node should contain all bodies,
     * lights etc.
     */
    osg::ref_ptr<osg::Group> _root_vis;

    osg::ref_ptr<osg::Group> _g_bodies; //!< Group of VisBodies
    osg::ref_ptr<osg::Group> _lights;

    osg::ref_ptr<osg::Switch> _coord_frame;

    osg::ref_ptr<osg::Group> _off_scene;

    std::list<VisBody *> _bodies;

    bool _window; /*!< Show a window? */

  public:
    VisWorld();
    virtual ~VisWorld();

    osgViewer::CompositeViewer *viewer() { return _viewer.get(); }
    const osgViewer::CompositeViewer *viewer() const { return _viewer.get(); }
    osgViewer::View *viewMain() { return _view_main.get(); }
    const osgViewer::View *viewMain() const { return _view_main.get(); }
    bool window() const { return _window; }
    void setWindow(bool yes = true) { _window = yes; }

    const std::list<VisBody *> &bodies() const { return _bodies; }
    std::list<VisBody *> &bodies() { return _bodies; }

    /**
     * Returns root of scene graph.
     */
    osg::Node *sceneRoot() { return _root_vis; }

    /**
     * Adds visualisable object.
     */
    void addBody(VisBody *obj);
    void rmBody(VisBody *obj);

    void addView(osgViewer::View *view);
    void rmView(osgViewer::View *view);

    void addCam(osg::Camera *cam);
    void rmCam(osg::Camera *cam);

    void addOffScene(osg::Node *n);
    void rmOffScene(osg::Node *n);

    /**
     * Initializes world.
     */
    void init();

    /**
     * Finalize world.
     */
    void finish();

    /**
     * Performes one step.
     */
    void step();

    bool done();

    void toggleWireframe();

	void toggleAlpha();

	void toggleAxis();

  protected:
    void _setUpStateSet();
    void _setUpLights();

    void _createCoordFrame();

};

}

#endif /* _SIM_VIS_WORLD_HPP_ */
