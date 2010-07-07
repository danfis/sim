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

#ifndef COMPONENT_HPP_
#define COMPONENT_HPP_

TEST(compSetUp);
TEST(compTearDown);
TEST(compPrePostStep);
TEST(compMsg);

TEST_SUITE(TSComponent) {
    TEST_ADD(compSetUp),

    TEST_ADD(compPrePostStep),
    TEST_ADD(compMsg),

    TEST_ADD(compTearDown),
    TEST_SUITE_CLOSURE
};

#endif
