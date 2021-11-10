/*
 *    Copyright (C) 2021 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
	\brief
	@author authorname
*/



#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>
#include "abstract_graphic_viewer/abstract_graphic_viewer.h"
#include "grid2d/grid.h"
#include <eigen3/Eigen/Eigen>

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(TuplePrx tprx, bool startup_check);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);



public slots:
	void compute();
	int startup_check();
	void initialize(int period);
    void new_target_slot(QPointF point);
private:
	std::shared_ptr < InnerModel > innerModel;
	bool startup_check_flag;
    AbstractGraphicViewer *viewer;
    const int ROBOT_LENGTH = 400;
    QGraphicsPolygonItem *robot_polygon;
    QGraphicsRectItem *laser_in_robot_polygon;

    // GRID
    const int TILE_SIZE = 200;
    Grid grid;

    enum MoveStates_t {ADVANCE, TURN, BORDER, STOP} moveState;

    const int MAX_ADV_SPEED = 1000;

    Eigen::Vector2f robotToWorld(Eigen::Vector2f p_world, Eigen::Vector2f p_robot, float angle);
    Eigen::Vector2f worldToRobot(Eigen::Vector2f p_world, Eigen::Vector2f p_robot, float angle);

    void draw_laser(const RoboCompLaser::TLaserData &ldata);
    void update_map(const RoboCompLaser::TLaserData &ldata, RoboCompFullPoseEstimation::FullPoseEuler r_state);
};

#endif
