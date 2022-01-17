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
#include <algorithm>

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
    const int TILE_SIZE = 100;
    Grid grid;

    enum class MoveStates_t {IDLE, EXPLORE, SELECT_DOOR, GOTO_CLOSE_DOOR, REFINE_DOOR, GOTO_DOOR, GOTO_CENTER, INIT_TURN};
    MoveStates_t moveState = MoveStates_t::INIT_TURN;

    struct Target_t {
        QPointF point;
        bool activo = false;
    } target;

    const int MAX_ADV_SPEED = 1000;

    Eigen::Vector2f robotToWorld(Eigen::Vector2f p_world, Eigen::Vector2f p_robot, float angle);
    Eigen::Vector2f worldToRobot(Eigen::Vector2f p_world, Eigen::Vector2f p_robot, float angle);

    void draw_laser(const RoboCompLaser::TLaserData &ldata);
    void update_map(const RoboCompLaser::TLaserData &ldata, RoboCompFullPoseEstimation::FullPoseEuler r_state);

    float reduce_speed_if_turning(float angle);
    float reduce_speed_if_close_to_target(float dist);
    void calculateDistRot(float &vel, float &rot, RoboCompFullPoseEstimation::FullPoseEuler bState);

    std::vector<Eigen::Vector2f> get_peaks(auto ldata, auto r_state);

    struct Door
    {
        Eigen::Vector2f p1, p2;
        Eigen::Vector2f get_midpoint() const {return p1 + ((p2-p1)/2.0);};
        Eigen::Vector2f get_external_midpoint() const
        {
            Eigen::ParametrizedLine<float, 2> r =  Eigen::ParametrizedLine<float, 2>(get_midpoint(), (p1-p2).unitOrthogonal());
            //qInfo() << __FUNCTION__ << r.pointAt(800.0).x() << r.pointAt(800.0).y();
            return r.pointAt(1600.0);
        };

        // Valores negativos te dan el punto más cerca de ti
        Eigen::Vector2f get_point_closeDoor() const
        {
            Eigen::ParametrizedLine<float, 2> r =  Eigen::ParametrizedLine<float, 2>(get_midpoint(), (p1-p2).unitOrthogonal());
            //qInfo() << __FUNCTION__ << r.pointAt(800.0).x() << r.pointAt(800.0).y();
            return r.pointAt(-700);
        };

        // Guarda las habitaciones que conecta la puerta.
		std::set<int> to_rooms;
        void insertRoom(int room) {
            to_rooms.insert(room);
        }

        bool operator==(const Door& d){
            const int THRESHOLD = 500;
            return ((p1 - d.p1).norm() < THRESHOLD and (p2 - d.p2).norm() < THRESHOLD)
                or ((p1 - d.p2).norm() < THRESHOLD and (p2 - d.p1).norm() < THRESHOLD);
        };

        void refine_points(std::vector<Eigen::Vector2f> peaks) {
            // De cada punto (p1, p2) obtener el pico que más cerca esté
            float menorDistP1 = 100000, menorDistP2 = 100000;
            Eigen::Vector2f picoMenorP1, picoMenorP2;

            for(Eigen::Vector2f p : peaks) {
                if((p - p1).norm() < menorDistP1) {
                    menorDistP1 = (p - p1).norm();
                    picoMenorP1 = p;
                }
                if((p - p2).norm() < menorDistP2) {
                    menorDistP2 = (p - p2).norm();
                    picoMenorP2 = p;
                }
            }

            // Si la distancia del pico al punto es menor que cierto umbral redefinir el punto
            const int UMBRAL_CAMBIO = 300;
            if(menorDistP1 < UMBRAL_CAMBIO)
                p1 = picoMenorP1;
            if(menorDistP2 < UMBRAL_CAMBIO)
                p2 = picoMenorP2;
        }

    };
    std::vector<Door> doors;
    Door selectedDoor;

    float init_angle;

    void draw_doors();
    void draw_peaks(std::vector<Eigen::Vector2f> peaks);
    void draw_door_midpoint(Door d);

    void print_all_doors();
    void print_graph();
};

#endif
