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
#include "specificworker.h"
#include <cppitertools/range.hpp>
#include <cppitertools/sliding_window.hpp>
#include <cppitertools/enumerate.hpp>

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
	this->startup_check_flag = startup_check;
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
//	THE FOLLOWING IS JUST AN EXAMPLE
//	To use innerModelPath parameter you should uncomment specificmonitor.cpp readConfig method content
//	try
//	{
//		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
//		std::string innermodel_path = par.value;
//		innerModel = std::make_shared(innermodel_path);
//	}
//	catch(const std::exception &e) { qFatal("Error reading config params"); }






	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
	this->Period = period;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
		timer.start(Period);
	}
    QRect dimensions(-5000, -2500, 10000, 5000);
    viewer = new AbstractGraphicViewer(this, dimensions);
    this->resize(900,450);
    robot_polygon = viewer->add_robot(ROBOT_LENGTH);
    laser_in_robot_polygon = new QGraphicsRectItem(-10, 10, 20, 20, robot_polygon);
    laser_in_robot_polygon->setPos(0, 190);     // move this to abstract
    try
    {
        RoboCompGenericBase::TBaseState bState;
        differentialrobot_proxy->getBaseState(bState);
    }
    catch(const Ice::Exception &e) { std::cout << e.what() << std::endl;}
    connect(viewer, &AbstractGraphicViewer::new_mouse_coordinates, this, &SpecificWorker::new_target_slot);

    grid.initialize(dimensions, TILE_SIZE, &viewer->scene, false);
//    grid.add_hit(Eigen::Vector2f(0, 0));
}

void SpecificWorker::compute()
{
    // Leer y pintar robot
    RoboCompLaser::TLaserData ldata;
    RoboCompFullPoseEstimation::FullPoseEuler r_state;
    try
    {
        r_state = fullposeestimation_proxy->getFullPoseEuler();
        robot_polygon->setRotation(r_state.rz*180/M_PI);
        robot_polygon->setPos(r_state.x, r_state.y);

        // Leer y pintar laser
        ldata = laser_proxy->getLaserData();
        draw_laser(ldata);
        update_map(ldata, r_state);
    }
    catch(const Ice::Exception &e){ std::cout << e.what() << std::endl;}

    float advance = 0, rot = 0;
    int initial_num_doors  = doors.size();
    float init_angle = 0;
    switch(moveState)
    {
        case MoveStates_t::IDLE:
            moveState = MoveStates_t::INIT_TURN;
            break;

        case MoveStates_t::INIT_TURN:
            initial_num_doors = doors.size();
            init_angle = r_state.rz;
            moveState = MoveStates_t::EXPLORE;
            break;

        case MoveStates_t::EXPLORE:
        {
//            qInfo() << init_angle << r_state.rz;
            if((init_angle + r_state.rz) > 2*M_PI)
            {
                // if no more doors found
                if(doors.size() == initial_num_doors )
                {
                    moveState = MoveStates_t::IDLE;
                    break;
                }
            }
            else
            {
                rot = 0;
                advance = 0;

                // build derivative
                // Tenemos un vector que almacena las diferencias de distancias de dos puntos del lase (dist_derivate)
                // este se rellena comparando las posiciones del laser de dos en dos (con la libreria iter)
                std::vector<float> dist_derivative(ldata.size());
                for (auto &&[k, p]: iter::sliding_window(ldata, 2) | iter::enumerate)
                    dist_derivative[k] = fabs(p[1].dist - p[0].dist);

                // Guardas los dos primeros valores mas altos ( las distancias entre dos puntos más grande )
                // y comparamos si son mayores que 1000 para saber si pueden llegar a ser puertas
                auto first = std::ranges::max_element(dist_derivative);
                float first_f = *first;
                int first_pos = std::distance(dist_derivative.begin(), first-1);
                dist_derivative.erase(dist_derivative.begin() + first_pos);

                auto second = std::ranges::max_element(dist_derivative);
                float second_f = *second;
                int second_pos = std::distance(dist_derivative.begin(), second+1);

                qInfo() << first_f << second_f;
                if (first_f > 1000 and second_f > 1000) {
                    auto point_first = ldata.at(first_pos);
                    auto point_second = ldata.at(second_pos);
                    Eigen::Vector2f first_e(point_first.dist * sin(point_first.angle),
                                            point_first.dist * sin(point_first.angle));
                    Eigen::Vector2f second_e(point_second.dist * sin(point_second.angle),
                                             point_second.dist * sin(point_second.angle));



                    // Si la diferencia entre los puntos es menor que 600 no entra el robot y si es mayor que 1100
                    // no se considera puerta sino espacio abierto
                    qInfo() << (first_e - second_e).norm();
                    if ((first_e - second_e).norm() > 600 and (first_e - second_e).norm() < 1100)
                    {
                        qInfo() << first_e.x() << first_e.y() << " Segunda: " << second_e.x() << second_e.y();

                        Door b = Door{first_e, second_e};
                        if(auto res = std::ranges::find_if_not(doors, [b](auto a){ return a == b;}); res == doors.end()) {
                            doors.emplace_back(b);
                        }
                    }
                }
            }
            qInfo() << doors.size();
            break;
        }
        case MoveStates_t::GOTO_DOOR:
            break;

        case MoveStates_t::GOTO_CENTER:
                // find and store doors
            break;
    }

    try
    {
//        differentialrobot_proxy->setSpeedBase(advance, rot);
    } catch (const Ice::Exception &e) {std::cout << e.what() << std::endl;}
}
/////////////////////////////////////////////////////////////////////////////////
int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}

void SpecificWorker::new_target_slot(QPointF point)
{
    qInfo() << point;
}

Eigen::Vector2f SpecificWorker::worldToRobot(Eigen::Vector2f p_world, Eigen::Vector2f p_robot, float angle) {
    Eigen::Matrix2f m_rotation;
    m_rotation << cos(angle), sin(angle), -sin(angle), cos(angle);

    return m_rotation * (p_world - p_robot);
}

Eigen::Vector2f SpecificWorker::robotToWorld(Eigen::Vector2f p_world, Eigen::Vector2f p_robot, float angle) {
    Eigen::Matrix2f m_rotation;
    m_rotation << cos(angle), -sin(angle), sin(angle), cos(angle);

    return m_rotation * p_world + p_robot;
}

void SpecificWorker::draw_point(Eigen::Vector2f p) {

}

void SpecificWorker::draw_laser(const RoboCompLaser::TLaserData &ldata) // robot coordinates
{
    static QGraphicsItem *laser_polygon = nullptr;

    if(laser_polygon != nullptr)
        viewer->scene.removeItem(laser_polygon);
    // code to delete any existing laser graphic element

    QPolygonF poly;

    QPointF point(0, 0);
    poly << point;
    for(auto &p : ldata) {
        point.setX(p.dist * sin(p.angle));
        point.setY(p.dist * cos(p.angle));

        poly << point;
    }
    // code to fill poly with the laser polar coordinates (angle, dist) transformed to cartesian coordinates (x,y), all in the robot's  // reference system

    QColor color("LightGreen");
    color.setAlpha(40);
    laser_polygon = viewer->scene.addPolygon(laser_in_robot_polygon->mapToScene(poly), QPen(QColor("DarkGreen"), 30), QBrush(color));
    laser_polygon->setZValue(3);
}

void SpecificWorker::update_map(const RoboCompLaser::TLaserData &ldata, RoboCompFullPoseEstimation::FullPoseEuler r_state)
{
    for(const auto &l : ldata)
    {
        if(l.dist > 450/2.0)
        {
            Eigen::Vector2f tip(l.dist*sin(l.angle), l.dist*cos(l.angle));
            Eigen::Vector2f p = robotToWorld(tip, Eigen::Vector2f(r_state.x, r_state.y), r_state.rz);
            int target_kx = (p.x() - grid.dim.left()) / grid.TILE_SIZE;
            int target_kz = (p.y() - grid.dim.bottom()) / grid.TILE_SIZE;
            int last_kx = -1000000;
            int last_kz = -1000000;

            int num_steps = ceil(l.dist/(TILE_SIZE/2.0));
            for(const auto &&step : iter::range(0.0, 1.0-(1.0/num_steps), 1.0/num_steps))
            {
                Eigen::Vector2f p = robotToWorld(tip*step,Eigen::Vector2f(r_state.x, r_state.y), r_state.rz);
                int kx = (p.x() - grid.dim.left()) / grid.TILE_SIZE;
                int kz = (p.y() - grid.dim.bottom()) / grid.TILE_SIZE;
                if(kx != last_kx and kx != target_kx and kz != last_kz and kz != target_kz)
                    grid.add_miss(robotToWorld(tip * step, Eigen::Vector2f(r_state.x, r_state.y), r_state.rz));
                last_kx = kx;
                last_kz = kz;
            }
            if(l.dist <= 4000)
                grid.add_hit(robotToWorld(tip, Eigen::Vector2f(r_state.x, r_state.y), r_state.rz));
        }
    }
}

/**************************************/
// From the RoboCompDifferentialRobot you can call this methods:
// this->differentialrobot_proxy->correctOdometer(...)
// this->differentialrobot_proxy->getBasePose(...)
// this->differentialrobot_proxy->getBaseState(...)
// this->differentialrobot_proxy->resetOdometer(...)
// this->differentialrobot_proxy->setOdometer(...)
// this->differentialrobot_proxy->setOdometerPose(...)
// this->differentialrobot_proxy->setSpeedBase(...)
// this->differentialrobot_proxy->stopBase(...)

/**************************************/
// From the RoboCompDifferentialRobot you can use this types:
// RoboCompDifferentialRobot::TMechParams

/**************************************/
// From the RoboCompLaser you can call this methods:
// this->laser_proxy->getLaserAndBStateData(...)
// this->laser_proxy->getLaserConfData(...)
// this->laser_proxy->getLaserData(...)

/**************************************/
// From the RoboCompLaser you can use this types:
// RoboCompLaser::LaserConfData
// RoboCompLaser::TData

