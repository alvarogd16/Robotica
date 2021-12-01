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
#include <cppitertools/combinations_with_replacement.hpp>
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
    QRect dimensions(-5100, -2600, 10200, 5200);
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
    static QGraphicsItem *left_point_draw=nullptr, *right_point_draw=nullptr;

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

    float advance = 0, rot = 0.5;
    int initial_num_doors  = doors.size();
    static int current_room = 0;
    float init_angle = 0;
    switch(moveState)
    {
        case MoveStates_t::IDLE:{
            moveState = MoveStates_t::INIT_TURN;
            break;}

        case MoveStates_t::INIT_TURN:{
            initial_num_doors = doors.size();
            init_angle = (r_state.rz < 0) ? (2 * M_PI + r_state.rz) : r_state.rz;
            moveState = MoveStates_t::EXPLORE;
            break;}

        case MoveStates_t::EXPLORE:{
            float current = (r_state.rz < 0) ? (2 * M_PI + r_state.rz) : r_state.rz;
            if (fabs(current - init_angle) < (M_PI + 0.1) and fabs(current - init_angle) > (M_PI - 0.1))
            {
                try
                { differentialrobot_proxy->setSpeedBase(0.0, 0.0); }
                catch (const Ice::Exception &e)
                { std::cout << e.what() << std::endl; }
                    moveState = MoveStates_t::SELECT_DOOR;
            }
            // search for corners
            // compute derivative wrt distance
            std::vector<float> derivatives(ldata.size());
            derivatives[0] = 0;
            for (const auto &&[k, l]: iter::sliding_window(ldata, 2) | iter::enumerate)
                derivatives[k + 1] = l[1].dist - l[0].dist;

            // filter derivatives greater than a threshold
            std::vector<Eigen::Vector2f> peaks;
            for (const auto &&[k, der]: iter::enumerate(derivatives))
            {
                if (der > 1000)
                {
                    const auto &l = ldata.at(k - 1);
                    peaks.push_back(robotToWorld(Eigen::Vector2f(l.dist * sin(l.angle), l.dist * cos(l.angle)), Eigen::Vector2f(r_state.x, r_state.y), r_state.rz));
                } else if (der < -1000)
                {
                    const auto &l = ldata.at(k);
                    peaks.push_back(robotToWorld(Eigen::Vector2f(l.dist * sin(l.angle), l.dist * cos(l.angle)), Eigen::Vector2f(r_state.x, r_state.y), r_state.rz));
                }
            }
            qInfo() << __FUNCTION__ << "peaks " << peaks.size();

            // pairwise comparison of peaks to filter in doors
            for (auto &&c: iter::combinations_with_replacement(peaks, 2))
            {
                qInfo() << __FUNCTION__ << "dist " << (c[0] - c[1]).norm();
                if ((c[0] - c[1]).norm() < 1100 and (c[0] - c[1]).norm() > 500)
                {
                    Door d{c[0], c[1]};
                    d.to_rooms.insert(current_room);
                    if (auto r = std::find_if(doors.begin(), doors.end(), [d](auto a) { return d == a; }); r == doors.end())
                        doors.emplace_back(d);
                }
            }

            // draw
            qInfo() << __FUNCTION__ << "---------- doors" << doors.size();
            static std::vector<QGraphicsItem *> door_lines;
            for (auto dp: door_lines) viewer->scene.removeItem(dp);
            door_lines.clear();
            for (const auto r: doors)
            {
                door_lines.push_back(viewer->scene.addLine(r.p1.x(), r.p1.y(), r.p2.x(), r.p2.y(), QPen(QColor("Magenta"), 100)));
                door_lines.back()->setZValue(200);
            }
            break;}

        case MoveStates_t::SELECT_DOOR:{
            // Seleccionar una puerta
            std::vector<Door> doorsToGo;
            auto gen = std::mt19937{std::random_device{}()};
            for(Door &d : doors) {
                for(const auto &to_r : d.to_rooms)
                    if(to_r == current_room)
                        doorsToGo.push_back(d);
            }

            if(doorsToGo.empty()) {
                qInfo() << "Todas las puertas visitadas";
                moveState = MoveStates_t::IDLE;
            } else {
                selectedDoor = doorsToGo.back();

                qInfo() << "Room i'am " << *selectedDoor.to_rooms.begin();

                float aux_p1_x = selectedDoor.get_midpoint().x();
                float aux_p1_y = selectedDoor.get_midpoint().y();

                qInfo() << "Puntos puerta" << selectedDoor.p1.x() << selectedDoor.p1.y() << selectedDoor.p2.x() << selectedDoor.p2.y();
                qInfo() << "Punto medio" << aux_p1_x << aux_p1_y;

                auto p = viewer->scene.addRect(QRectF(aux_p1_x-100, aux_p1_y-100, 200, 200), QPen(QColor("Blue"), 30), QBrush("Blue"));
                p->setZValue(200);

                target.point = QPointF(selectedDoor.get_midpoint().x(), selectedDoor.get_midpoint().y());
                target.activo = true;

                moveState = MoveStates_t::GOTO_DOOR;
            }
            break;}


        case MoveStates_t::GOTO_DOOR:{
            //Avanza hacia la puerta
            //Importante avanzar hacia el centro para no golpear las paredes

            if(target.activo) {
                calculateDistRot(advance, rot, r_state);
            } else {
                advance = 0;
                rot = 0;
                qInfo() << "Ha llegado a la mitad de la puerta";

                target.point = QPointF(selectedDoor.get_external_midpoint().x(), selectedDoor.get_external_midpoint().y());
                target.activo = true;
                current_room+=1;
                moveState = MoveStates_t::GOTO_CENTER;
            }

            break;}

        case MoveStates_t::GOTO_CENTER:{
                // find and store doors

                if(target.activo)
                    calculateDistRot(advance, rot, r_state);
                else {
                    advance = 0;
                    rot = 0;
                    qInfo() << "Ha llegado a la 'mitad' de la habitación";

                    moveState = MoveStates_t::INIT_TURN;
                }
            break;}
    }

    try
    {
        differentialrobot_proxy->setSpeedBase(advance, rot);
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


float SpecificWorker::reduce_speed_if_turning(float angle) {
    return std::exp(-std::pow(angle, 2) / 0.02714);
}

float SpecificWorker::reduce_speed_if_close_to_target(float dist) {
    if(dist >= 1000)
        return 1;
    else
        return dist / 1000;
}

void SpecificWorker::calculateDistRot(float &vel, float &rot, RoboCompFullPoseEstimation::FullPoseEuler bState) {
    float dist, angle;

    // Pasar target a robot coord
    Eigen::Vector2f p_world(target.point.x(), target.point.y());
    Eigen::Vector2f p_robot(bState.x, bState.y);
    Eigen::Vector2f target_robot = worldToRobot(p_world, p_robot, bState.rz);

    // Calcular angulo entre robot y target
    angle = atan2(-target_robot.y(), target_robot.x()) + M_PI_2;
    rot = angle;

    // Calcular velocidad de avance
    dist = sqrt(pow(target_robot.x(), 2) + pow(target_robot.y(), 2));
    vel = 1000 * reduce_speed_if_turning(angle) * reduce_speed_if_close_to_target(dist);

    if(angle < 0.2 && angle > -0.2) {
        if(dist < 300) {
            std::cout << "Parado" << std::endl;
            vel = 0;
            rot = 0;
            target.activo = false;
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

