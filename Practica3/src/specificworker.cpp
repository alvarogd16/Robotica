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
        last_point = QPointF(bState.x, bState.z);
    }
    catch(const Ice::Exception &e) { std::cout << e.what() << std::endl;}
    connect(viewer, &AbstractGraphicViewer::new_mouse_coordinates, this, &SpecificWorker::new_target_slot);

}

void SpecificWorker::compute()
{
    // Leer y pintar robot
    RoboCompGenericBase::TBaseState bState;
    differentialrobot_proxy->getBaseState(bState);
    robot_polygon->setRotation(bState.alpha*180/M_PI);
    robot_polygon->setPos(bState.x, bState.z);

    // Leer y pintar laser
    auto ldata = laser_proxy->getLaserData();
    draw_laser(ldata);


    if(target.activo) {
        float vel, rot;

        // Pasar target a robot coord
        Eigen::Vector2f p_world(target.point.x(), target.point.y());
        Eigen::Vector2f p_robot(bState.x, bState.z);
        Eigen::Vector2f target_robot = worldToRobot(p_world, p_robot, bState.alpha);

        // Calcular angulo entre robot y target
        rot = atan2(target_robot.x(), target_robot.y());
        std::cout << rot << std::endl;


        try {
            if(rot < 0.2) {
                rot = 0;
                target.activo = false;
                std::cout << "Girooo!!" << std::endl;
            }

            differentialrobot_proxy->setSpeedBase(0, rot);
        } catch (const Ice::Exception &e) {
            std::cout << e.what() << std::endl;
        }

        // Calcular velocidad de avance
        // vel =

        // Mandar a robot
        /*
        try {
            differentialrobot_proxy->setSpeedBase(vel, rot);
        } catch (const Ice::Exception &e) {
            std::cout << e.what() << std::endl;
        }*/
    }
}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}

void SpecificWorker::new_target_slot(QPointF point) {
    qInfo() << point;
    target.point = point;
    target.activo = true;
}

Eigen::Vector2f SpecificWorker::worldToRobot(Eigen::Vector2f p_world, Eigen::Vector2f p_robot, float angle) {
    Eigen::Matrix2f m_rotation;
    m_rotation << cos(angle), -sin(angle), sin(angle), cos(angle);

    return m_rotation * (p_world - p_robot);
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

