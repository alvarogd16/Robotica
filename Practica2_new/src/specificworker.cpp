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

    for(int i = 0; i < (10000/400); i++)
        for(int j = 0; j < (5000/400); j++)
            visitadosMap[i][j] = 0;

    changeState(SPIRAL);
}

void SpecificWorker::compute()
{
	try {

        // Obtención de datos
        RoboCompGenericBase::TBaseState rdata;
        differentialrobot_proxy->getBaseState(rdata);
        auto ldata = laser_proxy->getLaserData();

        // Tranformacion de coordenadas
        int xRobotMap, zRobotMap;
        coordRobotToMap(rdata.x, rdata.z, xRobotMap, zRobotMap);
        // std::cout << " " << xRobotMap << " " << zRobotMap << std::endl;

        // Actualización del mapa
        visitadosMap[xRobotMap][zRobotMap] = 1;


        std::sort(ldata.begin()+10, ldata.end()-10, [](RoboCompLaser::TData a, RoboCompLaser::TData b) { return a.dist < b.dist; });

        
        float minDist = ldata[10].dist;
        float minAngle = ldata[10].angle;
        // std::cout << "Min dist: " << minDist << std::endl;
        if(minDist == 0) std::cout << ldata[10].angle << std::endl;
        const float minTope = 500;


        giroDerecha = minAngle < 0;


        // Todos los estados mueven el robot
        differentialrobot_proxy->setSpeedBase(robotMove.adv, robotMove.rot);

        switch (moveState) {
            case ADVANCE:
                timeAdvance++;
                if(minDist <= minTope && minDist != 0){
                    changeState(OBSTACLE);
                    std::cout << "MinAngle: " << minAngle << std::endl;
                }
                else if(minDist > 700 && timeAdvance > 75)
                    changeState(SPIRAL);
            break;

            case SPIRAL:
                if(robotMove.rot < 0.5) 
                    robotMove.rot -= 0.0005;
                else 
                    robotMove.rot -= 0.0025;


                if(minDist <= minTope && minDist != 0)
                    changeState(OBSTACLE);
                else if(casillasCercanasVisitadas(xRobotMap, zRobotMap) || robotMove.rot < 0.3)
                    changeState(ADVANCE);
            break;

            case OBSTACLE:
                if(minDist > minTope)
                    changeState(ADVANCE);
            break;

            case STOP:
            break;
        }


    }
    catch(const Ice::Exception &e) { std::cout << e << std::endl; }
}

void SpecificWorker::changeState(MoveStates_t newState) {
    moveState = newState;

    switch(newState) {
        case ADVANCE:
            std::cout << "ADVANCE" << std::endl;
            robotMove.adv = 600;
            robotMove.rot = 0;
        break;

        case SPIRAL:
            std::cout << "SPIRAL" << std::endl;
            robotMove.adv = 600;
            robotMove.rot = 1.5;
            timeAdvance = 0;
        break;
        
        case OBSTACLE:
            std::cout << "OBSTACLE" << std::endl;
            robotMove.adv = 10;
            robotMove.rot = giroDerecha ? 1 : -1;
            timeAdvance = 0;
            // giroDerecha = !giroDerecha;
        break;

        case STOP:
            std::cout << "STOP" << std::endl;
            robotMove.adv = 0;
            robotMove.rot = 0;
        break;           
    }
}

void SpecificWorker::coordRobotToMap(float xR, float zR, int &xM, int &zM) {
    xM = (int) ((10000.0/400.0) / 10000.0 * (xR+5000));
    zM = (int) ((5000.0/400.0) / 5000.0 * (zR+2500));
}

bool SpecificWorker::casillasCercanasVisitadas(int x, int z) {
    return visitadosMap[x-1][z-1] && visitadosMap[x-1][z] && visitadosMap[x-1][z+1] 
            && visitadosMap[x][z-1] && visitadosMap[x][z] && visitadosMap[x][z+1]
            && visitadosMap[x+1][z-1] && visitadosMap[x+1][z] && visitadosMap[x+1][z+1];
}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
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

