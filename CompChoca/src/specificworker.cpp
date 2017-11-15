/*
 *    Copyright (C) 2017 by YOUR NAME HERE
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
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{

}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	innermodel = new InnerModel("/home/robocomp/robocomp/files/innermodel/simpleworld.xml");
	timer.start(Period);
	return true;
}


//Decala x y z dentro de  la estructura, despues un bool para comprobar si esta llena o no 

void SpecificWorker::compute()
{
     
    RoboCompDifferentialRobot::TBaseState bState;
    
    differentialrobot_proxy->getBaseState(bState);
    innermodel->updateTransformValues( "base", bState.x, 0, bState.z, 0, bState.alpha, 0);
     
    switch( state ) {
        case State::IDLE:
            if ( !target.isEmpty() ){
                target.calcularPuntos(bState.x, bState.z);
                state = State::GOTO;
            }
            break;

        case State::GOTO:
            gotoTarget();
            break;

        case State::BUG:
            bug();
            break;
    }
//     TLaserData laser ; 
//     
//     laser = laser_proxy->getLaserData();
//     std::sort(laser.end()-20,laser.end()-10,[](auto a, auto b){return a.dist>b.dist;});
//     std::cout<<laser[20].dist<<endl;
//     if(t.isEmpty() == false)
//     {     
//         QVec tR = innermodel->transform("base", QVec::vec3(tr.first, 0 , tr.second), "world");
//         float d=tR.norm2();        
//         if(d > 50)
//         {
//             //adv = d;
//             //if(adv > MAX_ADV)  adv = MAX_ADV;
// 	  
//             vrot = atan2(tR.x(), tR.z());
//             std::cout<<vrot<<endl;
//             if(vrot > MAX_VROT) vrot = MAX_VROT;
// 	    if(vrot < -MAX_VROT) vrot = -MAX_VROT;
// 	    
// 	    adv = MAX_ADV * sigmoid(d)  * gaussian(vrot, 0.5, 1);
// 	    differentialrobot_proxy->setSpeedBase(adv,vrot);
//         }
//         else
//         {
//             differentialrobot_proxy->setSpeedBase(0, 0);
//             t.setEmpty(true);    
//         }
//     }
  
}

void SpecificWorker::gotoTarget(){

    std::pair<float, float> tr = target.getValores();
    QVec rt = innermodel->transform("base", QVec::vec3(tr.first, 0 , tr.second), "world");
    
    float dist = rt.norm2();
    float ang  = atan2(rt.x(), rt.z());
    float adv;
    
    if( obstacle() == true){   // If ther is an obstacle ahead, then transit to BUG
        state = State::BUG;
        return;
    }
    
     if(dist < 100){         // If close to obstacle stop and transit to IDLE
        state = State::IDLE;
        target.setEmpty(true);
        differentialrobot_proxy->setSpeedBase(0,0); 
        return;
    }
    
    dist = distObstacle(dist);
    
    if ( fabs(ang) > 0.05 )
        adv = 0;
    else
        adv = dist * sigmoid(dist)  * gaussian(ang, 0.5, 1);
    
    if(ang > MAX_VROT) 
        ang = MAX_VROT;
    
    if(ang < -MAX_VROT) 
        ang = -MAX_VROT;
        
    differentialrobot_proxy->setSpeedBase(adv,ang);       
}

void SpecificWorker::bug()
{
    TLaserData laser ; 
    
    laser = laser_proxy->getLaserData();
    std::sort(laser.begin(),laser.end(),[](auto a, auto b){return a.dist<b.dist;});

    if(obstacle() == true){
        differentialrobot_proxy->setSpeedBase(0,0.3);
        state = State::BUG;
        return;
    }

    if(targetAtSight() == true && laser[18].dist>500){
        state = State::IDLE;
        return;
    }
    
    if(obstacle() == false){
        
       
        state = State::BUG;
        
        if(laser[19].dist>250 && laser[19].dist<300){
            differentialrobot_proxy->setSpeedBase(100,0.2);
        }
        if(laser[19].dist>300 && laser[19].dist<350){
            differentialrobot_proxy->setSpeedBase(100,0);
        }
        
        if(laser[15].dist>350){
            differentialrobot_proxy->setSpeedBase(100,-0.2);
        }
    }

}


float SpecificWorker::distObstacle(float dist)
{
    TLaserData laser ; 
    
    laser = laser_proxy->getLaserData();
    std::sort(laser.begin()+20,laser.end()-20,[](auto a, auto b){return a.dist<b.dist;}); 
    
    if(laser[20].dist < dist)
        return laser[20].dist;
    return dist;
}

bool SpecificWorker::obstacle()
{
    TLaserData laser ; 
    
    laser = laser_proxy->getLaserData();
    std::sort(laser.begin()+20,laser.end()-20,[](auto a, auto b){return a.dist<b.dist;});
 
    if (laser[20].dist < 300)
        return true;

    return false;
}

bool SpecificWorker::targetAtSight()
{
    TLaserData lasercopy ;
    QPolygonF polygon;
    
    int i = 0;
    bool existe = false;
    
    lasercopy = laser_proxy->getLaserData();
    
    for (auto l: lasercopy)
    {
        QVec lr = innermodel->laserTo("world", "laser", l.dist, l.angle);
            polygon << QPointF(lr.x(), lr.z());
    }
//     std::pair<float, float> tr = target.getValores();
    
    while(i < 50 && existe == false){
        existe = polygon.containsPoint( target.getLineaPuntos(i), Qt::WindingFill );
        i++;
    }
    return existe;
//     return  polygon.containsPoint( QPointF(tr.first, tr.second ), Qt::WindingFill ) ;
}


float SpecificWorker::sigmoid(float d)
{	
  return (1.f / (1.f + exp(-d))) - 0.5;
}

float SpecificWorker::gaussian(float vr, float vx, float h)
{
  float landa = -(vx*vx)/log(h);
  return exp(-(vr*vr)/landa);
}


////////////////////////////
/// FROM COMPONENT INTERFACE
////////////////////////////

void SpecificWorker::setPick(const Pick &myPick){
 
  std::cout<<myPick.x<<myPick.y<<endl;
  target.setCopy(myPick.x, myPick.z);
  
  
} 

void SpecificWorker::go(const string& nodo, const float x, const float y, const float alpha)
{
    target.setCopy(x, y);
    activo = true;
}

void SpecificWorker::turn(const float speed)
{
    differentialrobot_proxy->setSpeedBase(0,0.3);
}

bool SpecificWorker::atTarget()
{
    return activo;
}

void SpecificWorker::stop()
{
    differentialrobot_proxy->setSpeedBase(0,0);
    activo = false;
}

  
// 	try
// 	{
// 		camera_proxy->getYImage(0,img, cState, bState);
// 		memcpy(image_gray.data, &img[0], m_width*m_height*sizeof(uchar));
// 		searchTags(image_gray);
// 	}
// 	catch(const Ice::Exception &e)
// 	{
// 		std::cout << "Error reading from Camera" << e << std::endl;
// 	}

//}




//     TLaserData data ;
//     data = laser_proxy->getLaserData();
//     std::sort(data.begin()+20,data.end()-20,[](auto a, auto b){return a.dist<b.dist;});
//     
//       
//    if (data[20].dist < 300)
//    {
//       differentialrobot_proxy->setSpeedBase(0,0.3);
//       int tiempo = rand() % 10 + 1;
//       usleep(tiempo*100000);   
//    }
