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
	timer.start(Period);
	return true;
}


//Decala x y z dentro de  la estructura, despues un bool para comprobar si esta llena o no 

void SpecificWorker::compute()
{
//    qDebug()<< "hola";
//    differentialrobot_proxy->setSpeedBase(200,0);
      RoboCompDifferentialRobot::TBaseState bState;
  
  
    differentialrobot_proxy->getBaseState(bState);
    innermodel->updateTransformValues( "base", bState.x, 0, bState.z, 0, bState.alpha, 0);
    if(t.isEmpty() == false){
      std::pair<float, float> tr = t.getValores();
        QVec tR = innermodel->transform("robot", QVec::vec3(tr.first, 0 , tr.second), "world");
	float adv, vrot;
        float d=tR.norm2();        
        if(d > 50){
            adv = d;
            if(adv > MAX_ADV)
                adv = MAX_ADV;
            vrot = atan2(tR.x(), tR.z());
            if(vrot > MAX_VROT)
	        vrot = MAX_VROT;
	    dRobot->setSpeedBase(adv,vrot);
        }
        else{
            dRobot->setSpeedBase(0, 0);
            t.setEmpty();
	    
        }
        
    }
  
}

void SpecificWorker::setPick(const Pick &myPick){
 
  std::cout<<myPick.x<<myPick.y<<endl;
  t.setCopy(myPick.x, myPick.z);
  
  
} 
  
  /*qDebug()<<myPick.x;
  qDebug()<<myPick.y;
  qDebug()<<"----------";
  qDebug()<<"Error push";*/


/*  
    for(auto d:data)
        qDebug()<<d.angle<<d.dist;
    */
  
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
