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

void SpecificWorker::compute()
{
    RoboCompDifferentialRobot::TBaseState bState;
    
    differentialrobot_proxy->getBaseState(bState);
    
    innermodel->updateTransformValues( "base", bState.x, 0, bState.z, 0, bState.alpha, 0);
    
     switch( state ) {
         case State::BUSCAR:
             gotopoint_proxy->turn(0.2);
                std::cout<<"buscando cuadro"<<endl;
                if(tag.emptyId(actual)){
                    gotopoint_proxy->stop();
                    state = State::GOTO;
                    std::cout<<"manda parar al robot"<<endl;
                    std::cout<<"valor x: "<<tag.getValorX()<<"  valor y: "<< tag.getValorY()<<endl;
                }
             
             break;
             
         case State::GOTO:
             sendGoTo();
             state = State::WAIT;
             break;
             
         case State::WAIT:
             if(tag.emptyId(actual)){
                
                actual = actual + 1;
                actual = actual % 4;
                std::cout<<"manda mover al robot"<<endl;
             }
            
             if(gotopoint_proxy->atTarget()){

                tag.setVacia(true);
                std::cout<<"cambio de cuadro"<<endl;
                std::cout<<actual<<endl;
                state = State::BUSCAR;
                 
             }
             break;
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
}

void SpecificWorker::sendGoTo(){
    
    std::pair<float, float> tr = tag.getValores();
    QVec rt = innermodel->transform("world", QVec::vec3(tr.first, 0 , tr.second), "base");
  
    gotopoint_proxy->go("nodo 1",rt.x(), rt.z(),0.0);
}

void SpecificWorker::newAprilTag(const tagsList &tags)
{
    tag.copiaValores(tags[0].id, tags[0].tx, tags[0].tz);
}






