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
         case State::BUSCARPARED:
             gotopoint_proxy->turn(0.2);
                if(tag.emptyId(actualPared)){
                    actualPared = actualPared + 1;
                    actualPared = actualPared % 4;
                    gotopoint_proxy->stop();
                    state = State::GOTO;
		     std::cout<<"GO TO DE PARED!!!"<<endl;
		    stateLast=State::BUSCARPARED;
               }             
             break;
	     
	case State::BUSCARTAZA :
           gotopoint_proxy->turn(0.2);
	    std::cout<<"Valor actual taza "<<actualTaza<<endl;
             if(tag.emptyId(actualTaza)){
                    actualTaza = actualTaza + 1 ;
                   // actualTaza = actualTaza % 4;
                    gotopoint_proxy->stop();
                    state = State::GOTO;
		    stateLast=State::BUSCARTAZA;
                    std::cout<<"GO TO DE TAZA!!!"<<endl;
		    std::cout<<"Valor actual taza "<<actualTaza<<endl;
               }             
	    break;   
	     
             
         case State::GOTO:
             sendGoTo();
             state = State::WAIT;
             break;
             
         case State::WAIT:
          
              switch( stateLast ){
		case State::BUSCARPARED:
		     if(gotopoint_proxy->atTarget()){
			tag.setVacia(true);
			state = State::BUSCARTAZA;
			std::cout<<"MANDA A BUSCAR TAZA!!!"<<endl;
		
		    }
		break;   
		
		case State::BUSCARTAZA:
		     if(gotopoint_proxy->atTarget()){
			tag.setVacia(true);
			state = State::BUSCARPARED;	
			std::cout<<"MANDA BUSCAR PARED!!!"<<endl;
		    }
		break;
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
    std::cout<<"Tag ID: "<<tags[0].id<<endl;

}






