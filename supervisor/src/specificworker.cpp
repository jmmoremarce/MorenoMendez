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
    innermodel = new InnerModel("/home/robocomp/robocomp/files/innermodel/betaWorldArm.xml");	
	timer.start(Period);
	    
	return true;
}

void SpecificWorker::compute()
{
    try
    {
        RoboCompGetAprilTags::listaMarcas tags = getapriltags_proxy->checkMarcas();
        if(tags.size() > 0)
        {
            tag.copiaValores(tags[0].id, tags[0].tx, tags[0].tz);
            if(stopGiro == -1 && tags[0].id < 11)
            {
                stopGiro = tags[0].id;
            }
        }
    }
    catch(const Ice::Exception &e)
    {
        std::cout<<e<<endl;
    }
    
    RoboCompDifferentialRobot::TBaseState bState;
    try
    {
        differentialrobot_proxy->getBaseState(bState);
    }
    catch(const Ice::Exception &e)
    {
        std::cout<<e<<endl;
    }
    
  
    innermodel->updateTransformValues("robot", bState.x, 0, bState.z, 0, bState.alpha, 0);
    
     switch( state ) {
        case State::BUSCARPARED:
            gotopoint_proxy->turn(0.3);
            if(tag.emptyId(actualPared)){
                actualPared = actualPared + 1;
                actualPared = actualPared % 4;
                gotopoint_proxy->stop();
                std::cout<<"SALE DE BUSCAR PARED A GOTO"<<endl;
                state = State::GOTO;
            }             
            break;
	     
        case State::BUSCARTAZA :
            gotopoint_proxy->turn(0.2);
            if(tag.getVacia() == false && tag.CajasCogidas() == true){
                gotopoint_proxy->stop();
                tag.marcarCaja();
                std::cout<<"SALE DE BUSCAR TAZA A GOTO"<<endl;    
                state = State::GOTO;
            } 
            else{
                if(tag.emptyId(stopGiro) == true && salidaGiro > 0){ //para que el robot pueda dar una vuelta completa
                    std::cout<<"SE VA DE PATRULLA"<<endl;
                    state = State::PATRULLA;
                }
                if(tag.emptyId(stopGiro) == false && stopGiro > -1)
                    salidaGiro = salidaGiro + 1;
            }
            break;   
	                 
        case State::GOTO:
            sendGoTo();
            if(patrulla == true){
                std::cout<<"GOTO ---- PATRULLA"<<endl;
                patrulla = false;
                state = State::WAIT_PATRULLA;                
            }
            else{
                std::cout<<"GOTO"<<endl;
                state = State::WAIT;
            }
            break;
             
        case State::WAIT:
            if(Taza == false){
                if(gotopoint_proxy->atTarget()){
                    tag.setVacia(true);
                    std::cout<<"MANDA A BUSCAR TAZA!!!"<<endl;
                    Taza = true;
                    stopGiro = -1;
                    salidaGiro = 0;
                    state = State::BUSCARTAZA;
                }
            }
            else{
                if(gotopoint_proxy->atTarget()){
                    tag.setVacia(true);
                    Taza = false;
                    std::cout<<"MANDA BUSCAR PARED!!!"<<endl;
                    state = State::BUSCARPARED;	
                }
            }
            break;
            
        case State::PATRULLA:
            patrulla = true;
            state = State::GOTO;
            break;
                
        case State::WAIT_PATRULLA:
            if(tag.getVacia() == false && tag.CajasCogidas() == true){
                gotopoint_proxy->stop();
                std::cout<<"PATRULLA HA ENCONTRADO TAZA!!!"<<endl;
                state = State::GOTO;
            }
            if(gotopoint_proxy->atTarget()){
                std::cout<<"MANDA A BUSCAR TAZA TRAS PATRULLA!!!"<<endl;
                stopGiro = -1;
                salidaGiro = 0;
                state = State::BUSCARTAZA;
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
    QVec rt = innermodel->transform("world", QVec::vec3(tr.first, 0 , tr.second), "robot");
    
    if(patrulla == true)
        gotopoint_proxy->go("nodo 1",rt.x(), rt.z(),1.0);
    else
        gotopoint_proxy->go("nodo 1",rt.x(), rt.z(),0.0);
}








