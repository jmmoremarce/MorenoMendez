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
    
    goHome();
    sleep(1);
    
    try 
	{ 
        mList = jointmotor_proxy->getAllMotorParams();
        
    }
	catch(const Ice::Exception &e){ std::cout << e << std::endl;}
	
	joints << "shoulder_right_1"<<"shoulder_right_2"<<"shoulder_right_3"<<"elbow_right"<<"wrist_right_1"<<"wrist_right_2";
	// Check that these names are in mList
	motores = QVec::zeros(joints.size());
    
	timer.start(Period);
	return true;
}


//Decala x y z dentro de  la estructura, despues un bool para comprobar si esta llena o no 

void SpecificWorker::compute()
{
    try
    {
        RoboCompGetAprilTags::listaMarcas tags = getapriltags_proxy->checkMarcas();
        if(tags.size() > 0)
        {
            caja.setCopy(tags[0].id, tags[0].tx, tags[0].ty, tags[0].tz);
//             std::cout<<"id: "<<tags[0].id<<" valor x: "<< tags[0].tx<<" valor y: "<< tags[0].ty<<" Valor z: "<<tags[0].tz<<endl;
        }
        else{
            caja.setEmpty(true);
        }
    }
    catch(const Ice::Exception &e)
    {
        std::cout<<e<<endl;
    }
    
    RoboCompDifferentialRobot::TBaseState bState;
    
    differentialrobot_proxy->getBaseState(bState);
    innermodel->updateTransformValues( "robot", bState.x, 0, bState.z, 0, bState.alpha, 0);

    switch( state ) 
    {
        case State::IDLE:
            if (target.isEmpty() == false)
            {
                std::cout<<"Entra para cambiar estado GOTO"<<endl;
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
            
        case State::PATRULLA:
            Patrulla();
            break;
            
        case State::COLOCAR_BOX:
            box = true;
            if(ColocarBrazo() == true)
            {
                stopBrazo();
                
                state = State::COGER_CAJA;
            }
            else
            {
                MoverBrazo();
            }
            break;
            
        case State::COGER_CAJA:
            if(ColocarBrazo() == true)
            {
                downSlot();   
            }
            MoverBrazo();
            if(caja.isEmpty() == true)
            {
                std::cout<<"COJE LA CAJITA ASQUEROSA"<<endl;
                stopBrazo();
                Picking_box();
                subirCaja();
                sleep(1);
                
//                 disPared = 600;
                box = false;
                state = State::IDLE;
            }
            break;
            
        case State::SOLTAR_CAJA:
            bajarCaja();
            releasing_box();
            goHome();
            sleep(1);
            std::cout<<"SUELTA LA CAJITA"<<endl;
            box = false;
//             disPared = 100;
            
            state = State::IDLE;
            
            break;
            
        case State::COMPARE:
            if(box == true && disPared > 400)
            {
                ok_goto = false;
                state = State::SOLTAR_CAJA;
            }
            else
            {
                if(caja.isEmpty() == false)
                {
                    ok_goto = false;
                    state = State::COLOCAR_BOX;
                }
                else
                {
                    state = State::IDLE;
                }
            }
            break;
    }  
}

void SpecificWorker::gotoTarget()
{

    std::pair<float, float> tr = target.getValores();
    QVec rt = innermodel->transform("robot", QVec::vec3(tr.first, 0 , tr.second), "world");
    
    float ang  = atan2(rt.x(), rt.z());
    float adv;

    if( obstacle() == true)
    {   // If ther is an obstacle ahead, then transit to BUG
        if(gotoEnd() == true)
            return;
        
        state = State::BUG;
	
        return;
    }
    
    if(gotoEnd() == true)
        return;
    
    float dist = distObstacle(rt.norm2());
    
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
    try
    {
        laser = laser_proxy->getLaserData();
    }
    catch(const Ice::Exception &e)
    {
        std::cout<<e<<endl;
    }
    
    std::sort(laser.begin(),laser.end(),[](auto a, auto b){return a.dist<b.dist;});

    bool noObstaculo = true;
        
    if( gotoEnd() == true)
        return;
    
    if(giro == 0.0)
    {
        if( laser[20].angle > 0 )
        {
            giro = -0.1;
        }
        else
        {
            giro = 0.1;
        }
    }
    
    if(laser[19].dist<400)
    {
        differentialrobot_proxy->setSpeedBase(0,3*giro);
        noObstaculo = false;
        return;
    }
    
    if(laser[19].dist>250 && laser[19].dist<300)
    {
            differentialrobot_proxy->setSpeedBase(100,2*giro);
            noObstaculo = false;
    }
    
    if(laser[19].dist>300 && laser[19].dist<350)
    {
        differentialrobot_proxy->setSpeedBase(100,0);
        noObstaculo = false;
    }
        
    if(laser[15].dist>350)
    {
        differentialrobot_proxy->setSpeedBase(100,-2*giro);
    }
    
    if(noObstaculo == true && salida() == true)
    {
        giro = 0.0;
        state = State::IDLE;
        return;
    }
}

bool SpecificWorker::gotoEnd()
{
    std::pair<float, float> tr = target.getValores();
    QVec rt = innermodel->transform("robot", QVec::vec3(tr.first, 0 , tr.second), "world");
    
    float dist = rt.norm2();
    
    if(dist < disPared || caja.isEmpty() == false)
    {
        target.setEmpty(true);
        caja.setEmpty(true);
        differentialrobot_proxy->setSpeedBase(0,0); 
        state = State::COMPARE;
        return true;
    }
    return false;
}

bool SpecificWorker::salida()
{
    TLaserData laser ; 
    
    try
    {
        laser = laser_proxy->getLaserData();
    }
    catch(const Ice::Exception &e)
    {
        std::cout<<e<<endl;
    }
    
    bool exit = true;
    for(int i = 7; i < 92; i++)
    {
        if(laser[i].dist < 350)
            exit = false;
    }
    
    if(exit == true && targetAtSight() == true)
        return true;
    
    return false;
}

float SpecificWorker::distObstacle(float dist)
{
    TLaserData laser ; 
    
    try
    {
        laser = laser_proxy->getLaserData();
    }
    catch(const Ice::Exception &e)
    {
        std::cout<<e<<endl;
    }
    
    std::sort(laser.begin()+45,laser.end()-45,[](auto a, auto b){return a.dist<b.dist;}); 

    if(laser[20].dist < dist)
        return laser[20].dist;
    
    float aux = dist / 4;
    return (aux * 3);
}

bool SpecificWorker::obstacle()
{
    TLaserData laser ; 
    
    try
    {
        laser = laser_proxy->getLaserData();
    }
    catch(const Ice::Exception &e)
    {
        std::cout<<e<<endl;
    }
    
    std::sort(laser.begin()+20,laser.end()-20,[](auto a, auto b){return a.dist<b.dist;});
 
    if (laser[20].dist < 400)
        return true;

    return false;
}

bool SpecificWorker::targetAtSight()
{
    TLaserData lasercopy ;
    QPolygonF polygon;
    
    int i = 0;
    bool existe = false;
    
    try
    {
        lasercopy = laser_proxy->getLaserData();
    }
    catch(const Ice::Exception &e)
    {
        std::cout<<e<<endl;
    }
    
    for (auto l: lasercopy)
    {
        QVec lr = innermodel->laserTo("world", "laser", l.dist, l.angle);
            polygon << QPointF(lr.x(), lr.z());
    }
    
    while(i < 50 && existe == false){
        existe = polygon.containsPoint( target.getLineaPuntos(i), Qt::WindingFill );
        i++;
    }
    return existe;
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
  target.setCopy(myPick.x, myPick.z);  
} 

void SpecificWorker::Patrulla()
{
    state = State::IDLE;
    switch(patru)
    {
        case patrulla::PUNTO_0:
            target.setCopy(0.0, 0.0);
            patru = patrulla::PUNTO_1;
            break;
        case patrulla::PUNTO_1:
            target.setCopy(-1178.8, 1084.05);
            patru = patrulla::PUNTO_2;
            break;
        case patrulla::PUNTO_2:
            target.setCopy(-1389.98, -1560.17);
            patru = patrulla::PUNTO_3;
            break;
        case patrulla::PUNTO_3:
            target.setCopy(1450.61, -1425.51);
            patru = patrulla::PUNTO_4;
            break;
        case patrulla::PUNTO_4:
            target.setCopy(1336.79, 1626.18);
            patru = patrulla::PUNTO_0;
            break;
    }
}

void SpecificWorker::go(const string& nodo, const float x, const float y, const float alpha)
{
    if(alpha == 1.0)
    {
        state = State::PATRULLA;
    }
    else
    {
        if(ok_goto == true)
            target.setCopy(x , y);
    }
}

void SpecificWorker::turn(const float speed)
{
    if(box == false)
        disPared = 100;
    if(box == true)
        disPared = 600;
    
    ok_goto = true;
    
    differentialrobot_proxy->setSpeedBase(0,speed);
}

bool SpecificWorker::atTarget()
{
    if(caja.isEmpty() == true && target.isEmpty() == true)
    {
        std::cout<<"SE VA PARA EL SUPERVISOR, YA PUEDE MOVERSE"<<endl;
        if(box == true)
        {
            return false;
        }
        else
        {
            box = true;
            return true;
        }
    }
    return false;
}

void SpecificWorker::stop()
{
    differentialrobot_proxy->setSpeedBase(0,0);
    state = State::IDLE;
    target.setEmpty(true);
}

bool SpecificWorker::ColocarBrazo()
{
    if(abs(caja.getX()) > 10.0)
    {
        if(caja.getX() > 0.0)
            rightSlot();
        else
            leftSlot();
            
        return false;
    }
    if(abs(caja.getY()) > 10.0)
    {
        if(caja.getY() > 0.0)
            backSlot();
        else
            frontSlot();
        return false;
    }
    return true;
}

void SpecificWorker::MoverBrazo()
{
    RoboCompJointMotor::MotorStateMap mMap;
    RoboCompJointMotor::MotorGoalVelocityList vl;
    
 	try
	{
		jointmotor_proxy->getAllMotorState(mMap);
        
		for(auto m: mMap)
		{
			innermodel->updateJointValue(QString::fromStdString(m.first),m.second.pos);//m.first nombre y m.sencond.pos posicion
		}
	}
	catch(const Ice::Exception &e)
	{	std::cout << e.what() << std::endl;}

	try 
	{
        QMat jacobian = innermodel->jacobian(joints, motores, "cameraHand");
    
        QVec incs = jacobian.invert() * error;	
        int i = 0;
	
        chapu.append(incs);
	
        for(auto m: joints)
        {
            RoboCompJointMotor::MotorGoalVelocity vg{FACTOR*incs[i], 1.0, m.toStdString()};
            vl.push_back(vg);
            i++;
        }
    }
    catch(const Ice::Exception &e)
    {
        std::cout<<e.what() << endl;
    }

    try
    {
        jointmotor_proxy->setSyncVelocity(vl);
    }
    catch(const Ice::Exception &e)
    {
        std::cout<<e<<endl;
    }
}

void SpecificWorker::Picking_box()
{
    RoboCompJointMotor::MotorStateMap mMap;
	try
	{
	  jointmotor_proxy->getAllMotorState(mMap);
	  for(auto m: mMap)
	  {
        RoboCompJointMotor::MotorGoalPosition mg;
        if(m.first == "finger_right_1")
            mg = { -0.6, 1.0, m.first };
        if(m.first == "finger_right_2")
            mg = { 0.6, 1.0, m.first };
	    jointmotor_proxy->setPosition(mg);
	  }
	  sleep(1);
	}
	catch(const Ice::Exception &e)
	{	std::cout << e.what() << std::endl;}
}


void SpecificWorker::releasing_box()
{
    RoboCompJointMotor::MotorStateMap mMap;
	try
	{
	  jointmotor_proxy->getAllMotorState(mMap);
	  for(auto m: mMap)
	  {
        RoboCompJointMotor::MotorGoalPosition mg;
        if(m.first == "finger_right_1")
            mg = { 0.0, 1.0, m.first };
        if(m.first == "finger_right_2")
            mg = { -0.0, 1.0, m.first };
	    jointmotor_proxy->setPosition(mg);
	  }
	  sleep(1);
	}
	catch(const Ice::Exception &e)
	{	std::cout << e.what() << std::endl;}
}



void SpecificWorker::subirCaja()
{
	RoboCompJointMotor::MotorStateMap mMap;
	try
	{
		jointmotor_proxy->getAllMotorState(mMap);
		for(auto m: mMap)
		{
		    RoboCompJointMotor::MotorGoalPosition mg;
		    if(m.first != "wrist_right_2" && m.first !="finger_right_1" && m.first !="finger_right_2")
			mg = { innermodel->getJoint(m.first)->home, 1.0, m.first };
		    jointmotor_proxy->setPosition(mg);
		}
	}
	catch(const Ice::Exception &e)
	{	std::cout << e.what() << std::endl;}	
}


void SpecificWorker::bajarCaja()
{
    RoboCompJointMotor::MotorGoalVelocityList vl;
    int j = 0;
    for(auto c: chapu)
    {
      QVec incs = chapu[j];
      int i = 0;
      for(auto m: joints)
      {
	RoboCompJointMotor::MotorGoalVelocity vg{FACTOR*incs[i], 1.0, m.toStdString()};
	vl.push_back(vg);
	i++;
      }
      chapu.remove(j);

      try
      {
	  jointmotor_proxy->setSyncVelocity(vl);
      }
      catch(const Ice::Exception &e)
      {
	  std::cout<<e<<endl;
      }
    }
    usleep(1000200);
    stopBrazo();
}

void SpecificWorker::goHome()
{
	RoboCompJointMotor::MotorStateMap mMap;
	try
	{
		jointmotor_proxy->getAllMotorState(mMap);
		for(auto m: mMap)
		{
            RoboCompJointMotor::MotorGoalPosition mg;
            if(m.first == "wrist_right_2")
                mg = { 1.4, 1.0, m.first };
            else
                mg = { innermodel->getJoint(m.first)->home, 1.0, m.first };
			jointmotor_proxy->setPosition(mg);
		}
	}
	catch(const Ice::Exception &e)
	{	std::cout << e.what() << std::endl;}	
}


void SpecificWorker::stopBrazo(){
    RoboCompJointMotor::MotorGoalVelocityList vl;
    
    for(auto m: joints)
	{
		RoboCompJointMotor::MotorGoalVelocity vg{0.0, 1.0, m.toStdString()};
		vl.push_back(vg);
	}
    try
	{ 
		jointmotor_proxy->setSyncVelocity(vl);
	}
	catch(const Ice::Exception &e)
	{
        std::cout << e.what() << std::endl;
    }
}

void SpecificWorker::leftSlot()
{
	error = QVec::vec6(-caja.getX(),0,0,0,0,0);
}

void SpecificWorker::rightSlot()
{
	error = QVec::vec6(-caja.getX(),0,0,0,0,0);
}

void SpecificWorker::frontSlot()
{
	error = QVec::vec6(0,-caja.getY(),0,0,0,0);
}

void SpecificWorker::backSlot()
{
	error = QVec::vec6(0,-caja.getY(),0,0,0,0);
}

void SpecificWorker::upSlot()
{
	error = QVec::vec6(0,0,-caja.getZ(),0,0,0);
}

void SpecificWorker::downSlot()
{
	error = QVec::vec6(0,0,-caja.getZ(),0,0,0);
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
