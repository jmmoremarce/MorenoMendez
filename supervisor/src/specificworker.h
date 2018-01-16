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

/**
       \brief
       @author authorname
*/







#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>

using namespace std;

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);


public slots:
	void compute(); 	

private:
	struct Tag {
        mutable QMutex mutex;
        bool vacia = true;
        bool vector[10]={false,false,false,false,false,false,false,false,false,false};
        int id = -1;
        float valorX = 0.0;
        float valorY = 0.0;
        
        
        void copiaValores(int id_, float x, float y){
            QMutexLocker block (&mutex);
            id = id_;
            valorX = x;
            valorY = y;
            vacia = false;
        }
        
        std::pair<float, float> getValores(){
            QMutexLocker block (&mutex);
            return std::make_pair(valorX, valorY);
	    }
    
        void marcarCaja(){
            vector[id -11] = true;
        }
        
	    bool CajasCogidas(){
            if(id > 10)
                return !vector[id - 11];
            return false;
        }
        
	    bool emptyId(int _id){
            if(id == _id)
                return true;
            return false;
        }
        
	    bool getVacia(){
            return vacia;
        }
        
        void setVacia(bool v){
            vacia = false;
        }
    };
    
    Tag tag;
    InnerModel *innermodel;
    
    enum State {BUSCARPARED,BUSCARTAZA, WAIT, GOTO, PATRULLA, WAIT_PATRULLA};
    State state = State::BUSCARTAZA;
    
    bool Taza = true;
    bool patrulla = false;

    void sendGoTo();
    
    int actualPared = 0;
    int stopGiro = -1;
    int salidaGiro = 0;
};

#endif

