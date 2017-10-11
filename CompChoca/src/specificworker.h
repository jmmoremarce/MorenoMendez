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



#define MAX_ADV 400
#define MAX_VROT 0.5



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
	void setPick(const Pick &myPick);
	//coger coordenadas del rat´on y mostrarla y luego cuando tenemos las coordenadas decirle al robot que vaya a esas coordenadas 

private:
	struct Target{
	    mutable QMutex mutex;
	    bool vacia=true; 
	    float valorX=0.0;
	    float valorZ=0.0; 
	     
	    void setEmpty (){
	      QMutexLocker block(&mutex);	      
	      vacia=true;
	    }
	    
	    bool isEmpty(){
	      QMutexLocker block(&mutex);
	      return vacia;
	    } 
	    
	    void setCopy (float x, float z){
	      
		QMutexLocker block (&mutex);
		vacia=false;		
		valorX=x;
		valorZ=z;
	    }
	    
	    std::pair<float, float> getValores(){
		QMutexLocker block (&mutex);
		return std::make_pair(valorX, valorZ);
	    }
				  
	    
	};
	
	Target t;
	InnerModel *innermodel;
	
	float gaussian(float vr, float vx, float h);
	float sigmoid(float d);
};

#endif

