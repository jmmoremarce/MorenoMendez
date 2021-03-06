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
#define VECTOR 25

using namespace std;
 
class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
    
    void go(const string &nodo, const float x, const float y, const float alpha);
	void turn(const float speed);
	bool atTarget();
	void stop();
	void Picking_box();
	void releasing_box();
	
    
public slots:
	void compute(); 	
	void setPick(const Pick &myPick);
    void goHome();
    void leftSlot();
	void rightSlot();
	void upSlot();
	void downSlot();
	void frontSlot();
	void backSlot();
	//coger coordenadas del rat´on y mostrarla y luego cuando tenemos las coordenadas decirle al robot que vaya a esas coordenadas 
   
private:
	struct Target{
	    mutable QMutex mutex;
	    bool vacia = true; 
	    float valorX=0.0;
	    float valorZ=0.0; 
        float vectorPuntos[100];
	     
	    void setEmpty (bool vaciar){
	      QMutexLocker block(&mutex);	      
	      vacia=vaciar;
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
	    
	    void calcularPuntos(float x, float z){
            float m = (valorZ - z) / (valorX - x);
            float aux = valorX - x;
            if(aux != 0){
                aux = aux / VECTOR;
                for(int i = 0; i < 2*VECTOR; i = i+2){
                    x = x + aux;
                    vectorPuntos[i] = x;
                    vectorPuntos[i+1] = m * ( valorX - x) + z; 
                }
            }
            if(aux == 0){
                aux = valorZ - z;
                aux = aux / 50;
                for(int i = 0; i < 2*VECTOR; i = i+2){
                    z = z + aux;
                    vectorPuntos[i] = ((valorZ - z) / m) + x;
                    vectorPuntos[i+1] = z; 
                }
            }
        }
	    
	    QPointF getLineaPuntos(int i){
            return QPointF(vectorPuntos[i], vectorPuntos[i+1]);
        }
	};
	struct Caja{
        mutable QMutex mutex;
        bool pick_box[10] = {false, false, false, false, false, false, false, false, false, false};
	    bool vacia = true; 
	    float valorX = 0.0;
	    float valorZ = 0.0; 
        float valorY = 0.0;
        int id = -1;
        
        void setCopy (int i, float x, float y, float z){
	      
            QMutexLocker block (&mutex);
            vacia = false;		
            valorX = x;
            valorY = y;
            valorZ = z;
            id = i;
	    }
	    
	    void setEmpty (bool vaciar){
	      QMutexLocker block(&mutex);	      
	      vacia=vaciar;
          id = -1;
	    }
	    
        bool isEmpty(){
	      QMutexLocker block(&mutex);
	      return vacia;
	    }
	    
	    float getZ(){
            return valorZ;
        }
        
        float getX(){
            return valorX;
        }
        
        float getY(){
            return valorY;
        }
        
        void marcarBox()
        {
            pick_box[id-11] = true;
        }
        
        bool cajaCogida()
        {
            if( id > 10)
                return pick_box[id-11];
            return false;
        }
    };
    
    Caja caja;
	Target target;
	InnerModel *innermodel;
    
    float giro = 0.0;
    QVector <QVec> chapu;
    bool box = false;
    int disPared = 100;
    bool ok_goto = false;
    bool goto_patrulla = false;
    bool cambio_pared = true;
    
    enum State {IDLE, GOTO, BUG, PATRULLA, COLOCAR_BOX, COGER_CAJA, SOLTAR_CAJA, COMPARE};
    enum patrulla {PUNTO_1, PUNTO_2, PUNTO_3, PUNTO_4};
    
    State state = State::IDLE;
//     State state = State::COLOCAR_BOX;
    patrulla patru = patrulla::PUNTO_1;
	
    float gaussian(float vr, float vx, float h);
	float sigmoid(float d);
        
    void gotoTarget();
    void bug();
    bool obstacle();
    bool targetAtSight();
    
    float distObstacle(float dist);
    bool salida();
    void Patrulla();
    bool gotoEnd();
    
    QStringList joints;
	QVec motores;
    QVec error;
    RoboCompJointMotor::MotorParamsList mList;
    int const FACTOR = 1;
    
    void MoverBrazo();
    bool ColocarBrazo();
    void stopBrazo();
    void subirCaja();
    void bajarCaja();
};

#endif

