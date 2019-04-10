#include <iostream>
#include <cstdlib>
#include <pthread.h>
#include <vector>
#include <queue>
#include "web.hpp"
#include "dspic.hpp"

// DStarIncludes 
#include <utility>
#include <algorithm>
#include <cmath>
#include <bits/stdc++.h>
#include <limits>
#include <map>
#include "mapGeneration.hpp"
#include "dStarLite.hpp"

//DStarGlobal 
int mapRows {10};  
int mapColumns {10};  
float km {0}; 
std::vector<std::vector<int>> mapVector;
 
Node startNode = {infinity,infinity,0,std::pair<int,int>(0,0)};
Node goalNode = {infinity,0,0,std::pair<int,int>(9,9), false};

priorityList uList; // priority List
mappedNodes knownNodes; // node the robot can see

#define TX_CODE_VAR     1
#define TX_CODE_LOG     2
#define TX_CODE_PLOT    3
#define CODE_VAR_X      1
#define CODE_VAR_Y      2
#define CODE_VAR_T      3
#define CODE_VAR_RUPT   4

void *print(void *ptr);

int main()
{
    /*Init variables & threads*/
	DsPIC dspic;
    pthread_t thread_print;

    dspic.async_read(); //flush rx buffer

	Web web(&dspic);
    web.startThread();

    int rc;
    rc = pthread_create(&thread_print, NULL, print, &web);

    if (rc) {
    std::cout << "Error:unable to create thread," << rc << std::endl;
    exit(-1);
    }

	dspic.getVar(CODE_VAR_BAT);

	dspic.initVarDspic();  //Init PID,odometry,acceleration,speed
	dspic.initPos(1000,1500,0);    //initialize position & angle of the robot
	char c = 0;
	char started = 0;

    puts("Robot initialized. Press enter to start...");
    getchar();
    
    dspic.setVar8(CODE_VAR_VERBOSE,1);  //Allow the dspic to speak on UART channel
    dspic.start();  //Start the motors
    std::cout << "dspic start" << std::endl; 
    /*=============DStarImplementation===================*/
    //dsPic initialization 

    // Map Generation 
    generateMap(mapVector,mapRows,mapColumns); // generates empty map 
    std::cout << "Map generated" << std::endl; 
    std::cout << "x " << mapVector.size() << " y " << mapVector[0].size() << std::endl; 
    createRectangle(4,4,5, 5, mapVector); // creates a 5x5 obstacle rectangle  at (4,4) 
    printMap(mapRows, mapColumns, mapVector);
    

    //DStarLite first run
    Node lastNode = startNode;
    initialize(mapVector, knownNodes, uList, startNode, goalNode);
    goalNode = knownNodes.at(goalNode.coord);
    computeShortestPath(uList, knownNodes, startNode.coord, goalNode);
    startNode = knownNodes.at(startNode.coord); // we update the start node
    goalNode = knownNodes.at(goalNode.coord); // we update the goal node
    std::cout << "Dstar first run" << std::endl; 

    while(startNode.coord != goalNode.coord){

        if(startNode.costG == infinity){
            std::cerr << "NOT KNOWN PATH" << std::endl;
            break;
        }

        startNode = bestNode(startNode, knownNodes); // we "move" the robot
        findPath(mapVector,knownNodes,startNode,goalNode); // prints the path in the terminal 
        
        int xSetpoint = startNode.coord.first *10+1000; 
        int ySetpoint = startNode.coord.second *10+1500; 

        dspic.setVarDouble64b(CODE_VAR_XC_LD,xSetpoint);
        dspic.setVarDouble64b(CODE_VAR_YC_LD,ySetpoint);
        dspic.setVarDouble64b(CODE_VAR_XF_LD,goalNode.coord.first);
        dspic.setVarDouble64b(CODE_VAR_YF_LD,goalNode.coord.second);

        delay(100); 
        // dspic.go(xSetpoint, ySetpoint,0,0); // we move the robot to the next point
        // dspic.arrived = 0;  //force arrived to 0
        // while(!dspic.arrived){
        //     delay(50);  //wait before asking so the dspic can start the movement /  and don't SPAM the UART channel
        //     dspic.getVar(CODE_VAR_ARRIVED); //send a request to update the arrived variable
        // }


        // Debug 
        std::cout << "Press enter to continue" << std::endl; 
        getchar();
    }
    /*=============DStarImplementation===================*/
	
	while(c != 's'){
		puts("Press 's' to stop or any other button to start/stop the robot");
		c = getchar();
		if(c != 's'){
			if(!started){
				started = 1;
				dspic.start();

			}
			else{
				started = 0;
				dspic.stop();
				//dspic.setVarDouble64b(CODE_VAR_TC_LD,0);
			}
		}
	}
	//Test circle
	/*double radius = 200;
	dspic.setVarDouble64b(CODE_VAR_DISTANCE_MAX_LD,1);	//reducing the arrival distance for more precise path following
	dspic.setVarDouble64b(CODE_VAR_X_LD,1000+radius);
	dspic.setVarDouble64b(CODE_VAR_Y_LD,1500);
	for(double t = 0; t < 2*3.14159;t+=0.01){
		double x = 1000 + radius * cos(t);
		double y = 1500 + radius * sin(t);
		dspic.setVarDouble64b(CODE_VAR_XC_LD,x);
		dspic.setVarDouble64b(CODE_VAR_YC_LD,y);
		dspic.setVarDouble64b(CODE_VAR_XF_LD,x);
		dspic.setVarDouble64b(CODE_VAR_YF_LD,y);
		delay(20);
	}*/
	dspic.stop();
    //dspic.initPos(1000,1500,0);
	dspic.setVar8(CODE_VAR_VERBOSE,0);
	puts("verbose set to 0");

    puts("exiting ...");
    //pthread_exit(NULL);

    return 0;
}

/*
Interface between the RPI and DSPIC :

Infinite loop where it waits for the messages from the dspic and it uses-it as a command for : 
    - ...?
*/
void *print(void *ptr) {
   /*long tid;
   tid = (long)threadid;*/
   Web *w = (Web*)ptr;
   //DsPIC *dspic = (DsPIC*)ptr;
   DsPIC *dspic = w->dspic;
   while(1){

        std::vector<uint8_t> msg = dspic->readMsg(); // get message from dspic 
        uint8_t checksum = 0;
        for(unsigned int i = 0; i < msg.size() - 1; i++){
            checksum += msg[i];
        }
        /*
        Last element of the message is the checksum 
        We verify if they are the same 
        */
        if(checksum != msg[msg.size() - 1]){ 
            std::cout << "CHECKSUM ERROR !" << std::endl;
			std::cout << "CE dec :";
            for(unsigned int i = 0; i < msg.size(); i++){
                std::cout << " & [" << i << "] = " << (int)msg[i];
            }
			std::cout << std::endl << "CE char :";
			
            for(unsigned int i = 0; i < msg.size(); i++){
                if(msg[i] > 31 && msg[i] < 127)
                    std::cout << msg[i];
            }
        }
        else{
            if(msg.size() > 1){
                switch(msg[1]){
                    case 0 :    //error

                        break;
                    case TX_CODE_VAR :    //variable
                        switch(msg[2]){
							case CODE_VAR_BAT :{
                                if(msg.size() > 6){
                                    float vbat;
									float *ptr = &vbat;
									uint8_t *ptrChar = (uint8_t*)ptr;
									for(int i = 0; i < 4; i++){
										ptrChar[i] = msg[3+i];
									}
									std::cout << "received from DsPIC : VBAT = " << vbat << std::endl;
									dspic->bat = vbat;
                                }
							break;
							}
                            case CODE_VAR_X :
                                if(msg.size() > 4){
                                    dspic->x = ((msg[3] << 8) + msg[4]);
                                    //std::cout << "received from DsPIC : x = " << dspic->x << std::endl;
                                }
                                break;
                            case CODE_VAR_Y :
                                if(msg.size() > 4){
                                    dspic->y = ((msg[3] << 8) + msg[4]);
                                    //std::cout << "received from DsPIC : y = " << dspic->y << std::endl;
                                }
                                break;
                            case CODE_VAR_T :
                                if(msg.size() > 4){
                                    dspic->t = ((msg[3] << 8) + msg[4]);
                                    //std::cout << "received from DsPIC : t = " << dspic->t << " & H = " << (int)msg[3] << " & L = " << (int)msg[4] << std::endl;
                                }
                            case CODE_VAR_X_LD :
                                if(msg.size() > 8){
                                    double x_ld;
                                    double *ptr = &x_ld;
                                    uint8_t *ptrChar = (uint8_t*)ptr;
                                    for(int i = 0; i < 8; i++){
                                        ptrChar[i] = msg[3+i];
                                    }
                                    std::cout.precision(11);
                                    std::cout << "received from DsPIC : x_ld = " << x_ld << std::endl;
                                    std::cout.precision(6);
                                    //dspic->bat = vbat;
                                }
                                break;
                            case CODE_VAR_Y_LD :
                                if(msg.size() > 8){
                                    double y_ld;
                                    double *ptr = &y_ld;
                                    uint8_t *ptrChar = (uint8_t*)ptr;
                                    for(int i = 0; i < 8; i++){
                                        ptrChar[i] = msg[3+i];
                                    }
                                    std::cout.precision(11);
                                    std::cout << "received from DsPIC : y_ld = " << y_ld << std::endl;
                                    std::cout.precision(6);
                                    //dspic->bat = vbat;
                                }
                                break;
                            case CODE_VAR_T_LD :
                                if(msg.size() > 8){
                                    double t_ld;
                                    double *ptr = &t_ld;
                                    uint8_t *ptrChar = (uint8_t*)ptr;
                                    for(int i = 0; i < 8; i++){
                                        ptrChar[i] = msg[3+i];
                                    }
                                    std::cout.precision(11);
                                    std::cout << "received from DsPIC : t_ld = " << t_ld << std::endl;
                                    std::cout.precision(6);
                                    //dspic->bat = vbat;
                                }
                                break;
                            case CODE_VAR_RUPT :
                                if(msg.size() > 4){
                                    //printf("msg[3] = %d / 0x01 = %d / msg[3] & 0x01 = %d\n",msg[3],0x01,msg[3] & 0x01);
                                    //printf("msg[3] = %d / 0x02 = %d / msg[3] & 0x02 = %d\n",msg[3],0x02,msg[3] & 0x02);
                                    //printf("msg[3] = %d / 0x04 = %d / msg[3] & 0x04 = %d\n",msg[3],0x04,msg[3] & 0x04);
                                    //printf("msg[3] = %d / 0x08 = %d / msg[3] & 0x08 = %d\n",msg[3],0x08,msg[3] & 0x08);
                                    dspic->rupt.ass0 = msg[3] & 0x01;
                                    dspic->rupt.ass1 = (msg[3] & 0x02) >> 1;
                                    dspic->rupt.ass2 = (msg[3] & 0x04) >> 2;
                                    dspic->rupt.ass3 = (msg[3] & 0x08) >> 3;
                                    dspic->rupt.act0 = (msg[3] & 0x10) >> 4;
                                    dspic->rupt.act1 = (msg[3] & 0x20) >> 5;
                                    dspic->rupt.act2 = (msg[3] & 0x40) >> 6;
                                    dspic->rupt.act3 = (msg[3] & 0x80) >> 7;
                                    dspic->rupt.act4 = msg[4] & 0x01;
                                    dspic->rupt.act5 = (msg[4] & 0x02) >> 1;
                                    //std::cout << "received from DsPIC : rupt = H:" << (int)msg[3] << " & L: " << (int)msg[4] << " & " << dspic->rupt.ass0 << "/" << dspic->rupt.ass1 << "/" << dspic->rupt.ass2 << "/" << dspic->rupt.ass3 << "/" << dspic->rupt.act0 << "/" << dspic->rupt.act1 << "/" << dspic->rupt.act2 << "/" << dspic->rupt.act3 << "/" << dspic->rupt.act4 << "/" << dspic->rupt.act5 << "/"  << std::endl;
                                }
                                break;
                            case 100 :
                                if(msg.size() > 4){
                                    dspic->US[0] = ((msg[3] << 8) + msg[4]);
                                    //std::cout << "received from DsPIC : US[0] = " << dspic->US[0] << " (H = " << (int)msg[3] << " & L = " << (int)msg[4] << ")" << std::endl;
                                }
                                break;
                            case 101 :
                                if(msg.size() > 4){
                                    dspic->US[1] = ((msg[3] << 8) + msg[4]);
                                    //std::cout << "received from DsPIC : US[1] = " << dspic->US[1] << " (H = " << (int)msg[3] << " & L = " << (int)msg[4] << ")" << std::endl;
                                }
                                break;
                            case 102 :
                                if(msg.size() > 4){
                                    dspic->US[2] = ((msg[3] << 8) + msg[4]);
                                    //std::cout << "received from DsPIC : US[2] = " << dspic->US[2] << " (H = " << (int)msg[3] << " & L = " << (int)msg[4] << ")" << std::endl;
                                }
                                break;
                            case 103 :
                                if(msg.size() > 4){
                                    dspic->US[3] = ((msg[3] << 8) + msg[4]);
                                    //std::cout << "received from DsPIC : US[3] = " << dspic->US[3] << " (H = " << (int)msg[3] << " & L = " << (int)msg[4] << ")" << std::endl;
                                }
                                break;
                            case 104 :
                                if(msg.size() > 4){
                                    dspic->US[4] = ((msg[3] << 8) + msg[4]);
                                    //std::cout << "received from DsPIC : US[4] = " << dspic->US[4] << " (H = " << (int)msg[3] << " & L = " << (int)msg[4] << ")" << std::endl;
                                }
                                break;
                            case 105 :
                                if(msg.size() > 4){
                                    dspic->US[5] = ((msg[3] << 8) + msg[4]);
                                    //std::cout << "received from DsPIC : US[5] = " << dspic->US[5] << " (H = " << (int)msg[3] << " & L = " << (int)msg[4] << ")" << std::endl;
                                }
                                break;
                            case CODE_VAR_COEF_DISSYMETRY_LD:
                                if(msg.size() > 8){
                                    double var;
                                    double *ptr = &var;
                                    uint8_t *ptrChar = (uint8_t*)ptr;
                                    for(int i = 0; i < 8; i++){
                                        ptrChar[i] = msg[3+i];
                                    }
                                    std::cout << "received from DsPIC : coef_Dissymetry_ld = " << var << std::endl;
                                }
                                break;
                            case CODE_VAR_MM_PER_TICKS_LD:
                                if(msg.size() > 8){
                                    double var;
                                    double *ptr = &var;
                                    uint8_t *ptrChar = (uint8_t*)ptr;
                                    for(int i = 0; i < 8; i++){
                                        ptrChar[i] = msg[3+i];
                                    }
                                    std::cout << "received from DsPIC : mm_per_ticks_ld = " << var << std::endl;
                                }
                                break;
                            case CODE_VAR_RAD_PER_TICKS_LD:
                                if(msg.size() > 8){
                                    double var;
                                    double *ptr = &var;
                                    uint8_t *ptrChar = (uint8_t*)ptr;
                                    for(int i = 0; i < 8; i++){
                                        ptrChar[i] = msg[3+i];
                                    }
                                    std::cout << "received from DsPIC : rad_per_ticks_ld = " << var << std::endl;
                                }
                                break;
								
								
								
                            case CODE_VAR_P_SPEED_0_LD:
                                if(msg.size() > 8){
                                    double *ptr = &w->dspic->pidSpeed0.Kp;
                                    uint8_t *ptrChar = (uint8_t*)ptr;
                                    for(int i = 0; i < 8; i++){
                                        ptrChar[i] = msg[3+i];
                                    }
									//std::cout << "P_speed0 = " << w->dspic->pidSpeed0.Kp << std::endl;
                                }
								break;
                            case CODE_VAR_I_SPEED_0_LD:
                                if(msg.size() > 8){
                                    double *ptr = &w->dspic->pidSpeed0.Ki;
                                    uint8_t *ptrChar = (uint8_t*)ptr;
                                    for(int i = 0; i < 8; i++){
                                        ptrChar[i] = msg[3+i];
                                    }
                                }
								break;
                            case CODE_VAR_D_SPEED_0_LD:
                                if(msg.size() > 8){
                                    double *ptr = &w->dspic->pidSpeed0.Kd;
                                    uint8_t *ptrChar = (uint8_t*)ptr;
                                    for(int i = 0; i < 8; i++){
                                        ptrChar[i] = msg[3+i];
                                    }
                                }
								break;
								
                            case CODE_VAR_P_SPEED_1_LD:
                                if(msg.size() > 8){
                                    double *ptr = &w->dspic->pidSpeed1.Kp;
                                    uint8_t *ptrChar = (uint8_t*)ptr;
                                    for(int i = 0; i < 8; i++){
                                        ptrChar[i] = msg[3+i];
                                    }
                                }
								break;
                            case CODE_VAR_I_SPEED_1_LD:
                                if(msg.size() > 8){
                                    double *ptr = &w->dspic->pidSpeed1.Ki;
                                    uint8_t *ptrChar = (uint8_t*)ptr;
                                    for(int i = 0; i < 8; i++){
                                        ptrChar[i] = msg[3+i];
                                    }
                                }
								break;
                            case CODE_VAR_D_SPEED_1_LD:
                                if(msg.size() > 8){
                                    double *ptr = &w->dspic->pidSpeed1.Kd;
                                    uint8_t *ptrChar = (uint8_t*)ptr;
                                    for(int i = 0; i < 8; i++){
                                        ptrChar[i] = msg[3+i];
                                    }
                                }
								break;
								
                            case CODE_VAR_P_SPEED_2_LD:
                                if(msg.size() > 8){
                                    double *ptr = &w->dspic->pidSpeed2.Kp;
                                    uint8_t *ptrChar = (uint8_t*)ptr;
                                    for(int i = 0; i < 8; i++){
                                        ptrChar[i] = msg[3+i];
                                    }
                                }
								break;
                            case CODE_VAR_I_SPEED_2_LD:
                                if(msg.size() > 8){
                                    double *ptr = &w->dspic->pidSpeed2.Ki;
                                    uint8_t *ptrChar = (uint8_t*)ptr;
                                    for(int i = 0; i < 8; i++){
                                        ptrChar[i] = msg[3+i];
                                    }
                                }
								break;
                            case CODE_VAR_D_SPEED_2_LD:
                                if(msg.size() > 8){
                                    double *ptr = &w->dspic->pidSpeed2.Kd;
                                    uint8_t *ptrChar = (uint8_t*)ptr;
                                    for(int i = 0; i < 8; i++){
                                        ptrChar[i] = msg[3+i];
                                    }
                                }
								break;
								
								
                            case CODE_VAR_P_DISTANCE_LD:
                                if(msg.size() > 8){
                                    double *ptr = &w->dspic->pidDistance.Kp;
                                    uint8_t *ptrChar = (uint8_t*)ptr;
                                    for(int i = 0; i < 8; i++){
                                        ptrChar[i] = msg[3+i];
                                    }
                                }
								break;
                            case CODE_VAR_I_DISTANCE_LD:
                                if(msg.size() > 8){
                                    double *ptr = &w->dspic->pidDistance.Ki;
                                    uint8_t *ptrChar = (uint8_t*)ptr;
                                    for(int i = 0; i < 8; i++){
                                        ptrChar[i] = msg[3+i];
                                    }
                                }
								break;
                            case CODE_VAR_D_DISTANCE_LD:
                                if(msg.size() > 8){
                                    double *ptr = &w->dspic->pidDistance.Kd;
                                    uint8_t *ptrChar = (uint8_t*)ptr;
                                    for(int i = 0; i < 8; i++){
                                        ptrChar[i] = msg[3+i];
                                    }
                                }
								break;
								
                            case CODE_VAR_P_ANGLE_LD:
                                if(msg.size() > 8){
                                    double *ptr = &w->dspic->pidAngle.Kp;
                                    uint8_t *ptrChar = (uint8_t*)ptr;
                                    for(int i = 0; i < 8; i++){
                                        ptrChar[i] = msg[3+i];
                                    }
                                }
								break;
                            case CODE_VAR_I_ANGLE_LD:
                                if(msg.size() > 8){
                                    double *ptr = &w->dspic->pidAngle.Ki;
                                    uint8_t *ptrChar = (uint8_t*)ptr;
                                    for(int i = 0; i < 8; i++){
                                        ptrChar[i] = msg[3+i];
                                    }
                                }
								break;
                            case CODE_VAR_D_ANGLE_LD:
                                if(msg.size() > 8){
                                    double *ptr = &w->dspic->pidAngle.Kd;
                                    uint8_t *ptrChar = (uint8_t*)ptr;
                                    for(int i = 0; i < 8; i++){
                                        ptrChar[i] = msg[3+i];
                                    }
									w->dspic->isPIDUpdated = true;
									//std::cout << "Updated !" << std::endl;
                                }
								break;
							case CODE_VAR_ARRIVED:
                                w->dspic->arrived = msg[3];
                                break;
                            default :
                                std::cout << "Received wrong variable code from DsPIC : " << (int)msg[2] << std::endl;
                                break;

                        }
                        break;
                    case TX_CODE_LOG :{    //log
                        std::string s;
                        for(unsigned int i = 2; i < msg.size() - 1; i++)
                            s += msg[i];
                        std::cout << "Received log from DsPIC : " << s << std::endl;
                        dspic->logs.push(s);
                        //w->sendMsg("l=" + s);
                        break;
                    }
                    case TX_CODE_PLOT :{    //plot
                        uint8_t id = msg[2];
                        //std::cout << "plot id = " << id << "/" << (int)msg[2] << std::endl;
                        uint32_t x = (msg[3] << 24) + (msg[4] << 16) + (msg[5] << 8) + msg[6];
                        int32_t y = (msg[7] << 24) + (msg[8] << 16) + (msg[9] << 8) + msg[10];
                        point p = {id, x, y};
                        dspic->plots.push(p);
                        break;
                    }
                    default :
                        std::cout << "Received wrong message code from DsPIC : " << msg[1] << std::endl;
                        break;
                }
            }
        }


            //std::cout << "DsPIC >>" << s << std::endl;

	   //delay(100);
   }
   std::cout << "Hello World!" << std::endl;
   pthread_exit(NULL);
}
