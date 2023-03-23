#include "RF.h"
#include "NRF/NRF.h"
#include "timer/timer.h"
#include "hardware.h"

#define PITCH_POSITIVE_MAX_ANGLE_DEG    10.0
#define ROLL_POSITIVE_MAX_ANGLE_DEG    10.0
#define YAW_POSITIVE_MAX_ANGLE_DEG    30.0

static uint8_t internalBuffer[32];
static uint8_t address[6] = "00001";
static bool atLeastOneAvailable = false;
static bool timeOut = false;

static tim_id_t timeOutRF; 

static OrientationRF calibrationValues = {.pitch = 256/2, .roll = 256/2, .throttle = 0, .yaw = 256/2}; // default values
void RFinit(){
	timerInit();
    timeOutRF = timerGetId();
	RF24begin();
    openReadingPipe(0, address);
    setPALevel(RF24_PA_MIN);
}


void RFbegin(){
	startListening();
}
void RFcalibrate(){
    while(!atLeastOneAvailable){
        if(available()){
        	read_payLoad(4);
            atLeastOneAvailable = true;
        }
    }
    double sums[4] = {0, 0, 0, 0};
    for (uint8_t i = 0; i < 5; i++)
    {
        OrientationRF tmp;
        RFgetDeNormalizedData(&tmp);
        timerDelay(TIMER_MS2TICKS(100));
        sums[0] += tmp.throttle; 
        sums[1] += tmp.pitch; 
        sums[2] += tmp.roll; 
        sums[3] += tmp.yaw; 
    }
        calibrationValues.throttle = sums[0] / 5.0; 
        calibrationValues.pitch = sums[1] / 5.0; 
        calibrationValues.roll = sums[2] / 5.0; 
        calibrationValues.yaw = sums[3] / 5.0; 
}
void callBackTimeOut(){
    timeOut = true;
}
void RFgetDeNormalizedData(OrientationRF * dataPtr){
    static bool triggerTimeOut = true;
    if(triggerTimeOut){
        timerStart(timeOutRF, TIMER_MS2TICKS(1000), TIM_MODE_SINGLESHOT, callBackTimeOut);
        triggerTimeOut = false;
    }
    if(available()){
    	read_payLoad(4);
        timerStop(timeOutRF);
        triggerTimeOut = true;
    }
    readLastData(internalBuffer, 4);
    if(timeOut)
        internalBuffer[0] = 0;
    dataPtr->throttle = internalBuffer[0];
    dataPtr->pitch = internalBuffer[1];
    dataPtr->roll = internalBuffer[2];
    dataPtr->yaw = internalBuffer[3];
}
/*
	eqn1 = F1 + F2 + F3 + F4 == U1;
	eqn2 = F3 - F1 == U2;
	eqn3 = F4 - F2 == U3; 
	eqn4 = +c*F1 + c*F3 - c*F2 - c*F4 == U4;
*/
/*
rotacion alrededor del x del sensor (de y a z) -> GyroRates_deg[0] == p
rotacion alrededor del y del sensor (de x a z) -> GyroRates_deg[1] == q
rotacion alrededor del z del sensor (de y a x) -> GyroRates_deg[2] == r

newAngles[0] = roll ~ phi (de y a z) -> p
newAngles[1] = pitch ~ theta (de x a z) -> q
newAngles[2] = yaw ~ psi (de x a y) -> r
*/
void RF2Newton(OrientationRF * dataPtr){
    // basandonos en la curva de fuerza vs pwm en matlab, y a sabiendas de que 
    // U[0][0] = \sum F_i. U[0][0] MAX es 425g o 4.25N*4 = 17N. Pero se tomara como maxima 3N*4 = 12
    dataPtr->throttle = dataPtr->throttle*12/255; // Asumiendo que 0 es 0N y 255 es 12 N de fuerza total
    dataPtr->pitch = (dataPtr->pitch - (double)calibrationValues.pitch)*PITCH_POSITIVE_MAX_ANGLE_DEG/(256.0/2.0);
    dataPtr->roll = (dataPtr->roll - (double)calibrationValues.roll)*ROLL_POSITIVE_MAX_ANGLE_DEG/(256.0/2.0);
    dataPtr->yaw = (dataPtr->yaw - (double)calibrationValues.yaw)*YAW_POSITIVE_MAX_ANGLE_DEG/(256.0/2.0);
    if(dataPtr->throttle < 0) 
        dataPtr->throttle = 0;
    else if(dataPtr->throttle > 12)
        dataPtr->throttle = 12;
    if(dataPtr->pitch < -PITCH_POSITIVE_MAX_ANGLE_DEG) 
        dataPtr->pitch = -PITCH_POSITIVE_MAX_ANGLE_DEG;
    else if(dataPtr->pitch > PITCH_POSITIVE_MAX_ANGLE_DEG)
        dataPtr->pitch = PITCH_POSITIVE_MAX_ANGLE_DEG;
    if(dataPtr->pitch < -ROLL_POSITIVE_MAX_ANGLE_DEG) 
        dataPtr->pitch = -ROLL_POSITIVE_MAX_ANGLE_DEG;
    else if(dataPtr->pitch > ROLL_POSITIVE_MAX_ANGLE_DEG)
        dataPtr->pitch = ROLL_POSITIVE_MAX_ANGLE_DEG;
    if(dataPtr->pitch < -YAW_POSITIVE_MAX_ANGLE_DEG) 
        dataPtr->pitch = -YAW_POSITIVE_MAX_ANGLE_DEG;
    else if(dataPtr->pitch > YAW_POSITIVE_MAX_ANGLE_DEG)
        dataPtr->pitch = YAW_POSITIVE_MAX_ANGLE_DEG;
}
