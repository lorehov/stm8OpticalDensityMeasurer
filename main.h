#include "inc/stm8s208r8.h"
#include "inc/stm8s.h"

typedef enum {
    TRANSMITTING_DATA = 0,
    TRANSMITTING_TRESHHOLD = 1,
    RECEIVING_TRESHHOLD = 2
}Interaction_Status;


int treshhold = 100;
int state = 0;


void Periph_Init();
void DelayMS(int n);
void Concentration_Check(int con);

void Interaction_Do(int result);
void Treshhold_Receive();
void State_Get();
void Data_Send(int d);

int Conversion_Make();