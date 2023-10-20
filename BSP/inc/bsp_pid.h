#ifndef __BSP_PID_H
#define __BSP_PID_H

typedef struct
{
	float SetPos;
	float ActualPos;
	float err;
	float err_next;
	float err_last;
	float Kp,Ki,Kd;
	int  pid_out;
    int cnt;

}pid_TypeDef;

extern pid_TypeDef pid ;

int PID_send(int tarpos,int curpos);
void PID_init(void);


#endif

