#include "bsp_pid.h"


pid_TypeDef pid ;


int PID_send(int tarpos, int curpos)
{
	float incrementSpeed=0;

	pid.SetPos=tarpos;
	pid.ActualPos=curpos;
	pid.err=pid.SetPos-pid.ActualPos;
	
	incrementSpeed=pid.Kp*pid.err-pid .Ki *pid .err_next +pid .Kd *pid.err_last;
	pid.err_last =pid .err_next ;
	pid.err_next =pid .err ;
	pid.pid_out=incrementSpeed;
	return pid.pid_out ;
}


void PID_init()
{
	pid.SetPos=0.0;
	pid.ActualPos=0.0;
	pid.err=0.0;         //current err
	pid.err_next=0.0;    //pre  err
	pid.err_last=0.0;    //pre pre err

	pid.Kp = 2.0;
	pid.Ki = 0.1;
	pid.Kd = 0.2;
	pid.pid_out=0;
	pid.cnt=0;
}

