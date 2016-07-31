#ifndef PID_
#define PID_

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
typedef struct{
	float set_point;
	float KP;
	float KD;
	float KI;
	float output;
	float er;
	float pre_er;
	float pre_pre_er;
	
        float p_er;
	float p_pre_er;
	float p_pre_pre_er;
	
        float sampling_time;

	float P_term;
	float I_term;
	float D_term;
	float I_limit;

	float fb;//feedback
	float pre_fb;
	float pre_pre_fb;
}PID;

void PID_init();

void PID_controller(float feeback, PID *ptr_pid)
{
	ptr_pid->p_er=(ptr_pid->KP)+(ptr_pid->KI)*(ptr_pid->sampling_time)/(2.0)+(ptr_pid->KD)/(ptr_pid->sampling_time);
	ptr_pid->p_pre_er=-(ptr_pid->KP)+(ptr_pid->KI)*(ptr_pid->sampling_time)/(2.0)-(2.0)*(ptr_pid->KD)/(ptr_pid->sampling_time);
	ptr_pid->p_pre_pre_er=(ptr_pid->KD)/(ptr_pid->sampling_time);

	ptr_pid->er=ptr_pid->set_point-feeback;
	ptr_pid->output += ptr_pid->er*ptr_pid->p_er + ptr_pid->pre_er*ptr_pid->p_pre_er + ptr_pid->pre_pre_er*ptr_pid->p_pre_pre_er;

	ptr_pid->pre_pre_er=ptr_pid->pre_er;
	ptr_pid->pre_er=ptr_pid->er;
}
void PD_controller(float feeback, PID *ptr_pid)
{
	ptr_pid->p_er=(ptr_pid->KP)+(ptr_pid->KD)/(ptr_pid->sampling_time);
	ptr_pid->p_pre_er=-(ptr_pid->KP)-(2.0)*(ptr_pid->KD)/(ptr_pid->sampling_time);
	ptr_pid->p_pre_pre_er=(ptr_pid->KD)/(ptr_pid->sampling_time);

	ptr_pid->er=ptr_pid->set_point-feeback;
	ptr_pid->output += ptr_pid->er*ptr_pid->p_er + ptr_pid->pre_er*ptr_pid->p_pre_er + ptr_pid->pre_pre_er*ptr_pid->p_pre_pre_er;

	ptr_pid->pre_pre_er=ptr_pid->pre_er;
	ptr_pid->pre_er=ptr_pid->er;
}

void PID_type_3(float feedback, PID *ptr_pid)
{
	ptr_pid->er=ptr_pid->set_point - feedback;
	ptr_pid->fb=feedback;

	ptr_pid->P_term=ptr_pid->KP*ptr_pid->er;
//  if(ptr_pid->er==0)
//    ptr_pid->I_term=0;
//  else{
	ptr_pid->I_term += ptr_pid->KI*ptr_pid->sampling_time*(ptr_pid->er+ptr_pid->pre_er)/2.0;
	ptr_pid->I_term=constrain(ptr_pid->I_term,-ptr_pid->I_limit,ptr_pid->I_limit);
//  }
	ptr_pid->D_term = ptr_pid->KD*(-ptr_pid->fb + ptr_pid->pre_fb)/ptr_pid->sampling_time;
	ptr_pid->pre_er=ptr_pid->er;
	ptr_pid->pre_fb=ptr_pid->fb;


	ptr_pid->output=ptr_pid->P_term+ptr_pid->I_term+ptr_pid->D_term;
}

#endif
