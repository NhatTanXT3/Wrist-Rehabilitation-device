#ifndef COMMUNICATION_
#define COMMUNICATION_

#define CN_1 '~'
#define CN_2 '!'
#define CN_3 '@'
#define CN_4 '#'
#define CN_5 '$'
#define CN_6 '%'
#define CN_7 '^'
#define CN_8 '*'
#define CN_9 '('
#define CN_10 ')'

/*================================
   Nhận từ máy tính xuống
=================================*/
#define HELP_       'h'
#define SYSTEM_INFOR 'i'
#define TEST_MOTOR_   '"'//'a'
#define SET_FORCE_   'm'
#define SET_OMEGA_ 't'
#define SET_POSITION_ATTITUDE_ 'a'

#define RUN_       'c'
#define STOP_      'd'

#define DISPLAY_ON_  'e'
#define DISPLAY_OFF_ 'f'

#define SET_KP_ 'P'
#define SET_KI_ 'I'
#define SET_KD_ 'D'
#define SET_SETPOINT_ 'S'

#define PID_POSITION_     'P'
#define PID_ADMITTANCE_   'A'
#define PID_FORCE_        'F'
#define PID_VELO_         'V'

#define MODE_SET_     'w'
#define MODE_1_ '1'
#define MODE_2_ '2'
#define MODE_3_ '3'

void serialEvent();

#endif


