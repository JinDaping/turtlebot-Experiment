#ifndef PID_H
#define PID_H

typedef struct {
  float kp;
  float ki;
  float kd;

  float dErrP;
  float dErrI;
  float dErrD;

  float errNow;
  float errOld1;
  float errOld2;

  float dCtrOut;
  float dOutMAX;
  float ctrOut;
  float OutMAX;

} PID_IncrementType;

typedef struct {
  float kp;
  float ki;
  float kd;

  float ErrP;
  float ErrI;
  float ErrD;

  float errNow;
  float errOld;
  float errILim;
  float OutMAX;

  float ctrOut;

} PID_AbsoluteType;

void PID_IncrementMode(PID_IncrementType *pid);
void PID_AbsoluteMode(PID_AbsoluteType *pid);

#endif
