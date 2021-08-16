#ifndef __PID_H
#define __PID_H

#ifdef __cplusplus
extern "C" {
#endif

#define PID_DEFAULT_ACCURACY    0.0005

#define PID_DISABLE             0x00
#define PID_ENABLE              0x01
#define PID_UNINIT              0x02
#define PID_FAILED              0x03

#define Output               Uk
#define PID_RESET_OUPUT      0x00
#define PID_HOLD_OUTPUT      0x01

typedef enum __PID_StatusDef {

	pid_disable = PID_DISABLE,
	pid_reset_output = PID_RESET_OUPUT,
	pid_hold_output = PID_HOLD_OUTPUT,
	pid_enable = PID_ENABLE,
	pid_uninit = PID_UNINIT,
	pid_failed = PID_FAILED

} PID_StatusDef;

typedef struct __PID_HandleTypeDef {

	PID_StatusDef Status;
	float Accuracy;
	float SetValue;
	float ActualValue;
	float Offset;       //Offset: SetValue - ActualValue
	float OffsetNext;
	float OffsetLast;
	float Kp;           //Proportion coefficient
	float Ki;           //Integral coefficient
	float Kd;           //Differential coefficient
	float Integral;
	float Uk;           //Output

} PID_HandleTypeDef;

/**
 * PID Configuration
 * */
void PID_Init(PID_HandleTypeDef *pid, float kp, float ki, float kd);
void PID_DeInit(PID_HandleTypeDef *pid);
void PID_SetParam(PID_HandleTypeDef *pid, float kp, float ki, float kd);
void PID_ResetParam(PID_HandleTypeDef *pid, float kp, float ki, float kd);
void PID_SetAccuracy(PID_HandleTypeDef *pid, float accuracy);

/**
 * PID Algorithms
 * */
float PID_GetOffsetModul(float num);
float PID_Model_Positional(PID_HandleTypeDef *pid, float target);
float PID_Model_Incremental(PID_HandleTypeDef *pid, float target);
float PID_Model_IntegralSeparation(PID_HandleTypeDef *pid, float target,
		float moffset);
float PID_Model_Anti_Windup(PID_HandleTypeDef *pid, float target, float umax,
		float umin, float sepedge);
float PID_Model_TrapezoidalIntegral(PID_HandleTypeDef *pid, float target,
		float umax, float umin, float sepedge);
float PID_Model_VariableIntegral(PID_HandleTypeDef *pid, float target,
		float lowedge, float highedge);


#ifdef __cplusplus
}
#endif

#endif
