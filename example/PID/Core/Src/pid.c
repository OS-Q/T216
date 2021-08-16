/**
 ******************************************************************************
 * @file    : pid.c
 * @brief   : This file provides code for the configuration
 *            of the PID instances.
 * @version : v2.0.1
 ******************************************************************************
 * @help    When use PID, the following process should be abided
 *          PID_Model_Positional() for example:
 *
 *          0. Declare a PID_HandleTypeDef: pidx
 *
 * 			1. PID_Init()
 * 			   1.1 set accuracy(auto)
 * 			       also can set by user though call PID_SetAccuracy()
 *
 * 			2. Get PID taeget value: tar
 *
 * 			3. while(condition) {
 *
 * 			       // do something
 * 			       ......
 * 			       PID_Model_Positional(&pidx,tar);
 * 			       ......
 * 			       //do something
 *
 * 			   }
 *
 * 			(4. PID_DeInit() if it is needed)
 *
 * 			Call other functions if it is needed
 ******************************************************************************
 * */
#include "pid.h"
#include "main.h"
/**
 * @brief:  get modulus of a double
 *          the same function as fabs() in "math.h"
 *          but I don't need all functions in it
 * @return: modulus of the param
 * */
float PID_GetOffsetModul(float num) {

#ifndef _MATH_H_
	if (num >= 0.0)
		return num;
	else
		return -num;
#endif

#ifdef _MATH_H_
	return fabs(num);
#endif
}

/**
 * @brief: Init all params in pid
 *         Always call the function before use pid
 *         Some of the params with be set to default
 * @call:  Before all pid related functions
 * */
void PID_Init(PID_HandleTypeDef *pid, float kp, float ki, float kd) {

	pid->Status = pid_enable;
	pid->Accuracy = PID_DEFAULT_ACCURACY;
	pid->SetValue = 0.0;
	pid->ActualValue = 0.0;
	pid->Offset = 0.0;
	pid->OffsetNext = 0.0;
	pid->OffsetLast = 0.0;
	pid->Integral = 0.0;
	pid->Kp = kp;
	pid->Ki = ki;
	pid->Kd = kd;
	pid->Uk = 0.0;

}
/**
 * @brief: Deinit pid
 *         Set All params to default
 * @call:  After pid is unneeded
 * */
void PID_DeInit(PID_HandleTypeDef *pid) {

	pid->Status = pid_uninit;
	pid->SetValue = 0.0;
	pid->ActualValue = 0.0;
	pid->Offset = 0.0;
	pid->OffsetNext = 0.0;
	pid->OffsetLast = 0.0;
	pid->Kp = 0.0;
	pid->Ki = 0.0;
	pid->Kd = 0.0;
	pid->Integral = 0.0;
	pid->Uk = 0.0;

}

/**
 * @brief: Set pid related params
 *         Set Kp to kp
 *         Set Ki to ki
 *         Set Kd to kd
 * @call:  If it is needed to set params
 * */
void PID_SetParam(PID_HandleTypeDef *pid, float kp, float ki, float kd) {
	if (pid->Status == pid_enable) {
			pid->Kp = kp;
			pid->Ki = ki;
			pid->Kd = kd;
		} else if (pid->Status == pid_disable) {
			PID_Init(pid, kp, ki, kd);
			/* CODE TO SHOW MSG ABOUT PID IS DISABLE */

		} else if (pid->Status == pid_uninit) {
			PID_Init(pid, kp, ki, kd);
			/* CODE TO SHOW MSG ABOUT PID IS UNINIT */

		} else {
			/* SET PARAMS WITH OUT OTHER OPERATION */
			pid->Kp = kp;
			pid->Ki = ki;
			pid->Kd = kd;
		}
}

/**
 * @brief: Reset pid related params
 *         Set Kp to kp
 *         Set Ki to ki
 *         Set Kd to kd
 * @call:  If it is needed to reset params
 * */
void PID_ResetParam(PID_HandleTypeDef *pid, float kp, float ki, float kd) {

	PID_SetParam(pid, kp, ki, kd);

}

/**
 * @brief: set PID accuracy
 * @param accuracy: desired accuracy
 * */
void PID_SetAccuracy(PID_HandleTypeDef *pid, float accuracy) {
	pid->Accuracy = accuracy;
}

/**
 * @brief: positional pid
 * @param pid:     pid
 * @param target:  target value
 * */
float PID_Model_Positional(PID_HandleTypeDef *pid, float target) {

	if (pid->Status == pid_enable) {

		pid->SetValue = target;
		pid->Offset = (pid->SetValue) - (pid->ActualValue);
		pid->Integral += pid->Offset;
		pid->Uk = (pid->Kp * pid->Offset) + (pid->Ki * pid->Integral)
				+ pid->Kd * (pid->Offset - pid->OffsetLast);
		pid->OffsetLast = pid->Offset;
		pid->ActualValue = pid->Uk * 1.0;

		return pid->ActualValue;
	}

	else {
		return (float) pid->Status;
	}
}

/**
 * @brief: incremental pid
 * @param pid:     pid
 * @param target:  target value
 * */
float PID_Model_Incremental(PID_HandleTypeDef *pid, float target) {
	if (pid->Status == pid_enable) {

		float increment = 0.0;
		pid->SetValue = target;
		pid->Offset = (pid->SetValue) - (pid->ActualValue);
		increment = pid->Kp * (pid->Offset - pid->OffsetNext)
				+ pid->Ki * pid->Offset
				+ pid->Kd
						* (pid->Offset - 2 * pid->OffsetNext + pid->OffsetLast);
		pid->Uk += increment;
		pid->ActualValue = pid->Uk * 1.0;
		pid->OffsetLast = pid->OffsetNext;
		pid->OffsetNext = pid->Offset;

		return pid->ActualValue;

	} else {
		return (float) pid->Status;
	}
}

/**
 * @brief: integral separation pid
 * @param pid:        pid
 * @param target:     target value
 * @param moffset:    max offset
 * */
float PID_Model_IntegralSeparation(PID_HandleTypeDef *pid, float target,
		float moffset) {
	if (pid->Status == pid_enable) {

		uint8_t flag = 1;

		pid->SetValue = target;
		pid->Offset = (pid->SetValue) - (pid->ActualValue);

		if ((float) GetModul(pid->Offset) > moffset) {
			flag = 0;
		} else {
			flag = 1;
			pid->Integral += pid->Offset;
		}
		pid->Uk = (pid->Kp * pid->Offset) + (flag * pid->Ki * pid->Integral)
				+ pid->Kd * (pid->Offset - pid->OffsetLast);

		pid->OffsetLast = pid->Offset;
		pid->ActualValue = pid->Uk * 1.0;

		return pid->ActualValue;

	} else {
		return (float) pid->Status;
	}
}

/**
 * @brief: trapezoidal integral pid
 * @param pid:        pid
 * @param target:     target value
 * @param umax:       max integral offset of integral
 * @param umin:       min integral offset of integral
 * @param sepedge:    separation edge
 * */
float PID_Model_Anti_Windup(PID_HandleTypeDef *pid, float target, float umax,
		float umin, float sepedge) {
	if (pid->Status == pid_enable) {

		uint8_t flag = 0;
		pid->SetValue = target;
		pid->Offset = pid->SetValue - pid->ActualValue;

		if (pid->ActualValue > umax) {
			if (GetModul(pid->Offset) > sepedge) {
				flag = 0;
			} else {
				flag = 1;
				if (pid->Offset < 0) {
					pid->Integral += pid->Offset;
				}
			}
		} else if (pid->ActualValue < umin) {
			if (GetModul(pid->Offset) > sepedge) {
				flag = 0;
			} else {
				flag = 1;
				if (pid->Offset > 0) {
					pid->Integral += pid->Offset;
				}
			}
		} else {
			if (GetModul(pid->Offset) > sepedge) {
				flag = 0;
			} else {
				flag = 1;
				pid->Integral += pid->Offset;
			}
		}

		pid->Uk = (pid->Kp * pid->Offset) + (flag * pid->Ki * pid->Integral)
				+ pid->Kd * (pid->Offset - pid->OffsetLast);
		pid->OffsetLast = pid->Offset;
		pid->ActualValue = pid->Uk * 1.0;

		return pid->ActualValue;

	} else {
		return (float) pid->Status;
	}
}

/**
 * @brief: trapezoidal integral pid
 * @param pid:        pid
 * @param target:     target value
 * @param umax:       max integral offset
 * @param umin:       min integral offset
 * @param sepedge:    separation edge
 * */
float PID_Model_TrapezoidalIntegral(PID_HandleTypeDef *pid, float target,
		float umax, float umin, float sepedge) {
	if (pid->Status == pid_enable) {

		uint8_t flag = 0;
		pid->SetValue = target;
		pid->Offset = pid->SetValue - pid->ActualValue;

		if (pid->ActualValue > umax) {
			if (GetModul(pid->Offset) > sepedge) {
				flag = 0;
			} else {
				flag = 1;
				if (pid->Offset < 0) {
					pid->Integral += pid->Offset;
				}
			}
		} else if (pid->ActualValue < umin) {
			if (GetModul(pid->Offset) > sepedge) {
				flag = 0;
			} else {
				flag = 1;
				if (pid->Offset > 0) {
					pid->Integral += pid->Offset;
				}
			}
		} else {
			if (GetModul(pid->Offset) > sepedge) {
				flag = 0;
			} else {
				flag = 1;
				pid->Integral += pid->Offset;
			}
		}

		pid->Uk = (pid->Kp * pid->Offset) + (flag * pid->Ki * pid->Integral / 2)
				+ pid->Kd * (pid->Offset - pid->OffsetLast);
		pid->OffsetLast = pid->Offset;
		pid->ActualValue = pid->Uk * 1.0;

		return pid->ActualValue;

	} else {
		return (float) pid->Status;
	}
}

/**
 * @brief: variable integral pid
 * @param pid:        pid
 * @param target:     target value
 * @param lowedge:    lower limit of variable integral
 * @param highedge:   upper limit of variable integral
 * */
float PID_Model_VariableIntegral(PID_HandleTypeDef *pid, float target,
		float lowedge, float highedge) {
	if (pid->Status == pid_enable) {

		float Ki2;
		pid->SetValue = target;
		pid->Offset = pid->SetValue - pid->ActualValue;

		if ((float) GetModul(pid->Offset) > highedge) {
			Ki2 = 0.0;
		} else if ((float) GetModul(pid->Offset) < lowedge) {
			Ki2 = 1.0;
			pid->Integral += pid->Offset;
		} else {
			Ki2 = (highedge - (float) GetModul(pid->Offset))
					/ (highedge - lowedge);
			pid->Integral += pid->Offset;
		}
		pid->Uk = pid->Kp * pid->Offset + Ki2 * pid->Ki * pid->Integral
				+ pid->Kd * (pid->Offset - pid->OffsetLast);

		pid->OffsetLast = pid->Offset;
		pid->ActualValue = pid->Uk * 1.0;

		return pid->ActualValue;

	} else {
		return (float) pid->Status;
	}
}
