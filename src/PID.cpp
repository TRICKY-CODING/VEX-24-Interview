#include "PID.h"

#include <math.h>

#include <iostream>

#include "calc.h"
#include "geometry.h"
#include "my-timer.h"
#include "vex.h"
using namespace std;

/**
 * 本文件定义PID控制器
 * 需完成以下内容并在注释中完整解释：
 * PID::update()
 * DirPID::update()
 * PosPID::update()
 * 
 * 样例使用方法：
 * PID pid;
 * pid.setTarget(100);
 * pid.setErrorTolerance(1);
 * pid.setCoefficient(1, 0.1, 0.1);
 * while(!pid.taragetArrived()){
 *      pid.update(curr_value);
 *      output = pid.getOutput();
 *      // do something with output
 * }
 * 
 */

PID::PID() : first_time(true), arrived(false), I_max(20), I_range(50), jump_time(50), D_tol(10) { my_timer.reset(); }

void PID::setFirstTime() { first_time = true; }

void PID::setCoefficient(double _kp, double _ki, double _kd) {
    kp = _kp;
    ki = _ki;
    kd = _kd;
}
void PID::setTarget(double _target) { target = _target; }
void PID::setIMax(double _IMax) { I_max = _IMax; }
void PID::setIRange(double _IRange) { I_range = _IRange; }
void PID::setErrorTolerance(double _errorTol) { error_tol = _errorTol; }
void PID::setDTolerance(double _DTol) { D_tol = _DTol; }
void PID::setJumpTime(double _jumpTime) { jump_time = _jumpTime; }
void PID::setArrived(bool _arrived) { arrived = _arrived; }
bool PID::targetArrived() { return arrived; }
double PID::getOutput() { return output; }

void PID::update(double input) {
    error_curt = target - input;  // calculate current error
    //TODO: calculate the contribution of P, I, D with kp, ki, kd
     // 比例项：P = Kp * error
    P = kp * error_curt;

    // 积分项：I = Ki * ∫error dt
    // 如果误差较小，则积分累积
    if (abs(error_curt) <= I_range) {
        error_int += error_curt;
    } else {
        error_int = 0; // 如果误差过大，清空积分部分
    }

    // 限制积分值的最大值，防止积分饱和
    if (error_int > I_max) {
        error_int = I_max;
    } else if (error_int < -I_max) {
        error_int = -I_max;
    }

    I = ki * error_int;

    // 微分项：D = Kd * (error - prev_error) / dt
    if (!first_time) {
        error_dev = error_curt - error_prev;
        D = kd * error_dev;
    } else {
        D = 0; // 首次时没有历史误差，D项为0
    }

    // 保存当前误差用于下次计算微分项
    error_prev = error_curt;
    
    if (abs(error_curt) <= error_tol) {  // Exit when staying in tolerated region and
                                        // maintaining a low enough speed
        arrived = true;
    }
    output = P + I + D;
}

void PosPID::setTarget(Point p) { target_point = p; }

void PosPID::update(Point input) {
    Vector err = target_point - input;
    error_curt = err.mod();  // calculate current error
    //TODO: calculate the contribution of P, I, D with kp, ki, kd
    // 比例项：P = Kp * error
    P = kp * error_curt;

    // 积分项：I = Ki * ∫error dt
    // 如果误差较小，则积分累积
    if (abs(error_curt) <= I_range) {
        error_int += error_curt;
    } else {
        error_int = 0; // 如果误差过大，清空积分部分
    }

    // 限制积分值的最大值，防止积分饱和
    if (error_int > I_max) {
        error_int = I_max;
    } else if (error_int < -I_max) {
        error_int = -I_max;
    }

    I = ki * error_int;

    // 微分项：D = Kd * (error - prev_error) / dt
    if (!first_time) {
        error_dev = error_curt - error_prev;
        D = kd * error_dev;
    } else {
        D = 0; // 首次时没有历史误差，D项为0
    }

    // 保存当前误差用于下次计算微分项
    error_prev = error_curt;
    
    if (abs(error_curt) <= error_tol) {  // Exit when staying in tolerated region and
                                        // maintaining a low enough speed
        arrived = true;
    }
    output = P + I + D;
}

void DirPID::update(double input) {
    error_curt = degNormalize(target - input);  // calculate current error
    //TODO: calculate the contribution of P, I, D with kp, ki, kd
    // 比例项：P = Kp * error
    P = kp * error_curt;

    // 积分项：I = Ki * ∫error dt
    // 如果误差较小，则积分累积
    if (abs(error_curt) <= I_range) {
        error_int += error_curt;
    } else {
        error_int = 0; // 如果误差过大，清空积分部分
    }

    // 限制积分值的最大值，防止积分饱和
    if (error_int > I_max) {
        error_int = I_max;
    } else if (error_int < -I_max) {
        error_int = -I_max;
    }

    I = ki * error_int;

    // 微分项：D = Kd * (error - prev_error) / dt
    if (!first_time) {
        error_dev = error_curt - error_prev;
        D = kd * error_dev;
    } else {
        D = 0; // 首次时没有历史误差，D项为0
    }

    // 保存当前误差用于下次计算微分项
    error_prev = error_curt;
    
    if (abs(error_curt) <= error_tol) {  // Exit when staying in tolerated region and
                                        // maintaining a low enough speed
        arrived = true;
    }
    output = P + I + D;
}