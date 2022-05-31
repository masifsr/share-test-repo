/****************************************************************************
 *	Copyright (C) 2019-2024 Seven Robotics. All rights reserved.
 *	Author: Mohammed Talha Arif  <mtalha@sevenrobotics.in>
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in
 *     the documentation and/or other materials provided with the
 *     distribution.
 * 	3. Neither the name Seven Robotics nor the names of its contributors
 * 		 may be used to endorse or promote products derived from this
 * 		 software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 *  OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *  AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
#pragma once

#include <control/dataTypes/motorAllState.hpp>
#include <stateEstimation/ortegaObserver.hpp>
#include <ch.hpp>
#include <sys/OS.hpp>
#include <math/math.hpp>
#include <control/FOC.hpp>
#include <control/bldcMotorController.hpp>
#include <util/hw.hpp>
#include <util/MicroSecondTimer.hpp>

#define ARR TIM1->ARR

class motorController;


class FOC {
 private:
    float m1Currents[3];
    float m2Currents[3];
    float m1Voltages[3];
    float m2Voltages[3];
    motorController& controller;
    Observer<motorAllState>& observer;
    bool encoder_being_used = true;
    bool stopFOC=false;

    float tempOffset = 0.0;
    unsigned int rpmCounter = 0;
    unsigned int rpmControllerCounter = 0;
    float angleSum = 0.0f;
    float pre_angleSum = 0.0f;
    float rpm = 0.0f;
    int64_t micros_now = 0;
    int64_t micros_pre = 0;
    int64_t micros_detla = 0;
    

 public:
  FOC(motorController& controller, Observer<motorAllState>& obs): controller{controller},observer{obs} {}

  void FOC_Init();
  void motorAlign();
  inline void clarkeTransform(motorAllState& M1,motorAllState& M2);
  inline void parkTransform(motorAllState& M1,motorAllState& M2);
  //std::tuple<float,float,float,float> inverseParkTransform(motorAllState& M1,motorAllState& M2);
  inline void inverseParkTransform(motorAllState& M1,motorAllState& M2,float& modAlpha_M1,float& modAlpha_M2,float& modBeta_M1, float& modBeta_M2);
  inline void controlCurrent(motorAllState& M1, motorAllState& M2,float dt);
  void focISR(stm32plus::DmaEventType evt); // called by motorController at DMA EOT
  void sensorDetectionAndAlign();
  inline void svm(motorAllState& m,float& alpha, float& beta,uint32_t* T);
  inline void updateValphaVbeta(motorAllState& M, float modAlpha,float modBeta, bool isM1=false);
  inline void pll(motorAllState& M1, motorAllState& M2, float dt);
  inline void dutyControl(motorAllState& M1, motorAllState& M2);
  inline void rpmControl(motorAllState& M1, motorAllState& M2);
  inline void updateSinCos(motorAllState& M1, motorAllState& M2);
  inline void updateSinCosNow(motorAllState& M1, motorAllState& M2);
};


void FOC::FOC_Init() {
    // chibios_rt::System::lock();
    // //this->resetMotorStates();
    // controller = ctrlr;
    // chibios_rt::System::unlock();
    
}

void FOC::sensorDetectionAndAlign() {
        //sys::std_out("alignment starting\r\n");
        controller.setControlMode(CONTROL_STATE::DUTY);
        motorAllState& M1 = controller.getM1State();
        motorAllState& M2 = controller.getM2State();

        // M1.phase_override = true;
        // M1.idSet = 0;
        // // M1.iqSet = controller().detectCurrent;
        // float startAngle_M1 = controller.getM1EncoderAngle();
        // float startAngle_M2 = controller.getM2EncoderAngle();
        
        controller.stopIncrementalEncoder();
        M2.phase_override = true;
        M2.idSet = 2;
        M2.iqSet = 0;

        auto polePairs = controller().polePairs;
        float sinSum=0;
        float cosSum=0;
        float s,c;

        chThdSleep(MS2ST(500));

        //  M2.phase_override = true;
        controller.startIncrementalEncoder();
        for(int i =0; i<= 200;i++){
           float angle = (math::SR_2PI * (polePairs/2))/200;
           float override = static_cast<float>(i)*angle;
            while(M2.phase_override_now != override) { 
                math::stepTowards(&M2.phase_override_now,override,angle/100.0);
                chThdSleep(MS2ST(1));
            }
            float angleDiff = math::radDiff(controller.getM2EncoderAngle()*(polePairs/2.0),M2.phase_override_now);
            math::fastSinCos(angleDiff,s,c);
            sinSum += s;
            cosSum += c;
        }

       chThdSleep(MS2ST(500));

        for(int i =200; i>= 0;i--){
            float angle = (math::SR_2PI * (polePairs/2))/200;
            float override = static_cast<float>(i)*angle;
            while(M2.phase_override_now != override) { 
                math::stepTowards(&M2.phase_override_now,override,angle/100.0);           
                chThdSleep(MS2ST(1));
            }
            float angleDiff = math::radDiff(controller.getM2EncoderAngle()*(polePairs/2.0),M2.phase_override_now);
            math::fastSinCos(angleDiff,s,c);
            sinSum += s;
            cosSum += c;
        }
        
        tempOffset = atan2f(sinSum, cosSum);

        controller.resetM2Encoder();
        
        M1.idSet = 0;
        M1.iqSet = 0;
        M2.idSet = 0;
        M2.iqSet = 0;
        
        controller.motorAligned = true;
        M1.phase_override = false;
        M2.phase_override = false;
        M1.state.phase = 0;
        M2.state.phase = 0;
        M2.duty_set = 0;
        controller.setControlMode(CONTROL_STATE::DUTY);             
}

inline void FOC::clarkeTransform(motorAllState& M1, motorAllState& M2) {
    M2.state.iAlpha = M2.state.ia;
    M2.state.iBeta = math::ONE_BY_ROOT_THREE * M2.state.ia + math::TWO_BY_ROOT_THREE * M2.state.ib;
}

inline void FOC::parkTransform(motorAllState& M1, motorAllState& M2) {
    M2.state.id = M2.state.phase_cos_now * M2.state.iAlpha + M2.state.phase_sin_now * M2.state.iBeta;
    M2.state.iq = M2.state.phase_cos_now * M2.state.iBeta - M2.state.phase_sin_now * M2.state.iAlpha;
}

inline void FOC::inverseParkTransform(motorAllState& M1,motorAllState& M2,float& modAlpha_M1,float& modAlpha_M2,float& modBeta_M1, float& modBeta_M2) {

    modAlpha_M2 = M2.state.phase_cos_now * M2.state.modVd - M2.state.phase_sin_now * M2.state.modVq;
    modBeta_M2 = M2.state.phase_cos_now * M2.state.modVq + M2.state.phase_sin_now * M2.state.modVd;
}

__attribute__((flatten)) void FOC::focISR(stm32plus::DmaEventType evt) {
    // GPIOG->ODR |= GPIO_Pin_10;

    static unsigned int count = 0;
    bool is_v7 = !(TIM1->CR1 & TIM_CR1_DIR);

    motorAllState& M1 = controller.getM1State();
    motorAllState& M2 = controller.getM2State();
    controller.updateEncoders();

    if(M2.nextDutySet) {
        controller.updateM2DutyHW();
        M2.nextDutySet = false;
    }

   
    if(is_v7)
    {
       return;
    }


    controller.getMotorCurrents(m1Currents,m2Currents);

    M2.state.ia = m2Currents[0];
    M2.state.ib = m2Currents[1];
    M2.state.ic = -(M2.state.ia + M2.state.ib);

    float dt = 1/((controller().f_sw)/2.0);
    float angle = controller.getM2EncoderDeltaAngle()*(controller().polePairs/2);
    M2._obs.phase_encoder += angle;
    angleSum += angle;
    math::normAngleRad2Pi(M2._obs.phase_encoder);
    
    float busV = controller.getBusVoltage();
    math::LPFast(M1.vBus, busV,0.1f);
    M2.vBus = M1.vBus;
   
    if(controller.getControlMode() == CONTROL_STATE::NONE){
        return;
    }

    clarkeTransform(M1,M2);

    //running duty control every time ISR is called
    if(controller.isDutyMode()) {
        dutyControl(M1, M2);
    }

    //Running RPM controller at freq focISR/10
    // if(++rpmControllerCounter == 10) {
    //     rpmControl(M1, M2);
    //     rpmControllerCounter = 0;
    // }

    observer.setState(M2);
    observer.update(dt);

    M2._obs.phase_now += M2.pll_speed*dt;

   
    if(encoder_being_used){
        M2.state.phase = M2._obs.phase_encoder;
    }else{
        M2.state.phase = M2._obs.phase_now;
    }

    float idSetM2 = M2.idSet;
    float iqSetM2 = M2.iqSet;

    if(M2.phase_override){
        M2.state.phase = M2.phase_override_now;
    }
    
    //measuring RPM
    micros_detla = MicroTimer.micros_Since(micros_pre);
    if(++rpmCounter == 100){
        rpmCounter = 0;
        M2.state.rpm_measured = (((angleSum - pre_angleSum)*1000000.0f)/(static_cast<float>(micros_detla)))*(9.5493f/15.0f);
        pre_angleSum = angleSum;
        micros_pre = MicroTimer.micros();
    }

    math::LPFast(M2.state.filtered_rpm_measured,M2.state.rpm_measured,0.01f);

    updateSinCos(M1,M2);

    const float modQM2 = M2.state.modVqFiltered;

    float iqsetTemp = iqSetM2;
    if(modQM2 > 0.001) {
        math::truncate(iqSetM2, controller().in_current_Min/modQM2,controller().in_current_Max/modQM2);
    } else if(modQM2 < -0.001) {
        math::truncate(iqSetM2, controller().in_current_Max/modQM2,controller().in_current_Min/modQM2);
    }

    float current_max_abs = controller.getMaxCurrent();
    math::truncateAbs(idSetM2, current_max_abs);
    math::truncateAbs(iqSetM2, math::squareRoot(math::square(current_max_abs) - math::square(idSetM2)));

    M2.state.idTarget = idSetM2;
    M2.state.iqTarget = iqSetM2;
    
    controlCurrent(M1,M2,dt);

    M2.state.duty = math::SIGN(M2.state.vq)*math::squareRoot((math::square(M2.state.modVd)+math::square(M2.state.modVq)))/math::ROOT_THREE_BY_TWO;
    pll(M1,M2,dt);
}
 

inline void FOC::controlCurrent(motorAllState& M1, motorAllState& M2, float dt) {
    M2.state.phase_sin_now = M2.state.phase_sin;
    M2.state.phase_cos_now = M2.state.phase_cos;

    float maxDutyM2 = controller().max_Duty;

    math::truncate(maxDutyM2,0.0f,controller().max_Duty);

    parkTransform(M1,M2);

    math::LPFast(M2.state.idFiltered,M2.state.id,controller().lpFilterConst);
    math::LPFast(M2.state.iqFiltered,M2.state.iq,controller().lpFilterConst);

    float Ierror_d_M2 = M2.state.idTarget - M2.state.id;
    float Ierror_q_M2 = M2.state.iqTarget - M2.state.iq;

    M2.state.vd = M2.state.vdIntegral + Ierror_d_M2 * M2.foc_params.currentKp; 
    M2.state.vq = M2.state.vqIntegral + Ierror_q_M2 * M2.foc_params.currentKp;

    M2.state.vdIntegral += Ierror_d_M2* (M2.foc_params.currentKi*dt);
    M2.state.vqIntegral += Ierror_q_M2* (M2.foc_params.currentKi*dt);

    float maxV_mag_M2 = (maxDutyM2 * M2.vBus)/math::ROOT_THREE;
    float maxVq_M2 = math::squareRoot(math::square(maxV_mag_M2) - math::square(M2.state.vd));

    float vd_presat_M2 = M2.state.vd;
    float vq_presat_M2 = M2.state.vq;

    math::truncateAbs(M2.state.vd,maxV_mag_M2);
    math::truncateAbs(M2.state.vq,maxVq_M2);

    M2.state.vdIntegral += (M2.state.vd - vd_presat_M2);
    M2.state.vqIntegral += (M2.state.vq - vq_presat_M2);

    // if(M2.state.vdIntegral > 2.0) M2.state.vdIntegral = 2.0;
    // if(M2.state.vdIntegral < -2.0) M2.state.vqIntegral = -2.0;

    math::saturateVector2D(M2.state.vd,M2.state.vq,maxV_mag_M2);

    const float normalizedVoltage = 1.5/M2.vBus;

    M2.state.modVd = M2.state.vd * normalizedVoltage;
    M2.state.modVq = M2.state.vq * normalizedVoltage;
    math::NanZero( M2.state.modVd);
    math::NanZero( M2.state.modVq);
    math::LPFast(M2.state.modVqFiltered,M2.state.modVq,0.2f);

    M2.state.iAbs = math::squareRoot(math::square(M2.state.id) + math::square(M2.state.iq));
    M2.state.iAbsFiltered = math::squareRoot(math::square(M2.state.idFiltered) + math::square(M2.state.iqFiltered));
     
    float modAlpha_M1, modBeta_M1, modAlpha_M2, modBeta_M2;
    //Inverse park transform
    inverseParkTransform(M1,M2,modAlpha_M1,modAlpha_M2,modBeta_M1,modBeta_M2);

    updateValphaVbeta(M2,modAlpha_M2,modBeta_M2);

    svm(M2,modAlpha_M2,modBeta_M2,controller.getM2Duty());
     static int counter=0;

   if(counter++ >= 50)
    { 
        counter = 0;
        sys::std_out("%f %f\r\n", M2.state.vq, M2.state.vd);
    }
}

inline void FOC::svm(motorAllState& M, float& modAlpha,float& modBeta,uint32_t* T) {
    // static int count = 0;
    if(modBeta >= 0.0f) {
        if(modAlpha >= 0.0f) {
            if((math::ONE_BY_ROOT_THREE * modBeta) > modAlpha)
            {
                M.state.sector = 2;
            }
            else 
            {
                M.state.sector = 1;
            }
        }
        else {
            if(-(math::ONE_BY_ROOT_THREE * modBeta) > modAlpha) 
            {
                M.state.sector = 3;
            }
            else
            {
                M.state.sector = 2;
            }
        }
    }
    else {
        if(modAlpha >= 0.0f) {
            if(-(math::ONE_BY_ROOT_THREE * modBeta) > modAlpha)
            {
                M.state.sector = 5;
            }
            else
            {
                M.state.sector = 6;
            }
        }
        else
        {
            if((math::ONE_BY_ROOT_THREE * modBeta) > modAlpha)
            {
                M.state.sector = 4;
            }
            else 
            {
                M.state.sector = 5;
            }
        }
    }

    switch(M.state.sector) {
        case 1: {
            uint32_t t1 = (modAlpha - (math::ONE_BY_ROOT_THREE * modBeta)) * ARR;
            uint32_t t2 = math::TWO_BY_ROOT_THREE * modBeta * ARR;

            T[0] = (ARR + t1 + t2)/2;
            T[1] = T[0] - t1;
            T[2] = T[1] - t2;

            break;
        }

        case 2: {
            uint32_t t2 = (modAlpha + (math::ONE_BY_ROOT_THREE * modBeta)) * ARR;
            uint32_t t3 = (-modAlpha + (math::ONE_BY_ROOT_THREE * modBeta)) * ARR;

            T[1] = (ARR + t2 + t3)/2;
            T[0] = T[1] - t3; 
            T[2] = T[0] - t2;

            break;
        }

        case 3: {
            uint32_t t3 = math::TWO_BY_ROOT_THREE * modBeta * ARR;
            uint32_t t4 = (-modAlpha - (math::ONE_BY_ROOT_THREE * modBeta)) * ARR;
            
             T[1] = (ARR + t3 + t4)/2;
             T[2] = T[1] - t3;
             T[0] = T[2] - t4;

             break;
        }

        case 4: {
            uint32_t t4 = (-modAlpha + (math::ONE_BY_ROOT_THREE * modBeta)) * ARR;
            uint32_t t5 = -math::TWO_BY_ROOT_THREE * modBeta * ARR;

            T[2] = (ARR + t4 + t5)/2;
            T[1] = T[2] - t5;
            T[0] = T[1] - t4;

            break;
        }

        case 5: {
            uint32_t t5 = (-modAlpha - (math::ONE_BY_ROOT_THREE * modBeta)) * ARR;
            uint32_t t6 = (modAlpha - (math::ONE_BY_ROOT_THREE * modBeta)) * ARR;

            T[2] = (ARR + t5 + t6)/2;
            T[0] = T[2] - t5;
            T[1] = T[0] - t6;

            break;
        }

        case 6 : {
            uint32_t t6 = -math::TWO_BY_ROOT_THREE * modBeta *ARR;
            uint32_t t1 = (modAlpha + (math::ONE_BY_ROOT_THREE * modBeta)) * ARR;

            T[0] = (ARR + t6 + t1)/2;
            T[2] = T[0] - t1;
            T[1] = T[2] - t6;

            break;
        }
    }
    M.nextDutySet = true;
}

inline void FOC::updateValphaVbeta(motorAllState& M, float modAlpha,float modBeta, bool isM1) {

    controller.getMotorVoltages(m1Voltages,m2Voltages);

    const float i_alpha_filter = M.state.phase_cos_now * M.state.idFiltered - M.state.phase_sin_now * M.state.iqFiltered;
    const float i_beta_filter = M.state.phase_cos_now * M.state.iqFiltered + M.state.phase_sin_now * M.state.idFiltered;

    const float iaFiltered = i_alpha_filter;
    const float ibFiltered = -0.5 * i_alpha_filter + math::ROOT_THREE_BY_TWO * i_beta_filter;
    const float icFiltered = -0.5 * i_alpha_filter - math::ROOT_THREE_BY_TWO * i_beta_filter;

    const float mod_alpha_filter_sign = 1.0/3.0 * (2.0 * math::SIGN(iaFiltered) - math::SIGN(ibFiltered) - math::SIGN(icFiltered));
    const float mod_beta_filter_sign = math::ONE_BY_ROOT_THREE * (math::SIGN(ibFiltered) - math::SIGN(icFiltered));

    //dead-time compensation
    float modCompensationFactor = controller().dtUs * 1e-6 * controller().f_sw;
    float modAlphaCompensation = mod_alpha_filter_sign * modCompensationFactor;
    float modBetaCompensation = mod_beta_filter_sign * modCompensationFactor;

    modAlpha -= modAlphaCompensation;
    modBeta -= modBetaCompensation;

    if(isM1) {
        M.state.va = m1Voltages[0];
        M.state.vb = m1Voltages[1];
        M.state.vc = m1Voltages[2];
    }
    else {
        M.state.va = m2Voltages[0];
        M.state.vb = m2Voltages[1];
        M.state.vc = m2Voltages[2];
    }

    M.state.modAlphaMeasured = modAlpha;
    M.state.modBetaMeasured = modBeta;

    float vAlpha = (1/3)*(2*M.state.va - M.state.vb - M.state.vc);
    float vBeta = math::ONE_BY_ROOT_THREE * (M.state.vb - M.state.vc);

    float vMag = math::squareRoot(math::square(vAlpha) + math::square(vBeta));

    math::LPFast(M.state.vMagFiltered, vMag+ 0.1f * vMag * controller().lpFilterConst, controller().lpFilterConst);
    math::LPFast(M.state.modAlphaFiltered, modAlpha, controller().lpFilterConst);
    math::LPFast(M.state.modBetaFiltered, modBeta,controller().lpFilterConst);
    math::NanZero(M.state.vMagFiltered);
    math::NanZero(M.state.modAlphaFiltered);
    math::NanZero(M.state.modBetaFiltered);

    modAlpha = M.state.modAlphaFiltered;
    modBeta = M.state.modBetaFiltered;

    M.state.vAlpha = modAlpha * (2.0f/3.0f) * M.vBus;
    M.state.vBeta = modBeta * (2.0f/3.0f) * M.vBus;
}

inline void FOC::pll(motorAllState& M1, motorAllState& M2, float dt) {
    math::NanZero(M2.pll_phase);
    float delta_thetaM2 = M2.state.phase - M2.pll_phase;

    math::normAngleRad(delta_thetaM2);

    math::NanZero(M2.pll_speed);

    M2.pll_phase += (M2.pll_speed + M2.foc_params.pllKp * delta_thetaM2) * dt;

    math::normAngleRad(M2.pll_phase);

    M2.pll_speed += M2.foc_params.pllKi * delta_thetaM2 * dt; 
}

inline void FOC::updateSinCos(motorAllState& M1, motorAllState& M2) {
    math::fastSinCos(M2.state.phase,M2.state.phase_sin,M2.state.phase_cos);
}


inline void FOC::dutyControl(motorAllState& M1, motorAllState& M2) {
control::PID& M2PI = controller.getM2PID();

M2PI.setError(M2.duty_set - M2.state.duty);

auto output2 = M2PI.process();

//M2.iqSet = output2 * controller().current_Max;
M2.iqSet = 4;
    
}
inline void FOC::rpmControl(motorAllState& M1, motorAllState& M2) {
float error = 0.0f;

control::PID& M2PI = controller.getM2RPMPID();
error = M2.rpm - M2.state.filtered_rpm_measured;
M2PI.setError(error);

auto output2 = M2PI.process();

M2.duty_set = output2;

}
