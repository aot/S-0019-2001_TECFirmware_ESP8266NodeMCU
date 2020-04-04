//
// Created by tom on 14/11/16.
//

#ifndef _SLIMRUNNINGAVERAGE_H
#define _SLIMRUNNINGAVERAGE_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#elif __arm__
#include <stm32f3xx.h>
#else
#include <stdint.h>
#endif

/* Note - currently to be used with INTEGER TYPES ONLY */

template <typename T>
class RunningAverage
{
public:
    RunningAverage();
    void init(uint16_t windowDuration, uint16_t samplePeriod);
    void init(uint16_t noSamples);
    T addSample(T sample);

private:
    void initHelper();

    int64_t m_runningSum;
    uint16_t m_windowLength;
    T* m_window;
    uint16_t m_windowPtr;
    bool m_windowFull;
};

template <>
class RunningAverage<float>
{
public:
    RunningAverage();
    void init(uint16_t windowDuration, uint16_t samplePeriod);
    void init(uint16_t noSamples);
    float addSample(float sample);

private:
    void initHelper();

    float m_runningSum;
    uint16_t m_windowLength;
    float* m_window;
    uint16_t m_windowPtr;
    bool m_windowFull;
};


template <typename T>
RunningAverage<T>::RunningAverage(){
    m_runningSum = 0;
    m_windowLength = 0;
    m_windowPtr = 0;
    m_windowFull = false;
    m_window = 0;
}

template <typename T>
void RunningAverage<T>::init(uint16_t windowDuration, uint16_t samplePeriod)
{
    m_windowLength = windowDuration / samplePeriod;
    initHelper();
}

template <typename T>
void RunningAverage<T>::init(uint16_t noSamples){
    m_windowLength = noSamples;
    initHelper();
}

template <typename T>
void RunningAverage<T>::initHelper(){
    m_window = new T[m_windowLength];
    for (int i = 0; i < m_windowLength; i++) {
        m_window[i] = 0;
    }
}

template <typename T>
inline T RunningAverage<T>::addSample(T sample)
{
    if(m_windowPtr == m_windowLength){
        m_windowPtr = 0;
        m_windowFull = true;
    }

    T returnVal;
    if(m_windowFull){
        m_runningSum -= m_window[m_windowPtr];
        m_window[m_windowPtr] = sample;
        m_runningSum += m_window[m_windowPtr];
        returnVal = m_runningSum / m_windowLength;

    }else{
        m_window[m_windowPtr] = sample;
        m_runningSum += m_window[m_windowPtr];
        if (m_windowPtr == 0) {
        }
        returnVal = m_runningSum / (m_windowPtr+1);


    }
    m_windowPtr++;
    return returnVal;
}


RunningAverage<float>::RunningAverage(){
    m_runningSum = 0;
    m_windowLength = 0;
    m_windowPtr = 0;
    m_windowFull = false;
    m_window = 0;
}

void RunningAverage<float>::init(uint16_t windowDuration, uint16_t samplePeriod)
{
    m_windowLength = windowDuration / samplePeriod;
    initHelper();
}

void RunningAverage<float>::init(uint16_t noSamples){
    m_windowLength = noSamples;
    initHelper();
}

void RunningAverage<float>::initHelper(){
    m_window = new float[m_windowLength];
    for (int i = 0; i < m_windowLength; i++) {
        m_window[i] = 0;
    }
}

inline float RunningAverage<float>::addSample(float sample)
{
    if(m_windowPtr == m_windowLength){
        m_windowPtr = 0;
        m_windowFull = true;
    }

    float returnVal;
    if(m_windowFull){
        m_runningSum -= m_window[m_windowPtr];
        m_window[m_windowPtr] = sample;
        m_runningSum += m_window[m_windowPtr];
        returnVal = m_runningSum / m_windowLength;

    }else{
        m_window[m_windowPtr] = sample;
        m_runningSum += m_window[m_windowPtr];
        if (m_windowPtr == 0) {
        }
        returnVal = m_runningSum / (m_windowPtr+1);


    }
    m_windowPtr++;
    return returnVal;
}



#endif //_SLIMRUNNINGAVERAGE_H
