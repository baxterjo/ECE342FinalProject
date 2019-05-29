/**************************************************************************/
/*!
    File Name:     outlet.h
    Author:        Jordan Baxter

    Group: Blue(1) "Reverse Biased"
    Group Members: Jordan Baxter, Suyang Liu, Trenton Kilgore

    Description: This file contains the function prototypes for the outlet class.
*/
/**************************************************************************/

#ifndef _OUTLET_H_
#define _OUTLET_H_

#include <string.h>
#include <stdint.h>
#include <Arduino.h>


class Outlet
{
    protected:
        char* _name;
        bool _onOff;
        bool _timerOnOff;
        int _timerSeconds;
        float _avgCurrent;
        unsigned long _timerStart;
        int _onOffPin;
        int _currentPin;


    public:
        Outlet(char* name, int onOffPin, int currentPin);
        void switchOnOff();
        bool getOnOff();
        void timerCancel();
        bool getTimerOnOff();
        void setTimer(uint16_t seconds);
        uint16_t getTimeRemaining();
        uint16_t getCurrent();
        void timerRun();
};

#endif /*_OUTLET_H_*/