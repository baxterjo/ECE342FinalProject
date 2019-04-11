/**************************************************************************/
/*!
    File Name:     outlet.cpp
    Author:        Jordan Baxter

    Group: Blue(1) "Reverse Biased"
    Group Members: Jordan Baxter, Suyang Liu, Trenton Kilgore

    Description: This file contains the function declarations for the outlet class.
*/
/**************************************************************************/

#include "outlet.h"

/******************************************************************************/
/*!
    Description: Constructor
*/
/******************************************************************************/

Outlet::Outlet(int onOffPin, int currentPin)
{
    _onOff = false;
    _timerOnOff = false;
    _onOffPin = onOffPin;
    _currentPin = currrentPin;

}

void Outlet::switchOnOff(){
    _onOff = !_onOff;
}

bool Outlet::getOnOff(){
    return _onOff;
}

void Outlet::timerOnOff(){
    _timerOnOff = !_timerOnOff;
}

bool Outlet::getTimerOnOff(){
    return _timerOnOff;
}

void Outlet::setTimer(int seconds){
    
    _timerStart = millis();

    _timerSeconds = seconds;

    _timerOnOff = true;
}

int Outlet:::getTimer(){
    unsigned long timeElapsed = millis() - _timerStart;
    int timeRemaining = _timerSeconds - (timeElapsed%1000) + 1;
    return timeRemaining;
}

float getCurrent(){
    float current = analogread(_currentPin);
}



