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

Outlet::Outlet()
{
    _onOff = false;
    _timerOnOff = false;

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

