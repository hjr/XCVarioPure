/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include "setup/SetupMenuDisplay.h"

class ShowFlightInfo: public SetupMenuDisplay
{
public:

	ShowFlightInfo( const char* title ) : SetupMenuDisplay( title, nullptr ) {}
	virtual ~ShowFlightInfo() {}

	void display(int mode=0) override;
	void rot( int count ) override { display(1); }
};
