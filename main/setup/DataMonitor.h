/*
 * Datamonitor.h
 *
 *  Created on: Nov, 2021
 *      Author: iltis
 */
#pragma once

#include "setup/MenuEntry.h"
#include "comm/InterfaceCtrl.h"
#include "comm/Mutex.h"

class SetupAction;

typedef enum { DIR_RX, DIR_TX } e_dir_t;

class DataMonitor: public MenuEntry
{
public:
	DataMonitor();
    ~DataMonitor() = default;

    // main API
	void start(SetupAction *p, ItfTarget ch);
	void monitorString(e_dir_t dir, bool binary, const char *s, int len );
    // polymorphic API
	void display(int) override {}
	const char* value() const override { return ""; }
	void press() override;
	void rot( int count ) override {};
	void longPress() override;

private:
	const int LINE_WIDTH;
	const int SCROLL_BOTTOM;
	int maxChar( const char *s, int pos, int len);
	void header(int len=0, e_dir_t dir=DIR_RX);
	void printString(e_dir_t dir, const char *s, int len );
	void scroll(int scroll);
    // attributes
    mutable SemaphoreMutex _mutex;
	int map_pos;
	bool paused = true;
	ItfTarget channel = {};
	int rx_total = 0;
	int tx_total = 0;
	bool bin_mode = false;
};

extern DataMonitor *DM;
