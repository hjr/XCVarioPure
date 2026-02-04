/*
 * SetupEntry.h
 *
 *  Created on: Feb 4, 2018
 *      Author: iltis
 */

#pragma once

#include "ESPRotary.h"
#include "setup/SetupMenuCommon.h"

#include <string>
#include <cstdint>

struct bitfield {
    e_restart_mode_t _restart :2;
    bool _ext_handler         :1; // ??
    bool _end_setup           :1; // roll setup on exit
    bool _end_menu            :1; // just terminate the containing menu (two levels up)
    bool _locked              :1; // cannot enter
    bool _is_inline           :1; // inline menu
	bool _never_inline        :1; // never inline menu
    bool _live_update         :1; // update value on rotation
    uint8_t _precision        :4;
};

// restart bit field
constexpr uint8_t RESTART_SCHEDULED = 0x80;
constexpr uint8_t RESTART_BT_CHANGE = 0x01; // because of BT change
constexpr uint8_t RESTART_WIFI_CHANGE = 0x02; // because of WIFI change


class PressureSensor;
class SetupMenu;
class SetupRoot;

constexpr const int DEFAULT_HELP_Y_POS = 180;

class MenuEntry : public RotaryObserver
{
	friend class SetupRoot;

public:
	MenuEntry(const char *t) : RotaryObserver() { _title.assign(t); }
	virtual ~MenuEntry() = default;

	static void grabDisplaySize();

	// from Observer
	void release() override {} // not used
	void escape() override;

	// own API
	virtual void enter();
	virtual void exit(int ups=1);
	virtual void display( int mode=0 ) = 0;
	virtual void refresh() {} // reread temp values coping with side efects on refreshing the display
	virtual bool isLeaf() const { return true; }
	virtual const char* value() const = 0; // content as string
	virtual int freeBottomLines() const;

	void lock() { bits._locked = true; }
	void unlock() { bits._locked = false; }
	bool isLocked() const { return bits._locked; }
	void setNeverInline() { bits._never_inline = true; }

	// helper
	const char* getTitle() const { return _title.c_str(); }
	SetupMenu* getParent() const { return _parent; }
	void hookToParent(SetupMenu* p) { _parent = p; }
	void regParent(SetupMenu* p);
	bool isRoot() const;
	bool isSelected() const { return selected == this; }
	MenuEntry *getSelected() const { return selected; }
	void setHelp( const char *txt, int y=DEFAULT_HELP_Y_POS ) { helptext = (char*)txt; };
	void doHighlight(int sel) const;
    void unHighlight(int sel) const;
    void indentHighlight(int sel);
	void indentPrintLn(const char *str) const;
	void focusPosLn(const char *str, int16_t pos, bool mode=false) const;
	bool canInline() const;
	int nrOfHelpLines() const;
	bool showhelp(bool inln=false);
	static void clear();
	void clearHelpLines() const;
	const MenuEntry* findMenu(const char *title) const;
	void menuPrintLn(const char* str, int sel, int x=1) const;
	void menuPrintChar(char chr, int ln, int x) const;
	// void uprint( int x, int y, const char* str );
	void SavedDelay(bool showit=true);
	void scheduleReboot(uint8_t r = 0x80) { _restart |= r; }
	void unscheduleReboot(uint8_t r) { _restart &= ~r; }
	static void reBoot(int s=1);

public:
	static int16_t dwidth;
	static int16_t dheight;

protected:
	static uint8_t _restart; // restart bit field. 0x80 = scheduled, 0x01 = because of BT change, 0x02 = because of WIFI change

protected:
	SetupMenu  *_parent = nullptr;
	std::string _title;
	const char *helptext = nullptr;
	bitfield bits = {};
	static int16_t cur_indent;
	static int16_t cur_row;

private:
	static MenuEntry *selected;
};
