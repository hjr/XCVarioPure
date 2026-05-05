/*
 * SetupMenu.h
 *
 *  Created on: Feb 4, 2018
 *      Author: iltis
 */

#pragma once

#include "setup/MenuEntry.h"

#include <vector>

class SetupMenuValFloat;
class SetupMenuSelect;
extern char small_buf[64];
extern int set_parent_parent_dirty(SetupMenuSelect* p);
extern int set_parent_dirty(SetupMenuSelect* p);


class SetupMenu : public MenuEntry
{
public:
	using SetupMenuCreator_t = void (*)(SetupMenu*);

	SetupMenu() = delete;
	explicit SetupMenu(const char* title, SetupMenuCreator_t, int cont_id=0);
	virtual ~SetupMenu();
	void enter() override;
	void display( int mode=0 ) override;
	bool isLeaf() const override { return false; }
	const char *value() const override { return buzzword; };
	int16_t freeBottomLines() const;
    int16_t firstHelpLine() const;
	// accessors
	int getHighlight() const { return highlight; }
	int incHighlight() { return ++highlight; }
	void highlightTop() { highlight = -1; }
	void highlightFirst() { highlight = 0; }
	void highlightLast() { highlight = _childs.size()-1; }
	// void setHighlight(MenuEntry*);
	void setHighlight(int chnr);
	int getNrChilds() const { return _childs.size(); }
	void setDynContent() { dyn_content = true; }
	void setDirty() { dirty = true; }
	int getContId() const { return content_id; }
	void setBuzzword(const char *bz=nullptr) { buzzword = bz; }

	// the submenu structure
	MenuEntry* addEntry(MenuEntry* item);
	void       delEntry(MenuEntry* item);
	// const MenuEntry* findMenu(const char *title) const;
	// int findMenuIdx(int contId) const;
	MenuEntry* getEntry(int n) const;

	void rot( int count ) override;
	void press() override;
	void longPress() override;

	static gpio_num_t getGearWarningIO();
	static void initGearWarning();
	static int switch_alt_source(SetupMenuSelect* p);

	static SetupMenu* createTopSetup();
	static SetupMenu* createFactorySetup();
	static SetupMenuValFloat *createQNHMenu();
	static SetupMenuValFloat *createBallastMenu();
	static SetupMenuValFloat *createVoltmeterAdjustMenu();


protected:
	SetupMenuCreator_t populateMenu;
	std::vector<MenuEntry*> _childs;
	int highlight = -1;
	bool dyn_content = false; // reentrant create routine required, call it when dirty
	bool dirty = false; // need to refresh content/child list
	int content_id; // for dynamic content creation, eg device id for device menu
	const char *buzzword = nullptr;
private:
    bool _help_dirty = false;
};

