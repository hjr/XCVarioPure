/*
 * SetupMenu.cpp
 *
 *  Created on: Feb 4, 2018
 *      Author: iltis
 */

#include "SetupMenu.h"

#include "Colors.h"
#include "AdaptUGC.h"

#include "setup/SetupNG.h"
#include "setup/SetupMenuSelect.h"
#include "logdefnone.h"

#include <algorithm>
#include <cstdint>

extern AdaptUGC *MYUCG;

// some menu helper functions
char small_buf[64];
int set_parent_parent_dirty(SetupMenuSelect* p) {
    p->getParent()->getParent()->setDirty();
    return 0;
}
int set_parent_dirty(SetupMenuSelect* p) {
    p->getParent()->setDirty();
    return 0;
}


SetupMenu::SetupMenu(const char *title, SetupMenuCreator_t menu_create, int cont_id) :
	MenuEntry(title),
	populateMenu(menu_create),
	content_id(cont_id)
{
	// ESP_LOGI(FNAME,"SetupMenu::SetupMenu( %s ) ", title );
	setRotDynamic(1.f);
}

SetupMenu::~SetupMenu() {
	ESP_LOGI(FNAME,"del SetupMenu( %s ) ", _title.c_str() );
	for (auto *c : _childs) {
		delete c;
	}
	_childs.clear();
}

void SetupMenu::enter()
{
    if (isLocked()) { return; }

	ESP_LOGI(FNAME,"enter inSet %d, mptr: %p", gflags.inSetup, populateMenu );
	if ((_childs.empty() || dyn_content) && populateMenu) {
		(populateMenu)(this); // callback needs to be designed for this !!!
		ESP_LOGI(FNAME,"created childs %d", _childs.size());
	}
	MenuEntry::enter();
}

void SetupMenu::display(int mode)
{
	if (dirty && dyn_content && populateMenu) {
		// Cope with changes in menu item presence
		ESP_LOGI(FNAME,"SetupMenu display() dirty %d", dirty );
		(populateMenu)(this);
		ESP_LOGI(FNAME,"create_childs %d", _childs.size());
	}
	ESP_LOGI(FNAME,"SetupMenu display(%s-%d)", _title.c_str(), highlight );
	if ( highlight >= (int)(_childs.size()) ) {
		highlight = _childs.size()-1;
	}
	ESP_LOGI(FNAME,"SetupMenu display %d", highlight );
	clear();
	ESP_LOGI(FNAME,"Title: %s child size:%d", _title.c_str(), _childs.size());
	MYUCG->setFont(ucg_font_ncenR14_hr, true);
	MYUCG->setFontPosBottom();
	menuPrintLn("  <", 0);
	menuPrintLn(_title.c_str(), 0, 30);
	doHighlight(highlight);
	for (int i = 0; i < _childs.size(); i++) {
		MenuEntry *child = _childs[i];
		// cope with potential external change to the e.g. nvs representation of values
		if ( dirty ) { child->refresh(); }
		
		// Menu entry
		MYUCG->setColor(COLOR_WHITE);
        const char *cv = child->value();
		if ( ! child->isLeaf()  || cv ) {
			MYUCG->setColor(COLOR_BBLUE);
		}
		menuPrintLn(child->getTitle(), i+1);

		// Optional value or detail
		// ESP_LOGI(FNAME,"Child Title: %s - %p", child->getTitle(), child->value() );
		if (cv && *cv != '\0')
		{
			// Detail seperator
			const char *sep = "> ";
			int entry_len = MYUCG->getStrWidth(child->getTitle());
			if ( child->isLeaf() ) {
				sep = ": ";
			}
			menuPrintLn(sep, i+1, 1+entry_len);

			if (child->isLeaf() ) {
				MYUCG->setColor(COLOR_WHITE);
			} else {
				MYUCG->setColor(COLOR_HEADER_LIGHT);
			}
			menuPrintLn(cv, i+1, 1 + entry_len + MYUCG->getStrWidth(sep));
			ESP_LOGI(FNAME,"Child V: %s", cv );
		}
		// ESP_LOGI(FNAME,"Child: %s y=%d",child->getTitle() ,y );
	}
	showhelp(true);
	dirty = false;
}

int16_t SetupMenu::freeBottomLines() const
{
    return dheight/25 - getNrChilds() - 1;
}

int16_t SetupMenu::firstHelpLine() const
{
    return getNrChilds() + 1;
}

// void SetupMenu::setHighlight(MenuEntry *value)
// {
// 	int found = -1;
// 	for (int i = 0; i < _childs.size(); ++i) {
// 		if (_childs[i] == value) {
// 			found = i;
// 			break;
// 		}
// 	}
// 	if ( found >= 0 ) {
// 		highlight = found;
// 	}
// }
void SetupMenu::setHighlight(int chnr)
{
	if ( chnr >= 0 && chnr < (int)_childs.size() ) {
		highlight = chnr;
	}
}

MenuEntry* SetupMenu::addEntry( MenuEntry * item )
{
	ESP_LOGI(FNAME,"add to childs");
	item->regParent(this);
	_childs.push_back( item );
	return item;
}

void SetupMenu::delEntry( MenuEntry * item ) {
	ESP_LOGI(FNAME,"SetupMenu delMenu() title %s", item->getTitle() );
	std::vector<MenuEntry *>::iterator position = std::find(_childs.begin(), _childs.end(), item );
	if (position != _childs.end()) {
		ESP_LOGI(FNAME,"found entry, now erase" );
		delete *position;
		_childs.erase(position);
	}
}

// not yet used
// const MenuEntry* SetupMenu::findMenu(const char *title) const
// {
// 	ESP_LOGI(FNAME,"MenuEntry findMenu() %s %p", title, this );
// 	if( _title == title ) {
// 		ESP_LOGI(FNAME,"Menu entry found for start %s", title );
// 		return this;
// 	}
// 	for (const MenuEntry* child : static_cast<const SetupMenu*>(this)->_childs) {
// 		const MenuEntry* m = child->findMenu(title);
// 		if( m != nullptr ) {
// 			ESP_LOGI(FNAME,"Menu entry found for %s", title);
// 			return m;
// 		}
// 	}
// 	ESP_LOGW(FNAME,"Menu entry not found for %s", title);
// 	return nullptr;
// }
//
// int SetupMenu::findMenuIdx(int contId) const
// {
// 	int idx = 0;
// 	for (const MenuEntry* child : _childs) {
// 		if( !child->isLeaf() && static_cast<const SetupMenu*>(child)->getContId() == contId ) {
// 			ESP_LOGI(FNAME,"Menu entry found for %s", title);
// 			return idx;
// 		}
// 		idx++;
// 	}

// 	return -1; // not found
// }


MenuEntry *SetupMenu::getEntry(int n) const
{
	if ( n < _childs.size() ) {
		return _childs[n];
	}
	return nullptr;
}

static int modulo(int a, int b) {
	return (a % b + b) % b;
}

void SetupMenu::rot(int count)
{
    // ESP_LOGI(FNAME,"select %d: %d/%d", count, highlight, _childs.size() );
    unHighlight(highlight);
    highlight = modulo(highlight + 1 + count, _childs.size() + 1) - 1;
    doHighlight(highlight);

    // show help like a mouse over when menu get highlighted
    MenuEntry* child = (highlight >= 0) ? _childs[highlight] : nullptr;
    if (child && child->canInline() && child->hasHelp() ) {
        child->showhelp(true);
        _help_dirty = true;
    }
    else {
        if ( _help_dirty ) {
            if ( hasHelp() ) { showhelp(true); }
            else { clearHelpLines(firstHelpLine()); }
            _help_dirty = false;
        }
    }
    ESP_LOGI(FNAME, "new highlight %d dyn %.1f", highlight, getRotDynamic());
}

void SetupMenu::press()
{
	ESP_LOGI(FNAME,"press() inSet %d highl: %d", gflags.inSetup, highlight );
	if (highlight == -1) {
        if (factory_menu.get()) { // lock factory menu
            _parent->highlightTop();
            exit();
        }
	} else {
		ESP_LOGI(FNAME,"SetupMenu to child");
		if ((highlight >= 0) && (highlight < _childs.size())) {
			_childs[highlight]->enter();
		}
	}
}

void SetupMenu::longPress()
{
	if (highlight == -1) {
        if (factory_menu.get()) { // lock factory menu
            exit(-1); // fast exit
        }
	} else {
		press();
	}
}

