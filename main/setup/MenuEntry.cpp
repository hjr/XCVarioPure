/*
 * SetupMenu.cpp
 *
 *  Created on: Feb 4, 2018
 *      Author: iltis
 */
#include "setup/MenuEntry.h"

#include "setup/SetupMenu.h"
#include "setup/SetupCommon.h"
#include "Colors.h"
#include "driver/audio/ESPAudio.h"
#include "AdaptUGC.h"
#include "logdefnone.h"

#include <cstdint>

extern AdaptUGC *MYUCG;

MenuEntry* MenuEntry::current = nullptr;
SetupMenu* MenuEntry::current_menu = nullptr;
uint8_t MenuEntry::_restart = 0;
int16_t MenuEntry::cur_indent;
int16_t MenuEntry::cur_row;
int16_t MenuEntry::dwidth;
int16_t MenuEntry::dheight;

MenuEntry::MenuEntry(const char *t) :
    RotaryObserver(),
    _title(t)
{
    _help_line_start[0] = 0;
}

void MenuEntry::grabDisplaySize()
{
	dwidth = MYUCG->getDisplayWidth();
	dheight = MYUCG->getDisplayHeight();
}

void MenuEntry::escape()
{
	exit(-1); // exit all levels
}


// const MenuEntry* MenuEntry::findMenu(const char *title) const
// {
// 	ESP_LOGI(FNAME,"MenuEntry findMenu() %s %p", title, this );
// 	if( _title == title ) {
// 		ESP_LOGI(FNAME,"Menu entry found for start %s", title );
// 		return this;
// 	}
// 	return nullptr;
// }

// ln is the line enumeration, starting with 0
void MenuEntry::menuPrintLn(const char* str, int ln, int x) const {
    MYUCG->setPrintPos(x,(ln+1)*LINE_HEIGHT);
    MYUCG->print(str);
}

void MenuEntry::menuClearLn(int ln) const {
    MYUCG->setColor(COLOR_BLACK);
    MYUCG->drawBox(1, ln * LINE_HEIGHT + 1, dwidth - 2, LINE_HEIGHT);
    MYUCG->setColor(COLOR_WHITE);
}

void MenuEntry::menuPrintChar(char chr, int ln, int x) const {
    if ( chr == ' ' ) {
        MYUCG->drawDisc(x+2, (ln+1)*LINE_HEIGHT - 7, 2, UCG_DRAW_ALL);
    }
    else {
        MYUCG->setPrintPos(x,(ln+1)*LINE_HEIGHT);
        MYUCG->print(chr);
    }
}

void MenuEntry::reBoot(int s) {
    delete AUDIO;
    clear();
    MYUCG->setPrintPos(10, 50);
    MYUCG->print("...rebooting now");
    SetupCommon::commitDirty();
    vTaskDelay(s * 1000 / portTICK_PERIOD_MS);
    esp_restart();
}

// void MenuEntry::uprint( int x, int y, const char* str ) {
// 	MYUCG->setPrintPos(x,y);
// 	MYUCG->print( str );
// }

void MenuEntry::SavedDelay(bool showit)
{
	if ( showit ) {
		MYUCG->setColor( COLOR_BLACK );
		MYUCG->drawBox(1, dheight-LINE_HEIGHT, dwidth, LINE_HEIGHT);
		MYUCG->setPrintPos(1, dheight-3);
		MYUCG->setColor( COLOR_WHITE );
		MYUCG->print("Saved");
		vTaskDelay(pdMS_TO_TICKS(650));
	}
}


// Handle the basics to jump in- and out of setup menu levels
void MenuEntry::enter() {
    if (isLocked()) {
        return;
    }
    current = this;
    if ( ! isLeaf() ) { current_menu = static_cast<SetupMenu*>(this); }

    // enter a level of setup menu
    attach();  // set rotary focus
    if (isLeaf()) {
        if (canInline()) {
            showhelp(true);
            bits._is_inline = true;
        } else {
            clear();
            showhelp();
        }
    }
    display();
    ESP_LOGI(FNAME,"MenuEntry enter %p %p", this, current_menu );
}

void MenuEntry::exit(int ups) {
    // exit a level of setup menu
    if (ups != 0 && _parent != 0) {
        detach();
        current = _parent;
        current_menu = _parent;
        current->exit(--ups);
        return;
    }
    display();
    ESP_LOGI(FNAME, "MenuEntry enter %p %p", this, current_menu);
}

void MenuEntry::regParent(SetupMenu *p)
{
	if ( _parent == nullptr ) {
		_parent = p;
	}
}

bool MenuEntry::isFirstLevel() const
{
    // parent is root
    return  _parent->_parent == nullptr;
}

void MenuEntry::setHelp( const char *txt )
{
    helptext = (char*)txt;

    if (!helptext) {
        return;
    }
    MYUCG->setFont(ucg_font_ncenR14_hr);
    int16_t nlidx = MYUCG->IdxToTextWidth(helptext, dwidth);
    if (nlidx == 0) {
        return; // fits in one line, no need to split
    }

    const char* p = helptext + nlidx;
    int16_t nlidx_prev = 0;
    int lines = 0;
    while (lines < MAX_HELP_LINES) {
        // go backwards, find the last space before end_idx
        while ( (p > helptext) && (*p != ' ') ) {
            p--;
            nlidx--;
        }
        _help_line_start[lines] = nlidx + nlidx_prev;
        nlidx_prev = _help_line_start[lines];
        lines++;
        // get the potential next line end
        nlidx = MYUCG->IdxToTextWidth(p, dwidth);
        if ( nlidx == 0 ) {
            break;
        }
        p += nlidx;
    }
    
    if ( lines < MAX_HELP_LINES ) {
        _help_line_start[lines] = 0; // mark end of help text
    }
    ESP_LOGI(FNAME,"MenuEntry setHelp() lines %d, %d,%d,%d", lines, _help_line_start[0], _help_line_start[1], _help_line_start[2] );
}

void MenuEntry::doHighlight(int sel) const
{
	MYUCG->setColor(COLOR_WHITE);
	MYUCG->drawFrame(1, (sel+1)*LINE_HEIGHT+2, dwidth-2, LINE_HEIGHT-1 );
}

void MenuEntry::unHighlight(int sel) const
{
	MYUCG->setColor(COLOR_BLACK);
	MYUCG->drawFrame(1, (sel+1)*LINE_HEIGHT+2, dwidth-2, LINE_HEIGHT-1 );
}

void MenuEntry::indentHighlight(int sel)
{
	cur_indent = MYUCG->getStrWidth(_title.c_str()) + 4;
	cur_row = sel+1;
	MYUCG->setColor(COLOR_BLACK);
	MYUCG->drawFrame(1, cur_row*LINE_HEIGHT + 2, dwidth-2, LINE_HEIGHT-1);
	MYUCG->setColor(COLOR_WHITE);
	MYUCG->drawFrame(cur_indent, cur_row*LINE_HEIGHT + 2, dwidth-cur_indent-1, LINE_HEIGHT-1);
}

void MenuEntry::indentPrintLn(const char *str) const
{
	MYUCG->setColor(COLOR_BLACK);
	MYUCG->drawBox(cur_indent+3, cur_row*LINE_HEIGHT + 6, dwidth-3, 19);
	MYUCG->setColor(COLOR_WHITE);
	menuPrintLn(str, cur_row, cur_indent+7);
}

void MenuEntry::focusPosLn(const char *str, int16_t pos, bool mode) const
{
    indentPrintLn(str);
    if ( mode ) {
        MYUCG->setColor(COLOR_RED);
    } else {
        MYUCG->setColor(COLOR_YELLOW);
    }
    int16_t w = 0;
    if ( pos > 0) {
        char pbuf[100];
        strncpy(pbuf, str, pos);
        pbuf[pos] = '\0';
        w = MYUCG->getStrWidth(pbuf);
    }
    menuPrintChar(str[pos], cur_row, cur_indent+7 + w);
}

// simple heuristic based on "n" sized chars,
// how many lines the help text will allocate
bool MenuEntry::canInline() const
{
    return current_menu->freeBottomLines() >= nrOfHelpLines() && !isFirstLevel() && !bits._never_inline;
}

int MenuEntry::nrOfHelpLines() const
{
    int lines = 1;
    for (int i=0; i<MAX_HELP_LINES && _help_line_start[i] != 0; i++) {
        lines++;
    }
    return lines;
}

// In case inline is requested: Try to squeeze the help under
// the parents menu lines,
// returns true when inlining
bool MenuEntry::showhelp(bool inln)
{
    ESP_LOGI(FNAME, "MenuEntry showhelp() %s inline=%d", _title.c_str(), inln);
    bool ret = true;  // inlining w/o help text is always possible
    if (helptext) {
        // option to fit the help under the menu lines
        int needed_ln = nrOfHelpLines();
        int16_t first_hln = current_menu->firstHelpLine();
        if (inln) {
            if (current_menu->freeBottomLines() < needed_ln) {
                ret = false;
            }
        } else {
            first_hln = (dheight / LINE_HEIGHT + 1) / 2;  // Lower half of the display for help, default
            ret = false;
        }

        clearHelpLines(first_hln);
        MYUCG->setColor(COLOR_BBLUE);
        MYUCG->drawLine(30, first_hln * LINE_HEIGHT + 2, dwidth - 30, first_hln * LINE_HEIGHT + 2);  // separator
        MYUCG->setColor(COLOR_WHITE);

        MYUCG->setFont(ucg_font_ncenR14_hr);
        if ( _help_line_start[0] == 0 ) {
            // just one line
            menuPrintLn(helptext, first_hln);
            ESP_LOGI(FNAME,"MenuEntry showhelp() line 0: '%s'",  helptext );
        }
        else {
            // multiple lines
            int16_t start = 0;
            int16_t i;
            for (i=0; i < MAX_HELP_LINES && _help_line_start[i] != 0; i++) {
                std::string lbuf(helptext + start, _help_line_start[i] - start);
                ESP_LOGI(FNAME,"MenuEntry showhelp() line %d: '%s'", i, lbuf.data() );
                menuPrintLn(lbuf.data(), first_hln + i);
                start = _help_line_start[i];
            }
            menuPrintLn(helptext + start, first_hln + i);
        }
    }
    return ret;
}

void MenuEntry::clear() {
    MYUCG->setColor(COLOR_BLACK);
    MYUCG->drawBox(0, 0, dwidth, dheight);
    MYUCG->setFont(ucg_font_ncenR14_hr);
    MYUCG->setPrintPos(1, 30);
    MYUCG->setColor(COLOR_WHITE);
}

void MenuEntry::clearHelpLines(int16_t ln) const
{
	ESP_LOGI(FNAME,"clearHelpLines %s", _title.c_str());
	int16_t hy = ln * LINE_HEIGHT + 2;
	MYUCG->setColor(COLOR_BLACK);
	MYUCG->drawBox(0, hy, dwidth, dheight-hy-1);
	MYUCG->setFont(ucg_font_ncenR14_hr);
	MYUCG->setColor(COLOR_WHITE);
}
