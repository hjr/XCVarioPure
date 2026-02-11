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
#include "ESPAudio.h"
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
		MYUCG->drawBox(1, dheight-50, dwidth, 50);
		MYUCG->setPrintPos(1, dheight-3);
		MYUCG->setColor( COLOR_WHITE );
		MYUCG->print("Saved");
		vTaskDelay(pdMS_TO_TICKS(650));
	}
}


// Handle the basics to jump in- and out of setup menu levels
void MenuEntry::enter() {
    current = this;
    if ( ! isLeaf() ) { current_menu = static_cast<SetupMenu*>(this); }
    if (bits._locked) {
        return;
    }

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

void MenuEntry::doHighlight(int sel) const
{
	MYUCG->setColor(COLOR_WHITE);
	MYUCG->drawFrame(1, (sel+1)*LINE_HEIGHT+2, dwidth-2, LINE_HEIGHT );
}

void MenuEntry::unHighlight(int sel) const
{
	MYUCG->setColor(COLOR_BLACK);
	MYUCG->drawFrame(1, (sel+1)*LINE_HEIGHT+2, dwidth-2, LINE_HEIGHT );
}

void MenuEntry::indentHighlight(int sel)
{
	cur_indent = MYUCG->getStrWidth(_title.c_str()) + 4;
	cur_row = sel+1;
	MYUCG->setColor(COLOR_BLACK);
	MYUCG->drawFrame(1, cur_row*LINE_HEIGHT + 2, dwidth-2, LINE_HEIGHT);
	MYUCG->setColor(COLOR_WHITE);
	MYUCG->drawFrame(cur_indent, cur_row*LINE_HEIGHT + 2, dwidth-cur_indent-1, LINE_HEIGHT);
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
	int lines = 0;
    if( helptext ) {
		int nr_of_char_per_line = dwidth * 11 / (MYUCG->getStrWidth("n") * 10); // add 10% for safety
		int ll = 0;
		int last_ll = 0;
		const char *p = helptext;
		lines = 1;
		while (*p != '\0')
		{
			ll += 1;
			if ( *p++ != ' ' ) {
				continue;
			}
			if ( ll < nr_of_char_per_line ) {
				last_ll = ll; // still fits
			}
			else {
				ll = ll - last_ll; // back one word
				lines++;
			}
		}
	}
	return lines;
}

// In case inline is requested: Try to squeeze the help under
// the parents menu lines,
// returns true when inlining
bool MenuEntry::showhelp(bool inln)
{
	ESP_LOGI(FNAME,"MenuEntry showhelp() %s inline=%d", _title.c_str(), inln );
	bool ret = true; // inlining w/o help text is always possible
    if( helptext != 0 )
	{
		// option to fit the help under the menu lines
		int needed_ln = nrOfHelpLines();
        int16_t first_hln = current_menu->firstHelpLine();
		if ( inln ) {
            if (current_menu->freeBottomLines() < needed_ln ) {
                ret = false;
            }
		}
        else {
            first_hln = (dheight/LINE_HEIGHT + 1) / 2; // Lower half of the display for help, default
            ret = false;
        }
		clearHelpLines(first_hln);

		int y = (first_hln + 1) * LINE_HEIGHT;
		int line_length = dwidth;
		int w=0;
		char *buf = (char *)malloc(512);
		memset(buf, 0, 512);
		memcpy( buf, helptext, strlen(helptext));
		char *p = strtok (buf, " ");
		char *words[100];
		while (p != NULL)
		{
			words[w++] = p;
			p = strtok (NULL, " ");
		}
		// ESP_LOGI(FNAME,"showhelp number of words: %d", w);
		int x=1;

		MYUCG->setFont(ucg_font_ncenR14_hr);
		for( int p=0; p<w; p++ )
		{
			int len = MYUCG->getStrWidth( words[p] );
			// ESP_LOGI(FNAME,"showhelp pix len word #%d = %d, %s ", p, len, words[p]);
			if( x+len >= line_length ) {   // does still fit on line
				y+=LINE_HEIGHT;
				x=1;
			}
			MYUCG->setPrintPos(x, y);
			MYUCG->print( words[p] );
			x+=len+5;
		}
		free( buf );
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
	int16_t hy = ln * LINE_HEIGHT + 1;
	MYUCG->setColor(COLOR_BLACK);
	MYUCG->drawBox(0, hy, dwidth, dheight-hy);
	MYUCG->setFont(ucg_font_ncenR14_hr);
	MYUCG->setColor(COLOR_WHITE);
}
