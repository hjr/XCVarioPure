/*
 * SetupMenu.cpp
 *
 *  Created on: Feb 4, 2018
 *      Author: iltis
 */

#include "setup/SetupMenuChar.h"
#include "setup/SetupMenu.h"

#include "AdaptUGC.h"
#include "logdefnone.h"

#include <cstring>
#include <string_view>

extern AdaptUGC *MYUCG;

const std::string_view alternate_help[4] = {"Choose pos., press to edit", "Long press to exit", "Choose charachter", "Press to reposition"};

CharFilter::CharFilter( const char *chset )
{
    _flags.upper = strchr(chset, 'A') != NULL;
    _flags.lower = strchr(chset, 'a') != NULL;
    _flags.digit = strchr(chset, '0') != NULL;
    _flags.special = strchr(chset, '#') != NULL;
    _flags.signs = strchr(chset, '+') != NULL;
    _flags.whitespace = strchr(chset, ' ') != NULL;
    ESP_LOGI(FNAME, "CharFilter %s: upper=%d lower=%d digit=%d special=%d signs=%d whitespace=%d", chset,
             _flags.upper, _flags.lower, _flags.digit, _flags.special, _flags.signs, _flags.whitespace );
}
bool CharFilter::isValidChar( char c )
{
    if( _flags.upper && c >= 'A' && c <= 'Z' ) {
        return true; }
    if( _flags.lower && c >= 'a' && c <= 'z' ) {
        return true; }
    if( _flags.digit && c >= '0' && c <= '9' ) {
        return true; }
    if( _flags.special && ( (c >= '!' && c <= '/') || (c >= ':' && c <= '@') ||
                            (c >= '[' && c <= '`') || (c >= '{' && c <= '~') ) ) {
        return true; }
    if ( _flags.signs && ( c == '+' || c == '-' ) ) {
        return true; }
    if( _flags.whitespace && c == ' ' ) {
        return true; }
    ESP_LOGI(FNAME, "CharFilter: invalid char code %d", c );
    return false;
}
char CharFilter::succChar(char c)
{
    do {
        c++;
        if( c > 126 ) {
            c = 32;
        }
    } while( !isValidChar(c) );
    return c;
}
char CharFilter::predChar(char c)
{
    do {
        c--;
        if( c < 32 )
            c = 126;
    } while( !isValidChar(c) );
    return c;
}


SetupMenuChar::SetupMenuChar( const char* title, const char *chset, int mlen, e_restart_mode_t restart, int (*exit_action)(SetupMenuChar *p), const char *aval ) :
    MenuEntry(title),
    _chfilter(chset),
    _m_len(mlen),
    _exit_action(exit_action)
{
    ESP_LOGI(FNAME,"SetupMenuChar( %s )", title );
    if (aval) {
        _value.assign(aval);
    }
    bits._restart = restart;
    setRotDynamic(1.); // always setp == 1 for char editing
}

void SetupMenuChar::enter()
{
    MenuEntry::enter();
    if (!bits._locked) {
        if ( bits._is_inline && _parent->freeBottomLines() >= 2) {
            clearHelpLines(_parent->firstHelpLine());
            menuPrintLn(alternate_help[_mode*2].data(), _parent->firstHelpLine());
            menuPrintLn(alternate_help[_mode*2+1].data(), _parent->firstHelpLine()+1);
        }
    }

}

void SetupMenuChar::display(int mode)
{
    ESP_LOGI(FNAME,"display title:%s action: %x", _title.c_str(), (int)(_exit_action));
    // inline only !!
    indentHighlight(_parent->getHighlight());
    focusPosLn(_value.c_str(), _char_index, _mode);
}

void SetupMenuChar::rot(int count)
{
    if ( ! _mode ) {
        // selecting chr position
        int prev_idx = _char_index;
        _char_index += count;
        if ( _char_index < 0 ) { _char_index = 0; }
        else if ( _char_index >= _value.size() && _char_index < _m_len ) {
            _value += ' ';
        }
        else if ( _char_index >= _m_len ) {
            _char_index = _m_len-1;
        }

        if( _value.size() > _m_len ) {
            _char_index = prev_idx; // do not move
        }
        if ( prev_idx != _char_index )
        {
            focusPosLn(_value.c_str(), _char_index);
        }
    }
    else {
        // editing char at _char_index
        char co = _value[_char_index];
        char cn;
        if( count > 0 ) {
            cn = _chfilter.succChar(co);
        } else {
            cn = _chfilter.predChar(co);
        }
        ESP_LOGI(FNAME, "rot() change char from %c to %c at index %d", co, cn, _char_index);
        _value[_char_index] = cn;
        focusPosLn(_value.c_str(), _char_index, true);
    }
}

void SetupMenuChar::press()
{
    if ( _mode ) {
        // finish editing char
        _mode = false;
        indentPrintLn(_value.c_str());
        ESP_LOGI(FNAME,"press() leave edit mode at index %d", _char_index );
        _dirty = true;
    }
    else {
        // enter edit mode
        _mode = true;
        ESP_LOGI(FNAME,"press() enter edit mode at index %d", _char_index );
    }
    focusPosLn(_value.c_str(), _char_index, _mode);

    // additional help to handle the char menu
    if ( bits._is_inline && _parent->freeBottomLines() >= 2) {
        clearHelpLines(_parent->firstHelpLine());
        menuPrintLn(alternate_help[_mode*2].data(), _parent->firstHelpLine());
        menuPrintLn(alternate_help[_mode*2+1].data(), _parent->firstHelpLine()+1);
    }
}

void SetupMenuChar::longPress(){
    ESP_LOGI(FNAME, "long press() ");
    _mode = false;
    trim();

    if (_exit_action) {
        ESP_LOGI(FNAME, "calling exit action");
        (*_exit_action)(this);
    }
    if ( hasHelp() ) {
        SavedDelay(_dirty);
    }
    if (_dirty && bits._restart) {
        _restart = true;
    }
    if (bits._end_setup) {
        exit(-1);
        return;
    }
    exit();
}

const char *SetupMenuChar::value() const
{
    return _value.c_str();
}

void SetupMenuChar::reset()
{
    _value.clear();
    _char_index = 0;
    _mode = false;
}

void SetupMenuChar::trim()
{
    // left trim
    size_t first = _value.find_first_not_of(" \t\n\r");
    if (first == std::string::npos) {
        _value.clear();       // string is all whitespace
        return;
    }
    _value.erase(0, first);

    // right trim
    size_t last = _value.find_last_not_of(" \t\n\r");
    _value.erase(last + 1);
}

