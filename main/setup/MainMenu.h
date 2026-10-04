/*
 * MainMenu.h
 *
 *  Created on: Feb 4, 2018
 *      Author: iltis
 */

#pragma once

class SetupMenu;
class SetupMenuSelect;
class SetupMenuValFloat;

void setup_create_root(SetupMenu *top);
int do_display_test(SetupMenuSelect* p);
int factv_adj(SetupMenuValFloat *p);