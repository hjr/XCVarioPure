/*
 * SetupMenu.cpp
 *
 *  Created on: Feb 4, 2018
 *      Author: iltis
 */

#pragma once

class SetupMenu;
class SetupAction;
class SetupMenuValFloat;

void setup_create_root(SetupMenu *top);
int do_display_test(SetupAction* p);
int factv_adj(SetupMenuValFloat *p);