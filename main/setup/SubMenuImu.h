/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

class SetupMenu;
class SetupMenuSelect;

void screens_menu_create_gload(SetupMenu *top);
void screens_menu_create_extreme_records(SetupMenu *top);
void system_menu_create_hardware_imu(SetupMenu *top);
void system_menu_create_hardware_ahrs_parameter(SetupMenu *top);
extern int imu_calib(SetupMenuSelect* p);
void free_imu_menu();
