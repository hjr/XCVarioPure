
#pragma once

class SetupMenu;
class SetupMenuSelect;
class SetupMenuValFloat;

int getAudioProfile();
void audio_menu_create(SetupMenu *top);
int update_range_entry_s(SetupMenuSelect *p);
int audio_setup_enable(SetupMenuSelect *p);
void free_audio_menu();