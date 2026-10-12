/*
 * SetupCommon.h
 *
 *  Created on: August 23, 2020
 *      Author: iltis
 */

#pragma once

#include <string>
#include <cstdint>
#include <vector>

class XCVSyncMsg;

struct httpd_req;

// determines if data is reset to defaults on factory reset
enum e_reset_t { RESET_NONE, FACTORY_RST, USER_RST }; // -> reset threshold
// determines if data is synched from/to client. BIDIR means sync at commit from both sides
enum e_sync_t { SYNC_NONE, SYNC_FROM_MASTER, SYNC_FROM_CLIENT, SYNC_BIDIR };
enum class quantity_t : uint8_t { QUANT_NONE, QUANT_TEMPERATURE, QUANT_ALT, QUANT_HSPEED, QUANT_HSLEGACY, QUANT_VSPEED, QUANT_QNH, QUANT_MASS, QUANT_TIME };

struct t_setup_flags {
    uint8_t _reset : 2; // reset conditions: ==0: never (->use NVS to clear this one), >=FACTORY_RST: on factory reset, >=USER_RST: on user reset only
    bool _volatile : 1; // none nvs, run-time (black board) item only
    uint8_t _sync  : 2; // sync mode with client device
    uint8_t _quant : 3; // quantity
    bool _dirty    : 1; // has changed
    bool _valid    : 1; // valid data
    bool _hidden   : 1; // do not show in config backup
    bool _readonly : 1; // supress a change through setup
};

class SetupCommon {
public:
	SetupCommon(const char *k);
	virtual ~SetupCommon();

	// virtual methods to be implemented in derived class
	virtual char typeName() const = 0;
	virtual std::string getValueAsStr() const = 0;
	virtual void setValueFromStr( const char * val ) = 0;
	virtual void* getPtr() = 0;
	virtual int getSize() = 0;
	virtual bool isDefault() = 0;
	virtual bool inLimits() const = 0;

	bool init();
	bool erase();
	bool write();
	bool exists() const ;
	bool commit();
	int getReset() const { return flags._reset; }
	const char* key() const { return _key.data(); }
	bool sync();
	inline bool getDirty() const { return flags._dirty; }
	inline void setDirty() { flags._dirty = true; }
	inline bool getSync() const { return flags._sync; }
    inline bool getValid() const { return flags._valid; }
    inline void setInvalid() { flags._valid = false; }
    inline bool isHidden() const { return flags._hidden; }
    inline void setReadOnly() { flags._readonly = true; }
    inline bool isReadOnly() const { return flags._readonly; }

	static bool initSetup();  // returns false if at least one entry was blank
	static char *getID();
	static char *getDefaultID();
	static const char *getFixedID();
	static SetupCommon * getMember( const char * key );
	static bool syncEntry( int entry );
	static int numEntries();
	static void sendConfigChanges(httpd_req *req);
	static int restoreConfigChanges( int len, char *data );
	static bool getOldFloat( const char * key, float &val );
	static bool getOldInt( const char * key, int &val );
    static bool eraseEntry( const char * key );

	// housekeeping supporters
	static void setSyncProto(XCVSyncMsg *sp) { syncProto = sp; }
	static XCVSyncMsg* getSyncProto() { return syncProto; }
	static bool isMaster();
	static bool isClient();
    static bool isWired();
	static bool haveWLAN();
    static bool commitDirty();
	static bool factoryReset(int resetThershold);

    // variables
protected:
	virtual void setDefault() = 0;
	const std::string_view _key; // unique identification TAG
	t_setup_flags flags = {false, false, 0, 0, false, false, false, false};
	void (*_action)(); // action on a value change

private:
	static XCVSyncMsg *syncProto;
	static std::vector<SetupCommon *> instances;
	static char _ID[18];
	static char default_id[6];
};
