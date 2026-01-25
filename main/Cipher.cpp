#include "Cipher.h"
#include "cipherkey.h"

#include "setup/SetupNG.h"
#include "logdefnone.h"

#include <esp_mac.h>
#include <miniz.h>

#include <cstdlib>
#include <locale>


static int CalculateDistance(char a, char b) {
	// ESP_LOGI(FNAME,"CalculateDistance a:%c b:%c  ret:%d", a, b, abs(a - b));
	return abs(a - b);
}

static char ShiftChar(char a, int keyDistance) {
	// ESP_LOGI(FNAME,"ShiftChar %c, %d", a, keyDistance );
	for (int i = 0; i < keyDistance; i++) {
		a += 1; // shift our char once
		if( a < 65 && a > 57 )
			a+=7;  // skip ;:<...
		if (a > 90) // Check if we've gone past z
			a = 48; // Set our char back to a
	}
	// ESP_LOGI(FNAME,"ShiftChar ret: %c ", a );
	return a;
}

static char ShiftCharBack(char a, int keyDistance) {
	// ESP_LOGI(FNAME,"ShiftCharBack %c, %d", a, keyDistance );
	for (int i = 0; i < keyDistance; i++) {
		a -= 1; // shift our char once
		if( a < 65 && a > 57 )
			a-=7;  // skip ;:<...
		if (a < 48) // Check if we've gone before a
			a = 90; // Set our char back to z
	}
	// ESP_LOGI(FNAME,"ShiftCharBack ret: %c ", a );
	return a;
}

static void FormatKey(std::string& key, std::string plaintext) {
	// append key length to plaintext's size
	// note: key may end up larger than plaintext here, but will be trimmed in the next if statement
	if (key.length() < plaintext.length()) {
		while (key.length() < plaintext.length())
			key.append(key); // append key onto itself until >= plaintext.length()
	}
	// shrink key length to plaintext's size
	if (key.length() > plaintext.length())
		key.erase(key.length(), std::string::npos);
}

static void FormatEncrypted(std::string& encrypted) {
	std::locale loc;
	for (int i = 0; i < encrypted.length(); i++)
		encrypted.at(i) = std::toupper(encrypted.at(i), loc); // Make all character uppercase
}


static std::string Encrypt(std::string key, std::string plaintext) {
	// encrypted text
	std::string encrypted;
	// Format key
	FormatKey(key, plaintext);
	// Begin encryption
	for (int i = 0; i < plaintext.length(); i++) {
		int keyDistance = CalculateDistance(key.at(i), 'A'); // Key's current char distance from 'a'
		char EncryptedChar = ShiftChar(plaintext.at(i), keyDistance); // Encrypt the plaintext char shifting the plaintext char by the keyDistance
		encrypted.push_back(EncryptedChar); // Add char to encrypted string
	}
	FormatEncrypted(encrypted);
	return encrypted; // Return the encrypted string
}

static std::string Decrypt(std::string key, std::string plaintext) {
	// std::string to hold our encrypted text
	std::string encrypted;
	// Format key
	FormatKey(key, plaintext);
	// Begin encryption
	for (int i = 0; i < plaintext.length(); i++) {
		int keyDistance = CalculateDistance(key.at(i), 'A'); // Key's current char distance from 'a'
		char EncryptedChar = ShiftCharBack(plaintext.at(i), keyDistance); // Encrypt the plaintext char shifting the plaintext char by the keyDistance
		encrypted.push_back(EncryptedChar); // Add char to encrypted string
	}
	FormatEncrypted(encrypted);
	return encrypted; // Return the encrypted string
}


Cipher::Cipher()
{
	_id.assign(SetupCommon::getDefaultID()); // four diggits ID
}

void Cipher::initTest() {
    // check on old nvs variable first
    const char *ovar[4] = {"1_2", "2", "3", "4"};
    std::string oldlbl = "AHRS_LIC_" + std::string(ovar[0]);
    int dig;
    if (SetupCommon::getOldInt(oldlbl.c_str(), dig)) {
        ESP_LOGI(FNAME, "initTest Old var %s found, migrating..", ovar[0]);
        std::string oldid(1, '0' + dig);
        for (int i = 1; i < 4; i++) {
            oldlbl = "AHRS_LIC_" + std::string(ovar[i]);
            if (SetupCommon::getOldInt(oldlbl.c_str(), dig)) {
                oldid.push_back('0' + dig);
                ESP_LOGI(FNAME, "initTest Old var %s value %d", ovar[i], dig);
            }
        }
        ESP_LOGI(FNAME, "initTest Old ID %s", oldid.c_str());
        ahrs_licence.set(oldid.c_str());
    } else {
        std::string encid = Encrypt(CIPHER_KEY, _id);
        ESP_LOGI(FNAME, "initTest Encrypted ID %s", encid.c_str());
        ahrs_licence.set(encid.c_str());
    }
    SetupCommon::commitDirty();
}

bool Cipher::checkKeyAHRS()
{
	std::string decid = Decrypt(CIPHER_KEY, ahrs_licence.get().id );
	ESP_LOGI(FNAME,"checkKeyAHRS() ID/KEY/DECID %s %s %s returns %d", _id.c_str(), ahrs_licence.get().id, decid.c_str(), (_id == decid) );
	return (_id == decid);
}
