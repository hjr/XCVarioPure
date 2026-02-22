#include "Cipher.h"
#include "cipherkey.h"

#include "setup/SetupNG.h"
#include "logdefnone.h"


#include <cstdlib>

#define ALPHABET_LEN 36

static int char_to_index(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'Z') return 10 + (c - 'A');
    return -1;
}

static char index_to_char(int i)
{
    i = i % ALPHABET_LEN;
    return i <= 9 ? '0' + i : 'A' + (i - 10);
}

static std::string v_decrypt(const std::string& ciphertext, const std::string& key)
{
    std::string out;
    out.reserve(ciphertext.size());

    for (size_t i = 0; i < ciphertext.size(); ++i) {
        int c = char_to_index(ciphertext[i]);
        int k = char_to_index(key[i % key.size()]);
        out.push_back(index_to_char(c - abs(k - 10) + ALPHABET_LEN));
        // ESP_LOGI(FNAME, "v_decrypt() key: %c dist: %d -> %c", key[i % key.size()], abs(k - 10), out.back() );
    }
    return out;
}

std::string v_encrypt(const std::string& plaintext, const std::string& key)
{
    std::string out;
    out.reserve(plaintext.size());

    for (size_t i = 0; i < plaintext.size(); ++i) {
        int p = char_to_index(plaintext[i]);
        int k = char_to_index(key[i % key.size()]);
        out.push_back(index_to_char(p + abs(k - 10)));
        // ESP_LOGI(FNAME, "v_encrypt() key: %c dist: %d -> %c", key[i % key.size()], abs(k - 10), out.back() );
    }
    return out;
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
        std::string encid = v_encrypt(_id, CIPHER_KEY);
        ESP_LOGI(FNAME, "initTest Encrypted ID %s", encid.c_str());
        ahrs_licence.set(encid.c_str());
    }
    SetupCommon::commitDirty();
}

bool Cipher::checkKeyAHRS() {
    std::string decid = v_decrypt(ahrs_licence.get().id, CIPHER_KEY);
    ESP_LOGI(FNAME, "checkKeyAHRS() ID/KEY/DECID %s %s %s returns %d", _id.c_str(), ahrs_licence.get().id, decid.c_str(), (_id == decid));
    return (_id == decid);
}
