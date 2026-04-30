#include "ESP32NVS.h"

#include "logdef.h"

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <esp_partition.h>
#include <esp_err.h>
#include <nvs_flash.h>

#include <cstdlib>

static const char* nvsErr(esp_err_t e)
{
	switch (e) {
	   case ESP_OK: return "OK";
	   case ESP_ERR_NVS_NOT_FOUND: return "NOT_FOUND";
	   case ESP_ERR_NVS_NOT_ENOUGH_SPACE: return "NO_SPACE";
	   case ESP_ERR_NVS_INVALID_LENGTH: return "INVALID_LENGTH";
	   case ESP_ERR_NVS_INVALID_HANDLE: return "INVALID_HANDLE";
	   case ESP_ERR_NVS_NO_FREE_PAGES: return "NO_FREE_PAGES";
	   default: return "UNKNOWN";
	}
}

SemaphoreHandle_t nvMutex = NULL;
ESP32NVS* ESP32NVS::Instance = nullptr;

ESP32NVS::ESP32NVS(){
	nvMutex=xSemaphoreCreateMutex();
	Instance->begin(); // ensure NVS is initialized only once
}

ESP32NVS &ESP32NVS::CreateInstance()
{
	if( Instance == 0 ) {
		Instance = new ESP32NVS();
	}
	return *Instance;
}

bool ESP32NVS::begin()
{
	ESP_LOGI(FNAME,"ESP32NVS::begin()");
	esp_err_t _err = nvs_flash_init();
	if (_err == ESP_ERR_NVS_NO_FREE_PAGES) {
		const esp_partition_t* nvs_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_NVS, NULL);
		_err = (esp_partition_erase_range(nvs_partition, 0, nvs_partition->size));
	}
	if(_err != ESP_OK){
		ESP_LOGE(FNAME,"ESP32NVS partition find error err=%s(%d)", nvsErr(_err), _err);
			return false;
	}
	nvs_stats_t stats;
	esp_err_t err = nvs_get_stats(NULL, &stats);
	if(err == ESP_OK && stats.total_entries > 0){
		int percentage = (int)((100.0f * stats.used_entries) / stats.total_entries);
		ESP_LOGI(FNAME,"NVS Usage: %d of %d (%d %%)", stats.used_entries, stats.total_entries, percentage );
	}else{
		ESP_LOGW(FNAME,"NVS stats unavailable err=%d", err);
	}
	return true;
}

nvs_handle_t ESP32NVS::open(){
	// ESP_LOGI(FNAME,"ESP32NVS::open()");
	nvs_handle_t h;
	esp_err_t _err = nvs_open("SetupNG", NVS_READWRITE, &h);
	if(_err != ESP_OK){
		ESP_LOGE(FNAME,"ESP32NVS open storage error err=%s(%d)", nvsErr(_err), _err);
		return 0;
	}
	if( !h ) {
		ESP_LOGE(FNAME,"ESP32NVS open but no handle");
		return false;
	}
	return h;
}

void ESP32NVS::close( nvs_handle_t h ){
	// ESP_LOGI(FNAME,"ESP32NVS::close() %d", h );
	nvs_close(h);
}


bool ESP32NVS::commit(){
	// ESP_LOGI(FNAME,"ESP32NVS::commit()");
	bool ret=true;
	xSemaphoreTake(nvMutex,portMAX_DELAY );
	nvs_handle_t h = open();
	if( !h ) {
		xSemaphoreGive(nvMutex);
		return false;
	}
	esp_err_t _err = nvs_commit(h);
	if(_err != ESP_OK){
		ESP_LOGE(FNAME,"commit err=%s(%d)", nvsErr(_err), _err);
		ret=false;
	}else{
		ESP_LOGD(FNAME,"commit OK"); // should be a good idea so we see its happening not too frequent
	}
	close(h);
	xSemaphoreGive(nvMutex);
	return ret;
}

bool ESP32NVS::setBlob(const char * key, void* value, size_t length){
	// ESP_LOGI(FNAME,"ESP32NVS::setBlob(key:%s, addr:%p, len:%d)", key, value, length );
	bool ret=true;
	xSemaphoreTake(nvMutex,portMAX_DELAY );
	nvs_handle_t h = open();
	if( !h ) {
		xSemaphoreGive(nvMutex);
		return false;
	}
	esp_err_t _err = nvs_set_blob(h, (char*)key, value, length);
	if(_err != ESP_OK) {
		ESP_LOGE(FNAME,"set blob err=%s(%d) key=%s len=%d", nvsErr(_err), _err, key, (int)(length));
		ret=false;
	}
	// ESP_LOGI(FNAME,"set blob OK");
	close(h);
	xSemaphoreGive(nvMutex);
	return ret;
}

bool ESP32NVS::eraseAll(){
	bool ret=true;
	xSemaphoreTake(nvMutex,portMAX_DELAY );
	nvs_handle_t h = open();
	if( !h ) {
		xSemaphoreGive(nvMutex);
		return false;
	}
	esp_err_t _err = nvs_erase_all(h);
	if(_err != ESP_OK){
		ret = false;
	}
	close(h);
	xSemaphoreGive(nvMutex);
	return ret;
}

bool ESP32NVS::erase(const char * key){
	xSemaphoreTake(nvMutex,portMAX_DELAY );
	bool ret=true;
	nvs_handle_t h = open();
	if( !h ) {
		xSemaphoreGive(nvMutex);
		return false;
	}
	esp_err_t _err =  nvs_erase_key(h, key);
	if(_err != ESP_OK){
		ESP_LOGE(FNAME,"erase err=%s(%d) key=%s", nvsErr(_err), _err, key);
		ret = false;
	}
	close(h);
	xSemaphoreGive(nvMutex);
	return ret;
}

bool ESP32NVS::getBlob(const char * key, void *blob, size_t *length){
	xSemaphoreTake(nvMutex,portMAX_DELAY );
	bool ret=true;
	nvs_handle_t h = open();
	if( !h ) {
		xSemaphoreGive(nvMutex);
		return false;
	}
	esp_err_t err = nvs_get_blob(h, key, blob, length);
	if( err != ESP_OK ){
		ESP_LOGE(FNAME,"get blob err=%s(%d) key=%s len=%d", nvsErr(err), err, key, (int)(length));
		ret = false;
	}
	close(h);
	xSemaphoreGive(nvMutex);
	return ret;
}
