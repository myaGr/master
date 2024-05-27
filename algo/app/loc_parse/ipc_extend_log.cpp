#include "ipc_extend_log.h"
#include <iostream>

using namespace std;

/**
 * @brief construc
 */
IpcExtendLog::IpcExtendLog() {
  setFilePath(IPC_LOG_PVT_NMEA, "nmea_pvt_realtime.txt");
  setFilePath(IPC_LOG_PPP_NMEA, "nmea_ppp_realtime.txt");
}

/**
 * @brief destruct
 */
IpcExtendLog::~IpcExtendLog()
{
  for (log_ExtendTypeEnum i = 0; i < IPC_LOG_MAX; i++) {
    releaseFilePtr(i);
  }
}

/**
 * @brief set file path by type
 * @param[in] type
 * @param[in] path
 */
void IpcExtendLog::setFilePath(log_ExtendTypeEnum type, const std::string& path) {
  if (type >= IPC_LOG_MAX) {
    cout << "Error: IpcExtendLog::setFilePath type error" << endl;
    return;
  }
  auto it = _map_file_path.find(type);
  if ((it != _map_file_path.end()) && (path != it->second)) {
    releaseFilePtr(type);
  }
  if((it != _map_file_path.end()) && (path == it->second)){
    return;
  }
  _map_file_path[type] = path;
  _map_file_ptr[type] = fopen(path.c_str(), "wb");
}

/**
 * @brief Release file pointer
 * @param type
 */
void IpcExtendLog::releaseFilePtr(log_ExtendTypeEnum type) {
  auto it = _map_file_ptr.find(type);
  if (it != _map_file_ptr.end()) {
    fclose(it->second);
    it->second = NULL;
  }
}

/**
 * @brief fprint to file
 * @param[in] type log extend type
 * @param[in] buff content
 */
void IpcExtendLog::logPrint(log_ExtendTypeEnum type, const char* buff) {
  auto it = _map_file_ptr.find(type);
  if (it != _map_file_ptr.end()) {
    fprintf(it->second, "%s", buff);
  }
}

