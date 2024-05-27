#ifndef __IPC_EXTEND_LOG_H__
#define __IPC_PARSE_LOG_H__

#include <string>
#include <map>

using namespace std;

typedef enum {
 IPC_LOG_PVT_NMEA = 0,
 IPC_LOG_PPP_NMEA,
 IPC_LOG_MAX
} log_ExtendTypeEnumTypeValue;
typedef uint8_t log_ExtendTypeEnum;

/**
 * @brief ipc output log to specified file
*/
class IpcExtendLog
{
public:
  static IpcExtendLog& instance() {
    static IpcExtendLog instance;
    return instance;
  }
private:
  IpcExtendLog();
  ~IpcExtendLog();

public:

  /**
   * @brief fprint to file 
   * @param[in] type log extend type
   * @param[in] buff content
   */
  void logPrint(log_ExtendTypeEnum type, const char* buff);

  /**
   * @brief set file path by type
   * @param[in] type 
   * @param[in] path 
   */
  void setFilePath(log_ExtendTypeEnum type, const std::string& path);

private:
  /**
   * @brief Release file pointer
   * @param type
   */
  void releaseFilePtr(log_ExtendTypeEnum type);


/* data */
private:
  std::map<log_ExtendTypeEnum, std::string> _map_file_path;
  std::map<log_ExtendTypeEnum, FILE*> _map_file_ptr;
};

#endif //__IPC_EXTEND_LOG_H__
