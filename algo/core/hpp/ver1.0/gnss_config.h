#ifndef _GNSS_CONFIG_H_
#define _GNSS_CONFIG_H_

#include "macro.h"
#include "gnss_types.h"
#include "gnss.h"
#include "gnss_back.h"
#include "gnss_api_def.h"

#ifdef __cplusplus
extern "C" {
#endif

#if defined(OS_LINUX)
#	if defined(PLATFORM)
#		if (PLATFORM == PLT_THROUGH_SDK)
#		define LOCAL_DATA_FOLDER            "./data/"
#		define LOG_SUB_FOLDER_PREFIX        ""
#		define LOCAL_CFG_FILE_PATH          "./testCfg.ini"
#		else
#		define LOCAL_DATA_FOLDER            "/data/vendor/location/aggnss/"
#		define LOG_SUB_FOLDER_PREFIX        "ag_gnss_log_"
#		define LOCAL_CFG_FILE_PATH          "/etc/ag_gnss_hal_cfg.ini"
#		endif
#	else
#	define LOCAL_DATA_FOLDER                "./data/"
#	define LOG_SUB_FOLDER_PREFIX            ""
#	define LOCAL_CFG_FILE_PATH              "./testCfg.ini"
#	endif
#elif defined(_WIN32)
#	define LOCAL_DATA_FOLDER                ".\\data\\"
#	define LOG_SUB_FOLDER_PREFIX            ""
#	define LOCAL_CFG_FILE_PATH              ".\\testCfg.ini"
#else
#	define LOCAL_DATA_FOLDER                "/tmp/"
#	define LOG_SUB_FOLDER_PREFIX            ""
#	define LOCAL_CFG_FILE_PATH              ""
#endif
#define LOCAL_LOG_FOLDER             LOCAL_DATA_FOLDER

	typedef struct {
		size_t size;
		char   auth_server[64];
		uint16_t    auth_port;
	} sdk_server_cfg_t;

	typedef struct {
		char user[128];
		char password[128];
		char mountpoint[128];
	} NtripAccount_t;

	typedef enum {
		MEAS_RATE_1HZ = 1,
		MEAS_RATE_2HZ = 2,
		MEAS_RATE_5HZ = 5,
		MEAS_RATE_10HZ = 10
	} input_meas_rate_t;
	
	typedef struct
	{
		uint8_t       ppk_mode; // 0: rtk mode 1:ppk mode
		uint8_t       enable_pe;
		uint8_t       enable_waas;
		uint8_t       enable_indoor;
		uint8_t       dynamics;
        gnss_meas_type_t  chipType;
		sub_chiptype_e    sub_chipType;   // specific chiptype
		rinex_data_type_t rinexDataType; // rcv/board type of RINEX file
		uint8_t       automobile;	// flag to distinguish mobile phone(0) and mobile(1).
		algo_scenario_t   applyScenario; // flag to indicate what scenario the rcv antena is in
		uint8_t       rtd_usage_flag; // bit0: gps, bit1:glonass, bit2:beidou
		uint8_t       gnss_usage_flag; // #bit0:gps, bit1:gln, bit2:bds, bit3:gal
		meas_type_mask_t meas_type_mask;  // #bit0:PR, bit1:DR, bit2:CP, bit3:SNR
		uint8_t       is_playback;
		uint16_t      cno_mask;
		uint16_t      start_mode;
		double      altLowLimit;
		double      altHighLimit;
		double      pdop_mask;
        double      pos_res_thres;
		double      ele_mask;
        double      codeStdPhone;
        double      codeStdAuto;
        double      carrStdPhone;
		double      carrStdAuto;
		input_meas_rate_t meas_rate;
		rtk_algo_cfg_t  rtk_cfg;
	} Gnss_Cfg_t;

#define IS_MEAS_GPS_MASKED(gnss_cfg)	(((gnss_cfg)->gnss_usage_flag & 0x1) == 0)
#define IS_MEAS_GLN_MASKED(gnss_cfg)	(((gnss_cfg)->gnss_usage_flag & 0x2) == 0)
#define IS_MEAS_BDS_MASKED(gnss_cfg)	(((gnss_cfg)->gnss_usage_flag & 0x4) == 0)
#define IS_MEAS_GAL_MASKED(gnss_cfg)	(((gnss_cfg)->gnss_usage_flag & 0x8) == 0)
#define IS_MEAS_QZS_MASKED(gnss_cfg)	(((gnss_cfg)->gnss_usage_flag & 0x10) == 0)
#define IS_MEAS_SBS_MASKED(gnss_cfg)	(((gnss_cfg)->gnss_usage_flag & 0x20) == 0)

#define IS_MEAS_GPS_USED(gnss_cfg)	(((gnss_cfg)->gnss_usage_flag & 0x1) > 0)
#define IS_MEAS_GLN_USED(gnss_cfg)	(((gnss_cfg)->gnss_usage_flag & 0x2) > 0)
#define IS_MEAS_BDS_USED(gnss_cfg)	(((gnss_cfg)->gnss_usage_flag & 0x4) > 0)
#define IS_MEAS_GAL_USED(gnss_cfg)	(((gnss_cfg)->gnss_usage_flag & 0x8) > 0)
#define IS_MEAS_QZS_USED(gnss_cfg)	(((gnss_cfg)->gnss_usage_flag & 0x10) > 0)
#define IS_MEAS_SBS_USED(gnss_cfg)	(((gnss_cfg)->gnss_usage_flag & 0x20) > 0)

	typedef uint16_t   log_mask_t;
#define LOG_MASK_NO_LOG     (log_mask_t)(0x0)
#define LOG_MASK_STDOUT     (log_mask_t)(0x1<<0)
#define LOG_MASK_STDERR     (log_mask_t)(0x1<<1)
#define LOG_MASK_DEBUG_LOG  (log_mask_t)(0x1<<2)
#define LOG_MASK_PLAYBACK   (log_mask_t)(0x1<<3)
#define LOG_MASK_NMEA       (log_mask_t)(0x1<<4)
#define LOG_MASK_RAWIN      (log_mask_t)(0x1<<5)
#define LOG_MASK_PB_TXT     (log_mask_t)(0x1<<6)
#define LOG_MASK_ALL        (log_mask_t)(LOG_MASK_STDOUT|LOG_MASK_STDERR|LOG_MASK_DEBUG_LOG| \
                             LOG_MASK_PLAYBACK|LOG_MASK_NMEA|LOG_MASK_RAWIN)

	typedef struct
	{
		Log_Level_t log_level;
		uint8_t  flag_algo_enable; /* TRUE/FALSE */
		uint16_t hsm_port;
		uint16_t data_port;
		uint16_t rtcm_port;
		uint16_t re_port;
		uint16_t de_port;
		uint16_t al_port;
		char log_path[MAX_PATH_LEN];
		char debug_log_path[MAX_PATH_LEN];
		char un_sock_path[MAX_PATH_LEN];
		log_mask_t log_mask;    /* LOG_MASK_XXX macro */
		log_mask_t cb_log_mask; /* mask of the log output by callback interface. */
		int32_t log_rolling_size;   /* Max debug file size in bytes or -1. */
#ifdef HTC_UTTOOL
		int32_t htc_ut_tool_enabled;
#endif
		ag_sdk_params_t sdk_account;
		sdk_server_cfg_t serv_cfg;
		NtripAccount_t ntrip_account;
		char nmea_dr_file[MAX_PATH_LEN];
		char nmea_rtd_file[MAX_PATH_LEN];
		char nmea_chip_file[MAX_PATH_LEN];
		char ubxraw_bin_file[MAX_PATH_LEN];
		char grom_file[MAX_PATH_LEN];
		char serial_port[MAX_PATH_LEN];
		int32_t  serial_baudrate;
		input_meas_rate_t meas_rate;
        serial_data_type_t  serial_data_type;
	} Gnss_Sys_Cfg_t;

	typedef enum {	// for playback_mode
		PLAYBACK_MODE_NONE = 0,
		PLAYBACK_MODE_PE = 1,
		PLAYBACK_MODE_SYS = 2,
		PLAYBACK_MODE_RTCM = 3,
		PLAYBACK_MODE_RTK = 4,
		PLAYBACK_MODE_ZEROBL = 5,
		PLAYBACK_MODE_NMEA = 6,    /* OLY+IMU in txt playback file */
		PLAYBACK_MODE_QCDLF = 7,
		PLAYBACK_MODE_PE_TXT = 8,   /* MEAS+IMU in txt playback file */
		PLAYBACK_MODE_PE_NAV = 10
	} PlaybackMode_t;

    typedef enum {
        RTK_FILE_TYPE_RINEX,
        RTK_FILE_TYPE_RTCM3,
		RTK_FILE_TYPE_RTCM2,
		RTK_FILE_TYPE_RAW
    } RTK_File_Type_Enum;

#define MODULE_MASK_HSM_SRV		1
#define MODULE_MASK_RE			(1<<1)
#define MODULE_MASK_DE			(1<<2)
#define MODULE_MASK_DATA		(1<<3)
#define MODULE_MASK_RTCM		(1<<4)
#define MODULE_MASK_AL			(1<<5)
#define MODULE_MASK_ALL			(MODULE_MASK_HSM_SRV | MODULE_MASK_RE | MODULE_MASK_DE | MODULE_MASK_DATA | MODULE_MASK_RTCM | MODULE_MASK_AL)

	typedef struct
	{
		uint8_t  rtcm_interval_factor;
		uint32_t enable_playback;
		PlaybackMode_t playback_mode;
		uint32_t module_mask;
		uint32_t run_with_start_stop_flag;
		uint32_t n_skipped_meas;
		uint32_t enable_cfg_check;          /* FALSE:disable, TRUE:enable */
		char playback_file[MAX_PATH_LEN];
		char pb_txt_file[MAX_PATH_LEN];
		char rtcm_str_file[MAX_PATH_LEN];
        char rtk_base_file[MAX_PATH_LEN];
        char rtk_rover_file[MAX_PATH_LEN];
        char rtk_nav_file[MAX_PATH_LEN];
        RTK_File_Type_Enum  rtk_base_type;
        RTK_File_Type_Enum  rtk_rovr_type;
	} Gnss_Playback_cfg_t;

	typedef struct
	{
		char gyro_type[128];
		char acce_type[128];
		char magn_type[128];
		char press_type[128];
		char odom_type[128];
		char temp_type[128];
		char ospd_type[128];
	} DRHwParam_t;

	typedef struct
	{
		uint8_t enabled;
		//DRConfig_t cfg;
		qxdr_param_t hw_param;
	} Gnss_Dr_Cfg_t;

	typedef struct
	{
		uint32_t loaded;
		Gnss_Cfg_t gnss_cfg;
		Gnss_Sys_Cfg_t sys_cfg;
		Gnss_Dr_Cfg_t dr_cfg;
		Gnss_Playback_cfg_t playback_cfg;
	} Gnss_Test_Cfg_t;

#define LOG_IS_PB_BIN_ENABLED(cfg)       ((cfg)->sys_cfg.log_mask & LOG_MASK_PLAYBACK)
#define LOG_IS_NMEA_OUT_ENABLED(cfg)     ((cfg)->sys_cfg.log_mask & LOG_MASK_NMEA)
#define LOG_IS_DEBUG_LOG_ENABLED(cfg)    ((cfg)->sys_cfg.log_mask & LOG_MASK_DEBUG_LOG)
#define LOG_IS_RAWIN_ENABLED(cfg)        ((cfg)->sys_cfg.log_mask & LOG_MASK_RAWIN)
#define LOG_IS_PB_TXT_ENABLED(cfg)       ((cfg)->sys_cfg.log_mask & LOG_MASK_PB_TXT)
#define LOG_IS_RTCM_BIN_ENABLED(cfg)     ((cfg)->sys_cfg.log_mask & LOG_MASK_DEBUG_LOG)

#define CFG_IS_REALTIME_MODE(cfg)        ((cfg)->playback_cfg.enable_playback == 0)

#define PLAYBACK_MODE_IS_PE(p_cfg)		 ((p_cfg).enable_playback && \
										 ((p_cfg).playback_mode == PLAYBACK_MODE_RTK || \
										 (p_cfg).playback_mode == PLAYBACK_MODE_PE || \
										 (p_cfg).playback_mode == PLAYBACK_MODE_RTCM || \
										 (p_cfg).playback_mode == PLAYBACK_MODE_ZEROBL || \
										 (p_cfg).playback_mode == PLAYBACK_MODE_NMEA || \
										 (p_cfg).playback_mode == PLAYBACK_MODE_PE_NAV))

	typedef struct {
		ag_sdk_params_t          sdk_account;
		sdk_server_cfg_t         server_cfg;
        NtripAccount_t           ntrip_account;
		int8_t                       log_enable;   /* 1:enable all log; 0:disable all log. */
        int8_t                       applyScenario;
		char                     log_path[MAX_PATH_LEN];
		char                     unix_socket_path[MAX_PATH_LEN];
        char                     serial_port[MAX_PATH_LEN];
		char                     cfg_file_path[MAX_PATH_LEN];
		int32_t                      serial_baudrate;
		input_meas_rate_t        meas_rate;
        serial_data_type_t       serial_type;
	} gnss_ext_cfg_t;


#ifdef AG_GNSS_RTK_FUNCTION_IMPL
#	if (PLATFORM == PLT_QUALCOM)
#		define DEFAULT_AK "552982"
#		define DEFAULT_AS "b2793bfb987640ff133e5dbd8ffe9332bfc309704975fe994e513339cf8b7f21"
//#		define DEFAULT_AK "552491" // only used by ourselves
//#		define DEFAULT_AS "85696f9fa33bdfc2d9c35814c8c2a5323e46bdef4c9b4dfe1d6b37073adeec90"
#	elif (PLATFORM == PLT_HUAWEI_UBX)
//#		define DEFAULT_AK "521137" // Xukun's Account
//#		define DEFAULT_AS "75a0afad42d25bc92f9e5c672dfa24efce507ea66e1257662fcb008706d6cf81"
#		define DEFAULT_AK "555124"
#		define DEFAULT_AS "d56aab9c1bdea7a06eef4a1d31ccfe88b49d3007df7730eb28c7005a6c4b28e6"
#	elif (PLATFORM == PLT_NIO_UBX)
#		define DEFAULT_AK "552491"
#		define DEFAULT_AS "85696f9fa33bdfc2d9c35814c8c2a5323e46bdef4c9b4dfe1d6b37073adeec90"
#	else
#		define DEFAULT_AK "509811"
#		define DEFAULT_AS "307e8db9ce11efb51ead646add62607cbafb4f5f38d72dd9dbbf12ea0a7977e3"
#	endif
#	define DEFAULT_ID "sdk_mt2503_4"
#	define DEFAULT_NTRIP_USER "qxlhu005"
#	define DEFAULT_NTRIP_PASSWD "1562bba"
#	define DEFAULT_NTRIP_MOUNTPOINT "RTCM32_GGB"
#else
#	if (PLATFORM == PLT_XM)
#		define DEFAULT_AK "23306"
#		define DEFAULT_AS "21cc225dea8e50399847870afbd79b9bc1e6eeafa1a094423041f361f0d0b5f6"
#	elif (PLATFORM == PLT_SAMSUNG)
#		define DEFAULT_AK "514311"
#		define DEFAULT_AS "3eb5f74476630a04963ae688d1c8c43efdb589450d5d634345e9dc65f7670a99"
#	else //PLATFORM is HTC, SPRD or others
#		define DEFAULT_AK "18922"
#		define DEFAULT_AS "290d92d812763e28c5644bd7baf7e0eacf327e2c295ddb69fd92c8b23bd150dc"
#	endif
#	define DEFAULT_ID "sdk_mt2503_4"
#	define DEFAULT_NTRIP_USER "qxlhu005"
#	define DEFAULT_NTRIP_PASSWD "1562bba"
#	define DEFAULT_NTRIP_MOUNTPOINT "RTCM32_GGB"
#endif

	EXPORT_API char* file_date_tag(int32_t refresh); 
	EXPORT_API Gnss_Test_Cfg_t* GnssConfigGet();
	int32_t  gnss_hsm_cfg_init();

	void Gnss_PE_DefaultCfg(Gnss_Cfg_t* cfg);
	
	void GnssConfigSetAlgoEnable(uint8_t flag);
	uint8_t   GnssConfigGetAlgoEnable();
	void GnssConfigCtrlQXDR(uint8_t en_ctrl);
	void GnssCfgSetDRHwParam(qxdr_param_t *qxdr_params);
	void GnssCfgSetSDKConfig(ag_sdk_params_t *params);
	EXPORT_API const ag_sdk_params_t *GnssCfgGetSDKConfig();
	void GnssCfgSetPEStartMode(uint16_t mode);
	uint16_t  GnssCfgGetPEStartMode();
	void GnssConfigRestartDebugLogdir();
	int32_t gnss_cfg_is_cb_log_enabled(ag_gnss_log_t type);

	void GnssExtCfgInit();
	void GnssExtCfgFree();
	uint8_t   GnssExtCfgStatus();
	const char *GnssExtCfgFilePath();
	void GnssExtCfgSet(gnss_ext_cfg_t *ext_cfg);
	int32_t GnssExtCfgGetLogPath(char *path, size_t len);
	int32_t GnssExtCfgGetUnixSocketPath(char *path, size_t len);
	int32_t GnssExtCfgGetLogCtrl(uint16_t *mask, Log_Level_t *lvl);
	int32_t GnssExtCfgGetSDKAccount(ag_sdk_params_t *account);
    int32_t GnssExtCfgGetSerialPort(char* serial_port);
    int32_t GnssExtCfgGetSerialBaudRate(int32_t* serial_baudrate);
    int32_t GnssExtCfgGetNtripAccount(NtripAccount_t* ntrip_account);
    int32_t GnssExtCfgApplyScenario(uint8_t* apply_scenario);
	int32_t GnssExtCfgGetMeasRate(input_meas_rate_t *rate);
	int32_t GnssExtCfgGetQXServerCfg(sdk_server_cfg_t *server_cfg);
    int32_t GnssExtCfgGetSerialDataType(serial_data_type_t* p_type);


#ifdef __cplusplus
}
#endif

#endif

