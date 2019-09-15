#ifndef HVR_LOGGER_H__
#define HVR_LOGGER_H__

#ifdef HVR_LOGGER_STATIC_LIBRARY
#define HVR_LOG_API
#else
#ifdef HVR_LOGGER_DLL_EXPORT
#define HVR_LOG_API extern "C" __declspec(dllexport)
#else
#define HVR_LOG_API extern "C" __declspec(dllimport)
#endif
#endif	//STATIC_LIBRARY

HVR_LOG_API bool hvr_log_declare(char *module_name);
HVR_LOG_API void hvr_log_trace(char *logger, char *pFormat, ...);
HVR_LOG_API void hvr_log_debug(char *logger, char *pFormat, ...);
HVR_LOG_API void hvr_log_info(char *logger, char *pFormat, ...);
HVR_LOG_API void hvr_log_warn(char *logger, char *pFormat, ...);
HVR_LOG_API void hvr_log_error(char *logger, char *pFormat, ...);
HVR_LOG_API void hvr_log_fatal(char *logger, char *pFormat, ...);

//#define HVR_DECLARE_LOGGER(module_name) hvr_log_declare(#module_name)
#define HVR_DECLARE_LOGGER(module_name) static bool hvr_log_declare_##module_name = hvr_log_declare(#module_name)
#define HVR_LOG_TRACE(logger, ...)	hvr_log_trace(#logger, __VA_ARGS__)
#define HVR_LOG_DEBUG(logger, ...)	hvr_log_debug(#logger, __VA_ARGS__)
#define HVR_LOG_INFO(logger, ...)	hvr_log_info(#logger, __VA_ARGS__)
#define HVR_LOG_WARN(logger, ...)	hvr_log_warn(#logger, __VA_ARGS__)
#define HVR_LOG_ERROR(logger, ...)	hvr_log_error(#logger, __VA_ARGS__)
#define HVR_LOG_FATAL(logger, ...)	hvr_log_fatal(#logger, __VA_ARGS__)

#endif	//HVR_LOGGER_H__