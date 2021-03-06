#ifndef LOGGER_H_
#define LOGGER_H_

#include <stdint.h>
#include <stdbool.h>

#include "../firmware_common/logger_entry.h"

/* This logging interface is intended to be lightweight and tightly coupled
 * to the associated firmware. It is design to simply note that an event
 * occurred in a specific file, at a specific line, with an optional argument */

/* LOGGER_MAX_VERBOSITY defines the maximum (most verbose) level that we'll
 * compile into the firmware */
#ifndef LOGGER_MIN_LEVEL
#   define LOGGER_MIN_LEVEL LOGGER_LEVEL_INFO
#endif

#define LOGGER_LEVEL_DEBUG    0
#define LOGGER_LEVEL_INFO     1
#define LOGGER_LEVEL_WARNING  2
#define LOGGER_LEVEL_ERROR    3
#define LOGGER_LEVLE_SILENT   4

/**
 * Initialize the logger state. This must be called prior to use.
 *
 * @return true on success, false on failure
 */
bool logger_init();


/**
 * Log an event. This should not be used directly; one of the level-specific
 * macros should be used.
 *
 * @param   file_id     ID of source file that is logging the event
 * @param   line        Line number where the event is being logged
 * @param   data        Optional data to include with the event
 *
 * @return true on success, false on full log or failure
 */
bool logger_record(uint8_t file_id, uint16_t line, uint16_t data);

#if LOGGER_LEVEL_DEBUG >= LOGGER_MIN_LEVEL
#   define LOG_DEBUG(data) logger_record(THIS_FILE, __LINE__, data)
#else
#   define LOG_DEBUG(data)
#endif

#if LOGGER_LEVEL_INFO >= LOGGER_MIN_LEVEL
#   define LOG_INFO(data) logger_record(THIS_FILE, __LINE__, data)
#else
#   define LOG_INFO(data)
#endif

#if LOGGER_LEVEL_WARNING >= LOGGER_MIN_LEVEL
#   define LOG_WARNING(data) logger_record(THIS_FILE, __LINE__, data)
#else
#   define LOG_WARNING(data)
#endif

#if LOGGER_LEVEL_ERROR >= LOGGER_MIN_LEVEL
#   define LOG_ERROR(data) logger_record(THIS_FILE, __LINE__, data)
#else
#   define LOG_ERROR(data)
#endif

/**
 * Read and remove an entry from the log
 *
 * @return log entry on success, LOG_EOF if log is empty, and LOG_ERR if a
 *         failure occurred while reading the log.
 */
logger_entry logger_read();
#endif
