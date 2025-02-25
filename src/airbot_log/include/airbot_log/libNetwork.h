#ifndef LIBNETWOKR_H
#define LIBNETWOKR_H

#include <vector>
#include <string>

#define BASE_VERSION "0.31"

#ifdef LOG_SETTING
    #ifdef DEBUG_MODE
        #define VERSION BASE_VERSION ".2.1"
    #else
        #define VERSION BASE_VERSION ".2.0"
    #endif
#elif defined(OTA_SETTING)
    #ifdef DEBUG_MODE
        #define VERSION BASE_VERSION ".1.1"
    #else
        #define VERSION BASE_VERSION ".1.0"
    #endif
#else // NORMAL ¸ðµå
    #ifdef DEBUG_MODE
        #define VERSION BASE_VERSION ".0.1"
    #else
        #define VERSION BASE_VERSION ".0.0"
    #endif
#endif

#define getLogDataIdx 26

#define objectReq   0x01
#define objectRes   0x02
#define objectEmergency   0x0F
#define objectJig  0xC8
#define objectOTAData  0x0E
/******************************************** */

struct LogData_t {
    std::string	data;
};

#ifdef __cplusplus
extern "C" {
#endif

void start_server();
void stop_server();

/*********************************************************************/

void APIGetVersion(std::string &data);
void APISetEnc(bool enable);

/*********************************************************************/

bool reqGetSoftwareVersion(void);
void resGetSoftwareVersion(const std::string &version);

/*********************************************************************/

bool reqGetLogData(void);
bool resGetLogData(const std::string &folderPath);

#ifdef __cplusplus
}
#endif

#endif // LIBNETWOKR_H
