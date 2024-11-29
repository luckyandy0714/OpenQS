#pragma once
#include <stdio.h>

#ifdef __cplusplus
extern "C"
{
#endif

    void OTA_init();
    void OTA_diagnostic(uint32_t diagnosticPin);

    bool OTA_start(const size_t recrivLength, const unsigned char *hashCode);
    bool OTA_start_str(const size_t recrivLength, const char *hashCode_str);
    bool OTA_updata(const char *src, const size_t src_len);
    void OTA_finish();
    /**
     * @brief   OTA update data finish *
     * @return
     *    - true: Receiv data size and hash verification successfully.
     *    - false: Receiv data size and hash verification failed.
     */
    bool OTA_verify(bool restart);

#ifdef __cplusplus
}
#endif