#pragma once
#include <stdio.h>

#define STORAGE_ROOT "/spiflash"
#define STORAGE_LABEL "storage" // From Partition Table

class Storage
{
private:
    const char *root_fileName;

public:
    Storage(const char *fileName);
    ~Storage();
    const char *getPath();
    size_t fileSize();

    bool format();
    bool mount();
    void unmount();

    bool write(const void *src, const size_t len);
    bool read(void *dst, const size_t len);
    bool deleteFile();
};
