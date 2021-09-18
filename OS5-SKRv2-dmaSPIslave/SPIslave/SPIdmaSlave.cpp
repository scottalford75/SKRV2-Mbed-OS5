
#include "platform/mbed_debug.h"
# include "SPIdmaSlave.h"

#define SPI_DBG

SPIdmaSlave::SPIdmaSlave()
{

}


SPIdmaSlave::~SPIdmaSlave()
{
    if (_is_initialized)
    {
        deinit();
    }
}


SPIdmaSlave::init()
{
    debug_if(SPIdmaSlave, "init SPI DMA Slave\r\n");

    _is_initialized = true;
}

SPIdmaSlave::deinit()
{
    debug_if(SPIdmaSlave, "init SPI DMA Slave\r\n");

    _is_initialized = false;
}