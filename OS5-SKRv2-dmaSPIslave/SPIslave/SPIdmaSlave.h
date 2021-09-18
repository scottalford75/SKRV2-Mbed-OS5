
#include "spidma.h"


class SPIdmaSlave
{
    public:

    SPIdmaSlave();
    virtual ~SPIdmaSlave();

    virtual int init();
    virtual int deinit();


    private:

    bool _is_initialized;
}