#include "HoneybeeFramework.h"

using namespace HoneybeeFramework;

class ELRSReceiver : public SubSystem
{
    public:
        UART_ConnectionConfig data_config;
        void init();
        void update() override;
        CRSF_RC_ChannelData get_rc_channels();
    private:
        SerialConnection data_connection;
        CRSF_RC_ChannelData rc_channels;
        void update_rc_channels();
};