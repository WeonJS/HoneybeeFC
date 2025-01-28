#pragma once

namespace honeybee_utils {

    enum hb_drone_state_t
    {
        RUNNING,
        STOPPED,
        NONE
    };

    enum hb_err_t
    {
        HONEYBEE_OK,
        HONEYBEE_ERR,
        HONEYBEE_INVALID_CRC
    };
}