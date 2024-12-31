#pragma once

namespace honeybee_utils {

    enum drone_state_t
    {
        RUNNING,
        STOPPED,
        NONE
    };

    enum honeybee_err_t
    {
        HONEYBEE_OK,
        HONEYBEE_ERR,
        HONEYBEE_INVALID_CRC
    };
}