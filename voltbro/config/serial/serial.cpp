#include "serial.h"
#if defined(STM32G4) || defined(STM32_G)
#include "stm32g4xx_hal.h"
#if defined(HAL_UART_MODULE_ENABLED)

bool safe_stoi(const std::string& str, int& out_val) {
    if (str.empty()) return false;

    char* endptr = nullptr;
    long val = strtol(str.c_str(), &endptr, 10);

    // Check for conversion errors
    if (endptr != str.c_str() + str.size()) return false;
    if (val < INT32_MIN || val > INT32_MAX) return false;

    out_val = static_cast<int>(val);
    return true;
}

bool safe_stof(const std::string& str, float& out_val) {
    if (str.empty()) return false;

    char* endptr = nullptr;
    float val = strtof(str.c_str(), &endptr);

    if (endptr != str.c_str() + str.size()) return false;
    if (val == HUGE_VALF || val == -HUGE_VALF) return false;

    out_val = val;
    return true;
}

bool get_base_params(BaseConfigData* data, const std::string& param, UARTResponseAccumulator& responses) {
    if (param == NODE_ID_PARAM) {
        responses.append("node_id:%u\n\r", data->node_id);
    }
    else if (param == FDCAN_DATA_PARAM) {
        responses.append("d_baud:%u\n\r", to_underlying(data->fdcan_data_baud));
    }
    else if (param == FDCAN_NOMINAL_PARAM) {
        responses.append("n_baud:%u\n\r", to_underlying(data->fdcan_nominal_baud));
    }
    else {
        return false;
    }
    return true;
}

bool set_base_params(
    BaseConfigData* data,
    const std::string& param,
    const std::string& value,
    UARTResponseAccumulator& responses,
    bool& was_found
) {
    int new_int_value;
    bool is_converted = false;
    if (param == NODE_ID_PARAM ||
        param == FDCAN_DATA_PARAM ||
        param == FDCAN_NOMINAL_PARAM) {
        is_converted = safe_stoi(value, new_int_value);
        was_found = true;
    }
    else {
        was_found = false;
        return false;
    }
    if (!is_converted) {
        responses.append("ERROR: Invalid value\n\r");
        return false;
    }

    if (param == NODE_ID_PARAM) {
        data->node_id = static_cast<CanardNodeID>(new_int_value);
        responses.append("OK: node_id:%u\n\r", data->node_id);
    }
    else if (param == FDCAN_DATA_PARAM) {
        data->fdcan_data_baud = static_cast<FDCANDataBaud>(new_int_value);
        responses.append("OK: d_baud:%u\n\r", to_underlying(data->fdcan_data_baud));
    }
    else if (param == FDCAN_NOMINAL_PARAM) {
        data->fdcan_nominal_baud = static_cast<FDCANNominalBaud>(new_int_value);
        responses.append("OK: n_baud:%u\n\r", to_underlying(data->fdcan_nominal_baud));
    }
    else {
        responses.append("ERROR: Unknown parameter\n\r");
        return false;
    }

    return true;
}

#endif
#endif
