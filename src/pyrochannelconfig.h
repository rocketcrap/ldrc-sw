#pragma once

#include <stdint.h>
#include <ArduinoJson.h>

/**
 * @brief Stores the configurable items of a pyro channel. These are persisted
 *
 */
struct __attribute__((packed)) PyroChannelConfig {
    static uint8_t constexpr maxPyroChannels = 8; // who could need more than eight?

    PyroChannelConfig();
    enum ChannelType : uint8_t {
        DISABLED_CHANNEL = 0, /**< channel is disabled */
        DROUGE_CHANNEL, /**< channel triggers at apogee */
        MAIN_CHANNEL, /**< channel triggers at set altitude */
        AIRSTART_CHANNEL, /**< channel triggers at burnout with conditions */
        BURNOUT_CHANNEL, /**< channel triggers at burnout, specifying which burnout */
    } channelType;

    /**
     * @brief delay in seconds from trigger to activation
     *
     */
    uint8_t delaySeconds;

    /**
     * @brief altitude in meters above ground to activate main
     *
     */
    uint16_t mainAlt;

    /**
     * @brief burnouts to trigger. relevate to AIRSTART_CHANNEL and BURNOUT_CHANNEL
     *
     * booster is burnout 0, sustainer is 1. In multi-stage situations, there could be multiple burnout events
     *
     */
    uint8_t burnoutNumber;

    /**
     * @brief altitude in meters below which airstart will not activate. 0 to disable
     */
    uint16_t airStartLockoutAltitude;

    /**
     * @brief orientation off vertical axis in degrees; 0 to disable
     *
     */
    uint8_t airStartLockoutAngle;

    /**
     * @brief velocity in m/s required for air start; 0 to disable
     *
     */
    uint16_t airStartLockoutVelocity;
};

bool convertToJson(const PyroChannelConfig &src, JsonVariant dst);
void convertFromJson(JsonVariantConst src, PyroChannelConfig &dst);
