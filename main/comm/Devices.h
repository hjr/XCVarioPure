/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include <cstdint>

// List of supported devices
// Do not change sequence of the enum values as they are used in nvs permanent storage
enum DeviceId : uint8_t
{
    NO_DEVICE,
    ANEMOI_DEV,
    ATR833_HOST_DEV,
    FLARM_DEV,
    FLARM_HOST_DEV,
    FLARM2_DEV,
    FLARM_HOST2_DEV,
    JUMBO_DEV,
    MAGSENS_DEV,
    MAGLEG_DEV,
    CANREGISTRAR_DEV,  // <- 10 CAN id registry
    NAVI_DEV,
    NAVI2_DEV,
    RADIO_ATR833_DEV,
    RADIO_KRT2_DEV,
    RADIO_REMOTE_DEV,
    XCVARIOFIRST_DEV,
    XCVARIOSECOND_DEV,
    FLARM_PROXY,
    RADIO_PROXY,
    TEMPSENS_DEV,
    FLARM_HOST3_DEV,
    FLAP_SENS_DEV,
    TEST_DEV,
    TEST_DEV2
};


// Supported protocol id's
// Do not change sequence of the enum values as they are used in nvs permanent storage
enum ProtocolType : uint8_t
{
    NO_ONE = 0, // not a protocol
    ANEMOI_P,
    ATR833_REMOTE_P,
    BORGELT_P,
    CAMBRIDGE_P,
    FLARM_P,
    FLARMBIN_P,
    FLARMHOST_P,
    GARMIN_P,
    JUMBOCMD_P,
    KRT2_REMOTE_P, // <- 10
    MAGSENS_P,
    MAGSENSBIN_P,
    NMEASTD_P,
    OPENVARIO_P,
    REGISTRATION_P, // CAN id registration
    XCVARIO_P,
    XCVQUERY_P,
    XCVSYNC_P,
    XCNAV_P,
    SEEYOU_P, // <- 20
    XCVSENS_P,
    GPIO_P,
    TEST_P
};
// old ones .. P_EYE_PEYA, P_EYE_PEYI

constexpr int CAN_REG_PORT = 0x7f0;


