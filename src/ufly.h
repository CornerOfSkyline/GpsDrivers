/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file mtk.h
 *
 * @author Thomas Gubler <thomasgubler@student.ethz.ch>
 * @author Julian Oes <julian@oes.ch>
 */

#pragma once

#include "gps_helper.h"
#include "../../definitions.h"

#define SBP_SYNC        0x55
#define SBP_CRC_ERROR -1

#define UFLY_BAUDRATE   115200

typedef enum {
    SBP_DECODE_UNINIT = 0,
    SBP_DECODE_SYNC = 1,
    SBP_DECODE_MSG_ID1 = 2,
    SBP_DECODE_MSG_ID2 = 3,
    SBP_DECODE_MSG_SENDER1 = 4,
    SBP_DECODE_MSG_SENDER2 = 5,
    SBP_DECODE_PAYLOAD_LENGTH = 6,
    SBP_DECODE_PAYLOAD = 7,
    SBP_DECODE_CRC1 = 8,
    SBP_DECODE_CRC2 = 9
} sbp_decode_state_t;

/** GPS Time
 *
 * This message reports the GPS Time.
 */
#define SBP_MSG_GPS_TIME      0x0100
typedef struct __attribute__((packed)) {
  uint16_t wn;       /**< GPS week number [weeks] */
  uint32_t tow;      /**< GPS Time of Week rounded to the nearest ms [ms] */
  int32_t ns;       /**< Nanosecond remainder of rounded tow [ns] */
  uint8_t flags;    /**< Status flags (reserved) */
} msg_gps_time_t;

/** Geodetic Position
 *
 * This single-point position solution message reports the absolute
 * geodetic coordinates and the status (single point absolute vs
 * RTK) of the position solution. If the rover receiver knows the
 * surveyed position of the base station and has an RTK solution,
 * this reports a pseudo-absolute position solution using the base
 * station position and the rover's RTK baseline vector.
 */
#define SBP_MSG_POS_LLH       0x0201
typedef struct __attribute__((packed)) {
  uint32_t tow;           /**< GPS Time of Week [ms] */
  double lat;           /**< Latitude [deg] */
  double lon;           /**< Longitude [deg] */
  double height;        /**< Height [m] */
  uint16_t h_accuracy;    /**< Horizontal position accuracy estimate [mm] */
  uint16_t v_accuracy;    /**< Vertical position accuracy estimate [mm] */
  uint8_t n_sats;        /**< Number of satellites used in solution */
  uint8_t flags;         /**< Status flags */
} msg_pos_llh_t;

/** Velocity in NED
 *
 * This message reports the velocity in local North East Down (NED)
 * coordinates.
 */
#define SBP_MSG_VEL_NED       0x0205
typedef struct __attribute__((packed)) {
  uint32_t tow;           /**< GPS Time of Week [ms] */
  int32_t n;             /**< Velocity North coordinate [mm/s] */
  int32_t e;             /**< Velocity East coordinate [mm/s] */
  int32_t d;             /**< Velocity Down coordinate [mm/s] */
  uint16_t h_accuracy;    /**< Horizontal velocity accuracy estimate [mm/s] */
  uint16_t v_accuracy;    /**< Vertical velocity accuracy estimate [mm/s] */
  uint8_t n_sats;        /**< Number of satellites used in solution */
  uint8_t flags;         /**< Status flags (reserved) */
} msg_vel_ned_t;

/** Dilution of Precision
 *
 * This dilution of precision (DOP) message describes the effect of
 * navigation satellite geometry on positional measurement
 * precision.
 */
#define SBP_MSG_DOPS             0x0206
typedef struct __attribute__((packed)) {
  uint32_t tow;     /**< GPS Time of Week [ms] */
  uint16_t gdop;    /**< Geometric Dilution of Precision [0.01] */
  uint16_t pdop;    /**< Position Dilution of Precision [0.01] */
  uint16_t tdop;    /**< Time Dilution of Precision [0.01] */
  uint16_t hdop;    /**< Horizontal Dilution of Precision [0.01] */
  uint16_t vdop;    /**< Vertical Dilution of Precision [0.01] */
} msg_dops_t;

#define UFLY_RECV_BUFFER_SIZE 40

class GPSDriverUFLY : public GPSHelper
{
public:
    GPSDriverUFLY(GPSCallbackPtr callback, void *callback_user, struct vehicle_gps_position_s *gps_position);
    virtual ~GPSDriverUFLY();
	int receive(unsigned timeout);
    int configure(unsigned &baudrate, OutputMode output_mode);

private:
	/**
	 * Parse the binary MTK packet
	 */
    int parseChar(uint8_t b);

	/**
	 * Handle the package once it has arrived
	 */
    void handleMessage(int32_t msg_id);

	/**
	 * Reset the parse state machine for a fresh start
	 */
	void decodeInit();

    /** Calculate CCITT 16-bit Cyclical Redundancy Check (CRC16).
     *
     * This implementation uses parameters used by XMODEM i.e. polynomial is:
     * \f[
     *   x^{16} + x^{12} + x^5 + 1
     * \f]
     * Mask 0x11021, not reversed, not XOR'd
     * (there are several slight variants on the CCITT CRC-16).
     *
     * \param buf Array of data to calculate CRC for
     * \param len Length of data array
     * \param crc Initial CRC value
     *
     * \return CRC16 value
     */
    uint16_t crc16_ccitt (const uint8_t *buf, uint32_t len, uint16_t crc);

    msg_gps_time_t   msg_gps_time;
    msg_pos_llh_t      msg_pos_llh;
    msg_vel_ned_t     msg_vel_ned;
    msg_dops_t          msg_dpos;

	struct vehicle_gps_position_s *_gps_position;
    sbp_decode_state_t _decode_state;
    uint16_t _sbp_msg_id;
    uint16_t _sbp_msg_sender;
    uint8_t _sbp_payload_length;
    uint8_t _sbp_payload_receive_length;
    uint16_t _sbp_msg_crc;
    uint16_t _sbp_crc;
    int32_t parse_ret;
};
