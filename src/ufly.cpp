/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * @file ufly.cpp
 *
 * @author ZhangJF <zhangjianfei2005@126.com>
 */

#include "ufly.h"

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <ctime>

/**** Trace macros, disable for production builds */
#define UFLY_TRACE_PARSER(...)	{/*GPS_INFO(__VA_ARGS__);*/}	/* decoding progress in parse_char() */
#define UFLY_TRACE_RXMSG(...)		{/*GPS_INFO(__VA_ARGS__);*/}	/* Rx msgs in payload_rx_done() */
#define UFLY_TRACE_SVINFO(...)	{/*GPS_INFO(__VA_ARGS__);*/}	/* NAV-SVINFO processing (debug use only, will cause rx buffer overflows) */

/**** Warning macros, disable to save memory */
#define UFLY_WARN(...)		{GPS_WARN(__VA_ARGS__);}
#define UFLY_DEBUG(...)		{GPS_WARN(__VA_ARGS__);}

/* CRC16 implementation acording to CCITT standards */

static const uint16_t crc16tab[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
    0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};


GPSDriverUFLY::GPSDriverUFLY(GPSCallbackPtr callback, void *callback_user, struct vehicle_gps_position_s *gps_position) :
    GPSHelper(callback, callback_user),
    _gps_position(gps_position)
{

}

GPSDriverUFLY::~GPSDriverUFLY()
{
}

int
GPSDriverUFLY::configure(unsigned &baudrate, OutputMode output_mode)
{
    if (output_mode != OutputMode::GPS) {
        GPS_WARN("UFLY: Unsupported Output Mode %i", (int)output_mode);
        return -1;
    }

    /* set baudrate first */
    if (GPSHelper::setBaudrate(UFLY_BAUDRATE) != 0) {
        return -1;
    }

    baudrate = UFLY_BAUDRATE;

    UFLY_DEBUG("UFLY baudrate set %d!",baudrate);

    return 0;
}

int
GPSDriverUFLY::receive(unsigned timeout)
{
    uint8_t buf[GPS_READ_BUFFER_SIZE];

    /* timeout additional to poll */
    gps_abstime time_started = gps_absolute_time();

    int handled = 0;

    while (true) {
        bool ready_to_return = _configured ? (_got_posllh && _got_velned) : handled;

        int ret = read(buf, sizeof(buf), timeout);

        if(ret < 0){
            UFLY_WARN("UFLY poll_or_read err");
            return -1;
        } else if (ret == 0) {
            if(ready_to_return){
                _got_posllh = false;
                _got_velned = false;
                return handled;
            }
        } else {
            UFLY_DEBUG("read %d bytes",ret);

            for(int i = 0; i < ret; i++){
                parse_ret = parseChar(buf[i]);
                if(ret > 0)
                {
                    handled |= handleMessage(parse_ret);
                }
            }
        }

        /* abort after timeout if no useful packets received */
        if (time_started + timeout * 1000 < gps_absolute_time()) {
            UFLY_DEBUG("timed out, returning");
            return -1;
        }
    }
}

int
GPSDriverUFLY::parseChar(uint8_t b)
{
    int ret = 0;

    //UFLY_DEBUG("parseChar %X",b);

    switch(_decode_state)
    {
    case SBP_DECODE_SYNC:
        if(b == 0x55)
        {
            _sbp_msg_id = 0;
            _sbp_payload_receive_length = 0;
            _decode_state = SBP_DECODE_MSG_ID1;
        }
        break;
    case SBP_DECODE_MSG_ID1:
        _sbp_msg_id = b;
        _sbp_payload_receive_length = 0;
        _decode_state = SBP_DECODE_MSG_ID2;
        break;
    case SBP_DECODE_MSG_ID2:
        _sbp_msg_id |= b << 8u;
        _sbp_payload_receive_length = 0;
        _decode_state = SBP_DECODE_MSG_SENDER1;
        //UFLY_DEBUG("MSG_ID = %02X",_sbp_msg_id);
        break;
    case SBP_DECODE_MSG_SENDER1:
        _sbp_msg_sender = b;
        _sbp_payload_receive_length = 0;
        _decode_state = SBP_DECODE_MSG_SENDER2;
        break;
    case SBP_DECODE_MSG_SENDER2:
        _sbp_msg_sender |= b << 8u;
        _sbp_payload_receive_length = 0;
        _decode_state = SBP_DECODE_PAYLOAD_LENGTH;
        //UFLY_DEBUG("MSG_SENDER = %02X",_sbp_msg_sender);
        break;
    case SBP_DECODE_PAYLOAD_LENGTH:
        _sbp_payload_length = b;
        _sbp_payload_receive_length = 0;
        _decode_state = SBP_DECODE_PAYLOAD;
        //UFLY_DEBUG("MSG_LENGTH = %X",_sbp_payload_length);
        break;
    case SBP_DECODE_PAYLOAD:
        switch(_sbp_msg_id)
        {
        case SBP_MSG_GPS_TIME:
            *((uint8_t *)&msg_gps_time + _sbp_payload_receive_length) = b;
            _sbp_payload_receive_length ++;
            if(_sbp_payload_receive_length == _sbp_payload_length)
            {
                _decode_state = SBP_DECODE_CRC1;
            }
            break;
        case SBP_MSG_POS_LLH:
            *((uint8_t *)&msg_pos_llh + _sbp_payload_receive_length) = b;
            _sbp_payload_receive_length ++;
            if(_sbp_payload_receive_length == _sbp_payload_length)
            {
                _decode_state = SBP_DECODE_CRC1;
            }
            break;
        case SBP_MSG_DOPS:
            *((uint8_t *)&msg_dpos + _sbp_payload_receive_length) = b;
            _sbp_payload_receive_length ++;
            if(_sbp_payload_receive_length == _sbp_payload_length)
            {
                _decode_state = SBP_DECODE_CRC1;
            }
            break;
        case SBP_MSG_VEL_NED:
            *((uint8_t *)&msg_vel_ned + _sbp_payload_receive_length) = b;
            _sbp_payload_receive_length ++;
            if(_sbp_payload_receive_length == _sbp_payload_length)
            {
                _decode_state = SBP_DECODE_CRC1;
            }
            break;
        case SBP_MSG_HEARTBEAT:
            *((uint8_t *)&msg_heartbeat + _sbp_payload_receive_length) = b;
            _sbp_payload_receive_length ++;
            if(_sbp_payload_receive_length == _sbp_payload_length)
            {
                _decode_state = SBP_DECODE_CRC1;
            }
            break;
        default :
            ret = SBP_MSG_ERROR;
            //UFLY_DEBUG("Default msg received!")
            break;
        }
        break;
    case SBP_DECODE_CRC1:
         _sbp_msg_crc = b;
        _decode_state = SBP_DECODE_CRC2;
        break;
    case SBP_DECODE_CRC2:
        _sbp_msg_crc |= b << 8u;
        _decode_state = SBP_DECODE_SYNC;
        switch(_sbp_msg_id)
        {
        case SBP_MSG_GPS_TIME:
            _sbp_crc = crc16_ccitt((uint8_t *)&_sbp_msg_id,2,0);
            _sbp_crc = crc16_ccitt((uint8_t *)&_sbp_msg_sender,2,_sbp_crc);
            _sbp_crc = crc16_ccitt(&_sbp_payload_length,1,_sbp_crc);
            _sbp_crc = crc16_ccitt((uint8_t *)&msg_gps_time,_sbp_payload_length,_sbp_crc);
            if(_sbp_crc == _sbp_msg_crc)
            {
                ret = SBP_MSG_GPS_TIME;
                //UFLY_DEBUG("Time crc ok!")
            }
            else
                ret = SBP_CRC_ERROR;
            break;
        case SBP_MSG_POS_LLH:
            _sbp_crc = crc16_ccitt((uint8_t *)&_sbp_msg_id,2,0);
            _sbp_crc = crc16_ccitt((uint8_t *)&_sbp_msg_sender,2,_sbp_crc);
            _sbp_crc = crc16_ccitt(&_sbp_payload_length,1,_sbp_crc);
            _sbp_crc = crc16_ccitt((uint8_t *)&msg_pos_llh,_sbp_payload_length,_sbp_crc);
            if(_sbp_crc == _sbp_msg_crc)
            {
                ret = SBP_MSG_POS_LLH;
                //UFLY_DEBUG("Pos LLH crc ok!")
            }
            else
                ret = SBP_CRC_ERROR;
            break;
        case SBP_MSG_VEL_NED:
            _sbp_crc = crc16_ccitt((uint8_t *)&_sbp_msg_id,2,0);
            _sbp_crc = crc16_ccitt((uint8_t *)&_sbp_msg_sender,2,_sbp_crc);
            _sbp_crc = crc16_ccitt(&_sbp_payload_length,1,_sbp_crc);
            _sbp_crc = crc16_ccitt((uint8_t *)&msg_vel_ned,_sbp_payload_length,_sbp_crc);
            if(_sbp_crc == _sbp_msg_crc)
            {
                ret = SBP_MSG_VEL_NED;
                //UFLY_DEBUG("Vel Ned crc ok!")
            }
            else
                ret = SBP_CRC_ERROR;
            break;
        case SBP_MSG_DOPS:
            _sbp_crc = crc16_ccitt((uint8_t *)&_sbp_msg_id,2,0);
            _sbp_crc = crc16_ccitt((uint8_t *)&_sbp_msg_sender,2,_sbp_crc);
            _sbp_crc = crc16_ccitt(&_sbp_payload_length,1,_sbp_crc);
            _sbp_crc = crc16_ccitt((uint8_t *)&msg_dpos,_sbp_payload_length,_sbp_crc);
            if(_sbp_crc == _sbp_msg_crc)
            {
                ret = SBP_MSG_DOPS;
                //UFLY_DEBUG("DOPS crc ok!")
            }
            else
                ret = SBP_CRC_ERROR;
            break;
        case SBP_MSG_HEARTBEAT:
            _sbp_crc = crc16_ccitt((uint8_t *)&_sbp_msg_id,2,0);
            _sbp_crc = crc16_ccitt((uint8_t *)&_sbp_msg_sender,2,_sbp_crc);
            _sbp_crc = crc16_ccitt(&_sbp_payload_length,1,_sbp_crc);
            _sbp_crc = crc16_ccitt((uint8_t *)&msg_heartbeat,_sbp_payload_length,_sbp_crc);
            if(_sbp_crc == _sbp_msg_crc)
            {
                ret = SBP_MSG_HEARTBEAT;
                //UFLY_DEBUG("Heartbeat crc ok!")
            }
            else
                ret = SBP_CRC_ERROR;
        default:
            ret = SBP_MSG_ERROR;
            break;
        }
        break;
    default:
        _decode_state = SBP_DECODE_SYNC;
        break;
    }
    return ret;
}

uint32_t
GPSDriverUFLY::handleMessage(int32_t msg_id)
{
    uint32_t ret = 0;
    switch(msg_id)
    {
    case SBP_MSG_HEARTBEAT:
        last_heatbeat_received_ms = gps_absolute_time();
        break;
    case SBP_MSG_GPS_TIME:
        _gps_position->time_utc_usec = time_epoch_convert(msg_gps_time.wn,msg_gps_time.tow);
        break;
    case SBP_MSG_DOPS:
        _gps_position->hdop = msg_dpos.hdop;
        _gps_position->vdop = msg_dpos.vdop;
        //UFLY_WARN("hdop = %f",msg_dpos.hdop);
        //UFLY_WARN("hdop = %f",msg_dpos.vdop);
        break;
    case SBP_MSG_POS_LLH:
        _gps_position->lat = msg_pos_llh.lat * 1e7;
        _gps_position->lon = msg_pos_llh.lon * 1e7;
        _gps_position->eph = msg_pos_llh.h_accuracy * 1e-3;
        _gps_position->epv = msg_pos_llh.v_accuracy * 1e-3;
        UFLY_WARN("eph = %d",msg_pos_llh.h_accuracy);
        UFLY_WARN("epv = %d",msg_pos_llh.v_accuracy);
        switch(msg_pos_llh.flags & 0x07)
        {
        case 0x00:
            _gps_position->fix_type = 3;
            break;
        case 0x01:
            _gps_position->fix_type = 4;
            break;
        case 0x02:
            _gps_position->fix_type = 5;
            break;
        default:
            _gps_position->fix_type = 1;
            break;
        }

        if(msg_pos_llh.flags & 0x08)
            _gps_position->alt = msg_pos_llh.height * 1e3;
        else
            _gps_position->alt_ellipsoid = msg_pos_llh.height * 1e3;

        _gps_position->satellites_used = msg_pos_llh.n_sats;

        UFLY_WARN("llh.flags = %02X",msg_pos_llh.flags);
        UFLY_WARN("alt = %f",msg_pos_llh.height);

        _got_posllh = true;
        ret = 1;
        break;
    case SBP_MSG_VEL_NED:
        _gps_position->vel_n_m_s = msg_vel_ned.n * 1e-3;
        _gps_position->vel_e_m_s = msg_vel_ned.e * 1e-3;
        _gps_position->vel_d_m_s = msg_vel_ned.d * 1e-3;

        _gps_position->timestamp_time		= gps_absolute_time();
        _gps_position->timestamp_velocity 	= gps_absolute_time();
        _gps_position->timestamp_variance 	= gps_absolute_time();
        _gps_position->timestamp_position	= gps_absolute_time();

        _got_velned = true;
        ret = 1;

        // Position and velocity update always at the same time
        _rate_count_vel++;
        _rate_count_lat_lon++;

        break;
    default:
        break;
}
    return ret;
}

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

uint16_t
GPSDriverUFLY::crc16_ccitt(const uint8_t *buf, uint32_t len, uint16_t crc)
{
    for (uint32_t i = 0; i < len; i++)
        crc = (crc << 8) ^ crc16tab[((crc >> 8) ^ *buf++) & 0x00FF];
    return crc;
}

/**
   convert GPS week and milliseconds to unix epoch in milliseconds
 */
uint64_t GPSDriverUFLY::time_epoch_convert(uint16_t gps_week, uint32_t gps_ms)
{
    const uint64_t ms_per_week = 7000ULL*86400ULL;
    const uint64_t unix_offset = 17000ULL*86400ULL + 52*10*7000ULL*86400ULL - 15000ULL;
    uint64_t fix_time_ms = unix_offset + gps_week*ms_per_week + gps_ms;
    return fix_time_ms;
}

