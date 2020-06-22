/** @file
    Decoder for Ambient Weather TX-3102 (FCC ID: 2ALZ7-3102C1708).

    Copyright (C) 2020 Daniel J. Grinkevich

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
 */

/**
The device uses FSK_PCM encoding,

The device sends a transmission every 5 minutes or can be triggered with the refesh button.

A transmission starts with a preamble of 0xAAAAAAAAAA and codeword 0x2DD4

Data layout:

    01 02 03 04 05 06 07 09 10 11 12 13 14 15 16 17 18 19
    II II ?? ?? ?? ?? ?C ?? ?? ?? ?? ?? TT TB MM ?? ?? SS	
    1a 92 18 70 0c 31 41 ff ff ff 00 00 23 42 01 ff f0 a7
    c5 f9 18 70 08 22 42 ff ff ff 00 00 26 82 01 ff f0 76

- I: CRC-16/X-MODEM (bytes 03 to 18)
- C: Channel (1-7)
- T: 12 bit temperature, 234 = 23.4C (byte 13 might be used, couldn't get the probe below -9C)
- B: Battery status and sign for temperature (S?B?), ex: 0010 good voltage, 0000 voltage below 2.6V, 1010 negative temperature and good voltage, 1000 negative temperature and low voltage 
- M: 8 bit soil moisture, (values 01 - 16)
- S: SUM-8 of bytes 03 to 19 XOR with FF

Bytes 09 to 11 and 17 to 18 are probably for temperature/humidity/spa sensors.
Bytes 03 to 07 are probably the device type and unique ID, but the data I have is only for two soil probes.

From the manufacturer, "The WS-8482 can receive up to 7 sensors, including thermo-hygrometer (TX-3110B), 
floating pool and spa thermometer (TX-3107) and soil temperature and moisture (TX-3102)."

*/

#include "decoder.h"

#define TX3102_BITLEN      200
#define TX3102_STARTBYTE   0xAA

static int ambient_weather_tx3102_decode(r_device *decoder, bitbuffer_t *bitbuffer)
{
    data_t *data;
    uint8_t *b;
    char value[12];  
    uint8_t sensor_id;
    float temperature;
    uint8_t moisture_array[16] = { 0, 7, 13, 20, 27, 33, 40, 47, 53, 60, 67, 73, 80, 87, 93, 99 }; //device has 16 values mapped from 0% to 99%
    uint8_t moisture;
    uint8_t battery_ok;
    uint8_t sign;
    uint16_t r_crc;
    uint16_t c_crc;
    uint8_t dataforcrc[15];

    /*
     * Early debugging aid to see demodulated bits in buffer and
     * to determine if your limit settings are matched and firing
     * this decode callback.
     *
     * 1. Enable with -vvv (debug decoders)
     * 2. Delete this block when your decoder is working
     */
        if (decoder->verbose > 1) {
            bitbuffer_printf(bitbuffer, "%s: ", __func__);
        }

    b = bitbuffer->bb[0];

    if (b[0] != TX3102_STARTBYTE) {
        return 0;
    }

    if (b[5] != 0x2D && b[6] != 0xD4) { // check for codeword 2dd4
	return 0;
    }
    if (bitbuffer->bits_per_row[0] < TX3102_BITLEN) {
        return DECODE_ABORT_LENGTH;
    }
    r_crc = (b[7] << 8) | b[8];
    //there has to be a better way to do this
    dataforcrc[0] = b[9];
    dataforcrc[1] = b[10];
    dataforcrc[2] = b[11];
    dataforcrc[3] = b[12];
    dataforcrc[4] = b[13];
    dataforcrc[5] = b[14];
    dataforcrc[6] = b[15];
    dataforcrc[7] = b[16];
    dataforcrc[8] = b[17];
    dataforcrc[9] = b[18];
    dataforcrc[10] = b[19];
    dataforcrc[11] = b[20];
    dataforcrc[12] = b[21];
    dataforcrc[13] = b[22];
    dataforcrc[14] = b[23];

    c_crc = crc16(dataforcrc, 15, 0x1021, 0x0000);

    uint8_t c_sum = (add_bytes(dataforcrc, 15) ^ 0xFF) - b[24];
    if (c_sum) {
       return DECODE_FAIL_MIC;
    }

    if (c_crc != r_crc) {
	return DECODE_FAIL_MIC;
    }

    sprintf(value, "%02x%02x%02x%02x%02x", b[9], b[10], b[11], b[12], b[13]); // unknown data, maybe device type and unique id?
    sensor_id = (b[13] & 0xF);

    if (sensor_id > 7) { // channel can only be 1 to 7
	return DECODE_FAIL_SANITY;
    }

    battery_ok = ((b[20] & 0x02) >> 1);

    sign = ((b[20] & 0x08) >> 3);
    if (sign == 1) { // negative temperature
    	temperature = (b[18] & 0xF) * -10 + ((b[19] >> 4) & 0xF) * -1 + (b[19] & 0xF) * -0.1;
    }
    else {
    	temperature = ((b[19] >> 4) & 0xF) * 10 + (b[19] & 0xF) + ((b[20] >> 4) & 0xF) * 0.1;
    }
    moisture = moisture_array[(((b[21] >> 4) & 0xF) * 10 + (b[21] & 0xF)) - 1];

    /* clang-format off */
    data = data_make(
            "model", "", DATA_STRING, "Ambient Weather TX-3102",
	    "id", "", DATA_INT,	sensor_id,
	    "temperature", "", DATA_FORMAT, "%.1f C", DATA_DOUBLE, temperature,
	    "moisture", "", DATA_FORMAT, "%.1i %%", DATA_INT, moisture,
	    "battery", "", DATA_STRING, battery_ok ? "OK" : "LOW",
            "data",  "", DATA_STRING,    value,
            "mic", "Integrity", DATA_STRING, "CRC",
             NULL);
    /* clang-format on */

    decoder_output_data(decoder, data);

    // Return 1 if message successfully decoded
    return 1;
}

static char *output_fields[] = {
        "model",
	"channel",
	"temperature",
	"battery",
        "data",
	"mic",
         NULL,
};

r_device ambient_weather_tx3102 = {
        .name        = "Ambient Weather TX-2103",
        .modulation  = FSK_PULSE_PCM,
        .short_width = 130,  
        .long_width  = 130,  
        .reset_limit = 3000,
        .decode_fn   = &ambient_weather_tx3102_decode,
        .disabled    = 0,
        .fields      = output_fields,
};
