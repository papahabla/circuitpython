/*
 * This file is part of the Micro Python project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2017 Scott Shawcroft for Adafruit Industries
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <stdint.h>
#include <string.h>

#include "py/gc.h"
#include "py/mperrno.h"
#include "py/runtime.h"
#include "common-hal/audiobusio/PDMIn.h"
#include "shared-bindings/audiobusio/PDMIn.h"
#include "shared-bindings/microcontroller/Pin.h"

#include "asf/sam0/drivers/port/port.h"
#include "samd21_pins.h"

#include "shared_dma.h"

void pdmin_reset(void) {
    while (I2S->SYNCBUSY.reg & I2S_SYNCBUSY_ENABLE) {}
    I2S->INTENCLR.reg = I2S_INTENCLR_MASK;
    I2S->INTFLAG.reg = I2S_INTFLAG_MASK;
    I2S->CTRLA.reg &= ~I2S_SYNCBUSY_ENABLE;
    while (I2S->SYNCBUSY.reg & I2S_SYNCBUSY_ENABLE) {}
    I2S->CTRLA.reg = I2S_CTRLA_SWRST;
}

void common_hal_audiobusio_pdmin_construct(audiobusio_pdmin_obj_t* self,
                                           const mcu_pin_obj_t* clock_pin,
                                           const mcu_pin_obj_t* data_pin,
                                           uint32_t frequency,
                                           uint8_t bit_depth,
                                           bool mono,
                                           uint8_t oversample) {
    self->clock_pin = clock_pin; // PA10, PA20 -> SCK0, PB11 -> SCK1
    if (clock_pin == &pin_PA10 || clock_pin == &pin_PA20) {
        self->clock_unit = 0;
    } else if (clock_pin == &pin_PB11) {
        self->clock_unit = 1;
    } else {
        mp_raise_ValueError("Invalid clock pin");
    }

    self->data_pin = data_pin; // PA07, PA19 -> SD0, PA08, PB16 -> SD1

    if (data_pin == &pin_PA07 || data_pin == &pin_PA19) {
        self->serializer = 0;
    } else if (data_pin == &pin_PA08
        #ifdef PB16
        || data_pin == &pin_PB16) {
        #else
        ) {
        #endif
        self->serializer = 1;
    } else {
        mp_raise_ValueError("Invalid data pin");
    }

    claim_pin(clock_pin);
    claim_pin(data_pin);

    if (MP_STATE_VM(audiodma_block_counter) == NULL &&
        !allocate_block_counter()) {
        mp_raise_RuntimeError("Unable to allocate audio DMA block counter.");
    }

    if (frequency != 8000 || bit_depth != 8 || !mono || oversample != 64) {
        mp_raise_NotImplementedError("");
    }

    // TODO(tannewt): Use the DPLL to get a more precise sampling rate.
    // DFLL -> GCLK (/600 for 8khz, /300 for 16khz and /150 for 32khz) -> DPLL (*(63 + 1)) -> GCLK ( / 10) -> 512khz

    // For now sample at 8mhz (GCLK3) / 15 ~= 8333 khz

    i2s_init(&self->i2s_instance, I2S);
    struct i2s_clock_unit_config config_clock_unit;
    i2s_clock_unit_get_config_defaults(&config_clock_unit);
    config_clock_unit.clock.gclk_src = GCLK_GENERATOR_3;

    config_clock_unit.clock.mck_src = I2S_MASTER_CLOCK_SOURCE_GCLK;
    config_clock_unit.clock.mck_out_enable = false;

    config_clock_unit.clock.sck_src = I2S_SERIAL_CLOCK_SOURCE_MCKDIV;
    config_clock_unit.clock.sck_div = 15;

    config_clock_unit.frame.number_slots = 2;
    config_clock_unit.frame.slot_size = I2S_SLOT_SIZE_32_BIT;
    config_clock_unit.frame.data_delay = I2S_DATA_DELAY_0;

    config_clock_unit.mck_pin.enable = false;
    config_clock_unit.sck_pin.enable = true;
    config_clock_unit.sck_pin.gpio = self->clock_pin->pin;
    // Mux is always the same.
    config_clock_unit.sck_pin.mux = 6L;
    config_clock_unit.fs_pin.enable = false;
    if (i2s_clock_unit_set_config(&self->i2s_instance, self->clock_unit, &config_clock_unit) != STATUS_OK) {
        while(true) {}
    }

    struct i2s_serializer_config config_serializer;
    i2s_serializer_get_config_defaults(&config_serializer);
    config_serializer.clock_unit = self->clock_unit;
    config_serializer.mode = I2S_SERIALIZER_RECEIVE;
    config_serializer.data_size = I2S_DATA_SIZE_32BIT;
    config_serializer.data_pin.gpio = self->data_pin->pin;
    // Mux is always the same.
    config_serializer.data_pin.mux = 6L;
    config_serializer.data_pin.enable = true;
    if (i2s_serializer_set_config(&self->i2s_instance, self->serializer,
            &config_serializer) != STATUS_OK) {
                while(true) {}
            }
    i2s_enable(&self->i2s_instance);
}

void common_hal_audiobusio_pdmin_deinit(audiobusio_pdmin_obj_t* self) {
    i2s_disable(&self->i2s_instance);
    i2s_reset(&self->i2s_instance);
}

// Algorithm from https://en.wikipedia.org/wiki/Hamming_weight
static uint8_t hamming_weight(uint32_t word) {
    uint32_t b0 = (word >> 0) & 0x55555555;
    uint32_t b1 = (word >> 1) & 0x55555555;
    uint32_t c = b0 + b1;
    uint32_t d0 = (c >> 0) & 0x33333333;
    uint32_t d2 = (c >> 2) & 0x33333333;
    uint32_t e = d0 + d2;
    uint32_t f0 = (e >> 0) & 0x0f0f0f0f;
    uint32_t f4 = (e >> 4) & 0x0f0f0f0f;
    uint32_t g = f0 + f4;
    uint32_t h0 = (g >> 0) & 0x00ff00ff;
    uint32_t h8 = (g >> 8) & 0x00ff00ff;
    uint32_t i = h0 + h8;
    uint32_t j0 = (i >> 0) & 0x0000ffff;
    uint32_t j16 = (i >> 16) & 0x0000ffff;
    return j0 + j16;
}

uint32_t common_hal_audiobusio_pdmin_record_to_buffer(audiobusio_pdmin_obj_t* self, uint8_t* output_buffer, uint32_t length) {
    // Write the wave file header.

    // We allocate two 256 byte buffers on the stack to use for double buffering.
    // Our oversample rate is 64 (bits) so each buffer produces 32 samples.
    uint32_t first_buffer[64];
    first_buffer[0] = 0xdeadbeef;
    uint32_t second_buffer[64];
    second_buffer[0] = 0xdeadbeef;

    COMPILER_ALIGNED(16) DmacDescriptor second_descriptor;

    // Set up the DMA
    struct dma_descriptor_config descriptor_config;
    dma_descriptor_get_config_defaults(&descriptor_config);
    descriptor_config.beat_size = DMA_BEAT_SIZE_WORD;
    descriptor_config.step_selection = DMA_STEPSEL_SRC;
    descriptor_config.source_address = (uint32_t)&I2S->DATA[self->serializer];
    descriptor_config.src_increment_enable = false;
    // Block transfer count is the number of beats per block (aka descriptor).
    // In this case there are two bytes per beat so divide the length by two.
    uint16_t block_transfer_count = 64;
    if (length < block_transfer_count / 2) {
        block_transfer_count = 2 * length;
    }
    descriptor_config.block_transfer_count = block_transfer_count;
    descriptor_config.destination_address = ((uint32_t) first_buffer + sizeof(uint32_t) * block_transfer_count);
    descriptor_config.event_output_selection = DMA_EVENT_OUTPUT_BLOCK;
    if (length > 32) {
        descriptor_config.next_descriptor_address = ((uint32_t)&second_descriptor);
    } else {
        descriptor_config.next_descriptor_address = 0;
    }
    dma_descriptor_create(audio_dma.descriptor, &descriptor_config);

    if (length > block_transfer_count * 2) {
        block_transfer_count = 64;
        if (length < block_transfer_count) {
            block_transfer_count = 2 * (length - 32);
        }
        descriptor_config.block_transfer_count = block_transfer_count;
        descriptor_config.destination_address = ((uint32_t) second_buffer + sizeof(uint32_t) * block_transfer_count);
        if (length > 64) {
            descriptor_config.next_descriptor_address = ((uint32_t)audio_dma.descriptor);
        } else {
            descriptor_config.next_descriptor_address = 0;
        }
        dma_descriptor_create(&second_descriptor, &descriptor_config);
    }

    switch_audiodma_trigger(I2S_DMAC_ID_RX_0 + self->serializer);

    if (dma_start_transfer_job(&audio_dma) != STATUS_OK) {
        while(true) {}
    }
    tc_start_counter(MP_STATE_VM(audiodma_block_counter));
    i2s_clock_unit_enable(&self->i2s_instance, self->clock_unit);
    i2s_serializer_enable(&self->i2s_instance, self->serializer);
    I2S->DATA[1].reg = I2S->DATA[1].reg;

    // Record
    uint32_t buffers_processed = 0;
    uint32_t total_bytes = 0;
    while (total_bytes < length) {
        // Wait for the next buffer to fill
        while (tc_get_count_value(MP_STATE_VM(audiodma_block_counter)) == buffers_processed) {
            #ifdef MICROPY_VM_HOOK_LOOP
                MICROPY_VM_HOOK_LOOP
            #endif
        }
        if (tc_get_count_value(MP_STATE_VM(audiodma_block_counter)) != (buffers_processed + 1)) {
            break;
        }
        uint32_t* buffer = first_buffer;
        DmacDescriptor* descriptor = audio_dma.descriptor;
        if (buffers_processed % 2 == 1) {
            buffer = second_buffer;
            descriptor = &second_descriptor;
        }
        // Decimate the last buffer
        for (uint16_t i = 0; i < descriptor->BTCNT.reg; i++) {
            output_buffer[buffers_processed * 32 + i / 2] += hamming_weight(buffer[i]) * 4;
            total_bytes += i % 2;
        }
        buffers_processed++;

        if (length - total_bytes < 32) {
            descriptor->BTCNT.reg = (length - total_bytes) * 2;
            descriptor->DSTADDR.reg = ((uint32_t) buffer) + (length - total_bytes) * 8;
            descriptor->DESCADDR.reg = 0;
        }
        // Write it to the file (may or may not cause an actual write)
    }

    // Turn off the I2S clock and serializer. Peripheral is still enabled.
    i2s_serializer_disable(&self->i2s_instance, self->serializer);
    i2s_clock_unit_disable(&self->i2s_instance, self->clock_unit);

    // Shutdown the DMA
    tc_stop_counter(MP_STATE_VM(audiodma_block_counter));
    dma_abort_job(&audio_dma);

    // Flush the file
    return total_bytes;
}

void common_hal_audiobusio_pdmin_record_to_file(audiobusio_pdmin_obj_t* self, uint8_t* buffer, uint32_t length) {

}
