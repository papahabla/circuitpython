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

#include "lib/utils/context_manager_helpers.h"
#include "py/objproperty.h"
#include "py/runtime.h"
#include "shared-bindings/microcontroller/Pin.h"
#include "shared-bindings/audiobusio/PDMIn.h"

//| .. currentmodule:: audiobusio
//|
//| :class:`PDMIn` -- Record an input PDM audio stream
//| ========================================================
//|
//| PDMIn can be used to record an input audio signal on a given set of pins.
//|
//| .. class:: PDMIn(clock_pin, data_pin, \*, frequency=8000, bit_depth=8, mono=True, oversample=64)
//|
//|   Create a PDMIn object associated with the given pins. This allows you to
//|   record audio signals from the given pins.
//|
//|   :param ~microcontroller.Pin clock_pin: The pin to output the clock to
//|   :param ~microcontroller.Pin data_pin: The pin to read the data from
//|   :param int frequency: Frequency of the resulting samples
//|   :param int bit_depth: Final number of bits per sample. Must be divisible by 8
//|   :param bool mono: True when capturing a single channel of audio, captures two channels otherwise
//|   :param int oversample: Number of single bit samples to decimate into a final sample
//|
//|   Simple record to buffer::
//|
//|     import audiobusio
//|     import board
//|
//|     # Prep a buffer to record into
//|     b = bytearray(200)
//|     with audiobusio.PDMIn(board.MICROPHONE_DATA, board.MICROPHONE_CLOCK) as mic:
//|         mic.record(b, len(b))
//|
STATIC mp_obj_t audiobusio_pdmin_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    mp_arg_check_num(n_args, n_kw, 2, 2, true);
    mp_obj_t clock_pin_obj = args[0];
    assert_pin(clock_pin_obj, false);
    const mcu_pin_obj_t *clock_pin = MP_OBJ_TO_PTR(clock_pin_obj);
    assert_pin_free(clock_pin);

    mp_obj_t data_pin_obj = args[1];
    assert_pin(data_pin_obj, false);
    const mcu_pin_obj_t *data_pin = MP_OBJ_TO_PTR(data_pin_obj);
    assert_pin_free(data_pin);

    // create PDMIn object from the given pin
    audiobusio_pdmin_obj_t *self = m_new_obj(audiobusio_pdmin_obj_t);
    self->base.type = &audiobusio_pdmin_type;

    uint32_t frequency = 8000;
    uint8_t bit_depth = 8;
    bool mono = true;
    uint8_t oversample = 64;

    common_hal_audiobusio_pdmin_construct(self, clock_pin, data_pin, frequency, bit_depth, mono, oversample);

    return MP_OBJ_FROM_PTR(self);
}

//|   .. method:: deinit()
//|
//|      Deinitialises the PWMOut and releases any hardware resources for reuse.
//|
STATIC mp_obj_t audiobusio_pdmin_deinit(mp_obj_t self_in) {
    audiobusio_pdmin_obj_t *self = MP_OBJ_TO_PTR(self_in);
    common_hal_audiobusio_pdmin_deinit(self);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(audiobusio_pdmin_deinit_obj, audiobusio_pdmin_deinit);

//|   .. method:: __enter__()
//|
//|      No-op used by Context Managers.
//|
//  Provided by context manager helper.

//|   .. method:: __exit__()
//|
//|      Automatically deinitializes the hardware when exiting a context.
//|
STATIC mp_obj_t audiobusio_pdmin_obj___exit__(size_t n_args, const mp_obj_t *args) {
    (void)n_args;
    common_hal_audiobusio_pdmin_deinit(args[0]);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(audiobusio_pdmin___exit___obj, 4, 4, audiobusio_pdmin_obj___exit__);


//|   .. method:: record(destination, destination_length)
//|
//|     Records destination_length bytes of samples to destination. This is
//|     blocking. It does not filter the input samples, it only decimates them
//|     to produce a sample.
//|
//|     An IOError may be raised when the destination is too slow to record the
//|     audio at the given rate. For internal flash, writing all 1s to the file
//|     before recording is recommended to speed up writes.
//|
STATIC mp_obj_t audiobusio_pdmin_obj_record(mp_obj_t self_obj, mp_obj_t destination, mp_obj_t destination_length) {
    audiobusio_pdmin_obj_t *self = MP_OBJ_TO_PTR(self_obj);

    if (!MP_OBJ_IS_SMALL_INT(destination_length)) {
        mp_raise_TypeError("destination_length must be int");
    }
    uint32_t length = MP_OBJ_SMALL_INT_VALUE(destination_length);

    mp_buffer_info_t bufinfo;
    if (MP_OBJ_IS_TYPE(destination, &fatfs_type_fileio)) {
        mp_raise_NotImplementedError("");
    } else if (mp_get_buffer(destination, &bufinfo, MP_BUFFER_WRITE)) {
        if (bufinfo.len < length) {
            mp_raise_ValueError("Target buffer cannot hold destination_length bytes.");
        }
        common_hal_audiobusio_pdmin_record_to_buffer(self, ((uint8_t*)bufinfo.buf), length);
    }
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_3(audiobusio_pdmin_record_obj, audiobusio_pdmin_obj_record);

STATIC const mp_rom_map_elem_t audiobusio_pdmin_locals_dict_table[] = {
    // Methods
    { MP_ROM_QSTR(MP_QSTR_deinit), MP_ROM_PTR(&audiobusio_pdmin_deinit_obj) },
    { MP_ROM_QSTR(MP_QSTR___enter__), MP_ROM_PTR(&default___enter___obj) },
    { MP_ROM_QSTR(MP_QSTR___exit__), MP_ROM_PTR(&audiobusio_pdmin___exit___obj) },
    { MP_ROM_QSTR(MP_QSTR_record), MP_ROM_PTR(&audiobusio_pdmin_record_obj) },
};
STATIC MP_DEFINE_CONST_DICT(audiobusio_pdmin_locals_dict, audiobusio_pdmin_locals_dict_table);

const mp_obj_type_t audiobusio_pdmin_type = {
    { &mp_type_type },
    .name = MP_QSTR_PDMIn,
    .make_new = audiobusio_pdmin_make_new,
    .locals_dict = (mp_obj_dict_t*)&audiobusio_pdmin_locals_dict,
};
