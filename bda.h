/*
 * Copyright (c) 2018, Fabian Freyer <fabian.freyer@physik.tu-berlin.de>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once
#include <sys/types.h>
#include <stdint.h>

#define PACKED __attribute__((packed))

struct bios_data_area {
    uint16_t com1_base;
    uint16_t com2_base;
    uint16_t com3_base;
    uint16_t com4_base;

    uint16_t lpt1_base;
    uint16_t lpt2_base;
    uint16_t lpt3_base;
    uint16_t lpt4_base;

    uint16_t equipment_flags;

    uint8_t PCjr;                     /** infrared keyboard link error count */

    uint16_t memsize;                 /** Memory size in Kbytes */

    uint8_t reserved;

    uint8_t bios_control_flags;

    uint8_t kbd_flag0;
    uint8_t kbd_flag1;

    uint8_t keypad_entry;

    /* Keyboard buffer (circular queue buffer) */
    uint16_t kbd_head_offset;    /** Offset from 40:00 to keyboard buffer head */
    uint16_t kbd_tail_offset;    /** Offset from 40:00 to keyboard buffer tail */
    char kbd_buffer[32];
    
    uint8_t drive_recal;            /** Drive recalibration status */
    uint8_t floppy_motor_status;    /** Diskette motor status */
    uint8_t floppy_motor_shutoff;   /** Motor shutoff counter */
    uint8_t floppy_laststatus;      /** Status of last diskette operation */
    uint8_t floppy_nec_status[7];   /** NEC diskette controller status */

    uint8_t video_mode;             /** Current video mode */
    uint16_t video_columns;         /** Number of screen columns */
    uint16_t video_regen_buflen;    /** Size of current video regen buffer in */
    uint16_t video_page_offset;     /** Offset of current video page in regen */
    uint16_t video_page_cursor[8];  /** Cursor position of pages 1-8 */
    uint8_t video_cursor_bottom;    /** Cursor ending (bottom) scan line */
    uint8_t video_cursor_top;       /** Cursor starting (top) scan line */
    uint8_t video_page;             /** Active display page number */

#define CRT_MONO 0x3b4
#define CTF_COLOR 0x3d4

    uint16_t crt_base_addr;         /** Base port address for CRT controller */
    uint8_t crt_control_reg_val;    /** 6845 CRT mode control register value */
    uint8_t cga_pallette_mask;       /** CGA color palette mask setting */

    uint8_t cassete_tape_ctrl[5];   /** Cassette tape control (before AT) */

    uint32_t daily_timer_counter;   /** Daily timer counter, 0 at midnight */
    uint8_t daily_timer_rollover;   /** Clock rollover flag: counter > 24h */
    uint8_t bios_break_flag;        /** bit 7 is set if Ctrl-Break ever hit */

#define SOFTRESET_BYPASS_MEMTEST 0x1234
#define SOFTRESET_PRESERVE_MEM 0x4321
#define SOFTRESET_SUSPEND 0x5678
#define SOFTRESET_MANUFACTURER_TEST 0x9ABC
#define SOFTRESET_POSTLOOP 0xABCD

    uint16_t soft_reset_flag;       /** Soft reset  via CA-Del or JMP FFFF:0 */

    uint8_t hd_laststatus;          /** Status of last hard disk operation */
    uint8_t hd_count;               /** Number of hard disks attached */
    uint8_t hd_ctrl_xt;             /** XT fixed disk drive control byte */
    uint8_t hd_adapter_offset;      /** Port offset to fixed disk adapter */

    uint8_t com_timeout[4];
    uint8_t lpt_timeout[4];

    uint16_t kbd_buf_start_offset;  /** Keyboard buffer start offset */
    uint16_t kbd_buf_end_offset;    /** Keyboard buffer end offset */

    uint8_t rows_visible;           /** Rows on the screen less 1 (EGA+) */
    uint16_t point_height;          /** Point height of character matrix */
    uint8_t video_mode_options;     /** Video mode options (EGA+) */
    uint8_t video_mode_features;    /** EGA feature bit switches */
    uint8_t video_display_data;     /** Video display data area */
    uint8_t dcc_table_index;        /** Display Combination Code table index */

    uint8_t last_diskette_datarate; /** Last diskette data rate selected */

    uint8_t hd_status;          /** Hard disk status returned by controller */
    uint8_t hd_error;           /** Hard disk error returned by controller */
    uint8_t hd_interrupt_ctl;       /** Hard disk interrupt control flag */
    uint8_t hd_floppy_combi;        /** Combination hard/floppy disk card */
    uint8_t media_state[4];         /** Drive 0,1,2,3 media state */
    uint8_t track_seeked_drive0;    /** Track currently seeked to on drive 0 */
    uint8_t track_seeked_drive1;    /** Track currently seeked to on drive 1 */

    uint8_t kbd_mode_type;          /** Keyboard mode/type */
    uint8_t kbd_led_flags;          /** Keyboard LED flags */

    uint32_t uwait_complete;        /** Pointer to user wait complete flag */
    uint32_t uwait_timeout;         /** User wait Time-Out value in us */

    uint8_t rtc_wait_flag;          /** RTC wait function flag */

    uint8_t lana_dma_chan_flags;    /** LANA DMA channel flags */
    uint16_t lana_status;           /** Status of LANA 0,1 */

    uint32_t saved_hd_iv;           /** Saved hard disk interrupt vector */
    uint32_t bios_video_save;       /** BIOS Video Save/Override Ptr Table */

    uint8_t reserved1[8];

    uint8_t kbd_nmi_ctl_flags;      /** Keyboard NMI control flags */
    uint32_t kbd_break_pending;     /** Keyboard break pending flags */

    uint8_t port60_queue;           /** Port 60 single byte queue */

    uint8_t last_scancode;          /** Scan code of last key */
    uint8_t nmi_head_ptr;           /** NMI buffer head pointer */
    uint8_t nmi_tail_ptr;           /** NMI buffer tail pointer */
    uint8_t nmi_scancode_buf[16];   /** NMI scan code buffer */

    uint8_t padding1;

    uint16_t day_counter;           /** Day counter */

    uint8_t padding2[32];

    uint8_t intra_applc_comm[16];   /** Intra-Applications Communications */
} PACKED;
