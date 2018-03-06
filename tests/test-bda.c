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

#include <atf-c.h>
#include <bda.h>

ATF_TC(bda_offsets);
ATF_TC_HEAD(bda_offsets, tc)
{
    atf_tc_set_md_var(tc, "descr", "Check BIOS data area offsets");
}
ATF_TC_BODY(bda_offsets, tc)
{
    ATF_CHECK_EQ(0x00, offsetof(struct bios_data_area, com1_base));
    ATF_CHECK_EQ(0x02, offsetof(struct bios_data_area, com2_base));
    ATF_CHECK_EQ(0x04, offsetof(struct bios_data_area, com3_base));
    ATF_CHECK_EQ(0x06, offsetof(struct bios_data_area, com4_base));

    ATF_CHECK_EQ(0x08, offsetof(struct bios_data_area, lpt1_base));
    ATF_CHECK_EQ(0x0A, offsetof(struct bios_data_area, lpt2_base));
    ATF_CHECK_EQ(0x0C, offsetof(struct bios_data_area, lpt3_base));
    ATF_CHECK_EQ(0x0E, offsetof(struct bios_data_area, lpt4_base));

    ATF_CHECK_EQ(0x10, offsetof(struct bios_data_area, equipment_flags));
    ATF_CHECK_EQ(0x12, offsetof(struct bios_data_area, PCjr));
    ATF_CHECK_EQ(0x13, offsetof(struct bios_data_area, memsize));
    ATF_CHECK_EQ(0x15, offsetof(struct bios_data_area, reserved));
    ATF_CHECK_EQ(0x16, offsetof(struct bios_data_area, bios_control_flags));

    ATF_CHECK_EQ(0x17, offsetof(struct bios_data_area, kbd_flag0));
    ATF_CHECK_EQ(0x18, offsetof(struct bios_data_area, kbd_flag1));
    ATF_CHECK_EQ(0x19, offsetof(struct bios_data_area, keypad_entry));
    ATF_CHECK_EQ(0x1A, offsetof(struct bios_data_area, kbd_head_offset));
    ATF_CHECK_EQ(0x1C, offsetof(struct bios_data_area, kbd_tail_offset));
    ATF_CHECK_EQ(0x1E, offsetof(struct bios_data_area, kbd_buffer));

    ATF_CHECK_EQ(0x3E, offsetof(struct bios_data_area, drive_recal));
    ATF_CHECK_EQ(0x3F, offsetof(struct bios_data_area, floppy_motor_status));
    ATF_CHECK_EQ(0x40, offsetof(struct bios_data_area, floppy_motor_shutoff));
    ATF_CHECK_EQ(0x41, offsetof(struct bios_data_area, floppy_laststatus));
    ATF_CHECK_EQ(0x42, offsetof(struct bios_data_area, floppy_nec_status));

    ATF_CHECK_EQ(0x49, offsetof(struct bios_data_area, video_mode));
    ATF_CHECK_EQ(0x4A, offsetof(struct bios_data_area, video_columns));
    ATF_CHECK_EQ(0x4C, offsetof(struct bios_data_area, video_regen_buflen));
    ATF_CHECK_EQ(0x4E, offsetof(struct bios_data_area, video_page_offset));
    ATF_CHECK_EQ(0x50, offsetof(struct bios_data_area, video_page_cursor));
    ATF_CHECK_EQ(0x60, offsetof(struct bios_data_area, video_cursor_bottom));
    ATF_CHECK_EQ(0x61, offsetof(struct bios_data_area, video_cursor_top));
    ATF_CHECK_EQ(0x62, offsetof(struct bios_data_area, video_page));

    ATF_CHECK_EQ(0x63, offsetof(struct bios_data_area, crt_base_addr));
    ATF_CHECK_EQ(0x65, offsetof(struct bios_data_area, crt_control_reg_val));
    ATF_CHECK_EQ(0x66, offsetof(struct bios_data_area, cga_pallette_mask));

    ATF_CHECK_EQ(0x67, offsetof(struct bios_data_area, cassete_tape_ctrl));

    ATF_CHECK_EQ(0x6C, offsetof(struct bios_data_area, daily_timer_counter));
    ATF_CHECK_EQ(0x70, offsetof(struct bios_data_area, daily_timer_rollover));

    ATF_CHECK_EQ(0x71, offsetof(struct bios_data_area, bios_break_flag));
    ATF_CHECK_EQ(0x72, offsetof(struct bios_data_area, soft_reset_flag));

    ATF_CHECK_EQ(0x74, offsetof(struct bios_data_area, hd_laststatus));
    ATF_CHECK_EQ(0x75, offsetof(struct bios_data_area, hd_count));
    ATF_CHECK_EQ(0x76, offsetof(struct bios_data_area, hd_ctrl_xt));
    ATF_CHECK_EQ(0x77, offsetof(struct bios_data_area, hd_adapter_offset));

    ATF_CHECK_EQ(0x78, offsetof(struct bios_data_area, com_timeout));
    ATF_CHECK_EQ(0x7C, offsetof(struct bios_data_area, lpt_timeout));

    ATF_CHECK_EQ(0x80, offsetof(struct bios_data_area, kbd_buf_start_offset));
    ATF_CHECK_EQ(0x82, offsetof(struct bios_data_area, kbd_buf_end_offset));

    ATF_CHECK_EQ(0x84, offsetof(struct bios_data_area, rows_visible));
    ATF_CHECK_EQ(0x85, offsetof(struct bios_data_area, point_height));
    ATF_CHECK_EQ(0x87, offsetof(struct bios_data_area, video_mode_options));
    ATF_CHECK_EQ(0x88, offsetof(struct bios_data_area, video_mode_features));
    ATF_CHECK_EQ(0x89, offsetof(struct bios_data_area, video_display_data));
    ATF_CHECK_EQ(0x8A, offsetof(struct bios_data_area, dcc_table_index));

    ATF_CHECK_EQ(0x8B, offsetof(struct bios_data_area, last_diskette_datarate));
    ATF_CHECK_EQ(0x8C, offsetof(struct bios_data_area, hd_status));
    ATF_CHECK_EQ(0x8D, offsetof(struct bios_data_area, hd_error));
    ATF_CHECK_EQ(0x8E, offsetof(struct bios_data_area, hd_interrupt_ctl));
    ATF_CHECK_EQ(0x8F, offsetof(struct bios_data_area, hd_floppy_combi));
    ATF_CHECK_EQ(0x90, offsetof(struct bios_data_area, media_state));
    ATF_CHECK_EQ(0x94, offsetof(struct bios_data_area, track_seeked_drive0));
    ATF_CHECK_EQ(0x95, offsetof(struct bios_data_area, track_seeked_drive1));

    ATF_CHECK_EQ(0x96, offsetof(struct bios_data_area, kbd_mode_type));
    ATF_CHECK_EQ(0x97, offsetof(struct bios_data_area, kbd_led_flags));

    ATF_CHECK_EQ(0x98, offsetof(struct bios_data_area, uwait_complete));
    ATF_CHECK_EQ(0x9C, offsetof(struct bios_data_area, uwait_timeout));

    ATF_CHECK_EQ(0xA0, offsetof(struct bios_data_area, rtc_wait_flag));

    ATF_CHECK_EQ(0xA1, offsetof(struct bios_data_area, lana_dma_chan_flags));
    ATF_CHECK_EQ(0xA2, offsetof(struct bios_data_area, lana_status));
    ATF_CHECK_EQ(0xA4, offsetof(struct bios_data_area, saved_hd_iv));
    ATF_CHECK_EQ(0xA8, offsetof(struct bios_data_area, bios_video_save));

    ATF_CHECK_EQ(0xB4, offsetof(struct bios_data_area, kbd_nmi_ctl_flags));
    ATF_CHECK_EQ(0xB5, offsetof(struct bios_data_area, kbd_break_pending));
    ATF_CHECK_EQ(0xB9, offsetof(struct bios_data_area, port60_queue));

    ATF_CHECK_EQ(0xBA, offsetof(struct bios_data_area, last_scancode));
    ATF_CHECK_EQ(0xBB, offsetof(struct bios_data_area, nmi_head_ptr));
    ATF_CHECK_EQ(0xBC, offsetof(struct bios_data_area, nmi_tail_ptr));
    ATF_CHECK_EQ(0xBD, offsetof(struct bios_data_area, nmi_scancode_buf));

    ATF_CHECK_EQ(0xCE, offsetof(struct bios_data_area, day_counter));

    printf("0x%x\n", offsetof(struct bios_data_area, day_counter));

    ATF_CHECK_EQ(0xF0, offsetof(struct bios_data_area, intra_applc_comm));
    printf("0x%x\n", offsetof(struct bios_data_area, intra_applc_comm));
}

ATF_TP_ADD_TCS(tp)
{
    ATF_TP_ADD_TC(tp, bda_offsets);

    return atf_no_error();
}
