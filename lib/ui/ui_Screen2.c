// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.4.1
// LVGL version: 8.3.11
// Project name: SquareLine_Project

#include "ui.h"

void ui_Screen2_screen_init(void)
{
    ui_Screen2 = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_Screen2, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Screen2, lv_color_hex(0xE60B0B), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Screen2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_color(ui_Screen2, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui_Screen2, LV_GRAD_DIR_VER, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Screen2_Dropdown2 = lv_dropdown_create(ui_Screen2);
    lv_obj_set_width(ui_Screen2_Dropdown2, 219);
    lv_obj_set_height(ui_Screen2_Dropdown2, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Screen2_Dropdown2, -230);
    lv_obj_set_y(ui_Screen2_Dropdown2, -155);
    lv_obj_set_align(ui_Screen2_Dropdown2, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Screen2_Dropdown2, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags



    ui_Screen2_Button8 = lv_btn_create(ui_Screen2);
    lv_obj_set_width(ui_Screen2_Button8, 100);
    lv_obj_set_height(ui_Screen2_Button8, 50);
    lv_obj_set_x(ui_Screen2_Button8, -288);
    lv_obj_set_y(ui_Screen2_Button8, -90);
    lv_obj_set_align(ui_Screen2_Button8, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Screen2_Button8, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_Screen2_Button8, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Screen2_Button8, lv_color_hex(0xAC0808), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Screen2_Button8, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_color(ui_Screen2_Button8, lv_color_hex(0x940400), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui_Screen2_Button8, LV_GRAD_DIR_VER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_Screen2_Button8, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_Screen2_Button8, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_Screen2_Button8, 2, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Screen2_Label11 = lv_label_create(ui_Screen2_Button8);
    lv_obj_set_width(ui_Screen2_Label11, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Screen2_Label11, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Screen2_Label11, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Screen2_Label11, "SCAN");

    ui_Screen2_Button9 = lv_btn_create(ui_Screen2);
    lv_obj_set_width(ui_Screen2_Button9, 100);
    lv_obj_set_height(ui_Screen2_Button9, 50);
    lv_obj_set_x(ui_Screen2_Button9, -171);
    lv_obj_set_y(ui_Screen2_Button9, -89);
    lv_obj_set_align(ui_Screen2_Button9, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Screen2_Button9, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_Screen2_Button9, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Screen2_Button9, lv_color_hex(0xAC0808), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Screen2_Button9, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_color(ui_Screen2_Button9, lv_color_hex(0x940400), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_Screen2_Button9, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_Screen2_Button9, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_Screen2_Button9, 2, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Screen2_Label12 = lv_label_create(ui_Screen2_Button9);
    lv_obj_set_width(ui_Screen2_Label12, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Screen2_Label12, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Screen2_Label12, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Screen2_Label12, "CONNECT");

    ui_Screen2_Button10 = lv_btn_create(ui_Screen2);
    lv_obj_set_width(ui_Screen2_Button10, 212);
    lv_obj_set_height(ui_Screen2_Button10, 86);
    lv_obj_set_x(ui_Screen2_Button10, -229);
    lv_obj_set_y(ui_Screen2_Button10, -10);
    lv_obj_set_align(ui_Screen2_Button10, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Screen2_Button10, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_Screen2_Button10, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Screen2_Button10, lv_color_hex(0x8B0400), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Screen2_Button10, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_color(ui_Screen2_Button10, lv_color_hex(0x620400), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui_Screen2_Button10, LV_GRAD_DIR_VER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_Screen2_Button10, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_Screen2_Button10, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_Screen2_Button10, 2, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Screen2_Label13 = lv_label_create(ui_Screen2_Button10);
    lv_obj_set_width(ui_Screen2_Label13, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Screen2_Label13, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Screen2_Label13, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Screen2_Label13, "CHECK FOR UPDATE");

    ui_Screen2_Panel3 = lv_obj_create(ui_Screen2);
    lv_obj_set_width(ui_Screen2_Panel3, 288);
    lv_obj_set_height(ui_Screen2_Panel3, 211);
    lv_obj_set_x(ui_Screen2_Panel3, 47);
    lv_obj_set_y(ui_Screen2_Panel3, -73);
    lv_obj_set_align(ui_Screen2_Panel3, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Screen2_Panel3, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Screen2_Panel3, lv_color_hex(0xCA0808), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Screen2_Panel3, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_color(ui_Screen2_Panel3, lv_color_hex(0x6A0400), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui_Screen2_Panel3, LV_GRAD_DIR_VER, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Screen2_Keyboard1 = lv_keyboard_create(ui_Screen2);
    lv_obj_set_width(ui_Screen2_Keyboard1, 797);
    lv_obj_set_height(ui_Screen2_Keyboard1, 192);
    lv_obj_set_x(ui_Screen2_Keyboard1, -2);
    lv_obj_set_y(ui_Screen2_Keyboard1, 140);
    lv_obj_set_align(ui_Screen2_Keyboard1, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Screen2_Keyboard1, LV_OBJ_FLAG_HIDDEN);     /// Flags

    ui_Screen2_Button11 = lv_btn_create(ui_Screen2_Keyboard1);
    lv_obj_set_width(ui_Screen2_Button11, 100);
    lv_obj_set_height(ui_Screen2_Button11, 50);
    lv_obj_set_x(ui_Screen2_Button11, 335);
    lv_obj_set_y(ui_Screen2_Button11, 72);
    lv_obj_set_align(ui_Screen2_Button11, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Screen2_Button11, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_Screen2_Button11, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Screen2_Label14 = lv_label_create(ui_Screen2_Button11);
    lv_obj_set_width(ui_Screen2_Label14, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Screen2_Label14, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Screen2_Label14, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Screen2_Label14, "DONE");

    ui_Screen2_TextArea2 = lv_textarea_create(ui_Screen2);
    lv_obj_set_width(ui_Screen2_TextArea2, 262);
    lv_obj_set_height(ui_Screen2_TextArea2, 70);
    lv_obj_set_x(ui_Screen2_TextArea2, 45);
    lv_obj_set_y(ui_Screen2_TextArea2, -133);
    lv_obj_set_align(ui_Screen2_TextArea2, LV_ALIGN_CENTER);
    lv_textarea_set_placeholder_text(ui_Screen2_TextArea2, "Enter WIFI password here");



    ui_Screen2_Button12 = lv_btn_create(ui_Screen2);
    lv_obj_set_width(ui_Screen2_Button12, 114);
    lv_obj_set_height(ui_Screen2_Button12, 50);
    lv_obj_set_x(ui_Screen2_Button12, 328);
    lv_obj_set_y(ui_Screen2_Button12, -211);
    lv_obj_set_align(ui_Screen2_Button12, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Screen2_Button12, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_Screen2_Button12, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Screen2_Button12, lv_color_hex(0xDE0808), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Screen2_Button12, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_color(ui_Screen2_Button12, lv_color_hex(0xC50808), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_Screen2_Button12, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_Screen2_Button12, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_Screen2_Button12, 2, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Screen2_Label15 = lv_label_create(ui_Screen2_Button12);
    lv_obj_set_width(ui_Screen2_Label15, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Screen2_Label15, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Screen2_Label15, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Screen2_Label15, "BACK TO\nMAIN MENUE");

    ui_Screen2_Panel5 = lv_obj_create(ui_Screen2);
    lv_obj_set_width(ui_Screen2_Panel5, 169);
    lv_obj_set_height(ui_Screen2_Panel5, 211);
    lv_obj_set_x(ui_Screen2_Panel5, 296);
    lv_obj_set_y(ui_Screen2_Panel5, -72);
    lv_obj_set_align(ui_Screen2_Panel5, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Screen2_Panel5, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Screen2_Panel5, lv_color_hex(0xCD0808), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Screen2_Panel5, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_color(ui_Screen2_Panel5, lv_color_hex(0x630400), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui_Screen2_Panel5, LV_GRAD_DIR_VER, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Screen2_Label19 = lv_label_create(ui_Screen2_Panel5);
    lv_obj_set_width(ui_Screen2_Label19, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Screen2_Label19, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Screen2_Label19, -49);
    lv_obj_set_y(ui_Screen2_Label19, -81);
    lv_obj_set_align(ui_Screen2_Label19, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Screen2_Label19, "LOGS:");
    lv_obj_set_style_text_color(ui_Screen2_Label19, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Screen2_Label19, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Screen2_Label20 = lv_label_create(ui_Screen2_Panel5);
    lv_obj_set_width(ui_Screen2_Label20, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Screen2_Label20, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Screen2_Label20, -1);
    lv_obj_set_y(ui_Screen2_Label20, -70);
    lv_obj_set_align(ui_Screen2_Label20, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Screen2_Label20, "_______________________");
    lv_obj_set_style_text_color(ui_Screen2_Label20, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Screen2_Label20, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Screen2_Label21 = lv_label_create(ui_Screen2_Panel5);
    lv_obj_set_width(ui_Screen2_Label21, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Screen2_Label21, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Screen2_Label21, -20);
    lv_obj_set_y(ui_Screen2_Label21, -44);
    lv_obj_set_align(ui_Screen2_Label21, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Screen2_Label21, "LOG MESSAGE");
    lv_obj_set_style_text_color(ui_Screen2_Label21, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Screen2_Label21, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_add_event_cb(ui_Screen2_Button11, ui_event_Screen2_Button11, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_Screen2_TextArea2, ui_event_Screen2_TextArea2, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_Screen2_Label15, ui_event_Screen2_Label15, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_Screen2_Button12, ui_event_Screen2_Button12, LV_EVENT_ALL, NULL);

}