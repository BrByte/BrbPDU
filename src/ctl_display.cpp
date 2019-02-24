/*
 * ctl_display.cpp
 *
 *  Created on: 2019-02-18
 *      Author: Luiz Fernando Souza Softov <softov@brbyte.com>
 *      Author: Guilherme Amorim de Oliveira Alves <guilherme@brbyte.com>
 *
 * Copyright (c) 2018 BrByte Software (Oliveira Alves & Amorim LTDA)
 * Todos os direitos reservados. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "main.h"

typedef enum
{
    DISPLAY_SCREEN_CORE,
    DISPLAY_SCREEN_RS485,
    DISPLAY_SCREEN_INFO,
    DISPLAY_SCREEN_SUPPLY,
    DISPLAY_SCREEN_TEMP,
    DISPLAY_SCREEN_LASTITEM
} BrbDisplayScreen;

static BrbGenericCBH BrbCtlDisplay_Timer;

static BrbGenericCBH BrbCtlDisplay_ScreenCore;
static BrbGenericCBH BrbCtlDisplay_ScreenRS485;
static BrbGenericCBH BrbCtlDisplay_ScreenInfo;
static BrbGenericCBH BrbCtlDisplay_ScreenSupply;
static BrbGenericCBH BrbCtlDisplay_ScreenTemp;

static const BrbDisplayScreenPrototype glob_display_screen_prototype[] =
    {
        DISPLAY_SCREEN_CORE,
        "CORE",
        BrbCtlDisplay_ScreenCore,

        DISPLAY_SCREEN_RS485,
        "RS485",
        BrbCtlDisplay_ScreenRS485,

        DISPLAY_SCREEN_INFO,
        "INFO",
        BrbCtlDisplay_ScreenInfo,

        DISPLAY_SCREEN_SUPPLY,
        "SUPPLY",
        BrbCtlDisplay_ScreenSupply,

        DISPLAY_SCREEN_TEMP,
        "TEMP",
        BrbCtlDisplay_ScreenTemp,

        DISPLAY_SCREEN_LASTITEM,
        NULL,
        NULL,
};
/**********************************************************************************************************************/
int BrbCtlDisplay_Setup(BrbBase *brb_base)
{
    BrbDisplayBase *display_base = (BrbDisplayBase *)&glob_display_base;

    /* Clean up base */
    memset(&glob_display_base, 0, sizeof(BrbDisplayBase));

    display_base->brb_base = brb_base;
    // display_base->screen_cur = DISPLAY_SCREEN_CORE;
    display_base->screen_cur = DISPLAY_SCREEN_INFO;
    // display_base->screen_cur = DISPLAY_SCREEN_SUPPLY;
    // display_base->screen_cur = DISPLAY_SCREEN_TEMP;

    display_base->pin_led = TFT_LED;
    display_base->pin_cs = TFT_CS;
    display_base->pin_rst = TFT_RST;
    display_base->pin_dc = TFT_DC;
    display_base->pin_mosi = TFT_MOSI;
    display_base->pin_clk = TFT_CLK;
    display_base->pin_miso = TFT_MISO;

    display_base->screen_arr_ptr = (BrbDisplayScreenPrototype *)&glob_display_screen_prototype;
    display_base->screen_arr_cnt = DISPLAY_SCREEN_LASTITEM;

    // display_base->tft = (ILI9341_due *)&tft;
    display_base->tft = new ILI9341_due(display_base->pin_cs, display_base->pin_dc, display_base->pin_rst);
    // display_base->tft = new TFT_eSPI();

    BrbDisplayBase_Init(display_base);
    BrbDisplayBase_ScreenAction(display_base, -1);

    BrbTimerAdd(display_base->brb_base, 5000, 0, BrbCtlDisplay_Timer, display_base);

    return 0;
}
/**********************************************************************************************************************/
/* DISPLAY */
/**********************************************************************************************************************/
static int BrbCtlDisplay_Timer(void *base_ptr, void *cb_data_ptr)
{
    // BrbTimer *timer = (BrbTimer *)base_ptr;
    BrbDisplayBase *display_base = (BrbDisplayBase *)cb_data_ptr;
    BrbPDUBase *pdu_base = (BrbPDUBase *)&glob_pdu_base;

    BrbDisplayBase_ScreenAction(display_base, -1);

    // display_base->screen_cur++;

    int delay = 5000;

    // switch (pdu_base->state.code)
    // {
    // case PDU_STATE_START_INIT:
    // case PDU_STATE_START_DELAY:
    // case PDU_STATE_START_CHECK:
    // case PDU_STATE_STOP_INIT:
    // case PDU_STATE_STOP_DELAY:
    // case PDU_STATE_STOP_CHECK:
    // {
    //     delay = 1500;
    //     break;
    // }
    // case PDU_STATE_FAILURE:
    // {
    //     delay = 2500;
    //     break;
    // }
    // case PDU_STATE_RUNNING:
    // case PDU_STATE_NONE:
    // default:
    // {
    //     delay = 5000;
    //     break;
    // }
    // }

    BrbTimerAdd(&glob_brb_base, delay, 0, BrbCtlDisplay_Timer, display_base);

    return 0;
}
/**********************************************************************************************************************/
int BrbCtlDisplay_ScreenInfo(void *brb_base_ptr, void *display_base_ptr)
{
    BrbDisplayBase *display_base = (BrbDisplayBase *)display_base_ptr;
    BrbPDUBase *pdu_base = (BrbPDUBase *)&glob_pdu_base;

    int pos_x;
    int pos_y;

    const char *title_ptr = NULL;
    const char *text_ptr = NULL;
    int color;

    if (display_base->screen_cur != display_base->screen_last)
    {
        BrbDisplayBase_SetBg(display_base);
        BrbDisplayBase_SetTitle(display_base, PSTR("INFO"));
        display_base->tft->fillRect(DISPLAY_SZ_MARGIN, 89, 310, 1, ILI9341_WHITESMOKE);
    }

    // if (!display_base->flags.on_action && display_base->flags.on_select)
    // {
    //     display_base->flags.on_action = 1;
    //     display_base->action_code = -1;
    // }

    pos_x = DISPLAY_SZ_MARGIN;
    pos_y = DISPLAY_SZ_TITLE_H + (DISPLAY_SZ_MARGIN * 2);

    display_base->tft->fillRect(DISPLAY_SZ_MARGIN, pos_y, 310, 49, DISPLAY_COLOR_BG);

    color = BrbPDUBase_GetStateColor(pdu_base);
    title_ptr = BrbPDUBase_GetStateText(pdu_base);
    text_ptr = BrbPDUBase_GetFailureText(pdu_base);

    display_base->tft->setTextColor(color, DISPLAY_COLOR_BG);
    display_base->tft->setFont(DISPLAY_FONT_BOX_VALUE);
    display_base->tft->setTextScale(1);
    display_base->tft->cursorToXY(DISPLAY_SZ_MARGIN, pos_y);
    display_base->tft->println((const __FlashStringHelper *)title_ptr);

    if (text_ptr)
    {
        display_base->tft->setTextColor(DISPLAY_COLOR_TEXT_DEFAULT, DISPLAY_COLOR_BG);
        display_base->tft->setFont(DISPLAY_FONT_BOX_SUB);
        display_base->tft->setTextScale(1);
        display_base->tft->cursorToXY(DISPLAY_SZ_MARGIN, pos_y + 35);
        display_base->tft->print((const __FlashStringHelper *)text_ptr);
    }

    pos_x = DISPLAY_SZ_MARGIN;
    pos_y = pos_y + 52;

    display_base->box.text_color = color;
    display_base->tft->fillRect(pos_x, pos_y + 12, 320 - (DISPLAY_SZ_MARGIN * 2), 35, DISPLAY_COLOR_BG);
    BrbDisplayBase_BoxUptime(display_base, pos_x, pos_y, PSTR("State"), (pdu_base->state.delta / 1000));

    pos_x = DISPLAY_SZ_MARGIN;
    pos_y = pos_y + 52;

    display_base->tft->fillRect(pos_x, pos_y + 12, 320 - (DISPLAY_SZ_MARGIN * 2), 35, DISPLAY_COLOR_BG);
    display_base->box.text_color = (pdu_base->sensor_power.value < PDU_POWER_MIN_VALUE) ? ILI9341_ORANGERED : ILI9341_SEAGREEN;
    BrbDisplayBase_BoxSub(display_base, pos_x, pos_y, PSTR("ENERGIA"), pdu_base->sensor_power.value, 1, PSTR("VAC"));

    display_base->box.text_color = (pdu_base->zero_power.value < PDU_POWER_MIN_HZ) ? ILI9341_ORANGERED : ILI9341_SEAGREEN;
    BrbDisplayBase_BoxSub(display_base, pos_x + 115, pos_y, PSTR("FREQ"), pdu_base->zero_power.value, 1, PSTR("Hz"));

    display_base->box.text_color = pdu_base->ms.power_delay > 0 ? ILI9341_SEAGREEN :  ILI9341_ORANGERED;
    BrbDisplayBase_BoxUptime(display_base, pos_x + 200, pos_y, PSTR("Uptime"), pdu_base->ms.power_delay / 1000);

    pos_x = DISPLAY_SZ_MARGIN;
    pos_y = pos_y + 52;

    display_base->tft->fillRect(pos_x, pos_y + 12, 320 - (DISPLAY_SZ_MARGIN * 2), 35, DISPLAY_COLOR_BG);
    display_base->box.text_color = (pdu_base->sensor_aux.value < PDU_AUX_MIN_VALUE) ? ILI9341_ORANGERED : ILI9341_SEAGREEN;
    BrbDisplayBase_BoxSub(display_base, pos_x, pos_y, PSTR("AUXILIAR"), pdu_base->sensor_aux.value, 1, PSTR("VAC"));

    display_base->box.text_color = (pdu_base->zero_aux.value < PDU_AUX_MIN_HZ) ? ILI9341_ORANGERED : ILI9341_SEAGREEN;
    BrbDisplayBase_BoxSub(display_base, pos_x + 115, pos_y, PSTR("FREQ"), pdu_base->zero_aux.value, 1, PSTR("Hz"));

    display_base->box.text_color = pdu_base->ms.aux_delay > 0 ? ILI9341_SEAGREEN :  ILI9341_ORANGERED;
    BrbDisplayBase_BoxUptime(display_base, pos_x + 200, pos_y, PSTR("Uptime"), pdu_base->ms.aux_delay / 1000);

    display_base->tft->setFont(DISPLAY_FONT_BOX_VALUE);
    display_base->tft->setTextScale(1);

    return 0;
}
/**********************************************************************************************************************/
int BrbCtlDisplay_ScreenSupply(void *brb_base_ptr, void *display_base_ptr)
{
    BrbDisplayBase *display_base = (BrbDisplayBase *)display_base_ptr;
    BrbPDUBase *pdu_base = (BrbPDUBase *)&glob_pdu_base;

    const char *title_ptr = NULL;
    const char *text_ptr = NULL;
    int color = ILI9341_ORANGERED;

    int pos_x;
    int pos_y;

    if (display_base->screen_cur != display_base->screen_last)
    {
        BrbDisplayBase_SetBg(display_base);
        BrbDisplayBase_SetTitle(display_base, PSTR("Supply"));
        display_base->tft->fillRect(DISPLAY_SZ_MARGIN, 89, 310, 1, ILI9341_WHITESMOKE);
    }

    pos_x = DISPLAY_SZ_MARGIN;
    pos_y = DISPLAY_SZ_TITLE_H + (DISPLAY_SZ_MARGIN * 2);

    display_base->tft->fillRect(DISPLAY_SZ_MARGIN, pos_y, 310, 49, DISPLAY_COLOR_BG);

    if ((pdu_base->sensor_sp01_in.value < 12.0) || (pdu_base->sensor_sp01_out.value < 5.0) || (pdu_base->sensor_sp01_out.value > 6.0))
    {
        title_ptr = PSTR("Trocar Fonte 01");
        text_ptr = PSTR("Verificar ou trocar equipamento");
        color = ILI9341_ORANGERED;
    }
    else if ((pdu_base->sensor_sp02_in.value < 12.0) || (pdu_base->sensor_sp02_out.value < 5.0) || (pdu_base->sensor_sp02_out.value > 6.0))
    {
        title_ptr = PSTR("Trocar Fonte 02");
        text_ptr = PSTR("Verificar ou trocar equipamento");
        color = ILI9341_ORANGERED;
    }
    else
    {
        title_ptr = PSTR("Ativo");
        text_ptr = PSTR("- - - -");
        color = ILI9341_SEAGREEN;
    }

    display_base->tft->setTextColor(color, DISPLAY_COLOR_BG);
    display_base->tft->setFont(DISPLAY_FONT_BOX_VALUE);
    display_base->tft->setTextScale(1);
    display_base->tft->cursorToXY(DISPLAY_SZ_MARGIN, pos_y);
    display_base->tft->println((const __FlashStringHelper *)title_ptr);

    if (text_ptr)
    {
        display_base->tft->setTextColor(DISPLAY_COLOR_TEXT_DEFAULT, DISPLAY_COLOR_BG);
        display_base->tft->setFont(DISPLAY_FONT_BOX_SUB);
        display_base->tft->setTextScale(1);
        display_base->tft->cursorToXY(DISPLAY_SZ_MARGIN, pos_y + 35);
        display_base->tft->print((const __FlashStringHelper *)text_ptr);
    }

    pos_x = DISPLAY_SZ_MARGIN;
    pos_y = 95;

    display_base->tft->fillRect(pos_x, pos_y + DISPLAY_SZ_BOX_H, 110, 30, DISPLAY_COLOR_BG);
    display_base->box.text_color = (pdu_base->sensor_sp01_in.value < 12.0) ? ILI9341_ORANGERED : ILI9341_MEDIUMSEAGREEN;
    BrbDisplayBase_BoxSub(display_base, pos_x, pos_y, PSTR("Fonte 01 - IN"), pdu_base->sensor_sp01_in.value, 1, PSTR("VDC"));

    display_base->tft->fillRect(pos_x + 160, pos_y + DISPLAY_SZ_BOX_H, 110, 30, DISPLAY_COLOR_BG);
    display_base->box.text_color = (pdu_base->sensor_sp01_out.value < 3.0) ? ILI9341_ORANGERED : ILI9341_MEDIUMSEAGREEN;
    BrbDisplayBase_BoxSub(display_base, pos_x + 160, pos_y, PSTR("Fonte 01 - OUT"), pdu_base->sensor_sp01_out.value, 1, PSTR("VDC"));

    pos_x = DISPLAY_SZ_MARGIN;
    pos_y = pos_y + 55;

    display_base->tft->fillRect(pos_x, pos_y + DISPLAY_SZ_BOX_H, 110, 30, ILI9341_WHITE);
    display_base->box.text_color = (pdu_base->sensor_sp02_in.value < 12.0) ? ILI9341_ORANGERED : ILI9341_MEDIUMSEAGREEN;
    BrbDisplayBase_BoxSub(display_base, pos_x, pos_y, PSTR("Fonte 02 - IN"), pdu_base->sensor_sp02_in.value, 1, PSTR("VDC"));

    display_base->tft->fillRect(pos_x + 160, pos_y + DISPLAY_SZ_BOX_H, 110, 30, ILI9341_WHITE);
    display_base->box.text_color = (pdu_base->sensor_sp02_out.value < 3.0) ? ILI9341_ORANGERED : ILI9341_MEDIUMSEAGREEN;
    BrbDisplayBase_BoxSub(display_base, pos_x + 160, pos_y, PSTR("Fonte 02 - OUT"), pdu_base->sensor_sp02_out.value, 1, PSTR("VDC"));

    return 0;
}
/**********************************************************************************************************************/
int BrbCtlDisplay_ScreenTemp(void *brb_base_ptr, void *display_base_ptr)
{
    BrbDisplayBase *display_base = (BrbDisplayBase *)display_base_ptr;
    BrbPDUBase *pdu_base = (BrbPDUBase *)&glob_pdu_base;

    int pos_x;
    int pos_y;

    if (display_base->screen_cur != display_base->screen_last)
    {
        BrbDisplayBase_SetBg(display_base);
        BrbDisplayBase_SetTitle(display_base, PSTR("Temperatura"));
    }

    pos_x = DISPLAY_SZ_MARGIN;
    pos_y = DISPLAY_SZ_TITLE_H + (DISPLAY_SZ_MARGIN * 2);

    BrbDisplayBase_DrawBarGraph(display_base, pos_x, pos_y, 130, pdu_base->dht_data.dht_temp, -50, 150);

    pos_x = 90;
    pos_y = DISPLAY_SZ_TITLE_H + (DISPLAY_SZ_MARGIN * 2);

    display_base->tft->fillRect(pos_x, pos_y + DISPLAY_SZ_BOX_H, 90, 30, DISPLAY_COLOR_BG);
    display_base->box.text_color = BrbDisplayBase_Rainbow(map(pdu_base->dht_data.dht_temp, -25, 100, 0, 127));
    BrbDisplayBase_BoxSub(display_base, pos_x, pos_y, PSTR("TEMP"), pdu_base->dht_data.dht_temp, 1, PSTR("C"));

    // BrbDisplayBase_DrawArcSeg(display_base, pdu_base->dht_data.dht_temp, 0, 130, pos_x, pos_y, 100, PSTR("Celsius"), DISPLAY_ARC_GREEN2RED, 0, 3, 5);

    pos_x = pos_x;
    pos_y = pos_y + 60;

    display_base->tft->fillRect(pos_x, pos_y + DISPLAY_SZ_BOX_H, 90, 30, DISPLAY_COLOR_BG);
    display_base->box.text_color = BrbDisplayBase_Rainbow(map(pdu_base->dht_data.dht_humi, -25, 100, 127, 0));
    BrbDisplayBase_BoxSub(display_base, pos_x, pos_y, PSTR("HUMIDADE"), pdu_base->dht_data.dht_humi, 1, PSTR("%"));

    pos_x = pos_x;
    pos_y = pos_y + 60;

    display_base->tft->fillRect(pos_x, pos_y + DISPLAY_SZ_BOX_H, 90, 30, DISPLAY_COLOR_BG);
    display_base->box.text_color = BrbDisplayBase_Rainbow(map(pdu_base->dht_data.dht_hidx, -25, 100, 0, 127));
    BrbDisplayBase_BoxSub(display_base, pos_x, pos_y, PSTR("HEAT INDEX"), pdu_base->dht_data.dht_hidx, 1, PSTR("C"));

    return 0;
}
/**********************************************************************************************************************/
int BrbCtlDisplay_ScreenCore(void *brb_base_ptr, void *display_base_ptr)
{
    BrbDisplayBase *display_base = (BrbDisplayBase *)display_base_ptr;
    BrbBase *brb_base = (BrbBase *)brb_base_ptr;

    int pos_x;
    int pos_y;

    if (display_base->screen_cur != display_base->screen_last)
    {
        BrbDisplayBase_SetBg(display_base);
        BrbDisplayBase_SetTitle(display_base, PSTR("Core"));
        display_base->tft->fillRect(DISPLAY_SZ_MARGIN, 89, 310, 1, ILI9341_WHITESMOKE);
    }

    pos_x = DISPLAY_SZ_MARGIN;
    pos_y = DISPLAY_SZ_TITLE_H + (DISPLAY_SZ_MARGIN * 2);

    display_base->tft->fillRect(pos_x, pos_y + DISPLAY_SZ_BOX_H, 310, 30, DISPLAY_COLOR_BG);
    display_base->box.text_color = ILI9341_MIDNIGHTBLUE;
    BrbDisplayBase_BoxUptime(display_base, pos_x, pos_y, PSTR("UpTime"), millis() / 1000);
    BrbDisplayBase_BoxUptime(display_base, pos_x + 140, pos_y, PSTR("LifeTime"), brb_base->data.lifetime_sec);

    pos_x = DISPLAY_SZ_MARGIN;
    pos_y = pos_y + 55;

    display_base->tft->fillRect(pos_x, pos_y + DISPLAY_SZ_BOX_H, 310, 30, DISPLAY_COLOR_BG);
    BrbDisplayBase_BoxFmt(display_base, pos_x, pos_y, PSTR("Memoria"), PSTR("%d"), BrbBase_FreeRAM());
    // BrbDisplayBase_BoxSub(display_base, pos_x + 160, pos_y, PSTR("Memoria"), BrbBase_FreeRAM(), 1, PSTR("%"));

    // pos_x = DISPLAY_SZ_MARGIN;
    // pos_y = pos_y + 55;

    // display_base->tft->fillRect(pos_x, pos_y + DISPLAY_SZ_BOX_H, 310, 30, DISPLAY_COLOR_BG);
    // BrbDisplayBase_BoxFmt(display_base, pos_x, pos_y, PSTR("RX/TX"), PSTR("%lu/%lu"), rs485_sess->stats.byte.rx, rs485_sess->stats.byte.tx);
    // BrbDisplayBase_BoxUnit(display_base, pos_x, pos_y + DISPLAY_SZ_BOX_H, PSTR("KB"));

    // BrbDisplayBase_BoxFmt(display_base, pos_x + 160, pos_y, PSTR("Error"), PSTR("%lu/%lu"),
    //                       (rs485_sess->stats.err.bad_char + rs485_sess->stats.err.crc + rs485_sess->stats.err.overflow + rs485_sess->stats.err.pkt),
    //                       (rs485_sess->stats.pkt.err.cmd_id + rs485_sess->stats.pkt.err.cmd_no_bcast + rs485_sess->stats.pkt.err.cmd_no_cb));

    return 0;
}
/**********************************************************************************************************************/
int BrbCtlDisplay_ScreenRS485(void *brb_base_ptr, void *display_base_ptr)
{
    BrbDisplayBase *display_base = (BrbDisplayBase *)display_base_ptr;
    BrbPDUBase *pdu_base = (BrbPDUBase *)&glob_pdu_base;
    BrbBase *brb_base = (BrbBase *)brb_base_ptr;
    BrbRS485Session *rs485_sess = (BrbRS485Session *)&glob_rs485_sess;

    int pos_x;
    int pos_y;

    if (display_base->screen_cur != display_base->screen_last)
    {
        BrbDisplayBase_SetBg(display_base);
        BrbDisplayBase_SetTitle(display_base, PSTR("RS485"));
        display_base->tft->fillRect(DISPLAY_SZ_MARGIN, 89, 310, 1, ILI9341_WHITESMOKE);
    }

    pos_x = DISPLAY_SZ_MARGIN;
    pos_y = DISPLAY_SZ_TITLE_H + (DISPLAY_SZ_MARGIN * 2);

    display_base->tft->fillRect(pos_x, pos_y + DISPLAY_SZ_BOX_H, 110, 30, DISPLAY_COLOR_BG);
    display_base->box.text_color = ILI9341_MIDNIGHTBLUE;

    display_base->tft->fillRect(pos_x, pos_y + DISPLAY_SZ_BOX_H, 310, 30, DISPLAY_COLOR_BG);
    BrbDisplayBase_BoxFmt(display_base, pos_x, pos_y, PSTR("UUID"), PSTR("%02x-%02x-%02x-%02x"), rs485_sess->data.uuid[0], rs485_sess->data.uuid[1], rs485_sess->data.uuid[2], rs485_sess->data.uuid[2]);
    BrbDisplayBase_BoxFmt(display_base, pos_x + 190, pos_y, PSTR("ADDR"), PSTR("%02x"), rs485_sess->data.address);

    pos_x = DISPLAY_SZ_MARGIN;
    pos_y = pos_y + 55;

    display_base->tft->fillRect(pos_x, pos_y + DISPLAY_SZ_BOX_H, 310, 30, DISPLAY_COLOR_BG);
    BrbDisplayBase_BoxFmt(display_base, pos_x, pos_y, PSTR("Packet RX/TX"), PSTR("%lu/%lu"), rs485_sess->stats.pkt.rx, rs485_sess->stats.pkt.tx);
    BrbDisplayBase_BoxFmt(display_base, pos_x + 130, pos_y, PSTR("Me"), PSTR("%lu"), rs485_sess->stats.pkt.me);
    BrbDisplayBase_BoxFmt(display_base, pos_x + 190, pos_y, PSTR("Broadcast"), PSTR("%lu"), rs485_sess->stats.pkt.bcast);

    pos_x = DISPLAY_SZ_MARGIN;
    pos_y = pos_y + 55;
    display_base->tft->fillRect(pos_x, pos_y + DISPLAY_SZ_BOX_H, 310, 30, DISPLAY_COLOR_BG);
    //
    // BrbDisplayBase_BoxFmt(display_base, pos_x + 190, pos_y, PSTR("Pkt Error"), PSTR("%lu/%lu/%lu"), rs485_sess->stats.pkt.err.cmd_id, rs485_sess->stats.pkt.err.cmd_no_bcast, rs485_sess->stats.pkt.err.cmd_no_cb);
    // BrbDisplayBase_BoxFmt(display_base, pos_x + 190, pos_y, PSTR("Pkt Error"), PSTR("%lu/%lu/%lu"), rs485_sess->stats.pkt.err.cmd_id, rs485_sess->stats.pkt.err.cmd_no_bcast, rs485_sess->stats.pkt.err.cmd_no_cb);

    char byte_rx[16];
    char byte_tx[16];

    dtostrf((double)(rs485_sess->stats.byte.rx / 1024.0), 5, 2, byte_rx);
    dtostrf((double)(rs485_sess->stats.byte.tx / 1024.0), 5, 2, byte_tx);

    BrbDisplayBase_BoxFmt(display_base, pos_x, pos_y, PSTR("Bytes RX/TX"), PSTR("%s/%s"), byte_rx, byte_tx);
    BrbDisplayBase_BoxFmt(display_base, pos_x + 190, pos_y, PSTR("Error"), PSTR("%lu/%lu/%lu"), rs485_sess->stats.err.bad_char, rs485_sess->stats.err.crc, rs485_sess->stats.err.overflow);

    return 0;
}
/**********************************************************************************************************************/
