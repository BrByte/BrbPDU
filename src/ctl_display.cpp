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
    DISPLAY_SCREEN_RELAY,
    DISPLAY_SCREEN_TRANSISTOR,
    DISPLAY_SCREEN_LASTITEM
} BrbDisplayScreen;

typedef enum
{
    DISPLAY_INFO_ACTION_AUTO,
    DISPLAY_INFO_ACTION_TRANSFER,
    DISPLAY_INFO_ACTION_LASTITEM
} BrbDisplayInfoAction;

static BrbGenericCBH BrbCtlDisplay_Timer;

static BrbGenericCBH BrbCtlDisplay_ScreenCore;
static BrbGenericCBH BrbCtlDisplay_ScreenRS485;
static BrbGenericCBH BrbCtlDisplay_ScreenInfo;
static BrbGenericCBH BrbCtlDisplay_ScreenSupply;
static BrbGenericCBH BrbCtlDisplay_ScreenTemp;
static BrbGenericCBH BrbCtlDisplay_ScreenRelay;
static BrbGenericCBH BrbCtlDisplay_ScreenTransistor;

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

        DISPLAY_SCREEN_RELAY,
        "RELAY",
        BrbCtlDisplay_ScreenRelay,

        DISPLAY_SCREEN_TRANSISTOR,
        "TRANSISTOR",
        BrbCtlDisplay_ScreenTransistor,

        DISPLAY_SCREEN_LASTITEM,
        NULL,
        NULL,
};

static int BrbCtlDisplay_ScreenStart(BrbDisplayBase *display_base);

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
    // display_base->screen_cur = DISPLAY_SCREEN_RELAY;

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
    BrbCtlDisplay_ScreenStart(display_base);

    BrbTimerAdd(display_base->brb_base, 2500, 0, BrbCtlDisplay_Timer, display_base);

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

    switch (pdu_base->state.code)
    {
    case PDU_STATE_RUNNING_POWER:
    case PDU_STATE_RUNNING_AUX:
    {
        delay = 3000;
        break;
    }
    case PDU_STATE_TRANSF_P2A_DELAY:
    case PDU_STATE_TRANSF_A2P_DELAY:
    case PDU_STATE_FAILURE:
    {
        delay = 2000;
        break;
    }
    case PDU_STATE_NONE:
    default:
    {
        delay = 5000;
        break;
    }
    }

    BrbTimerAdd(&glob_brb_base, delay, 0, BrbCtlDisplay_Timer, display_base);

    // glob_display_base.tft->setSPIClockDivider(SPI_CLOCK_DIV8);
    // glob_display_base.tft->screenshotToConsole();
    // glob_display_base.tft->setSPIClockDivider(ILI9341_SPI_CLKDIVIDER);

    return 0;
}
/**********************************************************************************************************************/
static int BrbCtlDisplay_ScreenStart(BrbDisplayBase *display_base)
{
    BrbDisplayBase_SetBg(display_base);

	display_base->tft->drawImage(brbyte_white, 100, 60, brbyte_whiteWidth, brbyte_whiteHeight);

    display_base->tft->setTextColor(ILI9341_BLACK, DISPLAY_COLOR_BG);
    display_base->tft->setFont(DISPLAY_FONT_TITLE);
    display_base->tft->setTextScale(1);
    display_base->tft->printAtPivoted((const __FlashStringHelper *)PSTR("PDU Control 1.0"), 160, 140, gTextPivotMiddleCenter);

    return 0;
}
/**********************************************************************************************************************/
int BrbCtlDisplay_ScreenInfo(void *brb_base_ptr, void *display_base_ptr)
{
    BrbDisplayBase *display_base = (BrbDisplayBase *)display_base_ptr;
    BrbPDUBase *pdu_base = (BrbPDUBase *)&glob_pdu_base;

    int pos_x;
    int pos_y;

    const char *icon_ptr = NULL;
    const char *title_ptr = NULL;
    const char *text_ptr = NULL;
    int color;

    if (!display_base->flags.on_action && display_base->flags.on_select)
    {
        display_base->flags.on_action = 1;
        display_base->action_code = DISPLAY_INFO_ACTION_LASTITEM;
        display_base->screen_last = -1;
    }

    if (display_base->screen_cur != display_base->screen_last)
    {
        BrbDisplayBase_SetBg(display_base);
        BrbDisplayBase_SetTitle(display_base, PSTR("INFO"));
        display_base->tft->fillRect(DISPLAY_SZ_MARGIN, 89, 310, 1, ILI9341_WHITESMOKE);
    }
    
    pos_x = DISPLAY_SZ_MARGIN;
    pos_y = DISPLAY_SZ_TITLE_H + (DISPLAY_SZ_MARGIN * 2);

    if (display_base->flags.on_action)
    {
        if (display_base->action_code == DISPLAY_ACTION_SELECT)
        {
            if (display_base->user_int >= 0 && display_base->user_int < DISPLAY_INFO_ACTION_LASTITEM)
            {
                switch (display_base->user_int)
                {
                    case DISPLAY_INFO_ACTION_AUTO:
                    {
                        BrbPDUBase_ActionCmd(&glob_pdu_base, !pdu_base->data.flags.auto_enabled ? PDU_ACTION_AUTO_ENABLE : PDU_ACTION_AUTO_DISABLE);

                        break;
                    }
                    case DISPLAY_INFO_ACTION_TRANSFER:
                    {
                        BrbPDUBase_ActionCmd(&glob_pdu_base, !pdu_base->data.flags.transfer_force ? PDU_ACTION_TRANSFER_ENABLE : PDU_ACTION_TRANSFER_DISABLE);

                        break;
                    }
                }
                
	            BrbPDUBase_Save(pdu_base);
            }
            else
            {
                display_base->flags.on_action = 0;
                display_base->user_int = 0;
                display_base->screen_last = -1;
            }

            return BrbDisplayBase_ScreenAction(display_base, -1);
        }
        else if (display_base->action_code == DISPLAY_ACTION_PREV)
        {
            display_base->user_int--;

            if (display_base->user_int < 0)
                display_base->user_int = DISPLAY_INFO_ACTION_LASTITEM;

            display_base->user_int = display_base->user_int % (DISPLAY_INFO_ACTION_LASTITEM + 1);
        }
        else if (display_base->action_code == DISPLAY_ACTION_NEXT)
        {
            display_base->user_int++;
            display_base->user_int = display_base->user_int % (DISPLAY_INFO_ACTION_LASTITEM + 1);
        }

        int i;

        for (i = 0; i < DISPLAY_INFO_ACTION_LASTITEM; i++)
        {
            const char *action_ptr;

            switch (i)
            {
                case DISPLAY_INFO_ACTION_AUTO:
                {
                    if (pdu_base->data.flags.auto_enabled)
                    {
                        action_ptr = PSTR("AUTO : ON");
                    }
                    else
                    {
                        action_ptr = PSTR("AUTO : OFF");
                    }

                    break;
                }
                case DISPLAY_INFO_ACTION_TRANSFER:
                {
                    if (pdu_base->data.flags.transfer_force)
                    {
                        action_ptr = PSTR("TRANSFER : ON");
                    }
                    else
                    {
                        action_ptr = PSTR("TRANSFER : OFF");
                    }

                    break;
                }
                default:
                    continue;
                    break;
            }
            
            display_base->tft->fillRect(pos_x, pos_y + (i * 50), 310, 45, display_base->user_int == i ? ILI9341_ORANGERED : ILI9341_LIGHTGREY);
            display_base->tft->setTextColor(display_base->user_int == i ? ILI9341_WHITE : ILI9341_BLACK, display_base->user_int == i ? ILI9341_ORANGERED : ILI9341_LIGHTGREY);
            display_base->tft->setFont(DISPLAY_FONT_BOX_VALUE);
            display_base->tft->setTextScale(1);
            display_base->tft->cursorToXY(pos_x + 20, pos_y + (i * 50) + 10);
            display_base->tft->println((const __FlashStringHelper *)action_ptr);
            
            continue;
        }

        pos_x = DISPLAY_SZ_MARGIN;
        pos_y = 195;

        display_base->tft->setTextColor(display_base->user_int >= DISPLAY_INFO_ACTION_LASTITEM ? ILI9341_ORANGERED : ILI9341_BLACK, DISPLAY_COLOR_BG);
        display_base->tft->setFont(DISPLAY_FONT_ICON2);
        display_base->tft->setTextScale(1);
        display_base->tft->cursorToXY(pos_x + 5, pos_y);
        display_base->tft->println((const __FlashStringHelper *)DISPLAY_FONT_ICON_C_R);
        display_base->tft->setFont(DISPLAY_FONT_BOX_VALUE);
        display_base->tft->setTextScale(1);
        display_base->tft->cursorToXY(pos_x + 40, pos_y + 5);
        display_base->tft->println((const __FlashStringHelper *)PSTR("SAIR"));
        display_base->tft->drawRect(pos_x, pos_y, 110, 40, display_base->user_int >= DISPLAY_INFO_ACTION_LASTITEM ? ILI9341_ORANGERED : ILI9341_BLACK);

        return 0;
    }

    display_base->tft->fillRect(DISPLAY_SZ_MARGIN, pos_y, 310, 49, DISPLAY_COLOR_BG);

    color = BrbPDUBase_GetStateColor(pdu_base);
    title_ptr = BrbPDUBase_GetStateText(pdu_base);
    icon_ptr = BrbPDUBase_GetStateIcon(pdu_base);
    text_ptr = BrbPDUBase_GetFailureText(pdu_base);

    display_base->tft->setTextColor(color, DISPLAY_COLOR_BG);
    display_base->tft->setFont(DISPLAY_FONT_ICON);
    display_base->tft->setTextScale(1);
    display_base->tft->cursorToXY(DISPLAY_SZ_MARGIN, pos_y - 5);
    display_base->tft->println((const __FlashStringHelper *)icon_ptr);


    display_base->tft->setTextColor(color, DISPLAY_COLOR_BG);
    display_base->tft->setFont(DISPLAY_FONT_BOX_VALUE);
    display_base->tft->setTextScale(1);
    display_base->tft->cursorToXY(DISPLAY_SZ_MARGIN + 30, pos_y);
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
    display_base->tft->fillRect(pos_x, pos_y + DISPLAY_SZ_BOX_H, 320 - (DISPLAY_SZ_MARGIN * 2), 35, DISPLAY_COLOR_BG);
    BrbDisplayBase_BoxUptime(display_base, pos_x, pos_y, PSTR("State"), ((millis() - pdu_base->state.ms_change) / 1000));

	if (display_base->screen_cur != display_base->screen_last)
	{
		BrbDisplayBase_BoxTitle(display_base, pos_x + 200, pos_y, PSTR("Transfer"));
	}

    display_base->tft->setTextColor(pdu_base->data.flags.auto_enabled ? ILI9341_SEAGREEN : ILI9341_LIGHTGREY, DISPLAY_COLOR_BG);
    display_base->tft->setFont(DISPLAY_FONT_ICON);
    display_base->tft->setTextScale(1);
    display_base->tft->cursorToXY(pos_x + 200, pos_y + 12);
    display_base->tft->println((const __FlashStringHelper *)(pdu_base->data.flags.auto_enabled ? DISPLAY_FONT_ICON_ACTIVE : DISPLAY_FONT_ICON_USER));

    display_base->tft->setTextColor(pdu_base->data.flags.transfer_force ? ILI9341_ORANGERED : ILI9341_LIGHTGREY, DISPLAY_COLOR_BG);
    display_base->tft->setFont(DISPLAY_FONT_ICON);
    display_base->tft->setTextScale(1);
    display_base->tft->cursorToXY(pos_x + 235, pos_y + 12);
    display_base->tft->println((const __FlashStringHelper *)(pdu_base->data.flags.transfer_force ? DISPLAY_FONT_ICON_P2A : DISPLAY_FONT_ICON_A2P));

    pos_x = DISPLAY_SZ_MARGIN;
    pos_y = pos_y + 52;

    display_base->tft->fillRect(pos_x, pos_y + DISPLAY_SZ_BOX_H, 320 - (DISPLAY_SZ_MARGIN * 2), 35, DISPLAY_COLOR_BG);
    display_base->box.text_color = (pdu_base->sensor_power.value < PDU_POWER_MIN_VALUE) ? ILI9341_ORANGERED : ILI9341_SEAGREEN;
    BrbDisplayBase_BoxSub(display_base, pos_x, pos_y, PSTR("ENERGIA"), pdu_base->sensor_power.value, 1, PSTR("VAC"));

    display_base->box.text_color = (pdu_base->zero_power.value < PDU_POWER_MIN_HZ) ? ILI9341_ORANGERED : ILI9341_SEAGREEN;
    BrbDisplayBase_BoxSub(display_base, pos_x + 115, pos_y, PSTR("FREQ"), pdu_base->zero_power.value, 1, PSTR("Hz"));

    display_base->box.text_color = pdu_base->ms.power_delay > 0 ? ILI9341_SEAGREEN :  ILI9341_ORANGERED;
    BrbDisplayBase_BoxUptime(display_base, pos_x + 200, pos_y, PSTR("Uptime"), pdu_base->ms.power_delay / 1000);

    pos_x = DISPLAY_SZ_MARGIN;
    pos_y = pos_y + 52;

    display_base->tft->fillRect(pos_x, pos_y + DISPLAY_SZ_BOX_H, 320 - (DISPLAY_SZ_MARGIN * 2), 35, DISPLAY_COLOR_BG);
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
int BrbCtlDisplay_ScreenRelay(void *brb_base_ptr, void *display_base_ptr)
{
    BrbDisplayBase *display_base = (BrbDisplayBase *)display_base_ptr;
    BrbPDUBase *pdu_base = (BrbPDUBase *)&glob_pdu_base;

    int pos_x;
    int pos_y;

    int relay_mode;
    int item_cnt = 8;
    int c;
    int i;

    if (display_base->screen_cur != display_base->screen_last)
    {
        BrbDisplayBase_SetBg(display_base);
        BrbDisplayBase_SetTitle(display_base, PSTR("RELAY"));
        display_base->tft->fillRect(DISPLAY_SZ_MARGIN, 89, 310, 1, ILI9341_TOMATO);
    }

    if (!display_base->flags.on_action && display_base->flags.on_select)
    {
        display_base->flags.on_action = 1;
        display_base->action_code = item_cnt;
    }

    pos_x = 0;
    pos_y = DISPLAY_SZ_TITLE_H + (DISPLAY_SZ_MARGIN * 2) + 35;

    if (display_base->flags.on_action)
    {
        if (display_base->action_code == DISPLAY_ACTION_SELECT)
        {            
            if (display_base->user_int >= 0 && display_base->user_int < item_cnt)
            {
                pinMode(PDU_RELAY_PIN + display_base->user_int, OUTPUT);
                digitalWrite(PDU_RELAY_PIN + display_base->user_int, !digitalRead(PDU_RELAY_PIN + display_base->user_int));
                
	            BrbPDUBase_Save(pdu_base);
            }
            else
            {
                display_base->flags.on_action = 0;
                display_base->user_int = -1;
            }

            return BrbDisplayBase_ScreenAction(display_base, -1);
        }
        else if (display_base->action_code == DISPLAY_ACTION_PREV)
        {
            display_base->user_int--;

            if (display_base->user_int < 0)
                display_base->user_int = item_cnt;

            display_base->user_int = display_base->user_int % (item_cnt + 1);
        }
        else if (display_base->action_code == DISPLAY_ACTION_NEXT)
        {
            display_base->user_int++;
            display_base->user_int = display_base->user_int % (item_cnt + 1);
        }
    }
    else
    {

    }

    for (i = 0; i < item_cnt; i++)
    {
        c = (i) % 4;

        if (i > 0 && c == 0)
        {
            pos_y += 80;
        }

        relay_mode = digitalRead(PDU_RELAY_PIN + i);
        display_base->tft->fillCircle(pos_x + (c * 80) + 40, pos_y, 30, DISPLAY_COLOR_BG);
        
        if (relay_mode == LOW)
        {
            display_base->tft->setTextColor(ILI9341_SEAGREEN, DISPLAY_COLOR_BG);
            display_base->tft->setFont(DISPLAY_FONT_ICON);
            display_base->tft->setTextScale(2);
            display_base->tft->printAtPivoted((const __FlashStringHelper *)DISPLAY_FONT_ICON_LAMP, pos_x + (c * 80) + 40, pos_y, gTextPivotMiddleCenter);
        }
        else 
        {
            display_base->tft->setTextColor(ILI9341_DARKGRAY, DISPLAY_COLOR_BG);
            display_base->tft->setFont(DISPLAY_FONT_ICON2);
            display_base->tft->setTextScale(2);
            display_base->tft->printAtPivoted((const __FlashStringHelper *)DISPLAY_FONT_ICON_C_OFF, pos_x + (c * 80) + 40, pos_y, gTextPivotMiddleCenter);
        }

        if (display_base->user_int == i && (display_base->flags.on_action))
        {
            display_base->tft->fillArc(pos_x + (c * 80) + 40, pos_y, 40, 7, 0, 360, ILI9341_ORANGERED);
        }
        else
        {
            display_base->tft->fillArc(pos_x + (c * 80) + 40, pos_y, 40, 7, 0, 360, DISPLAY_COLOR_BG);
        }        
    }

    pos_x = DISPLAY_SZ_MARGIN;
    pos_y = DISPLAY_SZ_TITLE_H + 170;

    display_base->tft->fillRect(DISPLAY_SZ_MARGIN, pos_y, 160, 40, DISPLAY_COLOR_BG);

    if (display_base->flags.on_action)
    {
        display_base->tft->setTextColor(display_base->user_int == item_cnt ? ILI9341_ORANGERED : ILI9341_BLACK, DISPLAY_COLOR_BG);
        display_base->tft->setFont(DISPLAY_FONT_ICON2);
        display_base->tft->setTextScale(1);
        display_base->tft->cursorToXY(pos_x + 5, pos_y);
        display_base->tft->println((const __FlashStringHelper *)DISPLAY_FONT_ICON_C_R);
        display_base->tft->setFont(DISPLAY_FONT_BOX_VALUE);
        display_base->tft->setTextScale(1);
        display_base->tft->cursorToXY(pos_x + 40, pos_y + 5);
        display_base->tft->println((const __FlashStringHelper *)PSTR("SAIR"));
        display_base->tft->drawRect(pos_x, pos_y, 110, 40, display_base->user_int == item_cnt ? ILI9341_ORANGERED : ILI9341_BLACK);
    }

    return 0;
}
/**********************************************************************************************************************/
int BrbCtlDisplay_ScreenTransistor(void *brb_base_ptr, void *display_base_ptr)
{
    BrbDisplayBase *display_base = (BrbDisplayBase *)display_base_ptr;
    BrbPDUBase *pdu_base = (BrbPDUBase *)&glob_pdu_base;

    int pos_x;
    int pos_y;

    int relay_mode;
    int relay_check;
    int item_cnt = 8;
    int c;
    int i;

    if (display_base->screen_cur != display_base->screen_last)
    {
        BrbDisplayBase_SetBg(display_base);
        BrbDisplayBase_SetTitle(display_base, PSTR("Transistor"));
        display_base->tft->fillRect(DISPLAY_SZ_MARGIN, 89, 310, 1, ILI9341_WHITESMOKE);
    }

    if (!display_base->flags.on_action && display_base->flags.on_select)
    {
        display_base->flags.on_action = 1;
        display_base->action_code = item_cnt;
    }

    pos_x = 0;
    pos_y = DISPLAY_SZ_TITLE_H + (DISPLAY_SZ_MARGIN * 2) + 35;

    if (display_base->flags.on_action)
    {
        if (display_base->action_code == DISPLAY_ACTION_SELECT)
        {            
            if (display_base->user_int >= 0 && display_base->user_int < item_cnt)
            {
                pinMode(PDU_TRANSISTOR_PIN + display_base->user_int, OUTPUT);
                digitalWrite(PDU_TRANSISTOR_PIN + display_base->user_int, !digitalRead(PDU_TRANSISTOR_PIN + display_base->user_int));
                
	            BrbPDUBase_Save(pdu_base);
            }
            else
            {
                display_base->flags.on_action = 0;
                display_base->user_int = -1;
            }

            return BrbDisplayBase_ScreenAction(display_base, -1);
        }
        else if (display_base->action_code == DISPLAY_ACTION_PREV)
        {
            display_base->user_int--;

            if (display_base->user_int < 0)
                display_base->user_int = item_cnt;

            display_base->user_int = display_base->user_int % (item_cnt + 1);
        }
        else if (display_base->action_code == DISPLAY_ACTION_NEXT)
        {
            display_base->user_int++;
            display_base->user_int = display_base->user_int % (item_cnt + 1);
        }
    }
    else
    {

    }

    for (i = 0; i < item_cnt; i++)
    {
        c = (i) % 4;

        if (i > 0 && c == 0)
        {
            pos_y += 80;
        }

        relay_mode = digitalRead(PDU_TRANSISTOR_PIN + i);
        display_base->tft->fillCircle(pos_x + (c * 80) + 40, pos_y, 30, DISPLAY_COLOR_BG);
        
        if (i == 2)
            relay_check = digitalRead(PDU_TTR_CHECK_PIN_2);
        else if (i == 3)
            relay_check = digitalRead(PDU_TTR_CHECK_PIN_3);
        else if (i == 4)
            relay_check = digitalRead(PDU_TTR_CHECK_PIN_4);
        else if (i == 5)
            relay_check = digitalRead(PDU_TTR_CHECK_PIN_5);
        else if (i == 6)
            relay_check = digitalRead(PDU_TTR_CHECK_PIN_6);
        else if (i == 7)
            relay_check = digitalRead(PDU_TTR_CHECK_PIN_7);
        else
            relay_check = !relay_mode;            

        if (relay_mode == HIGH)
        {
            display_base->tft->setTextColor((relay_check == LOW) ? ILI9341_SEAGREEN : ILI9341_ORANGERED, DISPLAY_COLOR_BG);
            display_base->tft->setFont(DISPLAY_FONT_ICON);
            display_base->tft->setTextScale(2);
            display_base->tft->printAtPivoted((const __FlashStringHelper *)(relay_check == LOW ? DISPLAY_FONT_ICON_LAMP : DISPLAY_FONT_ICON_ALERT), pos_x + (c * 80) + 40, pos_y - 5, gTextPivotMiddleCenter);            
        }
        else 
        {
            display_base->tft->setTextColor((relay_check != LOW) ? ILI9341_DARKGRAY : ILI9341_ORANGERED, DISPLAY_COLOR_BG);
            display_base->tft->setFont((relay_check != LOW ? DISPLAY_FONT_ICON2 : DISPLAY_FONT_ICON));
            display_base->tft->setTextScale(2);
            display_base->tft->printAtPivoted((const __FlashStringHelper *)(relay_check != LOW ? DISPLAY_FONT_ICON_C_OFF : DISPLAY_FONT_ICON_POWER), pos_x + (c * 80) + 40, pos_y - 5, gTextPivotMiddleCenter);
        }

        if (display_base->user_int == i && (display_base->flags.on_action))
        {
            display_base->tft->fillArc(pos_x + (c * 80) + 40, pos_y, 40, 5, 0, 360, ILI9341_ORANGERED);
        }
        else
        {
            display_base->tft->fillArc(pos_x + (c * 80) + 40, pos_y, 40, 5, 0, 360, DISPLAY_COLOR_BG);
        }
    }

    pos_x = DISPLAY_SZ_MARGIN;
    pos_y = DISPLAY_SZ_TITLE_H + 170;

    display_base->tft->fillRect(DISPLAY_SZ_MARGIN, pos_y, 160, 40, DISPLAY_COLOR_BG);

    if (display_base->flags.on_action)
    {
        display_base->tft->setTextColor(display_base->user_int == item_cnt ? ILI9341_ORANGERED : ILI9341_BLACK, DISPLAY_COLOR_BG);
        display_base->tft->setFont(DISPLAY_FONT_ICON2);
        display_base->tft->setTextScale(1);
        display_base->tft->cursorToXY(pos_x + 5, pos_y);
        display_base->tft->println((const __FlashStringHelper *)DISPLAY_FONT_ICON_C_R);
        display_base->tft->setFont(DISPLAY_FONT_BOX_VALUE);
        display_base->tft->setTextScale(1);
        display_base->tft->cursorToXY(pos_x + 40, pos_y + 5);
        display_base->tft->println((const __FlashStringHelper *)PSTR("SAIR"));
        display_base->tft->drawRect(pos_x, pos_y, 110, 40, display_base->user_int == item_cnt ? ILI9341_ORANGERED : ILI9341_BLACK);
    }

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

    if (pdu_base->sensor_sp01_in.value < 23.0)
    {
        title_ptr = PSTR("Verificar Fonte 01");
        text_ptr = PSTR("Equipamento abaixo de 24VDC");
        color = ILI9341_ORANGERED;
    }
    else if (pdu_base->sensor_sp01_out.value < 4.9)
    {
        title_ptr = PSTR("Verificar Fonte 01");
        text_ptr = PSTR("Equipamento abaixo de 5VDC");
        color = ILI9341_ORANGERED;
    }
    else if (pdu_base->sensor_sp01_out.value > 5.7)
    {
        title_ptr = PSTR("Verificar Fonte 01");
        text_ptr = PSTR("Equipamento acima de 5.7VDC");
        color = ILI9341_ORANGERED;
    }
    else if (pdu_base->sensor_sp02_in.value < 23.0)
    {
        title_ptr = PSTR("Verificar Fonte 02");
        text_ptr = PSTR("Equipamento abaixo de 24VDC");
        color = ILI9341_ORANGERED;
    }
    else if (pdu_base->sensor_sp02_out.value < 4.9)
    {
        title_ptr = PSTR("Verificar Fonte 02");
        text_ptr = PSTR("Equipamento abaixo de 5VDC");
        color = ILI9341_ORANGERED;
    }
    else if (pdu_base->sensor_sp02_out.value > 5.7)
    {
        title_ptr = PSTR("Verificar Fonte 02");
        text_ptr = PSTR("Equipamento acima de 5.7VDC");
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

    BrbDisplayBase_DrawBarGraph(display_base, pos_x, pos_y, 140, pdu_base->dht_data.dht_temp, -50, 120);

    pos_x = 125;
    pos_y = DISPLAY_SZ_TITLE_H + (DISPLAY_SZ_MARGIN * 2);
    display_base->tft->fillRect(pos_x, pos_y + DISPLAY_SZ_BOX_H, 90, 30, DISPLAY_COLOR_BG);
    display_base->box.text_color = BrbDisplayBase_Rainbow(map(pdu_base->dht_data.dht_temp, -25, 80, 0, 127));
    BrbDisplayBase_BoxSub(display_base, pos_x, pos_y, PSTR("TEMP"), pdu_base->dht_data.dht_temp, 1, PSTR("C"));

    pos_x = 90;
    display_base->tft->setTextColor(display_base->box.text_color, DISPLAY_COLOR_BG);
    display_base->tft->setFont(DISPLAY_FONT_ICON2);
    display_base->tft->setTextScale(1);
    display_base->tft->cursorToXY(pos_x, pos_y + DISPLAY_SZ_BOX_H);

    if (pdu_base->dht_data.dht_temp > 50)
        display_base->tft->println((const __FlashStringHelper *)PSTR("\""));
    else if (pdu_base->dht_data.dht_temp > 30)
        display_base->tft->println((const __FlashStringHelper *)PSTR("\xda"));
    else if (pdu_base->dht_data.dht_temp < 20)
        display_base->tft->println((const __FlashStringHelper *)PSTR("\xcc"));
    else
        display_base->tft->println((const __FlashStringHelper *)PSTR("\xec"));

    // BrbDisplayBase_DrawArcSeg(display_base, pdu_base->dht_data.dht_temp, 0, 130, pos_x, pos_y, 100, PSTR("Celsius"), DISPLAY_ARC_GREEN2RED, 0, 3, 5);

    pos_x = 125;
    pos_y = pos_y + 55;
    display_base->tft->fillRect(pos_x, pos_y + DISPLAY_SZ_BOX_H, 90, 30, DISPLAY_COLOR_BG);
    display_base->box.text_color = BrbDisplayBase_Rainbow(map(pdu_base->dht_data.dht_humi, -25, 100, 127, 0));
    BrbDisplayBase_BoxSub(display_base, pos_x, pos_y, PSTR("HUMIDADE"), pdu_base->dht_data.dht_humi, 1, PSTR("%"));

    pos_x = 90;
    display_base->tft->setTextColor(display_base->box.text_color, DISPLAY_COLOR_BG);
    display_base->tft->setFont(DISPLAY_FONT_ICON2);
    display_base->tft->setTextScale(1);
    display_base->tft->cursorToXY(pos_x, pos_y + DISPLAY_SZ_BOX_H);
    display_base->tft->println((const __FlashStringHelper *)PSTR("\xcb"));

    pos_x = 125;
    pos_y = pos_y + 55; 
    display_base->tft->fillRect(pos_x, pos_y + DISPLAY_SZ_BOX_H, 90, 30, DISPLAY_COLOR_BG);
    display_base->box.text_color = BrbDisplayBase_Rainbow(map(pdu_base->dht_data.dht_hidx, -25, 100, 0, 127));
    BrbDisplayBase_BoxSub(display_base, pos_x, pos_y, PSTR("HEAT INDEX"), pdu_base->dht_data.dht_hidx, 1, PSTR("C"));

    pos_x = 90;
    display_base->tft->setTextColor(display_base->box.text_color, DISPLAY_COLOR_BG);
    display_base->tft->setFont(DISPLAY_FONT_ICON2);
    display_base->tft->setTextScale(1);
    display_base->tft->cursorToXY(pos_x, pos_y + DISPLAY_SZ_BOX_H);
    display_base->tft->println((const __FlashStringHelper *)PSTR("P"));

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
    BrbDisplayBase_BoxFmt(display_base, pos_x + 180, pos_y, PSTR("Memoria"), PSTR("%d"), BrbBase_FreeRAM());

    pos_x = DISPLAY_SZ_MARGIN;
    pos_y = pos_y + 50;

    display_base->tft->fillRect(pos_x, pos_y + DISPLAY_SZ_BOX_H, 310, 30, DISPLAY_COLOR_BG);
    BrbDisplayBase_BoxUptime(display_base, pos_x, pos_y, PSTR("LifeTime"), brb_base->data.lifetime_sec);
    BrbDisplayBase_BoxFmt(display_base, pos_x + 180, pos_y, PSTR("UpCount"), PSTR("%d"), brb_base->data.upcount);

    pos_x = DISPLAY_SZ_MARGIN;
    pos_y = pos_y + 50;

    display_base->tft->fillRect(pos_x, pos_y + DISPLAY_SZ_BOX_H, 310, 30, DISPLAY_COLOR_BG);
    BrbDisplayBase_BoxFmt(display_base, pos_x, pos_y, PSTR("Delay"), PSTR("%d"), brb_base->ms.delay);
    BrbDisplayBase_BoxFmt(display_base, pos_x + 160, pos_y, PSTR("Loop"), PSTR("%d"), brb_base->stats.loop_cnt);

    return 0;
}
/**********************************************************************************************************************/
int BrbCtlDisplay_ScreenRS485(void *brb_base_ptr, void *display_base_ptr)
{
    BrbDisplayBase *display_base = (BrbDisplayBase *)display_base_ptr;
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
    BrbDisplayBase_BoxFmt(display_base, pos_x + 160, pos_y, PSTR("ADDR"), PSTR("%02x"), rs485_sess->data.address);

    pos_x = DISPLAY_SZ_MARGIN;
    pos_y = pos_y + 50;
    
    display_base->tft->fillRect(pos_x, pos_y + DISPLAY_SZ_BOX_H, 310, 30, DISPLAY_COLOR_BG);

    char byte_rx[16];
    char byte_tx[16];

    dtostrf((double)(rs485_sess->stats.byte.rx / 1024.0), 3, 2, byte_rx);
    dtostrf((double)(rs485_sess->stats.byte.tx / 1024.0), 3, 2, byte_tx);

    BrbDisplayBase_BoxFmt(display_base, pos_x, pos_y, PSTR("Bytes RX"), PSTR("%s"), byte_rx);
    BrbDisplayBase_BoxFmt(display_base, pos_x + 160, pos_y, PSTR("Bytes TX"), PSTR("%s"), byte_tx);

    pos_x = DISPLAY_SZ_MARGIN;
    pos_y = pos_y + 50;

    display_base->tft->fillRect(pos_x, pos_y + DISPLAY_SZ_BOX_H, 310, 30, DISPLAY_COLOR_BG);
    BrbDisplayBase_BoxFmt(display_base, pos_x, pos_y, PSTR("Packet RX/TX"), PSTR("%lu/%lu"), rs485_sess->stats.pkt.rx, rs485_sess->stats.pkt.tx);
    BrbDisplayBase_BoxFmt(display_base, pos_x + 160, pos_y, PSTR("Me/Broad"), PSTR("%lu/%lu"), rs485_sess->stats.pkt.me, rs485_sess->stats.pkt.bcast);

    pos_x = DISPLAY_SZ_MARGIN;
    pos_y = pos_y + 50;
    
    BrbDisplayBase_BoxFmt(display_base, pos_x, pos_y, PSTR("Error"), PSTR("%lu/%lu/%lu"), rs485_sess->stats.err.bad_char, rs485_sess->stats.err.crc, rs485_sess->stats.err.overflow);
    BrbDisplayBase_BoxFmt(display_base, pos_x + 160, pos_y, PSTR("Fail"), PSTR("%lu/%lu/%lu"), rs485_sess->stats.pkt.err.cmd_id, rs485_sess->stats.pkt.err.cmd_no_bcast, rs485_sess->stats.pkt.err.cmd_no_cb);

    return 0;
}
/**********************************************************************************************************************/
