/*
 * main.cpp
 *
 *  Created on: 2019-02-09
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

DHT dht_sensor(DHT_SENSOR_PIN, DHT11);

/* Global control structures */
BrbLogBase *glob_log_base;
BrbBase glob_brb_base;
BrbRS485Session glob_rs485_sess;

BrbBtnBase glob_btn_base;
BrbDisplayBase glob_display_base;
BrbToneBase glob_tone_base;

BrbPDUBase glob_pdu_base;
/**********************************************************************************************************************/
/* RUN ONE TIME ON START */
/**********************************************************************************************************************/
void BrbSetup(void)
{
    /* Clean up base */
    memset(&glob_brb_base, 0, sizeof(BrbBase));

    glob_log_base = BrbLogBase_New();
    BrbLogBase_Init(glob_log_base, &Serial);

    glob_brb_base.log_base = glob_log_base;

    BrbBaseInit(&glob_brb_base);

    return;
}
/**********************************************************************************************************************/
void BrbBtnSetup(void)
{
    /* Clean up base */
    memset(&glob_btn_base, 0, sizeof(BrbBtnBase));

    glob_btn_base.brb_base = &glob_brb_base;
    glob_btn_base.buttons[BRB_BTN_SELECT].pin = BTN_PIN_SELECT;
    glob_btn_base.buttons[BRB_BTN_NEXT].pin = BTN_PIN_NEXT;
    glob_btn_base.buttons[BRB_BTN_PREV].pin = BTN_PIN_PREV;

    BrbBtnBase_Init(&glob_btn_base);

    return;
}
/**********************************************************************************************************************/
void BrbToneSetup(void)
{
    /* Clean up base */
    memset(&glob_tone_base, 0, sizeof(BrbToneBase));

    glob_tone_base.pin = BUZZER_PIN;
    BrbToneBase_Init(&glob_tone_base);

    return;
}
/**********************************************************************************************************************/
static void BrbCtlPDU_IntZeroCrossPower()
{
    glob_pdu_base.zero_power.counter++;
    return;
}
/**********************************************************************************************************************/
static void BrbCtlPDU_IntZeroCrossAux()
{
    glob_pdu_base.zero_aux.counter++;
    return;
}
/**********************************************************************************************************************/
void BrbCtlPDU_Setup(void)
{
    /* Clean up base */
    memset(&glob_pdu_base, 0, sizeof(BrbPDUBase));

    glob_pdu_base.brb_base = (BrbBase *)&glob_brb_base;
    glob_pdu_base.tone_base = (BrbToneBase *)&glob_tone_base;

    glob_pdu_base.sensor_sp01_in.pin = SENSOR_DC_SUPPLY_01_IN_PIN;
    glob_pdu_base.sensor_sp01_out.pin = SENSOR_DC_SUPPLY_01_OUT_PIN;

    glob_pdu_base.sensor_sp02_in.pin = SENSOR_DC_SUPPLY_02_IN_PIN;
    glob_pdu_base.sensor_sp02_out.pin = SENSOR_DC_SUPPLY_02_OUT_PIN;

    glob_pdu_base.sensor_power.pin = SENSOR_AC_POWER_PIN;
    glob_pdu_base.sensor_aux.pin = SENSOR_AC_AUX_PIN;

    glob_pdu_base.zero_power.pin = PDU_ZEROCROSS_POWER_PIN;
    glob_pdu_base.zero_aux.pin = PDU_ZEROCROSS_AUX_PIN;

    glob_pdu_base.dht_data.pin = DHT_SENSOR_PIN;
    glob_pdu_base.dht_data.type = DHT_SENSOR_TYPE;

    BrbPDUBase_Init(&glob_pdu_base);

    attachInterrupt(digitalPinToInterrupt(glob_pdu_base.zero_power.pin), BrbCtlPDU_IntZeroCrossPower, RISING);
    attachInterrupt(digitalPinToInterrupt(glob_pdu_base.zero_aux.pin), BrbCtlPDU_IntZeroCrossAux, RISING);

    return;
}
/**********************************************************************************************************************/
void setup()
{
    randomSeed(((analogRead(A0) + analogRead(A1)) / 2));

    /* Initialize Brb internal data */
    BrbSetup();

    /* Setup Display before anything, because it can display some info, eg logs */
    BrbCtlDisplay_Setup(&glob_brb_base);

    /* Setup Tone  */
    BrbToneSetup();

    /* Setup Buttons  */
    BrbBtnSetup();

    /* Setup RS485 Serial */
    BrbCtlRS485_Setup(&glob_brb_base);

    /* Setup System */
    BrbCtlPDU_Setup();

    LOG_NOTICE(glob_log_base, "BrbBox Panel Control - START [%u] - 0.1.2\r\n", micros());
    LOG_NOTICE(glob_log_base, "BRB [%p], RS485 [%p]\r\n", &glob_brb_base, &glob_rs485_sess);
    LOG_NOTICE(glob_log_base, "RS485 - ADDR 0x%02x UUID [%02x-%02x-%02x-%02x]\r\n",
               glob_rs485_sess.data.address, glob_rs485_sess.data.uuid[0], glob_rs485_sess.data.uuid[1], glob_rs485_sess.data.uuid[2], glob_rs485_sess.data.uuid[3]);
    LOG_HEAP(glob_log_base);

    // BrbMicroScript *script_test;
    // int i;

    // script_test = BrbMicroScriptGrabFree(&glob_brb_base.script_base);
    // script_test->flags.active = 1;
    // script_test->flags.persist = 1;
    
    // for (i = 0; i < PDU_TTR_COUNT; i++)
    // {
    //     BrbMicroScriptOPAddSetDig(&glob_brb_base.script_base, script_test, (glob_pdu_base.pin_ttr + i), OUTPUT, LOW);
    // }
    
    // BrbMicroScriptOPAddDelay(&glob_brb_base.script_base, script_test, 1000);
    
    // for (i = 0; i < PDU_TTR_COUNT; i++)
    // {
    //     BrbMicroScriptOPAddSetDig(&glob_brb_base.script_base, script_test, (glob_pdu_base.pin_ttr + i), OUTPUT, HIGH);
    // }

    // BrbMicroScriptOPAddDelay(&glob_brb_base.script_base, script_test, 1000);

    // LOG_NOTICE(glob_log_base, "CODE SIZE [%d]\r\n", script_test->code.size);
    

    return;
}
/**********************************************************************************************************************/
/* RUN FOREVER */
/**********************************************************************************************************************/
void loop()
{
    /* Dispatch */
    BrbBaseLoop(&glob_brb_base);

    /* Check for Buttons */
    BrbBtnBase_Loop((BrbBtnBase *)&glob_btn_base);

    if (glob_btn_base.buttons[BRB_BTN_SELECT].hit > 0)
    {
        glob_btn_base.buttons[BRB_BTN_SELECT].hit = BrbDisplayBase_ScreenAction((BrbDisplayBase *)&glob_display_base, DISPLAY_ACTION_SELECT);

    }
    else if (glob_btn_base.buttons[BRB_BTN_NEXT].hit > 0)
    {
        glob_btn_base.buttons[BRB_BTN_NEXT].hit = BrbDisplayBase_ScreenAction((BrbDisplayBase *)&glob_display_base, DISPLAY_ACTION_NEXT);
    }
    else if (glob_btn_base.buttons[BRB_BTN_PREV].hit > 0)
    {
        glob_btn_base.buttons[BRB_BTN_PREV].hit = BrbDisplayBase_ScreenAction((BrbDisplayBase *)&glob_display_base, DISPLAY_ACTION_PREV);
    }

    /* Do RS485 loop */
    BrbRS485Session_Loop(&glob_rs485_sess);

    /* Do PDU loop */
    BrbPDUBase_Loop(&glob_pdu_base);

    /* Do TONE loop */
    BrbToneBase_Loop(&glob_tone_base);

    // // send data only when you receive data:
    // if (Serial.available() > 0)
    // {
    //     // read the incoming byte:
    //     byte incomingByte = Serial.read();

    //     if (incomingByte == 'p')
    //     {
    //         glob_display_base.tft->setSPIClockDivider(SPI_CLOCK_DIV8);
    //         glob_display_base.tft->screenshotToConsole();
    //         glob_display_base.tft->setSPIClockDivider(ILI9341_SPI_CLKDIVIDER);
    //     }
    //     else if (incomingByte == 'n')
    //     {
    //         glob_btn_base.buttons[BRB_BTN_NEXT].hit = BrbDisplayBase_ScreenAction((BrbDisplayBase *)&glob_display_base, DISPLAY_ACTION_NEXT);
    //     }
        
    //     LOG_NOTICE(glob_log_base, "GOT [%c]\r\n", incomingByte);
    // }

    return;
}
/**********************************************************************************************************************/