/*
 * ctl_rs485.cpp
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

static BrbRS485SessionActionCBH BrbCtlRS485_SessionActionHandShakeCB;

static BrbRS485SessionActionCBH BrbCtlRS485_SessionActionGetAnalogCB;
static BrbRS485SessionActionCBH BrbCtlRS485_SessionActionSetAnalogCB;
static BrbRS485SessionActionCBH BrbCtlRS485_SessionActionSetAnalogBMPCB;

static BrbRS485SessionActionCBH BrbCtlRS485_SessionActionGetDigitalCB;
static BrbRS485SessionActionCBH BrbCtlRS485_SessionActionSetDigitalCB;
static BrbRS485SessionActionCBH BrbCtlRS485_SessionActionSetDigitalBMPCB;
static BrbRS485SessionActionCBH BrbCtlRS485_SessionActionSetScriptCB;
static BrbRS485SessionActionCBH BrbCtlRS485_SessionActionDataCB;
/**********************************************************************************************************************/
int BrbCtlRS485_Setup(BrbBase *brb_base)
{
    /* Clean up base */
    memset((BrbRS485Session *)&glob_rs485_sess, 0, sizeof(BrbRS485Session));
    BrbRS485Session *rs485_sess = (BrbRS485Session *)&glob_rs485_sess;

    rs485_sess->brb_base = brb_base;
    rs485_sess->pinRO = RS485_RO_PIN;
    rs485_sess->pinDI = RS485_DI_PIN;
    rs485_sess->pinREDE = RS485_REDE_PIN;
    rs485_sess->log_base = brb_base->log_base;
    rs485_sess->serial = &Serial3;
    rs485_sess->device_type = 0x11;

	/* Initialize SoftwareSerial to RS485 */
	// Serial3.begin(BRB_RS485_BAUDRATE);
	// rs485_sess->serial->begin(BRB_RS485_BAUDRATE);

    // char buf[32];
    // sprintf(buf, "%p %p %p", &Serial1, brb_base, brb_base->log_base);
    // Serial.println(buf);

	// LOG_DEBUG(brb_base->log_base, "SETTED - [%p]\r\n", rs485_sess);

    BrbRS485Session_SetEventCB(rs485_sess, RS485_PKT_TYPE_HANDSHAKE, BrbCtlRS485_SessionActionHandShakeCB, brb_base);
    BrbRS485Session_SetEventCB(rs485_sess, RS485_PKT_TYPE_CMD_GET_A, BrbCtlRS485_SessionActionGetAnalogCB, brb_base);
    BrbRS485Session_SetEventCB(rs485_sess, RS485_PKT_TYPE_CMD_SET_A, BrbCtlRS485_SessionActionSetAnalogCB, brb_base);
    BrbRS485Session_SetEventCB(rs485_sess, RS485_PKT_TYPE_CMD_SET_A_BMP, BrbCtlRS485_SessionActionSetAnalogBMPCB, brb_base);

    BrbRS485Session_SetEventCB(rs485_sess, RS485_PKT_TYPE_CMD_GET_D, BrbCtlRS485_SessionActionGetDigitalCB, brb_base);
    BrbRS485Session_SetEventCB(rs485_sess, RS485_PKT_TYPE_CMD_SET_D, BrbCtlRS485_SessionActionSetDigitalCB, brb_base);
    BrbRS485Session_SetEventCB(rs485_sess, RS485_PKT_TYPE_CMD_SET_D_BMP, BrbCtlRS485_SessionActionSetDigitalBMPCB, brb_base);

    BrbRS485Session_SetEventCB(rs485_sess, RS485_PKT_TYPE_CMD_SET_SCRIPT, BrbCtlRS485_SessionActionSetScriptCB, brb_base);

    BrbRS485Session_SetEventCB(rs485_sess, RS485_PKT_TYPE_DATA, BrbCtlRS485_SessionActionDataCB, brb_base);

    /* Initialize session data */
    BrbRS485Session_Init(rs485_sess);

    return 0;
}
/**********************************************************************************************************************/
static int BrbCtlRS485_SessionActionHandShakeCB(void *base_ptr, int action_code, const void *buffer_ptr, unsigned int buffer_sz, void *cb_data_ptr)
{
    // BrbRS485Session *rs485_sess             = (BrbRS485Session *)base_ptr;
    // BrbRS485PacketHandShake *pkt_recv_hs    = (BrbRS485PacketHandShake *)buffer_ptr;

    // LOG_INFO(rs485_sess->log_base, "GET HANDSHAKE [%02x][%02x][%02x][%02x]\n",
    // 		pkt_recv_hs->uuid[0], pkt_recv_hs->uuid[1], pkt_recv_hs->uuid[2], pkt_recv_hs->uuid[3]);

    return RS485_PKT_RETURN_QUIET;
}
/**********************************************************************************************************************/
static int BrbCtlRS485_SessionActionGetAnalogCB(void *base_ptr, int action_code, const void *buffer_ptr, unsigned int buffer_sz, void *cb_data_ptr)
{
    BrbRS485Session *rs485_sess = (BrbRS485Session *)base_ptr;
    BrbRS485PacketData *pkt_recv = (BrbRS485PacketData *)buffer_ptr;
    BrbRS485PacketData *pkt_reply = (BrbRS485PacketData *)&rs485_sess->pkt.out.data;
    BrbRS485PacketPinData *pin_data = (BrbRS485PacketPinData *)(pkt_reply + 1);

    int pin_code = pkt_recv->val.code;
    int pin_begin;
    int pin_max;
    int op_status;
    int i;

    /* Adjust pin query */
    if (pin_code < MIN_ANA_PIN || pin_code >= MAX_ANA_PIN)
    {
        pin_begin = MIN_ANA_PIN;
        pin_max = MAX_ANA_PIN;
    }
    else
    {
        pin_begin = pin_code;
        pin_max = pin_code + 1;
    }

    // LOG_DEBUG(glob_log_base, "GET ANALOG - [%u] - [%d][%d]\n", pin_code, pin_begin, pin_max);

    /* Inverse the order two reply */
    pkt_reply->hdr.dst = pkt_recv->hdr.src;
    pkt_reply->hdr.src = rs485_sess->data.address;
    pkt_reply->hdr.id = pkt_recv->hdr.id;
    pkt_reply->hdr.type = RS485_PKT_TYPE_CMD_GET_A;
    pkt_reply->hdr.len = sizeof(BrbRS485PacketData) + (sizeof(BrbRS485PacketPinData) * (pin_max - pin_begin));

    for (i = pin_begin; i < pin_max; i++)
    {
        pin_data->pin = i;
        pin_data->type = 0;
        pin_data->mode = BrbBase_PinGetMode(BrbBase_PinGetAnalogPin(i));
        pin_data->value = analogRead(i);

        // LOG_DEBUG(glob_log_base, "GET ANALOG [%u] - [%d]\n", i, pin_data->value);

        pin_data++;
    }

    /* Send CRC */
    op_status = BrbRS485Session_SendPacket(rs485_sess, (byte *)pkt_reply, pkt_reply->hdr.len);

    if (op_status <= 0)
        return RS485_PKT_RETURN_ACK_FAIL;

    return RS485_PKT_RETURN_QUIET;
}
/**********************************************************************************************************************/
static int BrbCtlRS485_SessionActionSetAnalogCB(void *base_ptr, int action_code, const void *buffer_ptr, unsigned int buffer_sz, void *cb_data_ptr)
{
    BrbRS485Session *rs485_sess = (BrbRS485Session *)base_ptr;
    BrbRS485PacketSetPin *pkt_recv_set = (BrbRS485PacketSetPin *)buffer_ptr;

    LOG_INFO(rs485_sess->log_base, "SET ANALOG - [%u] [%u] [%u]\n", pkt_recv_set->pin, pkt_recv_set->mode, pkt_recv_set->value);

    /* Check pinCode we can set */
    if (pkt_recv_set->pin < MIN_ANA_PIN || pkt_recv_set->pin >= MAX_ANA_PIN)
        return RS485_PKT_RETURN_ACK_FAIL;

    int analog_pin = BrbBase_PinGetAnalogPin(pkt_recv_set->pin);

    if (pkt_recv_set->mode == OUTPUT)
    {
        analogWrite(analog_pin, pkt_recv_set->value);
    }
    else if (pkt_recv_set->mode == INPUT)
        pinMode(analog_pin, INPUT);
    else if (pkt_recv_set->mode == INPUT_PULLUP)
        pinMode(analog_pin, INPUT_PULLUP);

    glob_brb_base.pin_data[MAX_DIG_PIN + pkt_recv_set->pin].value = pkt_recv_set->value;
    glob_brb_base.pin_data[MAX_DIG_PIN + pkt_recv_set->pin].mode = pkt_recv_set->mode;
    glob_brb_base.pin_data[MAX_DIG_PIN + pkt_recv_set->pin].persist = pkt_recv_set->persist;

    BrbBase_PinSave(rs485_sess->brb_base);

    return RS485_PKT_RETURN_ACK_SUCCESS;
}
/**********************************************************************************************************************/
static int BrbCtlRS485_SessionActionSetAnalogBMPCB(void *base_ptr, int action_code, const void *buffer_ptr, unsigned int buffer_sz, void *cb_data_ptr)
{
    BrbRS485PacketSetPinBmpAna *pkt_recv_set = (BrbRS485PacketSetPinBmpAna *)buffer_ptr;

    int analog_pin;
    int i;

    for (i = MIN_ANA_PIN; i < MAX_ANA_PIN && i < 32; i++)
    {
        if (pkt_recv_set->map[i].mode == 3)
            continue;

        analog_pin = BrbBase_PinGetAnalogPin(i);

        LOG_INFO(rs485_sess->log_base, "SET ANALOG [%u]/[%u] - [%u] [%u]\n",
                 i, analog_pin, pkt_recv_set->map[i].mode, pkt_recv_set->map[i].value);

        if (pkt_recv_set->map[i].mode == OUTPUT)
        {
            analogWrite(analog_pin, pkt_recv_set->map[i].value);
        }
        else if (pkt_recv_set->map[i].mode == INPUT)
            pinMode(analog_pin, INPUT);
        else if (pkt_recv_set->map[i].mode == INPUT_PULLUP)
            pinMode(analog_pin, INPUT_PULLUP);

        glob_brb_base.pin_data[MAX_DIG_PIN + i].value = pkt_recv_set->map[i].value;
        glob_brb_base.pin_data[MAX_DIG_PIN + i].mode = pkt_recv_set->map[i].mode;
        glob_brb_base.pin_data[MAX_DIG_PIN + i].persist = pkt_recv_set->map[i].persist;
    }

    BrbBase_PinSave(&glob_brb_base);

    return RS485_PKT_RETURN_ACK_SUCCESS;
}
/**********************************************************************************************************************/
static int BrbCtlRS485_SessionActionGetDigitalCB(void *base_ptr, int action_code, const void *buffer_ptr, unsigned int buffer_sz, void *cb_data_ptr)
{
    BrbRS485Session *rs485_sess = (BrbRS485Session *)base_ptr;
    BrbRS485PacketData *pkt_recv = (BrbRS485PacketData *)buffer_ptr;
    BrbRS485PacketData *pkt_reply = (BrbRS485PacketData *)&rs485_sess->pkt.out.data;
    BrbRS485PacketPinData *pin_data = (BrbRS485PacketPinData *)(pkt_reply + 1);
    
    int pin_code = pkt_recv->val.code;
    int pin_begin;
    int pin_max;
    int op_status;
    int i;

    /* Adjust pin query */
    if (pin_code < 0 || pin_code >= MAX_DIG_PIN)
    {
        pin_begin = 0;
        pin_max = MAX_DIG_PIN;
    }
    else
    {
        pin_begin = pin_code;
        pin_max = pin_code + 1;
    }

    LOG_INFO(rs485_sess->log_base, "GET DIGITAL - [%u] - [%d][%d]\n", pin_code, pin_begin, pin_max);

    /* Inverse the order two reply */
    pkt_reply->hdr.dst = pkt_recv->hdr.src;
    pkt_reply->hdr.src = rs485_sess->data.address;
    pkt_reply->hdr.id = pkt_recv->hdr.id;
    pkt_reply->hdr.type = RS485_PKT_TYPE_CMD_GET_D;
    pkt_reply->hdr.len = sizeof(BrbRS485PacketData) + (sizeof(BrbRS485PacketPinData) * (pin_max - pin_begin));

    for (i = pin_begin; i < pin_max; i++)
    {
        pin_data->pin = i;
        pin_data->type = 1;
        pin_data->value = digitalRead(i);
        pin_data->mode = BrbBase_PinGetMode(i);
        // pin_data->value = 0;
        // pin_data->mode = 0;
        pin_data++;
    }

    /* Send CRC */
    op_status = BrbRS485Session_SendPacket(rs485_sess, (byte *)pkt_reply, pkt_reply->hdr.len);

    if (op_status <= 0)
        return RS485_PKT_RETURN_ACK_FAIL;

    return RS485_PKT_RETURN_QUIET;
}
/**********************************************************************************************************************/
static int BrbCtlRS485_SessionActionSetDigitalCB(void *base_ptr, int action_code, const void *buffer_ptr, unsigned int buffer_sz, void *cb_data_ptr)
{
    BrbRS485Session *rs485_sess = (BrbRS485Session *)base_ptr;
    BrbRS485PacketSetPin *pkt_recv_set = (BrbRS485PacketSetPin *)buffer_ptr;

    LOG_WARN(rs485_sess->log_base, "SET DIGITAL - [%u] [%u] [%u]\n", pkt_recv_set->pin, pkt_recv_set->mode, pkt_recv_set->value);

    /* Check pinCode we can set */
    if (pkt_recv_set->pin < MIN_DIG_PIN || pkt_recv_set->pin >= MAX_DIG_PIN)
        return RS485_PKT_RETURN_ACK_FAIL;

    /* Check output, set value */
    if (pkt_recv_set->mode == OUTPUT)
    {
        pinMode(pkt_recv_set->pin, OUTPUT);

        digitalWrite(pkt_recv_set->pin, (pkt_recv_set->value == HIGH) ? HIGH : LOW);
    }
    if (pkt_recv_set->mode == INPUT_PULLUP)
        pinMode(pkt_recv_set->pin, INPUT_PULLUP);
    else if (pkt_recv_set->mode == INPUT)
        pinMode(pkt_recv_set->pin, INPUT);

    glob_brb_base.pin_data[pkt_recv_set->pin].value = (pkt_recv_set->value == HIGH) ? HIGH : LOW;
    glob_brb_base.pin_data[pkt_recv_set->pin].mode = pkt_recv_set->mode;
    glob_brb_base.pin_data[pkt_recv_set->pin].persist = pkt_recv_set->persist;
    glob_brb_base.pin_data[pkt_recv_set->pin].persist = 0;

    BrbBase_PinSave(&glob_brb_base);

    return RS485_PKT_RETURN_ACK_SUCCESS;
}
/**********************************************************************************************************************/
static int BrbCtlRS485_SessionActionSetDigitalBMPCB(void *base_ptr, int action_code, const void *buffer_ptr, unsigned int buffer_sz, void *cb_data_ptr)
{
    BrbRS485Session *rs485_sess = (BrbRS485Session *)base_ptr;
    BrbRS485PacketSetPinBmpDig *pkt_recv_set = (BrbRS485PacketSetPinBmpDig *)buffer_ptr;

    int i;
    for (i = MIN_DIG_PIN; i < MAX_DIG_PIN && i < 64; i++)
    {
        if (pkt_recv_set->map[i].mode == 3)
            continue;

        LOG_DEBUG(rs485_sess->log_base, "SET DIGITAL [%u] - [%u] [%u]\n", i, pkt_recv_set->map[i].value, pkt_recv_set->map[i].mode);

        if (pkt_recv_set->map[i].mode == OUTPUT)
        {
            pinMode(i, OUTPUT);
            
            digitalWrite(i, (pkt_recv_set->map[i].value == HIGH) ? HIGH : LOW);
        }
        else if (pkt_recv_set->map[i].mode == INPUT)
            pinMode(i, INPUT);
        else if (pkt_recv_set->map[i].mode == INPUT_PULLUP)
            pinMode(i, INPUT_PULLUP);

        glob_brb_base.pin_data[i].value = (pkt_recv_set->map[i].value == HIGH) ? HIGH : LOW;
        glob_brb_base.pin_data[i].mode = pkt_recv_set->map[i].mode;
        glob_brb_base.pin_data[i].persist = pkt_recv_set->map[i].persist;
        glob_brb_base.pin_data[i].persist = 0;
    }

    BrbBase_PinSave(&glob_brb_base);

    return RS485_PKT_RETURN_ACK_SUCCESS;
}
/**********************************************************************************************************************/
static int BrbCtlRS485_SessionActionSetScriptCB(void *base_ptr, int action_code, const void *buffer_ptr, unsigned int buffer_sz, void *cb_data_ptr)
{
    BrbRS485Session *rs485_sess = (BrbRS485Session *)base_ptr;
    BrbRS485PacketSetScript *pkt_recv_set = (BrbRS485PacketSetScript *)buffer_ptr;
    BrbBase *brb_base = (BrbBase *)cb_data_ptr;
    BrbMicroScript *script;

    LOG_WARN(rs485_sess->log_base, "SCRIPT SET [%p] [%p] [%d]\r\n", &glob_brb_base, brb_base, pkt_recv_set->script_id);

    script = BrbMicroScriptGrabFree(&brb_base->script_base);

    // LOG_WARN(rs485_sess->log_base, "SCRIPT SET [%p] [%p] [%o]\r\n", &glob_brb_base, brb_base, script);

    if (!script)
    {
        LOG_WARN(rs485_sess->log_base, "NO SCRIPT [%u]\r\n", pkt_recv_set->script_id);

        return RS485_PKT_RETURN_ACK_FAIL;
    }

    memcpy(&script->code, &pkt_recv_set->code, sizeof(BrbMicroCode));

    script->flags.persist = pkt_recv_set->persist;
    script->flags.active = 1;

    return RS485_PKT_RETURN_ACK_SUCCESS;
}
/**********************************************************************************************************************/
static int BrbCtlRS485_SessionActionDataCB(void *base_ptr, int action_code, const void *buffer_ptr, unsigned int buffer_sz, void *cb_data_ptr)
{
    BrbRS485Session *rs485_sess = (BrbRS485Session *)base_ptr;
    BrbRS485PacketData *pkt_data = (BrbRS485PacketData *)buffer_ptr;
    BrbRS485PacketVal *pkt_val = (BrbRS485PacketVal *)&pkt_data->val;
    BrbBase *brb_base = (BrbBase *)cb_data_ptr;

    LOG_WARN(rs485_sess->log_base, "ACTION DATA [%d] [%d] [%d]\r\n", pkt_data->hdr.id, pkt_val->type, pkt_val->code);

    switch (pkt_val->type)
    {
        case RS485_PKT_DATA_TYPE_ACTION:
        {
            BrbPDUBase_ActionCmd(&glob_pdu_base, pkt_val->code);

            break;
        }
        case RS485_PKT_DATA_TYPE_INFORM:
        {
            break;
        }
        case RS485_PKT_DATA_TYPE_NOTIFY:
        {
            break;
        }
        case RS485_PKT_DATA_TYPE_NONE:
        case RS485_PKT_DATA_TYPE_LAST_ITEM:
        default:
        {
            break;
        }

    }

    return RS485_PKT_RETURN_ACK_SUCCESS;
}
/**********************************************************************************************************************/