/*
 * ctl_pdu.cpp
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

static int BrbPDUBase_TransferPowerToAux(BrbPDUBase *pdu_base);
static int BrbPDUBase_TransferAuxToPower(BrbPDUBase *pdu_base);

static int BrbPDUBase_TransferON(BrbPDUBase *pdu_base);
static int BrbPDUBase_TransferOFF(BrbPDUBase *pdu_base);
static int BrbPDUBase_PowerSetState(BrbPDUBase *pdu_base, BrbPDUStateCode code, BrbPDUFailureCode fail);

/**********************************************************************************************************************/
int BrbPDUBase_Init(BrbPDUBase *pdu_base)
{
	/* Sanitize */
	if (!pdu_base)
		return -1;

	/* Read EEPROM */
	BrbBase_EEPROMRead(pdu_base->brb_base, (uint8_t *)&pdu_base->data, sizeof(pdu_base->data), PDU_EEPROM_OFFSET);

	if (pdu_base->dht_data.pin > 0)
		pdu_base->dht_sensor = new DHT(pdu_base->dht_data.pin, pdu_base->dht_data.type);

	// pinMode(pdu_base->pin_partida, OUTPUT);
	// digitalWrite(pdu_base->pin_partida, PDU_POWER_OFF);

	// pinMode(pdu_base->pin_parada, OUTPUT);
	// digitalWrite(pdu_base->pin_parada, PDU_POWER_OFF);

	/* Setup DC Sensor pins */
	if (pdu_base->sensor_sp01_in.pin > 0)
		pinMode(pdu_base->sensor_sp01_in.pin, INPUT);

	if (pdu_base->sensor_sp01_out.pin > 0)
		pinMode(pdu_base->sensor_sp01_out.pin, INPUT);

	if (pdu_base->sensor_sp02_in.pin > 0)
		pinMode(pdu_base->sensor_sp02_in.pin, INPUT);

	if (pdu_base->sensor_sp02_out.pin > 0)
		pinMode(pdu_base->sensor_sp02_out.pin, INPUT);

	/* Setup AC Sensor pins */
	if (pdu_base->sensor_power.pin > 0)
		pinMode(pdu_base->sensor_power.pin, INPUT);

	if (pdu_base->sensor_aux.pin > 0)
		pinMode(pdu_base->sensor_aux.pin, INPUT);

	return 0;
}
/**********************************************************************************************************************/
int BrbPDUBase_DHTCheck(BrbPDUBase *pdu_base)
{
	pdu_base->dht_data.ms_delta = (pdu_base->ms.cur - pdu_base->dht_data.ms_last);

	if ((pdu_base->dht_sensor) && ((pdu_base->dht_data.ms_last <= 0) || (pdu_base->dht_data.ms_delta >= PDU_TIMER_DHT_MS)))
	{
		pdu_base->dht_data.ms_last = pdu_base->ms.cur;

		pdu_base->dht_data.dht_temp = pdu_base->dht_sensor->readTemperature();
		pdu_base->dht_data.dht_humi = pdu_base->dht_sensor->readHumidity();

		/* Compute heat index in Celsius (isFahreheit = false) */
		pdu_base->dht_data.dht_hidx = pdu_base->dht_sensor->computeHeatIndex(pdu_base->dht_data.dht_temp, pdu_base->dht_data.dht_humi, false);

		pdu_base->dht_data.dht_temp = isnan(pdu_base->dht_data.dht_temp) ? 0.0 : pdu_base->dht_data.dht_temp;
		pdu_base->dht_data.dht_humi = isnan(pdu_base->dht_data.dht_humi) ? 0.0 : pdu_base->dht_data.dht_humi;
		pdu_base->dht_data.dht_hidx = isnan(pdu_base->dht_data.dht_hidx) ? 0.0 : pdu_base->dht_data.dht_hidx;
	}

	return 0;
}
/**********************************************************************************************************************/
int BrbPDUBase_SupplyCheck(BrbPDUBase *pdu_base)
{
#define RDC1 30000.0
#define RDC2 7500.0
#define RDCR (RDC2 / (RDC2 + RDC1))

	pdu_base->sensor_sp01_in.value = ((analogRead(pdu_base->sensor_sp01_in.pin) * 5.0) / 1024.0) / RDCR;
	pdu_base->sensor_sp01_out.value = ((analogRead(pdu_base->sensor_sp01_out.pin) * 5.0) / 1024.0) / RDCR;

	pdu_base->sensor_sp02_in.value = ((analogRead(pdu_base->sensor_sp02_in.pin) * 5.0) / 1024.0) / RDCR;
	pdu_base->sensor_sp02_out.value = ((analogRead(pdu_base->sensor_sp02_out.pin) * 5.0) / 1024.0) / RDCR;

	return 0;
}
/**********************************************************************************************************************/
int BrbPDUBase_PowerCheck(BrbPDUBase *pdu_base)
{
	pdu_base->zerocross.ms_delta = (pdu_base->ms.cur - pdu_base->zerocross.ms_last);

	/* We are waiting delay */
	if ((pdu_base->zerocross.ms_last <= 0) || (pdu_base->zerocross.ms_delta >= PDU_TIMER_ZERO_WAIT_MS))
	{
		pdu_base->zerocross.ms_last = pdu_base->ms.cur;

		noInterrupts();
		pdu_base->zero_power.value = (pdu_base->zero_power.counter / (pdu_base->zerocross.ms_delta / 1000.0)) / 2.0;
		pdu_base->zero_power.counter = 0;

		pdu_base->zero_aux.value = (pdu_base->zero_aux.counter / (pdu_base->zerocross.ms_delta / 1000.0)) / 2.0;
		pdu_base->zero_aux.counter = 0;
		interrupts();
	}

#define RAC1 220000.0
#define RAC2 10000.0
// #define RACR (RAC2 / (RAC2 + RAC1))
#define RACR 0.0165

	pdu_base->sensor.ms_delta = (pdu_base->ms.cur - pdu_base->sensor.ms_last);

	/* We are waiting delay */
	if ((pdu_base->sensor.ms_last <= 0) || (pdu_base->sensor.ms_delta >= PDU_TIMER_SENSOR_WAIT_MS))
	{
		pdu_base->sensor.ms_last = pdu_base->ms.cur;
		long samples_value;
		int i;

		/* Read sensor data */
		if (pdu_base->sensor_power.pin > 0)
		{
			for (i = 0, samples_value = 0; i < PDU_TIMER_SENSOR_SAMPLES; i++)
			{
				samples_value += analogRead(pdu_base->sensor_power.pin);
			}

			samples_value /= PDU_TIMER_SENSOR_SAMPLES;
			pdu_base->sensor_power.value = ((samples_value * 5.0) / 1023.0) / RACR;
			// pdu_base->sensor_power.value = ((analogRead(pdu_base->sensor_power.pin) * 5.0) / 1023.0) / RACR;
		}

		if (pdu_base->sensor_aux.pin > 0)
		{
			for (i = 0, samples_value = 0; i < PDU_TIMER_SENSOR_SAMPLES; i++)
			{
				samples_value += analogRead(pdu_base->sensor_aux.pin);
			}

			samples_value /= PDU_TIMER_SENSOR_SAMPLES;
			pdu_base->sensor_aux.value = ((samples_value * 5.0) / 1023.0) / RACR;
			// pdu_base->sensor_aux.value = ((analogRead(pdu_base->sensor_aux.pin) * 5.0) / 1023.0) / RACR;
		}

		if (pdu_base->sensor_power.value > PDU_POWER_MIN_VALUE && pdu_base->zero_power.value > PDU_POWER_MIN_HZ)
		{
			if (pdu_base->ms.power_time <= 0)
				pdu_base->ms.power_time = pdu_base->ms.cur;

			pdu_base->ms.power_delay = (pdu_base->ms.cur - pdu_base->ms.power_time);
		}
		else
		{
			pdu_base->ms.power_delay = -1;
			pdu_base->ms.power_time = -1;
		}

		if (pdu_base->sensor_aux.value > PDU_AUX_MIN_VALUE && pdu_base->zero_aux.value > PDU_AUX_MIN_HZ)
		{
			if (pdu_base->ms.aux_time <= 0)
				pdu_base->ms.aux_time = pdu_base->ms.cur;

			pdu_base->ms.aux_delay = (pdu_base->ms.cur - pdu_base->ms.aux_time);
		}
		else
		{
			pdu_base->ms.aux_delay = -1;
			pdu_base->ms.aux_time = -1;
		}
	}

	return 0;
}
/**********************************************************************************************************************/
int BrbPDUBase_Loop(BrbPDUBase *pdu_base)
{
	pdu_base->ms.delay = pdu_base->brb_base->ms.cur - pdu_base->ms.last;

	/* Loop Delay */
	if (pdu_base->ms.delay < 50)
		return -1;

	pdu_base->ms.last = pdu_base->ms.cur;
	pdu_base->ms.cur = pdu_base->brb_base->ms.cur;
	pdu_base->state.delta = (pdu_base->ms.cur - pdu_base->state.time);

	BrbPDUBase_DHTCheck(pdu_base);

	BrbPDUBase_SupplyCheck(pdu_base);

	BrbPDUBase_PowerCheck(pdu_base);

	switch (pdu_base->state.code)
	{
	case PDU_STATE_NONE:
	{
		/* Starting system, check for power */
		if (pdu_base->ms.power_delay > PDU_TIMER_POWER_MIN_MS)
		{
			BrbPDUBase_PowerSetState(pdu_base, PDU_STATE_RUNNING_POWER, PDU_FAILURE_NONE);

			BrbToneBase_PlayArrive(pdu_base->tone_base);

			break;
		}

		/* Update time */
		if (pdu_base->state.time == 0)
			pdu_base->state.time = pdu_base->ms.cur;

		break;
	}
	case PDU_STATE_RUNNING_POWER:
	{
		/* Check Power */
		if (pdu_base->sensor_power.value < PDU_POWER_MIN_VALUE && pdu_base->zero_power.value < PDU_POWER_MIN_HZ)
		{
			BrbPDUBase_PowerSetState(pdu_base, PDU_STATE_FAILURE, PDU_FAILURE_POWER_DOWN);

			BrbToneBase_PlayAlarm3(pdu_base->tone_base);

			break;
		}

		break;
	}
	case PDU_STATE_RUNNING_AUX:
	{
		/* We are waiting delay */
		if ((pdu_base->state.time > 0) && (pdu_base->state.delta < (PDU_TIMER_AUX_MIN_MS * 2)))
			break;

		/* We got energy back, transfer back */
		if (!pdu_base->flags.transfer_force && pdu_base->ms.power_delay > PDU_TIMER_AUX_MIN_MS)
		{
			BrbPDUBase_TransferAuxToPower(pdu_base);

			break;
		}

		/* Check Power */
		if (pdu_base->sensor_aux.value < PDU_AUX_MIN_VALUE && pdu_base->zero_aux.value < PDU_AUX_MIN_HZ)
		{
			BrbPDUBase_PowerSetState(pdu_base, PDU_STATE_FAILURE, PDU_FAILURE_AUX_DOWN);

			BrbToneBase_PlayAlarm3(pdu_base->tone_base);

			break;
		}

		break;
	}
	case PDU_STATE_TRANSF_P2A_DELAY:
	{
		/* Can't transfer P2A */
		if (!pdu_base->flags.transfer_enabled)
		{
			return BrbPDUBase_PowerSetState(pdu_base, PDU_STATE_FAILURE, PDU_FAILURE_CANT_P2A);
		}

		/* We are waiting delay */
		if ((pdu_base->state.time > 0) && (pdu_base->state.delta < PDU_TIMER_TRANSF_P2A_WAIT_MS))
			break;

		/* Min time without energy */
		if (!pdu_base->flags.transfer_force && pdu_base->ms.aux_delay < PDU_TIMER_AUX_MIN_MS)
		{
			pdu_base->state.time = PDU_TIMER_TRANSF_P2A_WAIT_MS / 2;

			break;
		}

		/* Transfer */
		BrbPDUBase_TransferON(pdu_base);

		BrbToneBase_PlayArrive(pdu_base->tone_base);

		break;
	}
	case PDU_STATE_TRANSF_A2P_DELAY:
	{
		/* We are waiting delay */
		if ((pdu_base->state.time > 0) && (pdu_base->state.delta < PDU_TIMER_TRANSF_A2P_WAIT_MS))
			break;

		/* Transfer */
		BrbPDUBase_TransferOFF(pdu_base);

		BrbToneBase_PlayLeave(pdu_base->tone_base);

		break;
	}
	case PDU_STATE_FAILURE:
	{
		/* We are waiting delay */
		if ((pdu_base->state.time > 0) && (pdu_base->state.delta < PDU_TIMER_FAIL_WAIT_MS))
			break;

		BrbToneBase_PlayAlarm3(pdu_base->tone_base);

		switch (pdu_base->state.fail)
		{
		case PDU_FAILURE_POWER_DOWN:
		{
			/* Power is up again, reset, so we can initiate */
			if (pdu_base->sensor_power.value >= PDU_POWER_MIN_VALUE && pdu_base->zero_power.value > PDU_POWER_MIN_HZ)
			{
				return BrbPDUBase_PowerSetState(pdu_base, PDU_STATE_NONE, PDU_FAILURE_NONE);
			}

			if (pdu_base->ms.aux_delay > 0)
			{
				/* We can transfer to auxiliary */
				if (pdu_base->flags.transfer_enabled)
				{
					return BrbPDUBase_TransferPowerToAux(pdu_base);
				}
			}

			break;
		}
		case PDU_FAILURE_AUX_DOWN:
		{
			if (pdu_base->sensor_aux.value >= PDU_AUX_MIN_VALUE && pdu_base->zero_aux.value > PDU_AUX_MIN_HZ)
			{
				return BrbPDUBase_PowerSetState(pdu_base, PDU_STATE_NONE, PDU_FAILURE_NONE);
			}

			break;
		}
		default:
		{
			return BrbPDUBase_PowerSetState(pdu_base, PDU_STATE_NONE, PDU_FAILURE_NONE);
			break;
		}
		}

		/* Reset failure */
		BrbPDUBase_PowerSetState(pdu_base, PDU_STATE_FAILURE, pdu_base->state.fail);

		break;
	}
	}

	return 0;
}
/**********************************************************************************************************************/
static int BrbPDUBase_TransferPowerToAux(BrbPDUBase *pdu_base)
{
	BrbToneBase_PlayAlarm2(pdu_base->tone_base);

	BrbPDUBase_PowerSetState(pdu_base, PDU_STATE_TRANSF_P2A_DELAY, PDU_FAILURE_NONE);

	pdu_base->state.retry++;

	return 0;
}
/**********************************************************************************************************************/
static int BrbPDUBase_TransferAuxToPower(BrbPDUBase *pdu_base)
{
	BrbToneBase_PlayAlarm3(pdu_base->tone_base);

	BrbPDUBase_PowerSetState(pdu_base, PDU_STATE_TRANSF_A2P_DELAY, PDU_FAILURE_NONE);

	pdu_base->state.retry++;

	return 0;
}
/**********************************************************************************************************************/
static int BrbPDUBase_TransferON(BrbPDUBase *pdu_base)
{
	/* Set pin data */
	digitalWrite(pdu_base->pin_transfer, PDU_POWER_ON);

	BrbPDUBase_PowerSetState(pdu_base, PDU_STATE_RUNNING_AUX, PDU_FAILURE_NONE);

	return 0;
}
/**********************************************************************************************************************/
static int BrbPDUBase_TransferOFF(BrbPDUBase *pdu_base)
{
	/* Set pin data */
	digitalWrite(pdu_base->pin_transfer, PDU_POWER_OFF);

	BrbPDUBase_PowerSetState(pdu_base, PDU_STATE_RUNNING_POWER, PDU_FAILURE_NONE);

	return 0;
}
/**********************************************************************************************************************/
static int BrbPDUBase_PowerSetState(BrbPDUBase *pdu_base, BrbPDUStateCode code, BrbPDUFailureCode fail)
{
	pdu_base->state.delta = 0;
	pdu_base->state.code = code;
	pdu_base->state.fail = fail;
	pdu_base->state.time = pdu_base->ms.cur;

	BrbRS485PacketVal rs485_pkt_val = {0};

	if (code == PDU_STATE_FAILURE)
	{
		rs485_pkt_val.type = RS485_PKT_DATA_TYPE_NOTIFY;
		rs485_pkt_val.code = fail;
	}
	else
	{
		rs485_pkt_val.type = RS485_PKT_DATA_TYPE_INFORM;
		rs485_pkt_val.code = code;
	}

	/* Send RS485 to notify the network */
	BrbRS485Session_SendPacketData(&glob_rs485_sess, 0xFF, &rs485_pkt_val);

	return 0;
}
/**********************************************************************************************************************/
int BrbPDUBase_ActionCmd(BrbPDUBase *pdu_base, int cmd_code)
{
	switch (cmd_code)
	{
	case PDU_ACTION_TRANSFER_ENABLE:
	{
		pdu_base->flags.transfer_enabled = 1;

		switch (pdu_base->state.code)
		{
		case PDU_STATE_TRANSF_P2A_DELAY:
		case PDU_STATE_RUNNING_AUX:
		{
			return 1;
		}
		case PDU_STATE_TRANSF_A2P_DELAY:
		case PDU_STATE_RUNNING_POWER:
		case PDU_STATE_FAILURE:
		default:

			BrbPDUBase_PowerSetState(pdu_base, PDU_STATE_TRANSF_P2A_DELAY, PDU_FAILURE_NONE);

			BrbToneBase_PlayAction(pdu_base->tone_base);
			break;
		}

		break;
	}
	case PDU_ACTION_TRANSFER_FORCE:
	{
		pdu_base->flags.transfer_force = 1;
		pdu_base->flags.transfer_enabled = 1;

		switch (pdu_base->state.code)
		{
		case PDU_STATE_TRANSF_P2A_DELAY:
		case PDU_STATE_RUNNING_AUX:
		{
			return 1;
		}
		case PDU_STATE_TRANSF_A2P_DELAY:
		case PDU_STATE_RUNNING_POWER:
		case PDU_STATE_FAILURE:
		default:

			BrbPDUBase_PowerSetState(pdu_base, PDU_STATE_TRANSF_P2A_DELAY, PDU_FAILURE_NONE);

			BrbToneBase_PlayAction(pdu_base->tone_base);
			break;
		}

		break;
	}
	case PDU_ACTION_TRANSFER_DISABLE:
	{
		pdu_base->flags.transfer_enabled = 0;

		switch (pdu_base->state.code)
		{
		case PDU_STATE_TRANSF_A2P_DELAY:
		case PDU_STATE_RUNNING_POWER:
		{
			return 1;
		}
		case PDU_STATE_TRANSF_P2A_DELAY:
		case PDU_STATE_RUNNING_AUX:
		case PDU_STATE_FAILURE:
		default:
			BrbPDUBase_PowerSetState(pdu_base, PDU_STATE_TRANSF_A2P_DELAY, PDU_FAILURE_NONE);

			BrbToneBase_PlayAction(pdu_base->tone_base);
			break;
		}

		break;
	}
	case PDU_ACTION_NONE:
	{
		BrbPDUBase_PowerSetState(pdu_base, PDU_STATE_NONE, PDU_FAILURE_NONE);

		break;
	}
	default:
		return -1;
	}

	return 0;
}
/**********************************************************************************************************************/
int BrbPDUBase_FailureConfirm(BrbPDUBase *pdu_base)
{
	// BrbBase *brb_base = pdu_base->brb_base;

	BrbPDUBase_PowerSetState(pdu_base, PDU_STATE_NONE, PDU_FAILURE_NONE);

	BrbToneBase_PlayAction(pdu_base->tone_base);

	return 0;
}
/**********************************************************************************************************************/
int BrbPDUBase_HourmeterReset(BrbPDUBase *pdu_base)
{
	if (!pdu_base)
		return -1;

	pdu_base->data.hourmeter_time = 0;
	pdu_base->data.hourmeter_reset++;

	BrbPDUBase_Save(pdu_base);

	return 0;
}
/**********************************************************************************************************************/
int BrbPDUBase_Save(BrbPDUBase *pdu_base)
{
	if (!pdu_base || !pdu_base->brb_base)
		return -1;

	/* Read EEPROM */
	BrbBase_EEPROMWrite(pdu_base->brb_base, (uint8_t *)&pdu_base->data, sizeof(pdu_base->data), PDU_EEPROM_OFFSET);

	return 0;
}
/**********************************************************************************************************************/
const char *BrbPDUBase_GetStateText(BrbPDUBase *pdu_base)
{
	const char *ret_ptr = PSTR("Parado");

	switch (pdu_base->state.code)
	{
	case PDU_STATE_RUNNING_POWER:
	{
		ret_ptr = PSTR("Carga Ativa");
		break;
	}
	case PDU_STATE_RUNNING_AUX:
	{
		ret_ptr = PSTR("Auxiliar Ativo");
		break;
	}
	case PDU_STATE_TRANSF_P2A_DELAY:
	{
		ret_ptr = PSTR("Transferindo");
		break;
	}
	case PDU_STATE_TRANSF_A2P_DELAY:
	{
		ret_ptr = PSTR("Desacoplando");
		break;
	}
	case PDU_STATE_FAILURE:
	{
		switch (pdu_base->state.fail)
		{
		case PDU_FAILURE_POWER_DOWN:
		{
			ret_ptr = PSTR("FALHA - Sem Carga");
			break;
		}
		case PDU_FAILURE_AUX_DOWN:
		{
			ret_ptr = PSTR("FALHA - Auxiliar");
			break;
		}
		default:
		{
			break;
		}
		}

		break;
	}
	case PDU_STATE_NONE:
	default:
	{
		/**/
		break;
	}
	}

	return ret_ptr;
}
/**********************************************************************************************************************/
uint16_t BrbPDUBase_GetStateColor(BrbPDUBase *pdu_base)
{
	uint16_t color = ILI9341_BLUE;

	switch (pdu_base->state.code)
	{
	case PDU_STATE_RUNNING_POWER:
	{
		color = ILI9341_SEAGREEN;
		break;
	}
	case PDU_STATE_RUNNING_AUX:
	{
		color = ILI9341_ORANGE;
		break;
	}
	case PDU_STATE_TRANSF_P2A_DELAY:
	case PDU_STATE_TRANSF_A2P_DELAY:
	{
		color = ILI9341_PURPLE;
		break;
	}
	case PDU_STATE_FAILURE:
	{
		color = ILI9341_ORANGERED;
		break;
	}
	case PDU_STATE_NONE:
	{
		color = ILI9341_DARKSEAGREEN;
		break;
	}
	default:
	{
		/**/
	}
	}

	return color;
}
/**********************************************************************************************************************/
const char *BrbPDUBase_GetFailureText(BrbPDUBase *pdu_base)
{
	const char *ret_ptr = PSTR("- - - - -");

	switch (pdu_base->state.fail)
	{
	case PDU_FAILURE_POWER_DOWN:
	{
		ret_ptr = PSTR("Verificar Carga");
		break;
	}
	case PDU_FAILURE_AUX_DOWN:
	{
		ret_ptr = PSTR("Verificar Auxiliar");
		break;
	}
	default:
	{
		break;
	}
	}

	return ret_ptr;
}
/**********************************************************************************************************************/