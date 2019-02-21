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

static int BrbPDUBase_PowerStart(BrbPDUBase *pdu_base);
static int BrbPDUBase_PowerStop(BrbPDUBase *pdu_base);
static int BrbPDUBase_PowerOff(BrbPDUBase *pdu_base);
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
int BrbPDUBase_Loop(BrbPDUBase *pdu_base)
{
	pdu_base->ms.delay = pdu_base->brb_base->ms.cur - pdu_base->ms.last;

	/* Loop Delay */
	if (pdu_base->ms.delay < 50)
		return -1;

	pdu_base->ms.last = pdu_base->ms.cur;
	pdu_base->ms.cur = pdu_base->brb_base->ms.cur;

	pdu_base->state.delta = (pdu_base->ms.cur - pdu_base->state.time);

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

	pdu_base->sensor_power.value = ((analogRead(pdu_base->sensor_power.pin) * 5.0) / 1024.0) / 0.013;
	pdu_base->sensor_aux.value = ((analogRead(pdu_base->sensor_aux.pin) * 5.0) / 1024.0) / 0.013;

#define RDC1 30000.0
#define RDC2 7500.0
#define RDCR (RDC2 / (RDC2 + RDC1))

	pdu_base->sensor_sp01_in.value = ((analogRead(pdu_base->sensor_sp01_in.pin) * 5.0) / 1024.0) / RDCR;
	pdu_base->sensor_sp01_out.value = ((analogRead(pdu_base->sensor_sp01_out.pin) * 5.0) / 1024.0) / RDCR;

	pdu_base->sensor_sp02_in.value = ((analogRead(pdu_base->sensor_sp02_in.pin) * 5.0) / 1024.0) / RDCR;
	pdu_base->sensor_sp02_out.value = ((analogRead(pdu_base->sensor_sp02_out.pin) * 5.0) / 1024.0) / RDCR;

	pdu_base->dht_data.ms_delta = (pdu_base->ms.cur - pdu_base->dht_data.ms_last);

	if ((pdu_base->dht_sensor) && ((pdu_base->dht_data.ms_last <= 0) || (pdu_base->dht_data.ms_delta >= 1000)))
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

	switch (pdu_base->state.code)
	{
	case PDU_STATE_NONE:
	{
		/* This can't happen here, do something */
		if (pdu_base->sensor_power.value > PDU_POWER_MIN_VALUE && pdu_base->zero_power.value > PDU_POWER_MIN_HZ)
		{
			BrbPDUBase_PowerSetState(pdu_base, PDU_STATE_RUNNING_POWER, PDU_FAILURE_NONE);

			BrbToneBase_PlayArrive(pdu_base->tone_base);

			break;
		}

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
		/* Check Power */
		if (pdu_base->sensor_aux.value < PDU_AUX_MIN_VALUE && pdu_base->zero_aux.value < PDU_AUX_MIN_HZ)
		{
			BrbPDUBase_PowerSetState(pdu_base, PDU_STATE_FAILURE, PDU_FAILURE_AUX_DOWN);

			BrbToneBase_PlayAlarm3(pdu_base->tone_base);

			break;
		}

		break;
	}
	case PDU_STATE_FAILURE:
	{
		/* We are waiting delay */
		if ((pdu_base->state.time > 0) && (pdu_base->state.delta < PDU_TIMER_FAIL_ALARM_MS))
			break;

		/* Power off pins */
		BrbPDUBase_PowerOff(pdu_base);

		BrbToneBase_PlayAlarm3(pdu_base->tone_base);

		switch (pdu_base->state.fail)
		{
		case PDU_FAILURE_POWER_DOWN:
		{
			if (pdu_base->sensor_power.value >= PDU_POWER_MIN_VALUE && pdu_base->zero_power.value > PDU_POWER_MIN_HZ)
			{
				BrbPDUBase_PowerSetState(pdu_base, PDU_STATE_NONE, PDU_FAILURE_NONE);
				break;
			}

			break;
		}
		case PDU_FAILURE_AUX_DOWN:
		{
			if (pdu_base->sensor_aux.value >= PDU_AUX_MIN_VALUE && pdu_base->zero_aux.value > PDU_AUX_MIN_HZ)
			{
				BrbPDUBase_PowerSetState(pdu_base, PDU_STATE_NONE, PDU_FAILURE_NONE);
				break;
			}

			break;
		}
		default:
		{
			BrbPDUBase_PowerSetState(pdu_base, PDU_STATE_NONE, PDU_FAILURE_NONE);
			break;
		}
		}

		/* Update time */
		pdu_base->state.time = pdu_base->ms.cur;

		break;
	}
	}

	// if (pdu_base->sensor_power.value > PDU_POWER_MIN_VALUE)
	// {
	// 	pdu_base->info.hourmeter_ms = pdu_base->info.hourmeter_ms + pdu_base->ms.delay;

	// 	/* 5 seconds delay */
	// 	if (pdu_base->info.hourmeter_ms > 5000)
	// 	{
	// 		pdu_base->info.hourmeter_ms = (pdu_base->info.hourmeter_ms - 5000);

	// 		pdu_base->info.hourmeter_sec = pdu_base->info.hourmeter_sec + 5;

	// 		/* 60 seconds delay */
	// 		if (pdu_base->info.hourmeter_sec > 60)
	// 		{
	// 			pdu_base->data.hourmeter_total++;
	// 			pdu_base->data.hourmeter_time++;
	// 			pdu_base->info.hourmeter_sec = (pdu_base->info.hourmeter_sec - 60);

	// 			BrbPDUBase_Save(pdu_base);
	// 		}
	// 	}

	// 	BrbPDUBase_PowerSetState(pdu_base, PDU_STATE_RUNNING_POWER);
	// }

	return 0;
}
/**********************************************************************************************************************/
static int BrbPDUBase_PowerStart(BrbPDUBase *pdu_base)
{
	// BrbBase *brb_base = pdu_base->brb_base;

	BrbToneBase_PlayAlarm2(pdu_base->tone_base);

	// BrbPDUBase_PowerSetState(pdu_base, PDU_STATE_START_DELAY, PDU_FAILURE_NONE);

	/* Set pin data */
	// digitalWrite(pdu_base->pin_partida, PDU_POWER_ON);

	pdu_base->state.retry++;

	return 0;
}
/**********************************************************************************************************************/
static int BrbPDUBase_PowerStop(BrbPDUBase *pdu_base)
{
	// BrbBase *brb_base = pdu_base->brb_base;

	BrbToneBase_PlayAlarm3(pdu_base->tone_base);

	// BrbPDUBase_PowerSetState(pdu_base, PDU_STATE_STOP_DELAY, PDU_FAILURE_NONE);

	/* Set pin data */
	// digitalWrite(pdu_base->pin_parada, PDU_POWER_ON);

	pdu_base->state.retry++;

	return 0;
}
/**********************************************************************************************************************/
static int BrbPDUBase_PowerOff(BrbPDUBase *pdu_base)
{
	// BrbBase *brb_base = pdu_base->brb_base;

	/* Set pin data */
	// digitalWrite(pdu_base->pin_partida, PDU_POWER_OFF);
	// digitalWrite(pdu_base->pin_parada, PDU_POWER_OFF);

	return 0;
}
/**********************************************************************************************************************/
static int BrbPDUBase_PowerSetState(BrbPDUBase *pdu_base, BrbPDUStateCode code, BrbPDUFailureCode fail)
{
	// BrbBase *brb_base = pdu_base->brb_base;

	pdu_base->state.delta = 0;
	pdu_base->state.code = code;
	pdu_base->state.fail = fail;
	pdu_base->state.time = pdu_base->ms.cur;

	return 0;
}
/**********************************************************************************************************************/
int BrbPDUBase_Start(BrbPDUBase *pdu_base)
{
	// BrbBase *brb_base = pdu_base->brb_base;

	// BrbPDUBase_PowerSetState(pdu_base, PDU_STATE_START_INIT, PDU_FAILURE_NONE);

	BrbToneBase_PlayAction(pdu_base->tone_base);

	return 0;
}
/**********************************************************************************************************************/
int BrbPDUBase_Stop(BrbPDUBase *pdu_base)
{
	// BrbBase *brb_base = pdu_base->brb_base;

	// BrbPDUBase_PowerSetState(pdu_base, PDU_STATE_STOP_INIT, PDU_FAILURE_NONE);

	BrbToneBase_PlayAction(pdu_base->tone_base);

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
const char *BrbPDUBase_GetState(BrbPDUBase *pdu_base)
{
	const char *ret_ptr = PSTR("Parado");

	switch (pdu_base->state.code)
	{
	case PDU_STATE_RUNNING_POWER:
	{
		ret_ptr = PSTR("Funcionando");
		break;
	}
	case PDU_STATE_RUNNING_AUX:
	{
		ret_ptr = PSTR("Auxiliar");
		break;
	}
	case PDU_STATE_FAILURE:
	{
		ret_ptr = PSTR("Falha");
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
const char *BrbPDUBase_GetStateAction(BrbPDUBase *pdu_base)
{
	const char *ret_ptr = PSTR("None");

	switch (pdu_base->state.code)
	{
	case PDU_STATE_RUNNING_POWER:
	{
		ret_ptr = PSTR("Acoplar?");
		break;
	}
	case PDU_STATE_RUNNING_AUX:
	{
		ret_ptr = PSTR("Desacoplar?");
		break;
	}
	case PDU_STATE_FAILURE:
	{
		ret_ptr = PSTR("Falha no Sistema!");
		break;
	}
	case PDU_STATE_NONE:
	{
		ret_ptr = PSTR("Iniciar Sistema?");
		break;
	}
	default:
	{
		/**/
		break;
	}
	}

	return ret_ptr;
}

/**********************************************************************************************************************/
const char *BrbPDUBase_GetStateButton(BrbPDUBase *pdu_base)
{
	const char *ret_ptr = PSTR("None");

	switch (pdu_base->state.code)
	{
	case PDU_STATE_RUNNING_POWER:
	{
		ret_ptr = PSTR("ACOMPLAR");
		break;
	}
	case PDU_STATE_RUNNING_AUX:
	{
		ret_ptr = PSTR("DESACOPLAR");
		break;
	}
	case PDU_STATE_FAILURE:
	{
		ret_ptr = PSTR("IGNORAR");
		break;
	}
	case PDU_STATE_NONE:
	default:
	{
		ret_ptr = PSTR("LIGAR");
		break;
	}
	}

	return ret_ptr;
}
/**********************************************************************************************************************/
const char *BrbPDUBase_GetFailure(BrbPDUBase *pdu_base)
{
	const char *ret_ptr = PSTR("- - - - -");

	switch (pdu_base->state.fail)
	{
	case PDU_FAILURE_POWER_DOWN:
	{
		ret_ptr = PSTR("Power Down");
		break;
	}
	case PDU_FAILURE_AUX_DOWN:
	{
		ret_ptr = PSTR("Aux Down");
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