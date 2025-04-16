#ifndef _SMS_CMD_H
#define _SMS_CMD_H

#include "sim800l.h"
#include "gps.h"
#include "gyro.h"
#include "ble_scan.h"
#include "state_vals.h"

#define NO_OUT_SMS() (gsm_state.sms_body_out[0] == 0)

extern char *fill_current_state_in_sms(char *str, int max_sz);
extern void send_echo_sms(char *phone);
extern void handle_cmd_sms();

#endif