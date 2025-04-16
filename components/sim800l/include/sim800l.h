#ifndef _SIM800L_H
#define _SIM800L_H

#include "esp_types.h"

#define MAX_SMS_BODY_SZ (512) // In text mode, the maximum length of an SMS depends on the used coding scheme: It is 1024 characters if the 7 bit GSM coding scheme isused.
#define MAX_SMS_OUT_NUM (5)
#define MAX_PNONE_SZ (30)

typedef struct
{
    //-------- info block
    char provider[256];
    int voltage, signal_level, sim_in_slot;
    //-------- state form sim800l
    bool is_ring, has_sms;
    char ring_phone[MAX_PNONE_SZ], sms_phone_in[MAX_PNONE_SZ], sms_body_in[MAX_SMS_BODY_SZ];
    //-------- external action
    bool do_hang_up, do_answer_incoming_call;
    char sms_phone_out[MAX_PNONE_SZ], sms_body_out[MAX_SMS_BODY_SZ*MAX_SMS_OUT_NUM];
    //--------
    char last_at_cmd[1024];
    bool is_ok_last_at_cmd;
} gsm_state_t;

extern gsm_state_t gsm_state;
extern void set_send_sms_flag();
extern bool is_set_send_sms_flag();

// UART_NUM_1, 16-RX, 17-TX
// UART_NUM_2, 18-RX, 19-TX
#define SIM800L_UART_NUM UART_NUM_1
#define SIM800L_RXD (16)
#define SIM800L_TXD (17)

#define SIM800L_DTR_GPIO (4)

#define TX_BUF_SIZE (0)
#define RX_BUF_SIZE (4096)

extern void start_sim800l();

#endif
