/*
 * 13UART_CLB.c
 *
 * Created: 18-05-2026 23:31:04
 * Author : sreer
 */ 
#define F_CPU 16000000UL 
#include "uart.h"
#include "pwm.h"
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdio.h>

#define CMD_ON  1
#define CMD_OFF 2
#define CMD_PWM 3

static uint8_t parse_command(const char *buf)
{
    if (strcmp(buf, "ON") == 0) return CMD_ON;
    if (strcmp(buf, "OFF") == 0) return CMD_OFF;
    for (uint8_t i=0; buf[i]; i++)
        if (!isdigit(buf[i])) return 0;
    int val = atoi(buf);
    if (val >= 0 && val <= 100) return CMD_PWM;
    return 0;
}

int main(void)
{
    uart_init(9600);
    pwm_init();        // OC0A on PD6
    char buffer[16];
    uint8_t idx = 0;

    uart_println("Ready. Send ON, OFF or 0-100");

    while (1)
    {
		    /* Replace with your application code */

        char c = uart_receive();
        if (c == '\n' || c == '\r')
        {
            if (idx > 0)
            {
                buffer[idx] = '\0';
                uint8_t cmd = parse_command(buffer);
                if (cmd == CMD_ON)
                {
                    pwm_set_duty(255);
                    uart_println("LED is now ON");
                }
                else if (cmd == CMD_OFF)
                {
                    pwm_set_duty(0);
                    uart_println("LED is now OFF");
                }
                else if (cmd == CMD_PWM)
                {
                    uint8_t duty = (uint8_t)((uint32_t)atoi(buffer) * 255 / 100);
                    pwm_set_duty(duty);
                    char resp[32];
                    sprintf(resp, "Brightness set to %s%%", buffer);
                    uart_println(resp);
                }
                idx = 0;
            }
        }
        else
        {
            if (idx < 15) buffer[idx++] = c;
        }
    }
}
