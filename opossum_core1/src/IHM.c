#include "main.h"

int IO_1_state, previous_IO_1_state = 0; //unused for now
int IO_2_state, previous_IO_2_state = 0; //unused for now
int IO_3_state, previous_IO_3_state = 0; //unused for now
int team_state, previous_team_state = 0; //unused for now
int leash_state, previous_leash_state = 0;

int timer_match = 0;
int start_timer_match = 0;


int current_mode = 0;

int validation_blue = 0;
int validation_yellow = 0;

void IHM_loop(void){
   if (leash_state != previous_leash_state){
       previous_leash_state = leash_state;
       if(leash_state == 1){
           printf("LEASH\n");
           eth_send_frame(ETH_MSG_LEASH, &leash_state, sizeof(leash_state));
           current_mode = 60;
           start_timer_match = Timer_ms1;
       }else{
           current_mode = 0;
       }
   }
   if(leash_state == 1){
       timer_match = Timer_ms1 - start_timer_match;
   }
}

uint8_t Version_cmd(void) {
    printf("VERSION ZYNQ\n");
    return 0;
}
