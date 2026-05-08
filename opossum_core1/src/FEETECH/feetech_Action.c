#include "../main.h" 

//// ACTION 2026 ////
Pince_t robot_pinces[NBR_PINCES];


uint8_t Send_FEETECH_Cmd(void){
    uint32_t val32;
    uint8_t Id;
    uint8_t Reg;
    uint16_t Consigne;
    if (Get_Param_u32(&val32))
        return PARAM_ERROR_CODE;
    Id = val32;
    if (Get_Param_u32(&val32))
        return PARAM_ERROR_CODE;
    Reg = val32;
    if (Get_Param_u32(&val32))
        return PARAM_ERROR_CODE;
    Consigne = val32;

    PutFEETECH(Id, Reg, Consigne);
    return 0;
}

uint8_t Get_FEETECH_Cmd(void){
    uint32_t val32;
    uint8_t Id;
    uint8_t Reg;

    if (Get_Param_u32(&val32))
        return PARAM_ERROR_CODE;
    Id = val32;
    if (Get_Param_u32(&val32))
        return PARAM_ERROR_CODE;
    Reg = val32;
    xil_printf("Get FEETECH Id=%d Reg=%d\r\n", Id, Reg);
    xil_printf("  Value=%d\r\n", GetFEETECH_Wait(Id, Reg));
    return 0;
}

uint8_t Send_FEETECH_SCS_Cmd(void){
    uint32_t val32;
    uint8_t Id;
    uint8_t Reg;
    uint16_t Consigne;
    if (Get_Param_u32(&val32))
        return PARAM_ERROR_CODE;
    Id = val32;
    if (Get_Param_u32(&val32))
        return PARAM_ERROR_CODE;
    Reg = val32;
    if (Get_Param_u32(&val32))
        return PARAM_ERROR_CODE;
    Consigne = val32;

    PutFEETECH_SCS(Id, Reg, Consigne);
    return 0;
}

uint8_t Get_FEETECH_SCS_Cmd(void){
    uint32_t val32;
    uint8_t Id;
    uint8_t Reg;

    if (Get_Param_u32(&val32))
        return PARAM_ERROR_CODE;
    Id = val32;
    if (Get_Param_u32(&val32))
        return PARAM_ERROR_CODE;
    Reg = val32;
    xil_printf("Get FEETECH Id=%d Reg=%d\r\n", Id, Reg);
    xil_printf("  Value=%d\r\n", GetFEETECH_Wait_SCS(Id, Reg));
    return 0;
}

uint8_t feetech_action_state=0;
uint32_t feetech_action_timer=0;
uint8_t feetech_action_done=0;
uint32_t feetech_action_position=0;
uint32_t feetech_test_ctnr=0;

void FEETECH_action_loop(void){
    if(Timer_ms1 - feetech_action_timer >= 1000){
        feetech_action_timer = Timer_ms1;
        feetech_test_ctnr++;
        printf("FEETECH action loop %d\r\n", feetech_test_ctnr);
        switch(feetech_action_state){
            case 0:
                PutFEETECH(10, PUMP_CMD_2, 255);
                feetech_action_state = 1;
                break;
            case 1:
                PutFEETECH(10, PUMP_CMD_2, 0);
                feetech_action_state = 0;
                break;
        }
    }
}


void pince_loop(void){
    for (int i = 0; i < NBR_PINCES; i++) {
        pince_action_loop(&robot_pinces[i]);
    }
}

void pince_action_loop(Pince_t *pince){
    /* ---------------------------------------------------- */
    /* ------------- WATCHDOG TIMER  -----------------------*/
    /* ---------------------------------------------------- */
    if (pince->action_step != pince->previous_step) {
        pince->watchdog_timer = Timer_ms1;
        pince->previous_step = pince->action_step;
    }

    if (pince->action_step > 0 && pince->action_step < 500) {
        // Si on passe plus de 4000 ms (4s) sur la MÊME étape -> Plantage critique
        if (Timer_ms1 - pince->watchdog_timer >= 4000) {
            #ifdef DEBUG_FEETECH_ACTION
                printf("\n[WATCHDOG] Pince %d bloquee a l'etape %d ! Forcage du sas de securite.\n", pince->id, pince->action_step);
            #endif
            // NOTE: le PINCEFEEDBACK sera envoyé par l'étape 506 à la fin de l'abandon,
            // avec pending_feedback_cmd et succes_left/right = 0 (déjà initialisés à 0 ici ou dans les séquences).
            pince->succes_left = 0;
            pince->succes_right = 0;
            pince->action_step = 500;
        }
    }

    switch(pince->action_step){
        case 0:
            pince->current_command = CMD_IDLE;

            pince->succes_left = 0;
            pince->succes_right = 0;
            break;

        /* ---------------------------------------------------- */
        /* ------------- RAMASSER_OBJETS -----------------------*/
        /* ---------------------------------------------------- */

        case 10: // baisser la pince avec overshoot
            pince->pending_feedback_cmd = 1;
            pince->retry_count = 0; 

            // reset succes flags
            pince->succes_left = 0;
            pince->succes_right = 0;

            // Calcul de la direction de l'overshoot (gestion du montage miroir)
            int16_t overshoot = 0;
            if (pince->gros_pos.ramasser_pos > pince->gros_pos.idle_position) {
                overshoot = OVERSHOOT_MARGIN;
            } else {
                overshoot = -OVERSHOOT_MARGIN;
            }
            
            // On recycle la variable last_position pour stocker notre consigne d'overshoot finale
            pince->gros_pos.last_position = pince->gros_pos.ramasser_pos + overshoot;

            PutFEETECH_Ext_Done(pince->id_gros, FEETECH_GOAL_POSITION_L, pince->gros_pos.last_position, &pince->action_done);
            pince->gros_pos.cmd_timer = Timer_ms1;
            pince->action_step++;
            break;

        case 11: // allumer les pompes ciblées
            if(pince->action_done){
                pince->action_done = 0;
                #ifdef DEBUG_FEETECH_ACTION
                    if (pince->retry_count > 0) {
                        printf("pince : %d : Tentative allumage pompes %d/3\n", pince->id, pince->retry_count + 1);
                    }
                #endif
                
                // On n'allume que les pompes demandées
                if (pince->current_command == CMD_RAMASSER_ALL) {
                    PutFEETECH(pince->id_pump, PUMP_CMD_1, PUMP_ON);
                    PutFEETECH_Ext_Done_SCS(pince->id_pump, PUMP_CMD_2, PUMP_ON, &pince->action_done);
                } else if (pince->current_command == CMD_RAMASSER_G) {
                    PutFEETECH_Ext_Done_SCS(pince->id_pump, PUMP_CMD_1, PUMP_ON, &pince->action_done);
                } else if (pince->current_command == CMD_RAMASSER_D) {
                    PutFEETECH_Ext_Done_SCS(pince->id_pump, PUMP_CMD_2, PUMP_ON, &pince->action_done);
                }

                pince->pump_right.cmd_timer = Timer_ms1;
                pince->pump_left.cmd_timer = Timer_ms1;
                pince->action_step++;
            }
            break;

        case 12: // check pump are on
            if(pince->action_done){
                if (Timer_ms1 - pince->pump_left.cmd_timer >= 500){ 
                    pince->action_done = 0; 
                    // On lit toujours les deux pour garder un temps de comm constant
                    GetFEETECH(pince->id_pump, ADDR_CURRENT_1_L, &pince->pump_left.pump_current);
                    GetFEETECH_Ext_Done(pince->id_pump, ADDR_CURRENT_2_L, &pince->pump_right.pump_current, &pince->action_done);
                    pince->action_step++;
                }
            }
            break;

        case 13: 
            if(pince->action_done){
                // On vérifie uniquement les pompes activées (les autres sont considérées "ok" par défaut)
                uint8_t left_ok = (pince->current_command != CMD_RAMASSER_G && pince->current_command != CMD_RAMASSER_ALL) || (pince->pump_left.pump_current > CURRENT_THRESHOLD_ON_ALLUMAGE);
                uint8_t right_ok = (pince->current_command != CMD_RAMASSER_D && pince->current_command != CMD_RAMASSER_ALL) || (pince->pump_right.pump_current > CURRENT_THRESHOLD_ON_ALLUMAGE);

                if(left_ok && right_ok){ 
                    pince->retry_count = 0; 
                    pince->action_done = 0;
                    
                    // Préparation pour l'échantillonnage de base (baseline)
                    pince->pump_right.sum_current = 0;
                    pince->pump_left.sum_current = 0;
                    pince->sample_idx = 0;
                    pince->action_step = 14; 
                } else {
                    pince->retry_count++;
                    if (pince->retry_count >= RETRY_COUNT_MAX) {
                        #ifdef DEBUG_FEETECH_ACTION
                            printf("pince : %d : ERREUR CRITIQUE: Impossible d'allumer les pompes après 3 essais. Abandon.\n", pince->id);
                        #endif
                        pince->retry_count = 0; 
                        pince->action_step = 500;  // PINCEFEEDBACK envoyé par 506
                    } else {
                        pince->action_step = 11; 
                    }
                }
            }
            break;

        // --- ECHANTILLONNAGE BASELINE (A VIDE) ---
        case 14:
            // Toujours les deux lectures pour stabiliser la durée de la boucle physique
            GetFEETECH(pince->id_pump, ADDR_CURRENT_1_L, &pince->pump_left.pump_current);
            GetFEETECH_Ext_Done(pince->id_pump, ADDR_CURRENT_2_L, &pince->pump_right.pump_current, &pince->action_done);
            pince->action_step = 15;
            break;

        case 15:
            if(pince->action_done){
                pince->action_done = 0;
                
                // On calcule le baseline pour les deux côtés en permanence
                pince->pump_left.sum_current += pince->pump_left.pump_current;
                pince->pump_right.sum_current += pince->pump_right.pump_current;
                
                pince->sample_idx++;
                
                if (pince->sample_idx >= NBR_VALUES_FOR_MEAN) { 
                    pince->pump_left.baseline_current = pince->pump_left.sum_current / NBR_VALUES_FOR_MEAN;
                    pince->pump_right.baseline_current = pince->pump_right.sum_current / NBR_VALUES_FOR_MEAN;
                    
                    #ifdef DEBUG_FEETECH_ACTION
                        printf("pince : %d : Baseline trouvé - D:%d, G:%d\n", pince->id, pince->pump_right.baseline_current, pince->pump_left.baseline_current);
                    #endif
                    pince->action_step = 16;
                } else {
                    pince->action_step = 14; 
                }
            }
            break;
        // -------------------------------------------------------------

        case 16: // Lire la position courante pendant la descente
            GetFEETECH_Ext_Done(pince->id_gros, FEETECH_PRESENT_POSITION_L, &pince->gros_pos.current_position, &pince->action_done);
            pince->action_step = 161;
            break;

        case 161:
            if(pince->action_done){
                pince->action_done = 0;

                if(ABS_DIFF(pince->gros_pos.current_position, pince->gros_pos.last_position) < 50){
                    #ifdef DEBUG_FEETECH_ACTION
                        printf("pince %d : Cible overshoot atteinte librement, pas de palet (pos=%d)\n",
                            pince->id, pince->gros_pos.current_position);
                    #endif
                    // *** RESET OBLIGATOIRE : sample_idx vaut encore NBR_VALUES_FOR_MEAN après le baseline ***
                    pince->sample_idx = 0;
                    pince->buffer_full = 0;
                    for(int i = 0; i < NBR_VALUES_FOR_MEAN; i++){
                        pince->pump_left.samples[i]  = 0;
                        pince->pump_right.samples[i] = 0;
                    }
                    pince->action_step = 18;
                } else {
                    pince->action_step = 162;
                }

                if(Timer_ms1 - pince->gros_pos.cmd_timer >= 3000){
                    printf("pince %d : Timeout descente (pos=%d)\n", pince->id, pince->gros_pos.current_position);
                    pince->action_step = 500;
                }
            }
            break;

        case 162: // Lancer la lecture du couple (Load)
            GetFEETECH_Ext_Done(pince->id_gros, FEETECH_PRESENT_LOAD_L, &pince->gros_pos.present_load, &pince->action_done);
            pince->action_step = 17;
            break;

        case 17: // Analyser le couple pour confirmer l'appui sur le palet
            if(pince->action_done){
                pince->action_done = 0;
                
                // On extrait la magnitude du couple en masquant le bit de direction (10ème bit)
                uint16_t load_magnitude = pince->gros_pos.present_load & ADDR_LOAD_MASK;

                if(load_magnitude >= LOAD_THRESHOLD_CONTACT){
                    // Contact confirmé : la pince écrase bien le palet !
                    #ifdef DEBUG_FEETECH_ACTION
                        printf("pince %d : Contact objet detecte par le couple (load=%d)\n", pince->id, load_magnitude);
                    #endif
                    
                    // SÉCURITÉ : On fige le servo à sa position actuelle pour éviter qu'il ne force indéfiniment dans le palet (overload)
                    PutFEETECH(pince->id_gros, FEETECH_GOAL_POSITION_L, pince->gros_pos.current_position);
                    
                    // Init buffer moyenne glissante pour analyse pompes
                    pince->pump_right.sum_current = 0;
                    pince->pump_left.sum_current  = 0;
                    pince->sample_idx  = 0;
                    pince->buffer_full = 0;
                    for(int i = 0; i < NBR_VALUES_FOR_MEAN; i++){
                        pince->pump_right.samples[i] = 0;
                        pince->pump_left.samples[i]  = 0;
                    }
                    pince->action_timer = 0;
                    pince->action_step = 18; // → on passe à l'analyse du vide des ventouses
                } else {
                    // Pas encore assez de couple, on retourne lire la position pour continuer le suivi de la descente
                    pince->action_step = 16;
                }
            }
            break;
        case 18: // Demande de mesure continue
            // Idem, on lit systématiquement les deux pour ne pas diviser par deux le temps d'analyse total
            GetFEETECH(pince->id_pump, ADDR_CURRENT_1_L, &pince->pump_left.pump_current);
            GetFEETECH_Ext_Done(pince->id_pump, ADDR_CURRENT_2_L, &pince->pump_right.pump_current, &pince->action_done);
            pince->action_step = 19;
            break;

        case 19: // Acquisition et Décision (Analyse par Moyenne Glissante)
            if(pince->action_done){
                pince->action_done = 0;
                
                // 1. On stocke l'échantillon courant
                pince->pump_left.samples[pince->sample_idx]  = pince->pump_left.pump_current;
                pince->pump_right.samples[pince->sample_idx] = pince->pump_right.pump_current;

                #ifdef DEBUG_FEETECH_ACTION
                    printf("pince : %d : Sample %d - Mesure courante (G:%d, D:%d)\n",
                        pince->id, pince->sample_idx,
                        pince->pump_left.pump_current, pince->pump_right.pump_current);
                #endif

                pince->sample_idx++;
                
                // 2. Tant qu'on n'a pas les N échantillons, on continue de lire
                if (pince->sample_idx < NBR_VALUES_FOR_MEAN) {
                    pince->action_step = 18; 
                } 
                // 3. Buffer plein : on prend la décision
                else {
                    uint32_t sum_gauche = 0;
                    uint32_t sum_droite = 0;

                    // Calcul de la somme pour la moyenne
                    for(int i = 0; i < NBR_VALUES_FOR_MEAN; i++){
                        sum_gauche += pince->pump_left.samples[i];
                        sum_droite += pince->pump_right.samples[i];
                    }

                    // Calcul de la moyenne
                    uint32_t mean_gauche = sum_gauche / NBR_VALUES_FOR_MEAN;
                    uint32_t mean_droite = sum_droite / NBR_VALUES_FOR_MEAN;

                    uint32_t delta_pct_l = 0;
                    uint32_t delta_pct_r = 0;

                    // Calcul du pourcentage de déviation par rapport à la baseline
                    if (pince->pump_left.baseline_current > 0) {
                        delta_pct_l = (ABS_DIFF(mean_gauche, pince->pump_left.baseline_current) * 100) / pince->pump_left.baseline_current;
                    }
                    if (pince->pump_right.baseline_current > 0) {
                        delta_pct_r = (ABS_DIFF(mean_droite, pince->pump_right.baseline_current) * 100) / pince->pump_right.baseline_current;
                    }

                    pince->succes_left  = 0;
                    pince->succes_right = 0;

                    // Validation du succès par rapport au seuil défini dans .h
                    if(pince->current_command == CMD_RAMASSER_G || pince->current_command == CMD_RAMASSER_ALL){
                        pince->succes_left  = (delta_pct_l >= CURRENT_RATIO_CATCH_PCT);
                    }
                    if(pince->current_command == CMD_RAMASSER_D || pince->current_command == CMD_RAMASSER_ALL){
                        pince->succes_right = (delta_pct_r >= CURRENT_RATIO_CATCH_PCT);
                    }

                    // Coupure des pompes qui ont raté + ouverture valve pour casser le vide inutile
                    if(pince->current_command == CMD_RAMASSER_G || pince->current_command == CMD_RAMASSER_ALL){
                        if(!pince->succes_left){
                            PutFEETECH(pince->id_pump, PUMP_CMD_1, PUMP_OFF);
                            PutFEETECH(pince->id_pump, VALVE_CMD_1, VALVE_ON);
                        }
                    }
                    if(pince->current_command == CMD_RAMASSER_D || pince->current_command == CMD_RAMASSER_ALL){
                        if(!pince->succes_right){
                            PutFEETECH(pince->id_pump, PUMP_CMD_2, PUMP_OFF);
                            PutFEETECH(pince->id_pump, VALVE_CMD_2, VALVE_ON);
                        }
                    }

                    #ifdef DEBUG_FEETECH_ACTION
                        printf("pince %d : G mean=%d baseline=%d delta=%d%% -> Succes:%d | D mean=%d baseline=%d delta=%d%% -> Succes:%d\n",
                            pince->id,
                            (int)mean_gauche, pince->pump_left.baseline_current,  (int)delta_pct_l, pince->succes_left,
                            (int)mean_droite, pince->pump_right.baseline_current, (int)delta_pct_r, pince->succes_right);
                    #endif

                    // 4. On passe à la remontée
                    pince->action_step = 20;
                }
            }
            break;
        
        case 20: //remonter la pince
            PutFEETECH_Ext_Done(pince->id_gros, FEETECH_GOAL_POSITION_L, pince->gros_pos.idle_position, &pince->action_done);
            pince->action_timer = Timer_ms1;
            pince->action_step = 21;
            break;

        case 21:
            if(pince->action_done){
                pince->action_done = 0;
                GetFEETECH_Ext_Done(pince->id_gros, FEETECH_PRESENT_POSITION_L, &pince->action_position, &pince->action_done);
                pince->action_step = 22;
            }
            break;

        case 22:
            if(pince->action_done){
                if((pince->gros_pos.idle_position - 100) <= pince->action_position && pince->action_position <= (pince->gros_pos.idle_position + 100)){
                    #ifdef DEBUG_FEETECH_ACTION
                        printf("pince : %d : Pince action done at position %d\n", pince->id, pince->action_position);
                    #endif
                    printf("PINCEFEEDBACK %d 1 %d %d\n", pince->id, pince->succes_left, pince->succes_right);
                    pince->action_done = 0;
                    pince->action_step = 0;
                } else {
                    pince->action_step = 21; // keep waiting
                }
            }

            // Timeout de 3s : la pince a un problème, on abandonne
            if (Timer_ms1 - pince->action_timer >= 3000){
                #ifdef DEBUG_FEETECH_ACTION
                    printf("pince : %d : Pince action timeout at position %d\n", pince->id, pince->action_position);
                #endif
                pince->succes_left = 0;
                pince->succes_right = 0;
                pince->action_step = 500; // PINCEFEEDBACK envoyé par 506
            }
            break;

        
        /* ---------------------------------------------------- */
        /* ------------- MONTER_PINCE --------------------------*/
        /* ---------------------------------------------------- */

        case 100:
            // CONTEXTE D'ABANDON : tout goto 500 depuis les étapes 100-102 enverra le feedback CMD 6
            pince->pending_feedback_cmd = 6;
            PutFEETECH_Ext_Done(pince->id_gros, FEETECH_GOAL_POSITION_L, pince->gros_pos.idle_position, &pince->action_done);
            pince->action_timer = Timer_ms1;
            pince->action_step = 101;
            break;
        case 101:
            if(pince->action_done){
                pince->action_done = 0;
                GetFEETECH_Ext_Done(pince->id_gros, FEETECH_PRESENT_POSITION_L, &pince->gros_pos.current_position, &pince->action_done);
                pince->action_step = 102;
            }
            break;
        case 102:
            if(pince->action_done){
                if((pince->gros_pos.idle_position - 100) <= pince->gros_pos.current_position && pince->gros_pos.current_position <= (pince->gros_pos.idle_position + 100)){
                    #ifdef DEBUG_FEETECH_ACTION
                        printf("pince : %d : Pince action done at position %d\n", pince->id, pince->gros_pos.current_position);
                    #endif
                    printf("PINCEFEEDBACK %d 6 1 1\n", pince->id);
                    pince->action_done = 0;
                    pince->action_step = 0;
                } else {
                    pince->action_step = 101; // keep waiting
                }
            }

            if (Timer_ms1 - pince->action_timer >= 3000){
                #ifdef DEBUG_FEETECH_ACTION
                    printf("pince : %d : Pince action timeout at position %d\n", pince->id, pince->gros_pos.current_position);
                #endif
                pince->succes_left = 0;
                pince->succes_right = 0;
                pince->action_step = 500; // PINCEFEEDBACK envoyé par 506
            }
            break;

        
        /* ---------------------------------------------------- */
        /* ------------- RETOURNER PALETS ----------------------*/
        /* ---------------------------------------------------- */

        case 200: // Ordonner la descente du bras
            // CONTEXTE D'ABANDON : tout goto 500 depuis les étapes 200-215 enverra le feedback CMD 3
            pince->pending_feedback_cmd = 3;
            PutFEETECH_Ext_Done(pince->id_gros, FEETECH_GOAL_POSITION_L, pince->gros_pos.lacher_pos, &pince->action_done);

            // set succes flags 
            pince->succes_left = 0;
            pince->succes_right = 0;

            pince->action_timer = Timer_ms1;
            pince->action_step = 201;
            break;

        case 201: // Demander la position
            if(pince->action_done){
                pince->action_done = 0;
                GetFEETECH_Ext_Done(pince->id_gros, FEETECH_PRESENT_POSITION_L, &pince->gros_pos.current_position, &pince->action_done);
                pince->action_step = 202;
            }
            break;

        case 202: // Vérifier l'arrivée en position
            if(pince->action_done){
                if((pince->gros_pos.lacher_pos - 50) <= pince->gros_pos.current_position && pince->gros_pos.current_position <= (pince->gros_pos.lacher_pos + 50)){
                    #ifdef DEBUG_FEETECH_ACTION
                        printf("pince : %d : Pince en position de lacher (%d)\n", pince->id, pince->gros_pos.current_position);
                    #endif
                    pince->action_done = 0;
                    pince->action_step = 203;
                } else {
                    pince->action_step = 201; // Continue de vérifier
                }
            }

            if (Timer_ms1 - pince->action_timer >= 3000){
                #ifdef DEBUG_FEETECH_ACTION
                    printf("pince : %d : Pince action timeout at position %d\n", pince->id, pince->gros_pos.current_position);
                #endif
                pince->action_step = 500; // PINCEFEEDBACK envoyé par 506
            }
            break;

        case 203: // Ouvrir les clapets (les sortir)
            if(pince->current_command == CMD_LACHER_G || pince->current_command == CMD_LACHER_ALL){
                #ifdef DEBUG_FEETECH_ACTION
                    printf("pince : %d : Ouverture clapet gauche a la position %d\n", pince->id, pince->petit_gauche_pos.sortie_pos);
                #endif
                PutFEETECH_Ext_Done_SCS(pince->id_gauche, FEETECH_GOAL_POSITION_L, pince->petit_gauche_pos.sortie_pos, &pince->action_done);
            }
            if(pince->current_command == CMD_LACHER_D || pince->current_command == CMD_LACHER_ALL){
                #ifdef DEBUG_FEETECH_ACTION
                    printf("pince : %d : Ouverture clapet droite a la position %d\n", pince->id, pince->petit_droite_pos.sortie_pos);
                #endif
                PutFEETECH_Ext_Done_SCS(pince->id_droite, FEETECH_GOAL_POSITION_L, pince->petit_droite_pos.sortie_pos, &pince->action_done);
            }
            pince->action_timer = Timer_ms1;
            pince->action_step = 204;
            break;

        case 204: // Demander la position des clapets
            if(pince->action_done){
                pince->action_done = 0;
                if(pince->current_command == CMD_LACHER_G || pince->current_command == CMD_LACHER_ALL){
                    GetFEETECH_Ext_Done_SCS(pince->id_gauche, FEETECH_PRESENT_POSITION_L, &pince->petit_gauche_pos.current_position, &pince->action_done);
                }
                if(pince->current_command == CMD_LACHER_D || pince->current_command == CMD_LACHER_ALL){
                    GetFEETECH_Ext_Done_SCS(pince->id_droite, FEETECH_PRESENT_POSITION_L, &pince->petit_droite_pos.current_position, &pince->action_done);
                }
                pince->action_step = 205;
            }
            break;

        case 205: // Vérifier l'ouverture des clapets
            if(pince->action_done){
                pince->succes_left = 1;
                pince->succes_right = 1;

                if(pince->current_command == CMD_LACHER_G || pince->current_command == CMD_LACHER_ALL){
                    if(!( (pince->petit_gauche_pos.sortie_pos - 100) <= pince->petit_gauche_pos.current_position && pince->petit_gauche_pos.current_position <= (pince->petit_gauche_pos.sortie_pos + 100) )){
                        pince->succes_left = 0;
                    }
                }
                if(pince->current_command == CMD_LACHER_D || pince->current_command == CMD_LACHER_ALL){
                    if(!( (pince->petit_droite_pos.sortie_pos - 100) <= pince->petit_droite_pos.current_position && pince->petit_droite_pos.current_position <= (pince->petit_droite_pos.sortie_pos + 100) )){
                        pince->succes_right = 0;
                    }
                }

                if(pince->succes_left && pince->succes_right){
                    pince->action_done = 0;
                    pince->retry_count = 0;
                    pince->action_step = 206;
                } else {
                    pince->action_step = 204; // Continue de vérifier
                }
            }

            if (Timer_ms1 - pince->action_timer >= 3000){
                #ifdef DEBUG_FEETECH_ACTION
                    printf("Timeout ouverture clapets\n");
                #endif
                pince->succes_left = 0;
                pince->succes_right = 0;
                pince->action_step = 500; // PINCEFEEDBACK envoyé par 506
            }
            break;

        case 206: // Éteindre les pompes
            if(pince->current_command == CMD_LACHER_G || pince->current_command == CMD_LACHER_ALL){
                PutFEETECH(pince->id_pump, PUMP_CMD_1, PUMP_OFF);
            }
            if(pince->current_command == CMD_LACHER_D || pince->current_command == CMD_LACHER_ALL){
                PutFEETECH(pince->id_pump, PUMP_CMD_2, PUMP_OFF);
            }
            pince->action_timer = Timer_ms1;
            pince->action_step = 207;
            break;

        case 207: // Demander les courants (après une courte pause de 100ms pour l'inertie)
            if(Timer_ms1 - pince->action_timer >= 100){
                pince->action_done = 0;
                if(pince->current_command == CMD_LACHER_G || pince->current_command == CMD_LACHER_ALL){
                    GetFEETECH_Ext_Done(pince->id_pump, ADDR_CURRENT_1_L, &pince->pump_left.pump_current, &pince->action_done);
                }
                if(pince->current_command == CMD_LACHER_D || pince->current_command == CMD_LACHER_ALL){
                    GetFEETECH_Ext_Done(pince->id_pump, ADDR_CURRENT_2_L, &pince->pump_right.pump_current, &pince->action_done);
                }
                pince->action_step = 208;
            }
            break;

        case 208: // Vérifier la chute de courant (Pompes bien éteintes)
            if(pince->action_done){
                pince->action_done = 0;
                uint8_t pump_ok = 1;
                
                if(pince->current_command == CMD_LACHER_G || pince->current_command == CMD_LACHER_ALL){
                    if(pince->pump_left.pump_current > CURRENT_THRESHOLD_ON_EXTINCTION) pump_ok = 0;
                }
                if(pince->current_command == CMD_LACHER_D || pince->current_command == CMD_LACHER_ALL){
                    if(pince->pump_right.pump_current > CURRENT_THRESHOLD_ON_EXTINCTION) pump_ok = 0;
                }

                if(pump_ok){
                    #ifdef DEBUG_FEETECH_ACTION
                        printf("pince : %d : Pompes coupees. Activation valves.\n", pince->id);
                    #endif
                    pince->retry_count = 0;
                    pince->action_step = 209;
                } else {
                    pince->retry_count++;
                    if (pince->retry_count >= RETRY_COUNT_MAX){ 
                        #ifdef DEBUG_FEETECH_ACTION
                            printf("pince : %d : ERREUR CRITIQUE: Le courant pompe ne chute pas apres 3 tentatives. Abandon.\n", pince->id);
                        #endif
                        pince->succes_left = 0;
                        pince->succes_right = 0;
                        pince->action_step = 500; // PINCEFEEDBACK envoyé par 506
                    } else {
                        #ifdef DEBUG_FEETECH_ACTION
                            printf("pince : %d : Echec extinction pompes, tentative %d/3...\n", pince->id, pince->retry_count + 1);
                        #endif
                        pince->action_step = 206;
                    }
                }
            }
            break;

        case 209: // Allumer les électrovannes pour casser le vide et lâcher l'objet
            if(pince->current_command == CMD_LACHER_G || pince->current_command == CMD_LACHER_ALL){
                PutFEETECH(pince->id_pump, VALVE_CMD_1, VALVE_ON);
            }
            if(pince->current_command == CMD_LACHER_D || pince->current_command == CMD_LACHER_ALL){
                PutFEETECH(pince->id_pump, VALVE_CMD_2, VALVE_ON);
            }
            pince->action_timer = Timer_ms1;
            pince->action_step = 210;
            break;

        case 210: // Attendre 1s que ça tombe, puis retirer les clapets
            if(Timer_ms1 - pince->action_timer >= 1000){
                if(pince->current_command == CMD_LACHER_G || pince->current_command == CMD_LACHER_ALL){
                    PutFEETECH_Ext_Done_SCS(pince->id_gauche, FEETECH_GOAL_POSITION_L, pince->petit_gauche_pos.retrait_pos, &pince->action_done);
                }
                if(pince->current_command == CMD_LACHER_D || pince->current_command == CMD_LACHER_ALL){
                    PutFEETECH_Ext_Done_SCS(pince->id_droite, FEETECH_GOAL_POSITION_L, pince->petit_droite_pos.retrait_pos, &pince->action_done);
                }
                pince->action_timer = Timer_ms1;
                pince->action_step = 211;
            }
            break;

        case 211: // Demander la position de retrait
            if(pince->action_done){
                pince->action_done = 0;
                if(pince->current_command == CMD_LACHER_G || pince->current_command == CMD_LACHER_ALL){
                    GetFEETECH_Ext_Done_SCS(pince->id_gauche, FEETECH_PRESENT_POSITION_L, &pince->petit_gauche_pos.current_position, &pince->action_done);
                }
                if(pince->current_command == CMD_LACHER_D || pince->current_command == CMD_LACHER_ALL){
                    GetFEETECH_Ext_Done_SCS(pince->id_droite, FEETECH_PRESENT_POSITION_L, &pince->petit_droite_pos.current_position, &pince->action_done);
                }
                pince->action_step = 212;
            }
            break;

        case 212: // Vérifier que les clapets sont bien rentrés
            if(pince->action_done){ 
                pince->succes_left = 1;
                pince->succes_right = 1;
                
                if(pince->current_command == CMD_LACHER_G || pince->current_command == CMD_LACHER_ALL){
                    if(!( (pince->petit_gauche_pos.retrait_pos - 100) <= pince->petit_gauche_pos.current_position && pince->petit_gauche_pos.current_position <= (pince->petit_gauche_pos.retrait_pos + 100) )){
                        pince->succes_left = 0;
                    }
                }
                if(pince->current_command == CMD_LACHER_D || pince->current_command == CMD_LACHER_ALL){
                    if(!( (pince->petit_droite_pos.retrait_pos - 100) <= pince->petit_droite_pos.current_position && pince->petit_droite_pos.current_position <= (pince->petit_droite_pos.retrait_pos + 100) )){
                        pince->succes_right = 0;
                    }
                }

                if(pince->succes_left && pince->succes_right){
                    #ifdef DEBUG_FEETECH_ACTION
                        printf("pince : %d : Clapets rentres, depot termine avec succes.\n", pince->id);
                    #endif
                    pince->action_done = 0;
                    pince->action_step = 213;
                } else {
                    pince->action_step = 211; // Continue de vérifier
                }
            }

            if (Timer_ms1 - pince->action_timer >= 3000){
                #ifdef DEBUG_FEETECH_ACTION
                    printf("pince : %d : Timeout retrait clapets\n", pince->id);
                #endif   
                pince->succes_left = 0;
                pince->succes_right = 0;                 
                pince->action_step = 500; // PINCEFEEDBACK envoyé par 506
            }   
            break;

        case 213: // Remonter la pince après le dépôt
            PutFEETECH_Ext_Done(pince->id_gros, FEETECH_GOAL_POSITION_L, pince->gros_pos.idle_position, &pince->action_done);
            pince->action_timer = Timer_ms1;
            pince->action_step = 214;
            break;

        case 214: // Vérifier la remontée
            if(pince->action_done){
                pince->action_done = 0;
                GetFEETECH_Ext_Done(pince->id_gros, FEETECH_PRESENT_POSITION_L, &pince->gros_pos.current_position, &pince->action_done);
                pince->action_step = 215;
            }
            break;

        case 215:
            if(pince->action_done){
                if((pince->gros_pos.idle_position - 100) <= pince->gros_pos.current_position && pince->gros_pos.current_position <= (pince->gros_pos.idle_position + 100)){
                    #ifdef DEBUG_FEETECH_ACTION
                        printf("pince : %d : Pince remontee en position idle (%d)\n", pince->id, pince->gros_pos.current_position);
                    #endif
                    printf("PINCEFEEDBACK %d 3 %d %d\n", pince->id, pince->succes_left, pince->succes_right);
                    pince->action_done = 0;
                    pince->action_step = 0;
                } else {
                    pince->action_step = 214; // Continue de vérifier
                }
            }

            if (Timer_ms1 - pince->action_timer >= 3000){
                #ifdef DEBUG_FEETECH_ACTION
                    printf("pince : %d : Timeout remontée pince après dépôt (pos: %d)\n", pince->id, pince->gros_pos.current_position);
                #endif
                pince->succes_left = 0;
                pince->succes_right = 0;
                pince->action_step = 500; // PINCEFEEDBACK envoyé par 506
            }
            break;
        
        /* ---------------------------------------------------- */
        /* ------------- DEPOSER PALETS ----------------------*/
        /* ---------------------------------------------------- */
        
        case 300: // 1. Ordre de baisser la pince
            // CONTEXTE D'ABANDON : tout goto 500 depuis les étapes 300-312 enverra le feedback CMD 2
            pince->pending_feedback_cmd = 2;
            pince->retry_count = 0; 
            PutFEETECH_Ext_Done(pince->id_gros, FEETECH_GOAL_POSITION_L, pince->gros_pos.deposer_pos, &pince->action_done); 
            pince->gros_pos.cmd_timer = Timer_ms1;

            //init succes flags for safety
            pince->succes_left = 0; 
            pince->succes_right = 0;

            pince->action_step = 301;
            break;
            
        case 301: // 2. Attendre que l'ordre de descente soit bien transmis
            if(pince->action_done){
                pince->action_done = 0;
                // On met une valeur "impossible" dans current_position pour forcer une vraie lecture
                pince->gros_pos.current_position = 0xFFFF; 
                pince->action_step = 302;
            }
            break;
            
        case 302: // 3. Demander la position actuelle
            GetFEETECH_Ext_Done(pince->id_gros, FEETECH_PRESENT_POSITION_L, &pince->gros_pos.current_position, &pince->action_done);
            pince->action_step = 303;
            break;
            
        case 303: // 4. Attendre la réponse et vérifier la position
            if(pince->action_done){
                pince->action_done = 0; 
                
                // On vérifie si on est bien arrivé en position basse
                if((pince->gros_pos.deposer_pos - 100) <= pince->gros_pos.current_position && pince->gros_pos.current_position <= (pince->gros_pos.deposer_pos + 100)){
                    #ifdef DEBUG_FEETECH_ACTION
                        printf("pince : %d : Pince en position de depot (%d)\n", pince->id, pince->gros_pos.current_position);
                    #endif
                    pince->action_step = 304;
                } else {
                    pince->action_step = 302; // Pas encore en bas : on redemande la position
                }
            }

            if (Timer_ms1 - pince->gros_pos.cmd_timer >= 3000){
                #ifdef DEBUG_FEETECH_ACTION
                    printf("pince : %d : Timeout descente pince depot (pos: %d)\n", pince->id, pince->gros_pos.current_position);
                #endif
                pince->action_step = 500; // PINCEFEEDBACK envoyé par 506
            }
            break;
            
        case 304: // 5. COUPER LES POMPES EN PREMIER
            if(pince->current_command == CMD_LACHER_G || pince->current_command == CMD_LACHER_ALL){
                PutFEETECH(pince->id_pump, PUMP_CMD_1, PUMP_OFF);
            }
            if(pince->current_command == CMD_LACHER_D || pince->current_command == CMD_LACHER_ALL){
                PutFEETECH(pince->id_pump, PUMP_CMD_2, PUMP_OFF);
            }
            
            pince->action_timer = Timer_ms1;
            pince->action_step = 305;
            break;
            
        case 305: // 6. LAISSER UN MINI TEMPS DE 100ms
            if(Timer_ms1 - pince->action_timer >= 100){
                pince->action_step = 306;
            }
            break;

        case 306: // 7. ACTIVER LES ELECTROVANNES POUR CASSER LE VIDE
            if(pince->current_command == CMD_LACHER_G || pince->current_command == CMD_LACHER_ALL){
                PutFEETECH(pince->id_pump, VALVE_CMD_1, VALVE_ON);
            }
            if(pince->current_command == CMD_LACHER_D || pince->current_command == CMD_LACHER_ALL){
                PutFEETECH(pince->id_pump, VALVE_CMD_2, VALVE_ON);
            }
            pince->action_step = 307;
            break;
            
        case 307: // 8. DEMANDER LES COURANTS POUR VERIFIER LA CHUTE
            pince->action_done = 0;
            if(pince->current_command == CMD_LACHER_G || pince->current_command == CMD_LACHER_ALL){
                GetFEETECH_Ext_Done(pince->id_pump, ADDR_CURRENT_1_L, &pince->pump_left.pump_current, &pince->action_done);
            }
            if(pince->current_command == CMD_LACHER_D || pince->current_command == CMD_LACHER_ALL){
                GetFEETECH_Ext_Done(pince->id_pump, ADDR_CURRENT_2_L, &pince->pump_right.pump_current, &pince->action_done);
            }
            pince->action_step = 308;
            break;
            
        case 308: // 9. VERIFIER LA CHUTE DE COURANT
            if(pince->action_done){
                pince->action_done = 0;
                uint8_t pump_ok = 1;
                
                if(pince->current_command == CMD_LACHER_G || pince->current_command == CMD_LACHER_ALL){
                    if(pince->pump_left.pump_current > CURRENT_THRESHOLD_ON_EXTINCTION){ 
                        pump_ok = 0;
                        pince->succes_left = 0;
                    }
                    else {
                        pince->succes_left = 1;
                    }
                }
                if(pince->current_command == CMD_LACHER_D || pince->current_command == CMD_LACHER_ALL){
                    if(pince->pump_right.pump_current > CURRENT_THRESHOLD_ON_EXTINCTION){ 
                        pump_ok = 0;
                        pince->succes_right = 0;
                    }else{
                        pince->succes_right = 1;
                    }
                }

                if(pump_ok){
                    #ifdef DEBUG_FEETECH_ACTION
                        printf("pince : %d : Courant bas, pompes coupees, palet relache.\n", pince->id);
                    #endif
                    pince->retry_count = 0;
                    pince->action_step = 309;
                } else {
                    pince->retry_count++;
                    if(pince->retry_count >= 50){ 
                        // Au bout de 50 essais (environ 1 seconde de boucle), on abandonne
                        #ifdef DEBUG_FEETECH_ACTION
                            printf("pince : %d : ERREUR CRITIQUE: Le courant ne chute pas (%d / %d). Abandon.\n", pince->id, pince->pump_left.pump_current, pince->pump_right.pump_current);
                        #endif
                        pince->action_step = 500; // PINCEFEEDBACK envoyé par 506
                    } else {
                        // Le rotor tourne encore un peu, on refait une mesure
                        pince->action_step = 307; 
                    }
                }
            }
            break;

        case 309: // 10. Ordre de remonter la pince
            PutFEETECH_Ext_Done(pince->id_gros, FEETECH_GOAL_POSITION_L, pince->gros_pos.idle_position, &pince->action_done);
            pince->action_timer = Timer_ms1;
            pince->action_step = 310;
            break;

        case 310: // 11. Attendre que l'ordre de remontée parte
            if(pince->action_done){
                pince->action_done = 0;
                pince->action_step = 311;
            }
            break;
            
        case 311: // 12. Demander position actuelle
            GetFEETECH_Ext_Done(pince->id_gros, FEETECH_PRESENT_POSITION_L, &pince->action_position, &pince->action_done);
            pince->action_step = 312;
            break;
            
        case 312: // 13. Verifier si on est en haut
            if(pince->action_done){
                pince->action_done = 0;
                if((pince->gros_pos.idle_position - 100) <= pince->action_position && pince->action_position <= (pince->gros_pos.idle_position + 100)){
                    #ifdef DEBUG_FEETECH_ACTION
                        printf("Pince remontee at position %d\n", pince->action_position);
                    #endif
                    pince->succes_left = 1;
                    pince->succes_right = 1;
                    printf("PINCEFEEDBACK %d 2 %d %d\n", pince->id, pince->succes_left, pince->succes_right);
                    pince->action_step = 0; // FIN DU CYCLE
                } else if (Timer_ms1 - pince->action_timer >= 3000){
                    #ifdef DEBUG_FEETECH_ACTION
                        printf("pince : %d : Timeout remontee pince (pos: %d)\n", pince->id, pince->action_position);
                    #endif
                    pince->succes_left = 0;
                    pince->succes_right = 0;
                    pince->action_step = 500; // PINCEFEEDBACK envoyé par 506
                } else {
                    pince->action_step = 311; // keep waiting
                }
            }
            break;

        /* ---------------------------------------------------- */
        /* ------ DEPOSER UN ET RETOURNER L'AUTRE (400) --------*/
        /* ---------------------------------------------------- */        
        case 400: // 1. Ordonner la descente totale au sol
            // CONTEXTE D'ABANDON : tout goto 500 depuis les étapes 400-413 enverra le feedback CMD 4
            pince->pending_feedback_cmd = 4;
            pince->retry_count = 0;
            pince->succes_left = 0;
            pince->succes_right = 0;
            PutFEETECH_Ext_Done(pince->id_gros, FEETECH_GOAL_POSITION_L, pince->gros_pos.deposer_pos, &pince->action_done);
            pince->action_timer = Timer_ms1;
            pince->action_step = 401;
            break;

        case 401: // 2. Attendre envoi et demander position
            if(pince->action_done){
                pince->action_done = 0;
                GetFEETECH_Ext_Done(pince->id_gros, FEETECH_PRESENT_POSITION_L, &pince->gros_pos.current_position, &pince->action_done);
                pince->action_step = 402;
            }
            break;

        case 402: // 3. Vérifier arrivée au sol
            if(pince->action_done){
                if(ABS_DIFF(pince->gros_pos.deposer_pos, pince->gros_pos.current_position) < 100){
                    #ifdef DEBUG_FEETECH_ACTION
                        printf("pince %d : Arrivée au sol pour dépose.\n", pince->id);
                    #endif
                    pince->action_done = 0;
                    pince->action_step = 403;
                } else {
                    pince->action_step = 401; // On reboucle lecture
                }
            }

            if (Timer_ms1 - pince->action_timer >= 3000){
                pince->action_step = 500; // PINCEFEEDBACK envoyé par 506
            }
            break;

        case 403: // 4. Lâcher le PREMIER objet (Dépose)
            if(pince->current_command == CMD_DEPOSE_G_RETOURNE_D) {
                PutFEETECH(pince->id_pump, PUMP_CMD_1, PUMP_OFF);
                PutFEETECH(pince->id_pump, VALVE_CMD_1, VALVE_ON);
                pince->succes_left = 1;
            } else {
                PutFEETECH(pince->id_pump, PUMP_CMD_2, PUMP_OFF);
                PutFEETECH(pince->id_pump, VALVE_CMD_2, VALVE_ON);
                pince->succes_right = 1;
            }
            pince->action_timer = Timer_ms1;
            pince->action_step = 404;
            break;

        case 404: // 5. Petite pause et remontée à la position clapet (lacher_pos)
            if(Timer_ms1 - pince->action_timer >= 50){
                PutFEETECH_Ext_Done(pince->id_gros, FEETECH_GOAL_POSITION_L, pince->gros_pos.lacher_pos, &pince->action_done);
                pince->action_timer = Timer_ms1;
                pince->action_step = 405;
            }
            break;

        case 405: // 6. Attendre envoi et demander position intermédiaire
            if(pince->action_done){
                pince->action_done = 0;
                GetFEETECH_Ext_Done(pince->id_gros, FEETECH_PRESENT_POSITION_L, &pince->gros_pos.current_position, &pince->action_done);
                pince->action_step = 406;
            }
            break;

        case 406: // 7. Vérifier arrivée position clapet
            if(pince->action_done){
                if(ABS_DIFF(pince->gros_pos.lacher_pos, pince->gros_pos.current_position) < 50){
                    pince->action_done = 0;
                    pince->action_step = 407;
                } else {
                    pince->action_step = 405;
                }
            }
            break;

        case 407: // 8. Sortir le clapet du SECOND objet (Retourne)
            if(pince->current_command == CMD_DEPOSE_G_RETOURNE_D){
                PutFEETECH_Ext_Done_SCS(pince->id_droite, FEETECH_GOAL_POSITION_L, pince->petit_droite_pos.sortie_pos, &pince->action_done);
            } else {
                PutFEETECH_Ext_Done_SCS(pince->id_gauche, FEETECH_GOAL_POSITION_L, pince->petit_gauche_pos.sortie_pos, &pince->action_done);
            }
            pince->action_timer = Timer_ms1;
            pince->action_step = 408;
            break;

        case 408: // 9. Attendre sortie clapet et demander position clapet
            if(pince->action_done){
                pince->action_done = 0;
                if(pince->current_command == CMD_DEPOSE_G_RETOURNE_D)
                    GetFEETECH_Ext_Done_SCS(pince->id_droite, FEETECH_PRESENT_POSITION_L, &pince->petit_droite_pos.current_position, &pince->action_done);
                else
                    GetFEETECH_Ext_Done_SCS(pince->id_gauche, FEETECH_PRESENT_POSITION_L, &pince->petit_gauche_pos.current_position, &pince->action_done);
                pince->action_step = 409;
            }
            break;

        case 409: // 10. Vérifier clapet sorti et couper pompe 2
            if(pince->action_done){
                uint16_t pos = (pince->current_command == CMD_DEPOSE_G_RETOURNE_D) ? pince->petit_droite_pos.current_position : pince->petit_gauche_pos.current_position;
                uint16_t target = (pince->current_command == CMD_DEPOSE_G_RETOURNE_D) ? pince->petit_droite_pos.sortie_pos : pince->petit_gauche_pos.sortie_pos;
                
                if(ABS_DIFF(target, pos) < 100){
                    if(pince->current_command == CMD_DEPOSE_G_RETOURNE_D){
                        PutFEETECH(pince->id_pump, PUMP_CMD_2, PUMP_OFF);
                        PutFEETECH(pince->id_pump, VALVE_CMD_2, VALVE_ON);
                        pince->succes_right = 1;
                    } else {
                        PutFEETECH(pince->id_pump, PUMP_CMD_1, PUMP_OFF);
                        PutFEETECH(pince->id_pump, VALVE_CMD_1, VALVE_ON);
                        pince->succes_left = 1;
                    }
                    pince->action_timer = Timer_ms1;
                    pince->action_step = 410;
                } else {
                    pince->action_step = 408; // On attend encore
                }
            }
            break;

        case 410: // 11. Attendre 350ms le retournement, puis rentrer clapet
            if(Timer_ms1 - pince->action_timer >= 350){
                if(pince->current_command == CMD_DEPOSE_G_RETOURNE_D)
                    PutFEETECH_Ext_Done_SCS(pince->id_droite, FEETECH_GOAL_POSITION_L, pince->petit_droite_pos.retrait_pos, &pince->action_done);
                else
                    PutFEETECH_Ext_Done_SCS(pince->id_gauche, FEETECH_GOAL_POSITION_L, pince->petit_gauche_pos.retrait_pos, &pince->action_done);
                pince->action_timer = Timer_ms1;
                pince->action_step = 411;
            }
            break;

        case 411: // 12. Attendre retrait clapet et ordonner remontée finale
            if(pince->action_done){
                pince->action_done = 0;
                PutFEETECH_Ext_Done(pince->id_gros, FEETECH_GOAL_POSITION_L, pince->gros_pos.idle_position, &pince->action_done);
                pince->action_timer = Timer_ms1;
                pince->action_step = 412;
            }
            break;

        case 412: // 13. Vérifier remontée finale (Fin de séquence)
            if(pince->action_done){
                pince->action_done = 0;
                GetFEETECH_Ext_Done(pince->id_gros, FEETECH_PRESENT_POSITION_L, &pince->gros_pos.current_position, &pince->action_done);
                pince->action_step = 413;
            }
            break;

        case 413:
            if(pince->action_done){
                if(ABS_DIFF(pince->gros_pos.idle_position, pince->gros_pos.current_position) < 100){
                    printf("PINCEFEEDBACK %d 4 %d %d\n", pince->id, pince->succes_left, pince->succes_right);
                    pince->action_step = 0;
                } else if (Timer_ms1 - pince->action_timer >= 3000){
                    pince->succes_left = 0;
                    pince->succes_right = 0;
                    pince->action_step = 500; // PINCEFEEDBACK envoyé par 506
                } else {
                    pince->action_step = 412;
                }
            }
            break;

        /* ---------------------------------------------------- */
        /* ------------- ARRET FORCE DES POMPES (600) --------- */
        /* ---------------------------------------------------- */
        case 600:
            // 1. Eteindre les pompes
            PutFEETECH(pince->id_pump, PUMP_CMD_1, PUMP_OFF);
            PutFEETECH(pince->id_pump, PUMP_CMD_2, PUMP_OFF);
            
            // 2. Ouvrir les valves pour relâcher instantanément un objet éventuel
            PutFEETECH(pince->id_pump, VALVE_CMD_1, VALVE_ON);
            PutFEETECH(pince->id_pump, VALVE_CMD_2, VALVE_ON);

            #ifdef DEBUG_FEETECH_ACTION
                printf("pince : %d : [ARRET FORCE] Pompes coupees, valves ouvertes.\n", pince->id);
            #endif
            
            // 3. Retour à l'état IDLE (pas de PINCEFEEDBACK : commande d'arret forcé externe)
            pince->action_done = 0;
            pince->action_step = 0;
            break;

        /* ---------------------------------------------------- */
        /* ------------- SEQUENCE D'ABANDON / ERREUR -----------*/
        /* ---------------------------------------------------- */
        case 500: // Sas d'initialisation de la mise en sécurité
            pince->retry_count = 0;
            pince->action_step = 501;
            break;

        case 501:
            #ifdef DEBUG_FEETECH_ACTION
            printf( "\n\n====================\n" );
            printf("pince : %d : Pince Abort ! Extinction pompes (Essai %d/3)...\n", pince->id, pince->retry_count + 1);
            #endif
            // 1. Eteindre les pompes
            PutFEETECH(pince->id_pump, PUMP_CMD_1, PUMP_OFF);
            PutFEETECH(pince->id_pump, PUMP_CMD_2, PUMP_OFF);
            
            pince->action_timer = Timer_ms1;
            pince->action_step = 502;
            break;

        case 502:
            // On laisse un court instant (250ms) pour que l'inertie du moteur tombe
            if(Timer_ms1 - pince->action_timer >= 250){
                GetFEETECH(pince->id_pump, ADDR_CURRENT_1_L, &pince->pump_right.pump_current);
                GetFEETECH_Ext_Done(pince->id_pump, ADDR_CURRENT_2_L, &pince->pump_left.pump_current, &pince->action_done);
                pince->action_step = 503;
            }
            break;

        case 503:
            if(pince->action_done){
                pince->action_done = 0;
                pince->succes_left = 0;
                pince->succes_right = 0;
                if(pince->pump_right.pump_current < CURRENT_THRESHOLD_ON_EXTINCTION && pince->pump_left.pump_current < CURRENT_THRESHOLD_ON_EXTINCTION){
                    #ifdef DEBUG_FEETECH_ACTION
                        printf("Pompes confirmées éteintes.\n");
                    #endif
                    pince->retry_count = 0;
                    pince->action_step = 504;
                } else {
                    pince->retry_count++;
                    if(pince->retry_count >= RETRY_COUNT_MAX){
                        #ifdef DEBUG_FEETECH_ACTION
                            printf("pince : %d : ERREUR CRITIQUE: Impossible d'éteindre les pompes après 3 essais. Poursuite de l'abandon.\n", pince->id);
                        #endif
                        pince->retry_count = 0;
                        pince->action_step = 504; // On force la suite pour ne pas bloquer le robot
                    } else {
                        #ifdef DEBUG_FEETECH_ACTION
                            printf("pince : %d : Echec extinction pompes (D:%d, G:%d), nouvel essai...\n", pince->id, pince->pump_right.pump_current, pince->pump_left.pump_current);
                        #endif
                        pince->action_step = 501;
                    }
                }
            }
            break;

        case 504:
            // Ouvrir les valves pour relâcher un éventuel objet en succion
            PutFEETECH(pince->id_pump, VALVE_CMD_1, VALVE_ON);
            PutFEETECH(pince->id_pump, VALVE_CMD_2, VALVE_ON);
            
            // Ordonner la remontée de sécurité
            PutFEETECH_Ext_Done(pince->id_gros, FEETECH_GOAL_POSITION_L, pince->gros_pos.idle_position, &pince->action_done);
            
            pince->action_timer = Timer_ms1;
            pince->action_step = 505;
            break;
            
        case 505:
            if(pince->action_done){
                pince->action_done = 0;
                GetFEETECH_Ext_Done(pince->id_gros, FEETECH_PRESENT_POSITION_L, &pince->action_position, &pince->action_done);
                pince->action_step = 506;
            }
            break;
            
        case 506:
            if(pince->action_done){
                if((pince->gros_pos.idle_position - 100) <= pince->action_position && pince->action_position <= (pince->gros_pos.idle_position + 100)){
                    #ifdef DEBUG_FEETECH_ACTION
                        printf("pince : %d : Pince action annulée et retour au repos ok\n", pince->id);
                    #endif
                    // Envoi du feedback d'abandon avec le code de la commande interrompue
                    printf("PINCEFEEDBACK %d %d 0 0\n", pince->id, pince->pending_feedback_cmd);
                    pince->action_done = 0;
                    pince->action_step = 0; // Prêt pour une nouvelle commande
                } else {
                    pince->action_step = 505; // Continue de vérifier la position
                }
            }

            if (Timer_ms1 - pince->action_timer >= 3000){
                #ifdef DEBUG_FEETECH_ACTION
                    printf("pince : %d : Timeout critique : impossible de remonter la pince.\n", pince->id);
                #endif
                // On envoie quand même le feedback pour que la stratégie ne reste pas suspendue
                printf("PINCEFEEDBACK %d %d 0 0\n", pince->id, pince->pending_feedback_cmd);
                pince->action_step = 0; // On force à 0 pour ne pas geler tout le robot
            }
            break;

        default:
            break;
        
    }
}

uint8_t pince_action_cmd(void){
    uint32_t ID_pince;
    if (Get_Param_u32(&ID_pince))
        return PARAM_ERROR_CODE;

    uint32_t command;
    if (Get_Param_u32(&command))
        return PARAM_ERROR_CODE;
    
    uint32_t param;
    if (Get_Param_u32(&param))
        return PARAM_ERROR_CODE;

    // --- COMMANDE GLOBALE : 10 0 0 ---
    if (ID_pince == 10 && command == 0 && param == 0) {
        #ifdef DEBUG_FEETECH_ACTION
            printf("Action globale: Envoi de toutes les pinces a l'etape 600 (Relachement).\n");
        #endif
        
        for (int i = 0; i < NBR_PINCES; i++) {
            robot_pinces[i].current_command = CMD_IDLE;
            robot_pinces[i].action_step = 600;
        }
        return 0; 
    }
    // ---------------------------------

    if(ID_pince < 0 || ID_pince > NBR_PINCES-1){
        printf("Invalid pince ID\n");
        return PARAM_ERROR_CODE;
    }

    Pince_t *pince = &robot_pinces[ID_pince];
    if(pince->action_step != 0){
        printf("Pince is busy\n");
        return PARAM_ERROR_CODE;
    }
    
    switch (command){
        case 1:
            if(param == 0){
                pince->current_command = CMD_RAMASSER_G;
            } else if (param == 1){
                pince->current_command = CMD_RAMASSER_D;
            } else if (param == 2){
                pince->current_command = CMD_RAMASSER_ALL;
            } else {
                printf("Invalid parameter for RAMASSER command\n");
                return PARAM_ERROR_CODE;
            }
            pince->action_step = 10;
            break;
        case 2:
            if(param == 0){
                pince->current_command = CMD_LACHER_G;
            } else if (param == 1){
                pince->current_command = CMD_LACHER_D;
            } else if (param == 2){
                pince->current_command = CMD_LACHER_ALL;
            } else {
                printf("Invalid parameter for LACHER command\n");
                return PARAM_ERROR_CODE;
            }
            pince->action_step = 300;
            break;
        case 3:
            if(param == 0){
                pince->current_command = CMD_LACHER_G;
            } else if (param == 1){
                pince->current_command = CMD_LACHER_D;
            } else if (param == 2){
                pince->current_command = CMD_LACHER_ALL;
            } else {
                printf("Invalid parameter for LACHER command\n");
                return PARAM_ERROR_CODE;
            }
            pince->action_step = 200;
            break;
        case 4:
            pince->current_command = CMD_DEPOSE_G_RETOURNE_D;
            pince->action_step = 400;
            break;
        case 5:
            pince->current_command = CMD_DEPOSE_D_RETOURNE_G;
            pince->action_step = 400;
            break;
        case 6:
            pince->current_command = CMD_MONTER;
            pince->action_step = 100;
            break;
    }
    return 0;
}


uint8_t pince_action_debug_cmd(void){
    uint32_t val_1;
    if (Get_Param_u32(&val_1))
        return PARAM_ERROR_CODE;

    uint32_t val_2;
    if (Get_Param_u32(&val_2))
        return PARAM_ERROR_CODE;

    Pince_t *pince_0 = &robot_pinces[6];
    Pince_t *pince_1 = &robot_pinces[7];
    
    if (val_1 == 0){
        pince_0->current_command = CMD_RAMASSER_ALL;
        pince_1->current_command = CMD_RAMASSER_ALL;
        pince_0->action_step = 10;
        pince_1->action_step = 10;
    } else {
        pince_0->current_command = CMD_LACHER_ALL;
        pince_1->current_command = CMD_LACHER_ALL;
        pince_0->action_step = 300;
        pince_1->action_step = 300;
    }
    return 0;
}



uint8_t init_pince_state = 0;
uint32_t init_pince_timer = 0;
uint8_t pinces_initialized = 0;

void AU_pinces(void){
    pinces_initialized = 0;
    init_pince_state = 0;

    for (int i = 0; i < NBR_PINCES; i++) {
        robot_pinces[i].action_step = 0;
        robot_pinces[i].current_command = CMD_IDLE;
    }
}

void Init_Pinces_Loop(void){

    switch(init_pince_state){
        case 0:
            if (pinces_initialized == 0){
                init_pince_state = 1;
            }
            break;
        case 1:
            for (uint8_t i = 0; i < NBR_PINCES; i++){
                robot_pinces[i].id = i; 

                robot_pinces[i].id_gros     = (i + 1)*10 + 1; 
                robot_pinces[i].id_droite   = (i + 1)*10 + 3;
                robot_pinces[i].id_gauche   = (i + 1)*10 + 2;
                robot_pinces[i].id_pump     = (i + 1)*10;

                robot_pinces[i].action_step = 0;
                robot_pinces[i].action_done = 0;
                robot_pinces[i].action_timer = 0;
                robot_pinces[i].action_position = 0;

                robot_pinces[i].pump_right.pump_current = 0;
                robot_pinces[i].pump_left.pump_current = 0;

                robot_pinces[i].retry_count = 0;

                robot_pinces[i].sample_count = 0;

                robot_pinces[i].sample_idx = 0;
                robot_pinces[i].buffer_full = 0;

                robot_pinces[i].succes_left = 0;
                robot_pinces[i].succes_right = 0;

                robot_pinces[i].watchdog_timer = 0;
                robot_pinces[i].previous_step = 0;

                robot_pinces[i].pending_feedback_cmd = 0; // Initialisé à 0 (inconnu)

                robot_pinces[i].current_command = CMD_IDLE;  
            }
            // PINCE_0
            robot_pinces[0].gros_pos.idle_position = PINCE_1_GROS_IDLE_POS;
            robot_pinces[0].gros_pos.ramasser_pos = PINCE_1_GROS_RAMASSER_POS;
            robot_pinces[0].gros_pos.deposer_pos = PINCE_1_GROS_DEPOSER_POS;
            robot_pinces[0].gros_pos.lacher_pos = PINCE_1_GROS_LACHER_POS;
            robot_pinces[0].petit_droite_pos.sortie_pos = PINCE_1_DROITE_SORTIE_POS;
            robot_pinces[0].petit_droite_pos.retrait_pos = PINCE_1_DROITE_RETRAIT_POS;
            robot_pinces[0].petit_gauche_pos.sortie_pos = PINCE_1_GAUCHE_SORTIE_POS;
            robot_pinces[0].petit_gauche_pos.retrait_pos = PINCE_1_GAUCHE_RETRAIT_POS;

            // PINCE_1
            robot_pinces[1].gros_pos.idle_position = PINCE_2_GROS_IDLE_POS;
            robot_pinces[1].gros_pos.ramasser_pos = PINCE_2_GROS_RAMASSER_POS;
            robot_pinces[1].gros_pos.deposer_pos = PINCE_2_GROS_DEPOSER_POS;
            robot_pinces[1].gros_pos.lacher_pos = PINCE_2_GROS_LACHER_POS;
            robot_pinces[1].petit_droite_pos.sortie_pos = PINCE_2_DROITE_SORTIE_POS;
            robot_pinces[1].petit_droite_pos.retrait_pos = PINCE_2_DROITE_RETRAIT_POS;
            robot_pinces[1].petit_gauche_pos.sortie_pos = PINCE_2_GAUCHE_SORTIE_POS;
            robot_pinces[1].petit_gauche_pos.retrait_pos = PINCE_2_GAUCHE_RETRAIT_POS;

            // PINCE_2
            robot_pinces[2].gros_pos.idle_position = PINCE_3_GROS_IDLE_POS;
            robot_pinces[2].gros_pos.ramasser_pos = PINCE_3_GROS_RAMASSER_POS;
            robot_pinces[2].gros_pos.deposer_pos = PINCE_3_GROS_DEPOSER_POS;
            robot_pinces[2].gros_pos.lacher_pos = PINCE_3_GROS_LACHER_POS;
            robot_pinces[2].petit_droite_pos.sortie_pos = PINCE_3_DROITE_SORTIE_POS;
            robot_pinces[2].petit_droite_pos.retrait_pos = PINCE_3_DROITE_RETRAIT_POS;
            robot_pinces[2].petit_gauche_pos.sortie_pos = PINCE_3_GAUCHE_SORTIE_POS;
            robot_pinces[2].petit_gauche_pos.retrait_pos = PINCE_3_GAUCHE_RETRAIT_POS;

            // PINCE_3
            robot_pinces[3].gros_pos.idle_position = PINCE_4_GROS_IDLE_POS;
            robot_pinces[3].gros_pos.ramasser_pos = PINCE_4_GROS_RAMASSER_POS;
            robot_pinces[3].gros_pos.deposer_pos = PINCE_4_GROS_DEPOSER_POS;
            robot_pinces[3].gros_pos.lacher_pos = PINCE_4_GROS_LACHER_POS;
            robot_pinces[3].petit_droite_pos.sortie_pos = PINCE_4_DROITE_SORTIE_POS;
            robot_pinces[3].petit_droite_pos.retrait_pos = PINCE_4_DROITE_RETRAIT_POS;
            robot_pinces[3].petit_gauche_pos.sortie_pos = PINCE_4_GAUCHE_SORTIE_POS;
            robot_pinces[3].petit_gauche_pos.retrait_pos = PINCE_4_GAUCHE_RETRAIT_POS;

            // PINCE_4
            robot_pinces[4].gros_pos.idle_position = PINCE_5_GROS_IDLE_POS;
            robot_pinces[4].gros_pos.ramasser_pos = PINCE_5_GROS_RAMASSER_POS;
            robot_pinces[4].gros_pos.deposer_pos = PINCE_5_GROS_DEPOSER_POS;
            robot_pinces[4].gros_pos.lacher_pos = PINCE_5_GROS_LACHER_POS;
            robot_pinces[4].petit_droite_pos.sortie_pos = PINCE_5_DROITE_SORTIE_POS;
            robot_pinces[4].petit_droite_pos.retrait_pos = PINCE_5_DROITE_RETRAIT_POS;
            robot_pinces[4].petit_gauche_pos.sortie_pos = PINCE_5_GAUCHE_SORTIE_POS;
            robot_pinces[4].petit_gauche_pos.retrait_pos = PINCE_5_GAUCHE_RETRAIT_POS;

            // PINCE_5
            robot_pinces[5].gros_pos.idle_position = PINCE_6_GROS_IDLE_POS;
            robot_pinces[5].gros_pos.ramasser_pos = PINCE_6_GROS_RAMASSER_POS;
            robot_pinces[5].gros_pos.deposer_pos = PINCE_6_GROS_DEPOSER_POS;
            robot_pinces[5].gros_pos.lacher_pos = PINCE_6_GROS_LACHER_POS;
            robot_pinces[5].petit_droite_pos.sortie_pos = PINCE_6_DROITE_SORTIE_POS;
            robot_pinces[5].petit_droite_pos.retrait_pos = PINCE_6_DROITE_RETRAIT_POS;
            robot_pinces[5].petit_gauche_pos.sortie_pos = PINCE_6_GAUCHE_SORTIE_POS;
            robot_pinces[5].petit_gauche_pos.retrait_pos = PINCE_6_GAUCHE_RETRAIT_POS;

            // PINCE_6
            robot_pinces[6].gros_pos.idle_position = PINCE_7_GROS_IDLE_POS;
            robot_pinces[6].gros_pos.ramasser_pos = PINCE_7_GROS_RAMASSER_POS;
            robot_pinces[6].gros_pos.deposer_pos = PINCE_7_GROS_DEPOSER_POS;
            robot_pinces[6].gros_pos.lacher_pos = PINCE_7_GROS_LACHER_POS;
            robot_pinces[6].petit_droite_pos.sortie_pos = PINCE_7_DROITE_SORTIE_POS;
            robot_pinces[6].petit_droite_pos.retrait_pos = PINCE_7_DROITE_RETRAIT_POS;
            robot_pinces[6].petit_gauche_pos.sortie_pos = PINCE_7_GAUCHE_SORTIE_POS;
            robot_pinces[6].petit_gauche_pos.retrait_pos = PINCE_7_GAUCHE_RETRAIT_POS;

            // PINCE_7
            robot_pinces[7].gros_pos.idle_position = PINCE_8_GROS_IDLE_POS;
            robot_pinces[7].gros_pos.ramasser_pos = PINCE_8_GROS_RAMASSER_POS;
            robot_pinces[7].gros_pos.deposer_pos = PINCE_8_GROS_DEPOSER_POS;
            robot_pinces[7].gros_pos.lacher_pos = PINCE_8_GROS_LACHER_POS;
            robot_pinces[7].petit_droite_pos.sortie_pos = PINCE_8_DROITE_SORTIE_POS;
            robot_pinces[7].petit_droite_pos.retrait_pos = PINCE_8_DROITE_RETRAIT_POS;
            robot_pinces[7].petit_gauche_pos.sortie_pos = PINCE_8_GAUCHE_SORTIE_POS;
            robot_pinces[7].petit_gauche_pos.retrait_pos = PINCE_8_GAUCHE_RETRAIT_POS;

            init_pince_state = 2;
            break;
        case 2:
            for (uint8_t i = 0; i < NBR_PINCES; i++){
                // move gros servo to IDLE position
                PutFEETECH_Ext_Done(robot_pinces[i].id_gros, FEETECH_GOAL_POSITION_L, robot_pinces[i].gros_pos.idle_position, &robot_pinces[i].action_done);
                // move petit servos to RETRAIT position
                PutFEETECH_Ext_Done_SCS(robot_pinces[i].id_droite, FEETECH_GOAL_POSITION_L, robot_pinces[i].petit_droite_pos.retrait_pos, &robot_pinces[i].action_done);
                PutFEETECH_Ext_Done_SCS(robot_pinces[i].id_gauche, FEETECH_GOAL_POSITION_L, robot_pinces[i].petit_gauche_pos.retrait_pos, &robot_pinces[i].action_done);
            }
            init_pince_timer = Timer_ms1;
            init_pince_state = 3;
            break;
        case 3:
            // wait for all servos to reach position
            if (Timer_ms1 - init_pince_timer >= 3000){
                for (uint8_t i = 0; i < NBR_PINCES; i++){
                    robot_pinces[i].action_done = 0;
                }
                init_pince_state = 4;
            }
            break;
        case 4:
            printf("Pinces initialized\n");
            pinces_initialized = 1;
            init_pince_state = 0;
            break;
        default:
            break;

    }
}


// Tu peux changer cet ID pour tester les autres pinces (10, 20, 30...)
#define CALIB_PUMP_ID 20 
#define CALIB_WINDOW 50
// Variables globales pour la calibration
uint8_t calib_step = 0;
uint32_t calib_timer = 0;
uint8_t calib_done = 0;
uint16_t calib_current_right = 0;
uint16_t calib_current_left = 0;

// Variables dédiées à la moyenne glissante
uint16_t samples_right[CALIB_WINDOW] = {0};
uint16_t samples_left[CALIB_WINDOW]  = {0};
uint32_t sum_right = 0;
uint32_t sum_left = 0;
uint8_t sample_idx = 0;
uint8_t buffer_full = 0;
uint8_t print_counter = 0;


void Pump_Calibration_Loop(void) {
    switch(calib_step) {
        case 0:
            printf("\n=== DEMARRAGE CALIBRATION POMPE %d ===\n", CALIB_PUMP_ID);
            
            // 1. Réinitialisation complète du buffer circulaire
            sum_right = 0;
            sum_left = 0;
            sample_idx = 0;
            buffer_full = 0;
            print_counter = 0;
            for(int i = 0; i < CALIB_WINDOW; i++){
                samples_right[i] = 0;
                samples_left[i] = 0;
            }
            
            PutFEETECH(CALIB_PUMP_ID, PUMP_CMD_1, PUMP_ON);
            PutFEETECH_Ext_Done(CALIB_PUMP_ID, PUMP_CMD_2, PUMP_ON, &calib_done);
            calib_timer = Timer_ms1;
            calib_step = 1;
            break;
            
        case 1:
            if (calib_done) {
                if (Timer_ms1 - calib_timer >= 500) { 
                    calib_done = 0;
                    calib_step = 2;
                }
            }
            break;
            
        case 2:
            GetFEETECH_Ext_Done(CALIB_PUMP_ID, ADDR_CURRENT_1_L, &calib_current_right, &calib_done);
            calib_step = 3;
            break;
            
        case 3:
            if(calib_done) {
                calib_done = 0;
                GetFEETECH_Ext_Done(CALIB_PUMP_ID, ADDR_CURRENT_2_L, &calib_current_left, &calib_done);
                calib_step = 4;
            }
            break;
            
        case 4:
            if(calib_done) {
                calib_done = 0;
                
                sum_right -= samples_right[sample_idx];
                sum_left  -= samples_left[sample_idx];
                
                samples_right[sample_idx] = calib_current_right;
                samples_left[sample_idx]  = calib_current_left;
                sum_right += calib_current_right;
                sum_left  += calib_current_left;
                
                sample_idx++;
                if(sample_idx >= CALIB_WINDOW) {
                    sample_idx = 0;
                    buffer_full = 1;
                }
                
                uint16_t avg_right, avg_left;
                if(buffer_full) {
                    avg_right = sum_right / CALIB_WINDOW;
                    avg_left  = sum_left  / CALIB_WINDOW;
                } else {
                    avg_right = sum_right / sample_idx;
                    avg_left  = sum_left  / sample_idx;
                }
                
                print_counter++;
                if (print_counter >= 10) {
                    printf("CALIB - Brut[D:%4d G:%4d] | MOYENNE LISSEE[D:%4d G:%4d]\n", 
                           calib_current_right, calib_current_left, avg_right, avg_left);
                    print_counter = 0;
                }
                
                calib_timer = Timer_ms1;
                calib_step = 5;
            }
            break;
            
        case 5:
            if (Timer_ms1 - calib_timer >= 20) { 
                calib_step = 2;
            }
            break;
    }
}

uint8_t setup_pince_set_pos_cmd(void){
    uint32_t ID_pince;
    if (Get_Param_u32(&ID_pince))
        return PARAM_ERROR_CODE;

    if(ID_pince < 0 || ID_pince > NBR_PINCES-1){
        printf("Invalid pince ID\n");
        return PARAM_ERROR_CODE;
    }

    uint32_t servo_type; 
    if (Get_Param_u32(&servo_type))
        return PARAM_ERROR_CODE;
    
    if (servo_type > 2){
        printf("Invalid servo type\n");
        return PARAM_ERROR_CODE;
    }
    
    uint32_t param;
    if (Get_Param_u32(&param))
        return PARAM_ERROR_CODE;

    uint32_t val;
    if (Get_Param_u32(&val))
        return PARAM_ERROR_CODE;

    Pince_t *pince = &robot_pinces[ID_pince];
    
    switch (servo_type){
        case 0:
            if(param == 0){
                pince->gros_pos.idle_position =  val;
            } else if (param == 1){
                pince->gros_pos.ramasser_pos = val;
            } else if (param == 2){
                pince->gros_pos.lacher_pos = val;
            } else {
                printf("Invalid parameter for GROS servo\n");
                return PARAM_ERROR_CODE;
            }
            printf ("Updated gros servo position: idle=%d, ramasser=%d, lacher=%d\n", 
                    pince->gros_pos.idle_position, pince->gros_pos.ramasser_pos, pince->gros_pos.lacher_pos);
            break;
        case 1:
            if(param == 0){
                pince->petit_droite_pos.sortie_pos = val;
            } else if (param == 1){
                pince->petit_droite_pos.retrait_pos = val;
            } else {
                printf("Invalid parameter for PETIT DROITE servo\n");
                return PARAM_ERROR_CODE;
            }
            printf ("Updated petit_droite servo position: sortie=%d, retrait=%d\n", 
                    pince->petit_droite_pos.sortie_pos, pince->petit_droite_pos.retrait_pos);
            break;
        case 2:
            if(param == 0){
                pince->petit_gauche_pos.sortie_pos = val;
            } else if (param == 1){
                pince->petit_gauche_pos.retrait_pos = val;
            } else {
                printf("Invalid parameter for PETIT GAUCHE servo\n");
                return PARAM_ERROR_CODE;
            }
            printf ("Updated petit_gauche servo position: sortie=%d, retrait=%d\n", 
                    pince->petit_gauche_pos.sortie_pos, pince->petit_gauche_pos.retrait_pos);
            break;
    }
    return 0;    
}