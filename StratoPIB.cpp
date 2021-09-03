/*
 *  StratoPIB.cpp
 *  Author:  Alex St. Clair
 *  Created: July 2019
 *  Updated for MonDo board: November 2020 (LEK)
 *
 *  This file implements an Arduino library (C++ class) that inherits
 *  from the StratoCore class. It serves as the overarching class
 *  for the RACHuTS Profiler Interface Board, or PIB.
 */

int PacketSize = 0;

//ISR for LoRa reception, needs to be outside the class for some reason
void onReceive(int Size)
{
    PacketSize = Size;
}

#include "StratoPIB.h"

StratoPIB::StratoPIB()
    : StratoCore(&ZEPHYR_SERIAL, INSTRUMENT, &DEBUG_SERIAL)
    , mcbComm(&MCB_SERIAL)
    , puComm(&PU_SERIAL)
{
}

// --------------------------------------------------------
// General instrument functions
// --------------------------------------------------------

// note serial setup occurs in main arduino file
void StratoPIB::InstrumentSetup()
{

    // safe pin required by Zephyr
    pinMode(SAFE_PIN, OUTPUT);
    digitalWrite(SAFE_PIN, LOW);

    // PU power switch
    pinMode(PU_PWR_ENABLE, OUTPUT);
    digitalWrite(PU_PWR_ENABLE, LOW);

    // Set up the second SPI Port for the LoRa Module
    SPI1.setSCK(20);
    SPI1.setMISO(5);
    SPI1.setMOSI(21);

    LoRa.setSPI(SPI1);
    LoRa.setPins(SS_PIN, RESET_PIN,INTERUPT_PIN);
    
    LoRaInit();  //initialize the LoRa modem

    LoRa.onReceive(onReceive);
    LoRa.receive();

    if (!pibConfigs.Initialize()) {
        ZephyrLogWarn("Error loading from EEPROM! Reconfigured");
    }

    mcbComm.AssignBinaryRXBuffer(binary_mcb, MCB_BUFFER_SIZE);
    puComm.AssignBinaryRXBuffer(binary_pu, PU_BUFFER_SIZE);
}

void StratoPIB::InstrumentLoop()
{
    WatchFlags();
    CheckTSEN();
    LoRaRX();
}

void StratoPIB::LoRaInit()
{
   if (!LoRa.begin(FREQUENCY)){
       ZephyrLogWarn("Starting LoRa failed!");
       Serial.println("WARN: LoRa Initializtion Failed");
    }
    delay(1);
    LoRa.setSpreadingFactor(SF);
    delay(1);
    LoRa.setSignalBandwidth(BANDWIDTH);
    delay(1);
    LoRa.setTxPower(RF_POWER);
}

void StratoPIB::LoRaRX()
{
    int i = 0;

    if (PacketSize > 0) //if LoRa data is available
    {
        PacketSize = 0;
        Serial.print("Received Packet with RSSI :");
        Serial.println(LoRa.packetRssi());
        int BytesToRead = LoRa.available();
        Serial.printf("Bytes to Read: %d\n",BytesToRead);
        for (i = 0; i <  BytesToRead; i++)
           LoRa_RX_buffer[i] = LoRa.read();
        
        for (i = 0; i< BytesToRead; i++ ) //for debug write buffer to consols
            Serial.write(LoRa_RX_buffer[i]);
        Serial.println();

        if (strncmp(LoRa_RX_buffer,"ST",2) == 0)//it is a status packet
        { 
            LoRa_RX_buffer[BytesToRead] = '\0'; //null terminate buffer to make a string
            ZephyrLogFine(LoRa_RX_buffer);

        }

        else if (strncmp(LoRa_RX_buffer,"TM",2) == 0) //it is a profile TM packet
        {
                Serial.print("TM Packet idx: ");
                Serial.println(LoRa_TM_buffer_idx);
                LoRa_rx_time = millis();  //record the time we received last LoRa TM
                if (LoRa_TM_buffer_idx + BytesToRead > 6005) //if the incomming packet will over fill a TM send what we have
                {
                    //send the LoRa PU data to zephyr as a TM
                    snprintf(log_array, LOG_ARRAY_SIZE, "PU TM Packet %u", ++pu_tm_counter);
                    zephyrTX.addTm(LoRa_TM_buffer,LoRa_TM_buffer_idx);
                    zephyrTX.setStateDetails(1, log_array);
                    zephyrTX.setStateFlagValue(1, FINE);
                    zephyrTX.setStateFlagValue(2, NOMESS);
                    zephyrTX.setStateFlagValue(3, NOMESS);
                    zephyrTX.TM();
                    log_nominal(log_array);
                    LoRa_TM_buffer_idx = 0; //reset the buffer
                    zephyrTX.clearTm();
                }
                
                for(i = 0; i < BytesToRead-2; i++)
                    LoRa_TM_buffer[LoRa_TM_buffer_idx++] = LoRa_RX_buffer[i+2];
                BytesToRead = 0;
            
        }

        else
        {
            snprintf(log_array, LOG_ARRAY_SIZE, "Received Unknown LoRa Packet");
            log_nominal(log_array);
        }

        
    }

    //if it has been a while since we received a LoRa TM, assume it is done and send remaining data
    if (LoRa_TM_buffer_idx > 0){
        if ((millis() - LoRa_rx_time) > LORA_TM_TIMEOUT*1000) 
        {
                    zephyrTX.addTm(LoRa_TM_buffer,LoRa_TM_buffer_idx);            
                    //send the LoRa_TM_Buffer to zephyr as a TM
                    snprintf(log_array, LOG_ARRAY_SIZE, "Last PU TM Packet %u", ++pu_tm_counter);
                    zephyrTX.setStateDetails(1, log_array);
                    zephyrTX.setStateFlagValue(1, FINE);
                    zephyrTX.setStateFlagValue(2, NOMESS);
                    zephyrTX.setStateFlagValue(3, NOMESS);
                    zephyrTX.TM();
                    log_nominal(log_array);
                    LoRa_TM_buffer_idx = 0; //reset the buffer
                    pu_tm_counter = 0; //reset the TM counter
        }
    }
}

// --------------------------------------------------------
// Action handler and action flag helper functions
// --------------------------------------------------------

void StratoPIB::ActionHandler(uint8_t action)
{
    // for safety, ensure index doesn't exceed array size
    if (action >= NUM_ACTIONS) {
        log_error("Out of bounds action flag access");
        return;
    }

    // set the flag and reset the stale count
    action_flags[action].flag_value = true;
    action_flags[action].stale_count = 0;
}

bool StratoPIB::CheckAction(uint8_t action)
{
    // for safety, ensure index doesn't exceed array size
    if (action >= NUM_ACTIONS) {
        log_error("Out of bounds action flag access");
        return false;
    }

    // check and clear the flag if it is set, return the value
    if (action_flags[action].flag_value) {
        action_flags[action].flag_value = false;
        action_flags[action].stale_count = 0;
        return true;
    } else {
        return false;
    }
}

void StratoPIB::SetAction(uint8_t action)
{
    action_flags[action].flag_value = true;
    action_flags[action].stale_count = 0;
}

void StratoPIB::WatchFlags()
{
    // monitor for and clear stale flags
    for (int i = 0; i < NUM_ACTIONS; i++) {
        if (action_flags[i].flag_value) {
            action_flags[i].stale_count++;
            if (action_flags[i].stale_count >= FLAG_STALE) {
                action_flags[i].flag_value = false;
                action_flags[i].stale_count = 0;
            }
        }
    }
}

// --------------------------------------------------------
// Profile helpers
// --------------------------------------------------------

bool StratoPIB::StartMCBMotion()
{
    bool success = false;

    switch (mcb_motion) {
    case MOTION_REEL_IN:
        snprintf(log_array, LOG_ARRAY_SIZE, "Retracting %0.1f revs", retract_length);
        success = mcbComm.TX_Reel_In(retract_length, pibConfigs.retract_velocity.Read());
        max_profile_seconds = 60 * (retract_length / pibConfigs.retract_velocity.Read()) + pibConfigs.motion_timeout.Read();
        break;
    case MOTION_REEL_OUT:
        PUUndock();
        snprintf(log_array, LOG_ARRAY_SIZE, "Deploying %0.1f revs", deploy_length);
        success = mcbComm.TX_Reel_Out(deploy_length, pibConfigs.deploy_velocity.Read());
        max_profile_seconds = 60 * (deploy_length / pibConfigs.deploy_velocity.Read()) + pibConfigs.motion_timeout.Read();
        break;
    case MOTION_DOCK:
        snprintf(log_array, LOG_ARRAY_SIZE, "Docking %0.1f revs", dock_length);
        success = mcbComm.TX_Dock(dock_length, pibConfigs.dock_velocity.Read());
        max_profile_seconds = 60 * (dock_length / pibConfigs.dock_velocity.Read()) + pibConfigs.motion_timeout.Read();
        break;
    case MOTION_IN_NO_LW:
        snprintf(log_array, LOG_ARRAY_SIZE, "Reel in (no LW) %0.1f revs", retract_length);
        success = mcbComm.TX_In_No_LW(retract_length, pibConfigs.dock_velocity.Read());
        max_profile_seconds = 60 * (retract_length / pibConfigs.dock_velocity.Read()) + pibConfigs.motion_timeout.Read();
        break;
    default:
        mcb_motion = NO_MOTION;
        log_error("Unknown motion type to start");
        return false;
    }

    if (autonomous_mode) {
        log_nominal(log_array);
    } else {
        ZephyrLogFine(log_array);
    }

    return success;
}

bool StratoPIB::ScheduleProfiles()
{
    // no matter the trigger, reset the time_trigger to the max value, new TC needed to set new value
    pibConfigs.time_trigger.Write(UINT32_MAX);

    // schedule the configured number of profiles starting in five seconds
    for (int i = 0; i < pibConfigs.num_profiles.Read(); i++) {
        if (!scheduler.AddAction(ACTION_BEGIN_PROFILE, i * pibConfigs.profile_period.Read() + 5)) {
            ZephyrLogCrit("Error scheduling profiles, scheduler failure");
            return false;
        }
    }

    snprintf(log_array, LOG_ARRAY_SIZE, "Scheduled profiles: %u, %0.2f, %0.2f, %0.2f, %u, %u", pibConfigs.num_profiles.Read(),
             pibConfigs.profile_size.Read(), pibConfigs.dock_amount.Read(), pibConfigs.dock_overshoot.Read(),
             pibConfigs.dwell_time.Read(), pibConfigs.profile_period.Read());
    ZephyrLogFine(log_array);
    return true;
}

void StratoPIB::AddMCBTM()
{
    // make sure it's the correct size
    if (mcbComm.binary_rx.bin_length != MOTION_TM_SIZE) {
        log_error("invalid motion TM size");
        return;
    }

    // if not in real-time mode, add the sync and time
    if (!pibConfigs.real_time_mcb.Read()) {
        // sync byte        
        MCB_TM_buffer[MCB_TM_buffer_idx++] = (uint8_t) 0xA5;
                
        // tenths of seconds since start
        uint16_t elapsed_time = (uint16_t)((millis() - profile_start) / 100);
        MCB_TM_buffer[MCB_TM_buffer_idx++] = (uint8_t) (elapsed_time >> 8);
        MCB_TM_buffer[MCB_TM_buffer_idx++] = (uint8_t) (elapsed_time & 0xFF);
    }

    // add each byte of data to the message
    for (int i = 0; i < MOTION_TM_SIZE; i++) {
        MCB_TM_buffer[MCB_TM_buffer_idx++] = mcbComm.binary_rx.bin_buffer[i];
    }

    // if real-time mode, send the TM packet
    if (pibConfigs.real_time_mcb.Read()) {
        snprintf(log_array, LOG_ARRAY_SIZE, "MCB TM Packet %u", ++mcb_tm_counter);
        zephyrTX.addTm(MCB_TM_buffer,MCB_TM_buffer_idx);
        zephyrTX.setStateDetails(1, log_array);
        zephyrTX.setStateFlagValue(1, FINE);
        zephyrTX.setStateFlagValue(2, NOMESS);
        zephyrTX.setStateFlagValue(3, NOMESS);
        zephyrTX.TM();
        log_nominal(log_array);
        MCB_TM_buffer_idx = 0; //reser the MCB buffer pointer
    }
}

void StratoPIB::NoteProfileStart()
{
    mcb_motion_ongoing = true;
    profile_start = millis();

    if (MOTION_DOCK == mcb_motion || MOTION_IN_NO_LW == mcb_motion) mcb_dock_ongoing = true;

    mcb_tm_counter = 0;
    //zephyrTX.clearTm(); // empty the TM buffer for incoming MCB motion data
    MCB_TM_buffer_idx = 0;
    // Add the start time to the MCB TM Header if not in real-time mode
    if (!pibConfigs.real_time_mcb.Read()) {
        //zephyrTX.addTm((uint32_t) now()); // as a header, add the current seconds since epoch
        uint32_t ProfileStartEpoch  = now();
        MCB_TM_buffer[MCB_TM_buffer_idx++] = (uint8_t) (ProfileStartEpoch >> 24);
        MCB_TM_buffer[MCB_TM_buffer_idx++] = (uint8_t) (ProfileStartEpoch >> 16);
        MCB_TM_buffer[MCB_TM_buffer_idx++] = (uint8_t) (ProfileStartEpoch >> 8);
        MCB_TM_buffer[MCB_TM_buffer_idx++] = (uint8_t) (ProfileStartEpoch & 0xFF);
    }


}

void StratoPIB::SendMCBTM(StateFlag_t state_flag, const char * message)
{
    // use only the first flag to report the motion
    zephyrTX.addTm(MCB_TM_buffer,MCB_TM_buffer_idx);
    zephyrTX.setStateDetails(1, message);
    zephyrTX.setStateFlagValue(1, state_flag);
    zephyrTX.setStateFlagValue(2, NOMESS);
    zephyrTX.setStateFlagValue(3, NOMESS);

    TM_ack_flag = NO_ACK;
    zephyrTX.TM();

    log_nominal(log_array);
    MCB_TM_buffer_idx = 0;
    if (!WriteFileTM("MCB")) {
        log_error("Unable to write MCB TM to SD file");
    }
}


void StratoPIB::SendMCBEEPROM()
{
    // the binary buffer has been prepared by the MCBRouter
    zephyrTX.clearTm();
    zephyrTX.addTm(mcbComm.binary_rx.bin_buffer, mcbComm.binary_rx.bin_length);

    // use only the first flag to preface the contents
    zephyrTX.setStateDetails(1, "MCB EEPROM Contents");
    zephyrTX.setStateFlagValue(1, FINE);
    zephyrTX.setStateFlagValue(2, NOMESS);
    zephyrTX.setStateFlagValue(3, NOMESS);

    // send as TM
    TM_ack_flag = NO_ACK;
    zephyrTX.TM();

    log_nominal("Sent MCB EEPROM as TM");
}

void StratoPIB::SendPIBEEPROM()
{
    // create a buffer from the EEPROM (cheat, and use the preallocated MCBComm Binary RX buffer)
    mcbComm.binary_rx.bin_length = pibConfigs.Bufferize(mcbComm.binary_rx.bin_buffer, MAX_MCB_BINARY);

    if (0 == mcbComm.binary_rx.bin_length) {
        log_error("Unable to bufferize PIB EEPROM");
        return;
    }

    // prepare the TM buffer
    zephyrTX.clearTm();
    zephyrTX.addTm(mcbComm.binary_rx.bin_buffer, mcbComm.binary_rx.bin_length);

    // use only the first flag to preface the contents
    zephyrTX.setStateDetails(1, "PIB EEPROM Contents");
    zephyrTX.setStateFlagValue(1, FINE);
    zephyrTX.setStateFlagValue(2, NOMESS);
    zephyrTX.setStateFlagValue(3, NOMESS);

    // send as TM
    TM_ack_flag = NO_ACK;
    zephyrTX.TM();

    log_nominal("Sent PIB EEPROM as TM");
}

void StratoPIB::SendTSENTM()
{
    if (0 < snprintf(log_array, LOG_ARRAY_SIZE, "PU TSEN: %lu, %0.2f, %0.2f, %0.2f, %0.2f, %u", pu_status.time, pu_status.v_battery, pu_status.i_charge, pu_status.therm1, pu_status.therm2, pu_status.heater_stat)) {
        zephyrTX.setStateDetails(1, log_array);
        zephyrTX.setStateFlagValue(1, FINE);
    } else {
        zephyrTX.setStateDetails(1, "PU TSEN: unable to add status info");
        zephyrTX.setStateFlagValue(1, WARN);
    }

    // use only the first flag to report the motion
    zephyrTX.setStateFlagValue(2, NOMESS);
    zephyrTX.setStateFlagValue(3, NOMESS);

    TM_ack_flag = NO_ACK;
    zephyrTX.TM();

    log_nominal(log_array);
}

void StratoPIB::SendProfileTM(uint8_t packet_num)
{
    if (0 < snprintf(log_array, LOG_ARRAY_SIZE, "PU Prof. Rec. %u.%u: %lu, %0.2f, %0.2f, %0.2f, %0.2f, %u", pibConfigs.profile_id.Read(), packet_num, pu_status.time, pu_status.v_battery, pu_status.i_charge, pu_status.therm1, pu_status.therm2, pu_status.heater_stat)) {
        zephyrTX.setStateDetails(1, log_array);
        zephyrTX.setStateFlagValue(1, FINE);
    } else {
        zephyrTX.setStateDetails(1, "PU Profile Record: unable to add status info");
        zephyrTX.setStateFlagValue(1, WARN);
    }

    // use only the first flag to report the motion
    zephyrTX.setStateFlagValue(2, NOMESS);
    zephyrTX.setStateFlagValue(3, NOMESS);

    TM_ack_flag = NO_ACK;
    zephyrTX.TM();

    log_nominal(log_array);
}

// every 10 minutes, aligned with the hour (called in InstrumentLoop)
void StratoPIB::CheckTSEN()
{
    static time_t last_tsen = 0;

    // check if it's been at least 9 minutes and the current minute is a multiple of 10
    if ((now() > last_tsen + 540) && (0 == minute() % 10)) {
        last_tsen = now();
        SetAction(COMMAND_SEND_TSEN);
    }
}

void StratoPIB::PUDock()
{
    pibConfigs.pu_docked.Write(true);
    digitalWrite(PU_PWR_ENABLE, HIGH);
}

void StratoPIB::PUUndock()
{
    pibConfigs.pu_docked.Write(false);
    digitalWrite(PU_PWR_ENABLE, LOW);
}

void StratoPIB::PUStartProfile()
{
    int32_t t_down = 60 * (deploy_length / pibConfigs.deploy_velocity.Read()) + pibConfigs.preprofile_time.Read();
    int32_t t_up = 60 * (retract_length / pibConfigs.retract_velocity.Read() + dock_length / pibConfigs.dock_velocity.Read())
                   + pibConfigs.motion_timeout.Read(); // extra time for dock delay

    puComm.TX_Profile(t_down, pibConfigs.dwell_time.Read(), t_up, pibConfigs.profile_rate.Read(), pibConfigs.dwell_rate.Read(),
                      pibConfigs.profile_TSEN.Read(), pibConfigs.profile_ROPC.Read(), pibConfigs.profile_FLASH.Read(),pibConfigs.lora_tx_tm.Read());
    Serial.printf("Profile Params Sent to PU: %d, %d, %d, %d,%d, %d, %d, %d, %d\n",t_down, pibConfigs.dwell_time.Read(), t_up, pibConfigs.profile_rate.Read(), pibConfigs.dwell_rate.Read(),
                      pibConfigs.profile_TSEN.Read(), pibConfigs.profile_ROPC.Read(), pibConfigs.profile_FLASH.Read(),pibConfigs.lora_tx_tm.Read());
    pibConfigs.profile_id.Write(pibConfigs.profile_id.Read()+1); //increment the profile counter
}