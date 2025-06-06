/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Noah Drakes

  (C) Copyright 2024 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <iostream>
#include <unistd.h>
#include <stdio.h>
#include <chrono>
#include <fstream>
#include <string>
#include <pthread.h>
#include <iomanip>

#include "udp_tx.h"
#include "data_collection.h"
#include "data_collection_shared.h"

using namespace std;

// Byteswap (bswap_32)
#ifdef _MSC_VER
#include <stdlib.h>   // for byteswap functions
inline uint32_t bswap_32(uint32_t data) { return _byteswap_ulong(data); }
#elif defined(__APPLE__)
#include <libkern/OSByteOrder.h>
#define bswap_32(x) OSSwapInt32(x)
#else
#include <byteswap.h>
#endif


///////////////////////
// UTILITY METHODS //
///////////////////////

static float convert_chrono_duration_to_float(chrono::high_resolution_clock::time_point start, chrono::high_resolution_clock::time_point end)
{
    std::chrono::duration<float> duration = end - start;
    return duration.count();
}

static string return_filename()
{
    time_t t = time(NULL);
    struct tm* ptr = localtime(&t);

    char buffer[32];
    // Format: MM-DD-YYYY_HHMMSS
    strftime(buffer, sizeof(buffer), "capture_%m-%d-%Y_%H%M%S.csv", ptr);

    return string(buffer);
}

static void hwVersToString(uint32_t val, char *str)
{
    val = bswap_32(val);

    const char *val_char =  reinterpret_cast<const char*> (&val); 
    char hw_vers[5];
    memcpy(str, val_char, 4);
    str[4] = '\0';
}


///////////////////////
// PROTECTED METHODS //
///////////////////////

void DataCollection:: process_sample(uint32_t *data_packet, int start_idx)
{
    if (start_idx + dc_meta.size_of_sample > UDP_MAX_QUADLET_PER_PACKET) {
        return;
    }

    int idx = start_idx;


    uint64_t timestamp_high = data_packet[idx++];
    uint32_t timestamp_low = data_packet[idx++];

    uint64_t raw_64bit_timestamp = (timestamp_high << 32) | (timestamp_low);

    proc_sample.timestamp = *reinterpret_cast<double *>(&raw_64bit_timestamp);

    for (int i = 0; i < dc_meta.num_encoders; i++) {
        proc_sample.encoder_position[i] = *reinterpret_cast<int32_t *> (&data_packet[idx++]);
    }

    for (int i = 0; i < dc_meta.num_encoders; i++) {
        proc_sample.encoder_velocity[i] = *reinterpret_cast<float *> (&data_packet[idx++]);
    }

    for (int i = 0; i < dc_meta.num_motors; i++) {
        proc_sample.motor_status[i] = static_cast<uint16_t> ((0xFFFF0000 & data_packet[idx]) >> 16);
        proc_sample.motor_current[i] = (uint16_t) (0x0000FFFF & data_packet[idx]);
        idx++;
    }

    if (use_ps_io){
        proc_sample.digital_io = data_packet[idx++];
        proc_sample.mio_pins = data_packet[idx++];
    }
    
}

int DataCollection::collect_data() {
    if (isDataCollectionRunning) {
        collect_data_ret = false;
        return false;
    }

    cout << "CAPTURE [" << data_capture_count << "] in Progress ... !" << endl;

    isDataCollectionRunning = true;
    stop_data_collection_flag = false;
    sm_state = SM_SEND_START_DATA_COLLECTIION_CMD_TO_PS;

    while (sm_state != SM_EXIT) {
        switch (sm_state) {
            case SM_SEND_START_DATA_COLLECTIION_CMD_TO_PS:
                udp_transmit(sock_id, (char *)HOST_START_DATA_COLLECTION, sizeof(HOST_START_DATA_COLLECTION));
                sm_state = SM_START_DATA_COLLECTION;
                break;

            case SM_START_DATA_COLLECTION:
                handle_data_collection();
                sm_state = SM_EXIT;
                break;

            case SM_CLOSE_SOCKET:
                handle_socket_closure();
                sm_state = SM_EXIT;
                break;

            default:
                std::cerr << "[ERROR] Unknown state: " << sm_state << std::endl;
                sm_state = SM_EXIT;
                break;
        }
    }

    return true;
}

void DataCollection::handle_data_collection() {
    curr_time.start = std::chrono::high_resolution_clock::now();
    udp_data_packets_recvd_count = 0;
    packet_misses_counter = 0;

    filename = return_filename();
    myFile.open(filename);

    write_csv_headers();

    while (!stop_data_collection_flag) {
        int ret_code = udp_nonblocking_receive(sock_id, data_packet, dc_meta.data_packet_size);

        if (ret_code > 0) {
            udp_data_packets_recvd_count++;
            packet_misses_counter = 0;
            process_and_write_data();
        } else if (ret_code == UDP_DATA_IS_NOT_AVAILABLE_WITHIN_TIMEOUT) {
            handle_packet_timeout();
            if (stop_data_collection_flag) {
                break;
            }
        } else {
            handle_udp_error(ret_code);
            stop_data_collection_flag = true;
            break;
        }
    }

    myFile.close();
}

void DataCollection::write_csv_headers() {
    myFile << "TIMESTAMP,";

    for (int i = 1; i <= dc_meta.num_encoders; i++) {
        myFile << "ENCODER_POS_" << i << ",";
    }
    for (int i = 1; i <= dc_meta.num_encoders; i++) {
        myFile << "ENCODER_VEL_" << i << ",";
    }
    for (int i = 1; i <= dc_meta.num_motors; i++) {
        myFile << "MOTOR_CURRENT_" << i << ",";
    }
    for (int i = 1; i <= dc_meta.num_motors; i++) {
        myFile << "MOTOR_STATUS_" << i;
        if (i < dc_meta.num_motors) myFile << ",";
    }
    if (use_ps_io) {
        myFile << ",DIGITAL_IO,MIO_PINS";
    }

    myFile << std::endl;
}

void DataCollection::process_and_write_data() {
    for (int i = 0; i < dc_meta.data_packet_size / 4; i += dc_meta.size_of_sample) {
        process_sample(data_packet, i);

        myFile << setprecision(12) << proc_sample.timestamp << ",";

        for (int j = 0; j < dc_meta.num_encoders; j++) {
            myFile << proc_sample.encoder_position[j] << ",";
        }
        for (int j = 0; j < dc_meta.num_encoders; j++) {
            myFile << proc_sample.encoder_velocity[j] << ",";
        }
        for (int j = 0; j < dc_meta.num_motors; j++) {
            myFile << proc_sample.motor_current[j] << ",";
        }

        for (int j = 0; j < dc_meta.num_motors; j++) {
            myFile << static_cast<uint16_t>(proc_sample.motor_status[j]);
            if (j < dc_meta.num_motors - 1) myFile << ",";
        }

        if (use_ps_io) {
            myFile << "," << proc_sample.digital_io << "," << proc_sample.mio_pins;
        }

        myFile << std::endl;
        memset(&proc_sample, 0, sizeof(proc_sample));
    }
}

void DataCollection::handle_packet_timeout() {
    packet_misses_counter++;

    if (packet_misses_counter >= 100000 && udp_data_packets_recvd_count != 0) {
        std::cerr << "[ERROR] Capture timeout. 100,000 data packet misses" << std::endl;
        std::cerr << "Restart Zynq and Host programs" << std::endl;
        sm_state = SM_CLOSE_SOCKET;
        stop_data_collection_flag = true;
    }
}

void DataCollection::handle_udp_error(int ret_code) {
    std::cerr << "[ERROR] UDP ERROR (ret code: " << ret_code << "). Check connection if Zynq program failed" << std::endl;
    sm_state = SM_CLOSE_SOCKET;
}

void DataCollection::handle_socket_closure() {
    std::cout << "Closing socket..." << std::endl;
    // Add socket closure logic here if needed
    isDataCollectionRunning = false;
}


void * DataCollection::collect_data_thread(void * args)
{
    DataCollection *dc = static_cast<DataCollection *>(args);

    dc->collect_data();
    return nullptr;
}



////////////////////
// PUBLIC METHODS //
////////////////////


DataCollection::DataCollection()
{
    cout << "New Data Collection Object !" << endl << endl;
    isDataCollectionRunning = false;
    stop_data_collection_flag = false;
}

// TODO: need to add useful return statements -> all the close socket cases are just returns
// make sure logic checks out 
bool DataCollection :: init(uint8_t boardID, bool usePSIO, bool useSampleRate, int sample_rate)
{
    if(!udp_init(&sock_id, boardID)) {
        return false;
    }

    if(useSampleRate){
        use_sample_rate = true; 
    }


    sm_state = SM_SEND_READY_STATE_TO_PS;
    int ret_code = 0;

    char recvBuffer[100] = {0};

    bool error_flag = false;

    use_ps_io = usePSIO;

    // Handshaking PS
    while(1) {
        switch(sm_state) {
            case SM_SEND_READY_STATE_TO_PS:

                if (!use_ps_io && !use_sample_rate){
                    udp_transmit(sock_id, (char *)HOST_READY_CMD, sizeof(HOST_READY_CMD));
                }

                if (use_ps_io){
                    udp_transmit(sock_id, (char *)HOST_READY_CMD_W_PS_IO, sizeof(HOST_READY_CMD_W_PS_IO));
                } 

                if(use_sample_rate){
                    udp_transmit(sock_id, (char *)HOST_READY_CMD_W_SAMPLE_RATE, sizeof(HOST_READY_CMD_W_SAMPLE_RATE));
                    udp_transmit(sock_id, (int *) &sample_rate, sizeof(sample_rate));
                }
                
                
                
                sm_state = SM_RECV_DATA_COLLECTION_META_DATA;
                break;

            case SM_RECV_DATA_COLLECTION_META_DATA:
                ret_code = udp_nonblocking_receive(sock_id, &dc_meta, sizeof(dc_meta));
                if (ret_code > 0) {
                    if (dc_meta.hwvers == dRA1_String || dc_meta.hwvers == QLA1_String || dc_meta.hwvers == DQLA_String) {
                        cout << "Received Message from Zynq: RECEIVED METADATA" << endl << endl;

                        char hw_vers[5];
                        hwVersToString(dc_meta.hwvers, hw_vers);

                        cout << "---- DATA COLLECTION METADATA ---" << endl;
                        cout << "Hardware Version: " << hw_vers << endl;
                        cout << "Num of Encoders:  " <<  +dc_meta.num_encoders << endl;
                        cout << "Num of Motors: " << +dc_meta.num_motors << endl;
                        cout << "Packet Size (in bytes): " << dc_meta.data_packet_size << endl;
                        cout << "Samples per Packet: " << dc_meta.samples_per_packet << endl;
                        cout << "Sizoef Samples (in quadlets): " << dc_meta.size_of_sample << endl;
                        cout << "----------------------------------" << endl << endl;

                        sm_state = SM_SEND_METADATA_RECV;
                    } else {
                        cout << "[ERROR] Host data collection is out of sync with Zynq State Machine. Restart Zynq and Host Program";
                        sm_state = SM_CLOSE_SOCKET;
                    }
                } else if (ret_code == UDP_DATA_IS_NOT_AVAILABLE_WITHIN_TIMEOUT || ret_code == UDP_NON_UDP_DATA_IS_AVAILABLE){
                    sm_state = SM_RECV_DATA_COLLECTION_META_DATA;
                } else {
                    cout << "[ERROR] - UDP fail, Check connection if zynq program failed" << endl;
                    sm_state = SM_CLOSE_SOCKET;
                }
                break;

            case SM_SEND_METADATA_RECV:
                udp_transmit(sock_id, (char *) HOST_RECVD_METADATA , sizeof(HOST_RECVD_METADATA ));
                sm_state = SM_WAIT_FOR_PS_HANDSHAKE;
                break;

            case SM_WAIT_FOR_PS_HANDSHAKE:
                ret_code = udp_nonblocking_receive(sock_id, recvBuffer, sizeof(recvBuffer));
                
                if (ret_code > 0) {
                    if (strcmp(recvBuffer, "ZYNQ: READY FOR DATA COLLECTION") == 0) {
                        cout << "Received Message " << ZYNQ_READY_CMD << endl;
                        sm_state = SM_SEND_START_DATA_COLLECTIION_CMD_TO_PS;
                        return true; 
                    } else {
                        cout << "[ERROR] Host data collection is out of sync with Processor State Machine. Restart Server";
                        sm_state = SM_CLOSE_SOCKET;
                    }
                } else if (ret_code == UDP_DATA_IS_NOT_AVAILABLE_WITHIN_TIMEOUT || ret_code == UDP_NON_UDP_DATA_IS_AVAILABLE) {
                    sm_state = SM_WAIT_FOR_PS_HANDSHAKE;
                } else {
                    cout << "[ERROR] - UDP fail, Check connection if zynq program failed" << endl;
                    sm_state = SM_CLOSE_SOCKET;
                }
                break;

            case SM_CLOSE_SOCKET:
                close(sock_id);
                return false;
        }
    }
}


bool DataCollection :: start()
{
    if (pthread_create(&collect_data_t, nullptr, DataCollection::collect_data_thread, this) != 0) {
        std::cerr << "Error collect data thread" << std::endl;
        return 1;
    }

    // clearing udp buffer of remaining packets not captured during data collection
    while (udp_nonblocking_receive(sock_id, data_packet, dc_meta.data_packet_size) > 0) {}

    return true;
}

bool DataCollection :: stop()
{
    // send end data collection cmd
    if (!udp_transmit(sock_id,(char *) HOST_STOP_DATA_COLLECTION, sizeof(HOST_STOP_DATA_COLLECTION)) ) {
        cout << "[ERROR]: UDP error. Check connection if zynq program failed!" << endl; // more descriptive eror message
    }

    usleep(1000);

    isDataCollectionRunning = false;

    stop_data_collection_flag = true;

    pthread_join(collect_data_t, nullptr);

    myFile.close();

    curr_time.end = std::chrono::high_resolution_clock::now();
    curr_time.elapsed = convert_chrono_duration_to_float(curr_time.start, curr_time.end);


    cout << "---------------------------------------------------------" << endl;
    cout << "STOPPED CAPTURE [" << data_capture_count++ << "] ! Time Elapsed: " << curr_time.elapsed << "s" << endl;
    cout << "Data stored to " << filename << "." << endl;
    cout << "---------------------------------------------------------" << endl << endl;

    collect_data_ret = true;

    usleep(1000);

    return true;
}


bool DataCollection :: terminate()
{
    char terminateClientAndServerCmd[] = "CLIENT: Terminate Server";
    char recvBuffer[100] = {0};

    if (!udp_transmit(sock_id, (char *) HOST_TERMINATE_SERVER, sizeof(HOST_TERMINATE_SERVER))) {
        cout << "[ERROR]: UDP error. check connection with host!" << endl;
    }

    while (1) {
        int ret = udp_nonblocking_receive(sock_id, recvBuffer, 31);

        if (ret > 0) {
            if (strcmp(recvBuffer,  ZYNQ_TERMINATATION_SUCCESSFUL) == 0) {
                cout << "Received Message:  " << ZYNQ_TERMINATATION_SUCCESSFUL << endl;
                break;
            } else {
                cout << "[ERROR] Zynq and Host out of sync" << endl;
                return false;
            }
        } else if (ret == UDP_SELECT_ERROR || ret == UDP_SOCKET_ERROR || ret == UDP_CONNECTION_CLOSED_ERROR) {
                cout << "Termination Failed: Check UDP connetion" << endl;
                return false;
            }
    }

    close(sock_id);
    return true;
}
