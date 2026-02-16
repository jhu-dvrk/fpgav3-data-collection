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

// stdlibs
#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <unistd.h>
#include <string>
#include <chrono>
#include <pthread.h>
#include <atomic>

// mmap mio pins
#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <sys/mman.h>

// dvrk libs
#include "BasePort.h"
#include "PortFactory.h"
#include "ZynqEmioPort.h"
#include "AmpIO.h"

// shared header
#include "data_collection_shared.h"

using namespace std;

// UDP_MAX_PACKET_SIZE (in bytes) is calculated from GetMaxWriteDataSize (based
// on the MTU) in EthUdpPort.cpp
// PK: This is now defined in data_collection_shared.h
const int UDP_MAX_PACKET_SIZE = UDP_REAL_MTU;

// defines and variables for MIO Memory mapping for reading MIO pins
const uint32_t GPIO_BASE_ADDR = 0xE000A000;
const unsigned int GPIO_BANK1_OFFSET = 0x8;
const uint32_t SCLR_CLK_BASE_ADDR = 0xF8000000;
volatile uint32_t *GPIO_MEM_REGION;

// FLAG for including Processor IO in data packets
bool use_ps_io_flag = false;
bool use_pot_flag = true;

///////////////////////////////////
///// STATE MACHINE VARIABLES /////
//////////////////////////////////

// Double Buffer Struct handling double buffer between collecting and transmitting data between 
// threads
struct Double_Buffer_Info {
    uint32_t double_buffer[2][UDP_MAX_PACKET_SIZE/4]; //note: changed from 1500 which makes sense
    uint16_t buffer_size; 
    uint8_t prod_buf;
    uint8_t cons_buf;
    atomic_uint8_t cons_busy; 
};

// have methods return new state
// slim down sm struct to non local 

DataCollectionMeta data_collection_meta;
Double_Buffer_Info db;
char recvd_cmd[CMD_MAX_STRING_SIZE] = {0};


struct Dvrk_Controller {
    BasePort *Port;
    AmpIO *Board;   
} dvrk_controller;

struct SM{
    // states
    int state = 0;
    int last_state = 0;  

    // return codes
    int ret = 0;
    int udp_ret;
};


// DEBUGGING VARIABLES 
int data_packet_count = 0;
int sample_count = 0;

// Motor Current/Status arrays to store data 
// for emio timeout error
int32_t emio_read_error_counter = 0; 

// last timestamp from last capture
double last_timestamp = 0;

// timespec vaiables for getting timestamp using gettime() method
// with CLOCK_MONOTONIC_RAW. 

timespec t_data_collection_start;

// keeps track of deadline to control sample rate
timespec deadline;

// expected period of sample capture
long period_ns;

// global sample rate variable to change sample rate
int SAMPLE_RATE = 0;
bool useSampleRate = false;

// FLAG set when the host terminates data collection
bool stop_data_collection_flag = false;

// State Machine states
enum DataCollectionStateMachine {
    SM_READY = 0,
    SM_SEND_READY_STATE_TO_HOST,
    SM_WAIT_FOR_HOST_HANDSHAKE,
    SM_WAIT_FOR_HOST_FLAG_CMD,
    SM_WAIT_FOR_HOST_FLAG_VALUE,
    SM_WAIT_FOR_HOST_SAMPLE_RATE_CMD,
    SM_WAIT_FOR_HOST_SAMPLE_RATE_VALUE,
    SM_WAIT_FOR_HOST_START_CMD,
    SM_START_DATA_COLLECTION,
    SM_CHECK_FOR_STOP_DATA_COLLECTION_CMD,
    SM_START_CONSUMER_THREAD,
    SM_PACKAGE_DATA_COLLECTION_METADATA,
    SM_SEND_DATA_COLLECTION_METADATA,
    SM_WAIT_FOR_HOST_RECV_METADATA,
    SM_PRODUCE_DATA,
    SM_CONSUME_DATA,
    SM_TERMINATE,
    SM_EXIT
};

// UDP Return Codes
enum UDP_RETURN_CODES {
    UDP_DATA_IS_AVAILABLE = 0,
    UDP_DATA_IS_NOT_AVAILABLE_WITHIN_TIMEOUT = -1,
    UDP_SELECT_ERROR = -2,
    UDP_CONNECTION_CLOSED_ERROR = -3,
    UDP_SOCKET_ERROR = -4,
    UDP_NON_UDP_DATA_IS_AVAILABLE = -5
};

// Socket Data struct
struct UDP_Info {
    int socket;
    struct sockaddr_in Addr;
    socklen_t AddrLen;
} udp_host; // this is global bc there will only be one


static int mio_mmap_init()
{
    int mem_fd;

    // Open /dev/mem for accessing physical memory
    if ((mem_fd = open("/dev/mem", O_RDWR | O_SYNC)) < 0) {
        cout << "Failed to open /dev/mem" << endl;
        return -1;
    }

    void *gpio_mmap = mmap(
        NULL,
        0x1000,
        PROT_READ | PROT_WRITE,
        MAP_SHARED,
        mem_fd,
        GPIO_BASE_ADDR
    );

    if (gpio_mmap == MAP_FAILED) {
        perror("Failed to mmap");
        close(mem_fd);  // Always close the file descriptor on failure
        return -1;
    }

    GPIO_MEM_REGION = (volatile uint32_t * ) gpio_mmap;

    // Following code ensures that APER_CLK is enabled; should not be necessary
    // since fpgav3_emio_mmap also does this.
    void *clk_map = mmap(
        NULL,
        0x00000130,
        PROT_READ | PROT_WRITE,
        MAP_SHARED,
        mem_fd,
        SCLR_CLK_BASE_ADDR
    );

    volatile unsigned long *clock_map = (volatile unsigned long *)clk_map;

    uint32_t bitmsk = (1 << 22);
    uint32_t aper_clk_reg = clock_map[0x12C/4];

    if ((aper_clk_reg & bitmsk) == 0) {
        clock_map[0x12C/4] |= bitmsk;
    }

    munmap(clk_map, 0x00000130);
    // End of APER_CLK check

    close(mem_fd);

    return 0;
}

uint8_t returnMIOPins(){

    if (GPIO_MEM_REGION == NULL) {
        cout << "[ERROR] MIO mmap region initialized incorrectly!" << endl;
        return 0;
    }

    const uint16_t MIO_PINS_MSK = 0x3C;
    uint32_t gpio_bank1 = GPIO_MEM_REGION[GPIO_BANK1_OFFSET/4];

    return (gpio_bank1 & MIO_PINS_MSK) >> 2;
}

// checks if data is available from udp buffer (for noblocking udp recv)
int udp_nonblocking_receive(UDP_Info *udp_host, void *data, int size)
{
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(udp_host->socket, &readfds);

    int ret_code;

    struct timeval timeout;

    // Timeout values
    timeout.tv_sec = 0;   
    timeout.tv_usec = 0;    

    int max_fd = udp_host->socket + 1;
    int activity = select(max_fd, &readfds, NULL, NULL, &timeout);

    if (activity < 0) {
        return UDP_SELECT_ERROR;
    } else if (activity == 0) {
        return UDP_DATA_IS_NOT_AVAILABLE_WITHIN_TIMEOUT;
    } else {
        if (FD_ISSET(udp_host->socket, &readfds)) {
            ret_code = recvfrom(udp_host->socket, data, size, 0, (struct sockaddr *)&udp_host->Addr, &udp_host->AddrLen);

            if (ret_code == 0) {
                return UDP_CONNECTION_CLOSED_ERROR;
            } else if (ret_code < 0) {
                return UDP_SOCKET_ERROR;
            } else {
                return ret_code; // Return the number of bytes received
            }
        }
        else {
            return UDP_NON_UDP_DATA_IS_AVAILABLE;
        }
    }
}

// udp transmit function. wrapper for sendo that abstracts the UDP_Info_struct
static int udp_transmit(UDP_Info *udp_host, void * data, int size)
{

    if (size > UDP_MAX_PACKET_SIZE) {
        return -1;
    }

    return sendto(udp_host->socket, data, size, 0, (struct sockaddr *)&udp_host->Addr, udp_host->AddrLen);
}


static bool initiate_socket_connection(int &host_socket)
{
    cout << endl << "Initiating Socket Connection with host..." << endl;

    udp_host.AddrLen = sizeof(udp_host.Addr);

    // Create a UDP socket
    host_socket = socket(AF_INET, SOCK_DGRAM, 0);

    if (host_socket < 0) {
        cerr << "[UDP ERROR] Failed to create socket [" << host_socket << "]" << endl;
        return false;
    }

    struct sockaddr_in serverAddr;
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(12345); 
    serverAddr.sin_addr.s_addr = INADDR_ANY;

    if (bind(host_socket, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) < 0) {
        cerr << "[UDP ERROR] Failed to bind socket" << endl;
        close(host_socket);
        return false;
    }

    cout << "UDP Connection Success !" << endl << endl;
    return true;
}

// calculate the size of a sample in quadlets
static uint16_t calculate_quadlets_per_sample(uint8_t num_encoders, uint8_t num_motors)
{
    // SAMPLE STRUCTURE

    // 1 quadlet = 4 bytes

    // Timestamp (Double -> 64 bits stored as two seperate quadlets)                                                           [1 quadlet]
    // Encoder Position (32 * num of encoders)                                      [1 quadlet * num of encoders]
    // Encoder Velocity Predicted (64 * num of encoders -> truncated to 32bits)     [1 quadlet * num of encoders]
    // Motur Current and Motor Status (32 * num of Motors -> each are 16 bits)      [1 quadlet * num of motors]
    // Digtial IO Values  (optional, used if PS IO is enabled ) 32 bits             [1 quadlet * digital IO]
    // MIO Pins (optional, used if PS IO is enabled ) 4 bits -> pad 32 bits         [1 quadlet * MIO PINS]  
    // POT pins (optional, used if Pot is enabled) num_encoders * 16 bits    
    
    int default_quadlets_per_sample = (2 + (2*(num_encoders)) + (num_motors));
    int quadlets_per_sample = default_quadlets_per_sample;

    if (use_ps_io_flag){
        quadlets_per_sample += 2;
    }

    if (use_pot_flag) {
        quadlets_per_sample += (1 * num_encoders);
    }

    return quadlets_per_sample;
}

// calculates the # of samples per packet in quadlets
static uint16_t calculate_samples_per_packet(uint8_t num_encoders, uint8_t num_motors)
{
    return ((UDP_MAX_PACKET_SIZE/4)/ calculate_quadlets_per_sample(num_encoders, num_motors) );
}

// calculate # of quadlets per packet
static uint16_t calculate_quadlets_per_packet(uint8_t num_encoders, uint8_t num_motors)
{
    return (calculate_samples_per_packet(num_encoders, num_motors) * calculate_quadlets_per_sample(num_encoders, num_motors));
}

// Compute elapsed seconds between two timespecs
static double ts_diff_s(const timespec &start, const timespec &end) {
    time_t  dsec  = end.tv_sec  - start.tv_sec;
    long    dnsec = end.tv_nsec - start.tv_nsec;
    return double(dsec) + double(dnsec) * 1e-9;
}

// loads data buffer for data collection
    // size of the data buffer is dependent on encoder count and motor count
    // see calculate_quadlets_per_sample method for data formatting
static bool load_data_packet(Dvrk_Controller dvrk_controller, uint32_t *data_packet, uint8_t num_encoders, uint8_t num_motors)
{   

    if (data_packet == NULL) {
        cout << "[ERROR - load_data_packet] databuffer pointer is null" << endl;
        return false;
    }

    if (sizeof(data_packet) == 0) {
        cout << "[ERROR - load_data_packet] len of databuffer == 0" << endl;
        return false;
    }

    uint16_t samples_per_packet = calculate_samples_per_packet(num_encoders, num_motors);
    uint16_t count = 0;

    // CAPTURE DATA 
    for (int j = 0; j < samples_per_packet; j++) {

        timespec t0;
        clock_gettime(CLOCK_MONOTONIC_RAW, &t0);

        if (!dvrk_controller.Port->ReadAllBoards()) {
            emio_read_error_counter++;
            return false;
        }

        if (!dvrk_controller.Board->ValidRead()) {
            cout << "[ERROR in load_data_packet] invalid read for ReadAllBoards" << endl;
            return false;
        }

        double time_elapsed = ts_diff_s(t_data_collection_start, t0);

        last_timestamp = time_elapsed;

        uint64_t timestamp_uint64 = *reinterpret_cast<uint64_t *>(&time_elapsed);

        uint32_t timestamp_high = (timestamp_uint64  >> 32);
        uint32_t timestamp_low =  (uint32_t) (timestamp_uint64 & 0xFFFFFFFF);

        // float time_elapsed_float = static_cast<float>(time_elapsed);
        data_packet[count++] = timestamp_high;
        data_packet[count++] = timestamp_low;

        // DATA 2: encoder position
        for (int i = 0; i < num_encoders; i++) {
            int32_t encoder_pos = dvrk_controller.Board->GetEncoderPosition(i);
            data_packet[count++] = static_cast<uint32_t>(encoder_pos + dvrk_controller.Board->GetEncoderMidRange());
        }

        // DATA 3: encoder velocity
        for (int i = 0; i < num_encoders; i++) {
            float encoder_velocity_float= static_cast<float>(dvrk_controller.Board->GetEncoderVelocityPredicted(i));
            data_packet[count++] = *reinterpret_cast<uint32_t *>(&encoder_velocity_float);
        }

        // DATA 4 & 5: motor current and motor status (for num_motors)
        for (int i = 0; i < num_motors; i++) {
            uint32_t motor_curr = dvrk_controller.Board->GetMotorCurrent(i); 
            // uint32_t motor_status = dvrk_controller.Board->GetMotorStatus(i);

            uint32_t raw_cmd_current;
            dvrk_controller.Port->ReadQuadlet(dvrk_controller.Port->GetBoardId(0), ((i+1) << 4) | 1, raw_cmd_current);
            // int16_t raw_cmd_current_16_bit = static_cast<int16_t>(raw_cmd_current);
            // uint16_t cmd_current_casted = *reinterpret_cast<uint16_t *>(&raw_cmd_current_16_bit);
  
            data_packet[count++] = (uint32_t)(((raw_cmd_current & 0x0000FFFF) << 16) | (motor_curr & 0x0000FFFF));
        }

        if (use_ps_io_flag){
            data_packet[count++] = dvrk_controller.Board->ReadDigitalIO();
            data_packet[count++] = (uint32_t) returnMIOPins();
        }

        if (use_pot_flag){
            for (int i = 0; i < num_encoders; i++) {
                data_packet[count++] = dvrk_controller.Board->GetAnalogInput(i);
            }
        }

        
        if (useSampleRate){
            
            deadline.tv_nsec += period_ns;
            if (deadline.tv_nsec >= 1'000'000'000) {
                deadline.tv_sec++;
                deadline.tv_nsec -= 1'000'000'000;
            }

            // 4) busy spin until deadline, subtracting work_ns if you like
            timespec now;
            do {
                clock_gettime(CLOCK_MONOTONIC_RAW, &now);
            } while ((  now.tv_sec  < deadline.tv_sec) ||
                        (now.tv_sec == deadline.tv_sec && now.tv_nsec < deadline.tv_nsec));
        }

        sample_count++;
    }

    return true;    
}

void package_meta_data(DataCollectionMeta *dc_meta, AmpIO *board)
{
    uint8_t num_encoders = (uint8_t) board->GetNumEncoders();
    uint8_t num_motors = (uint8_t) board->GetNumMotors();

    dc_meta->hwvers = board->GetHardwareVersion();
    dc_meta->num_encoders = (uint32_t) num_encoders;
    dc_meta->num_motors = (uint32_t) num_motors;

    dc_meta->data_packet_size = (uint32_t)calculate_quadlets_per_packet(num_encoders, num_motors) * 4;
    dc_meta->size_of_sample = (uint32_t) calculate_quadlets_per_sample(num_encoders, num_motors);
    dc_meta->samples_per_packet = (uint32_t) calculate_samples_per_packet(num_encoders, num_motors);
}

void reset_double_buffer_info(Double_Buffer_Info *db, AmpIO *board)
{
    db->cons_buf = 0;
    db->prod_buf = 0;
    db->cons_busy = 0;
    db->buffer_size = calculate_quadlets_per_packet(board->GetNumEncoders(), board->GetNumMotors()) * 4;

    memset(db->double_buffer, 0, sizeof(db->double_buffer));
}

void *consume_data(void *arg)
{
    Double_Buffer_Info* db = (Double_Buffer_Info*)arg;

    while (!stop_data_collection_flag) {

        if (db->prod_buf != db->cons_buf) {
            
            db->cons_busy = 1; 
            udp_transmit(&udp_host, db->double_buffer[db->cons_buf], db->buffer_size);
            data_packet_count++;
            db->cons_busy = 0; 

            db->cons_buf = (db->cons_buf + 1) % 2;
        }   
    }

    return nullptr;
}

SM wait_for_host_handshake( SM sm ){
    memset(recvd_cmd, 0, CMD_MAX_STRING_SIZE);
    sm.udp_ret = udp_nonblocking_receive(&udp_host, recvd_cmd, CMD_MAX_STRING_SIZE);

    if (sm.udp_ret > 0) {
        if (strcmp(recvd_cmd,  HOST_READY_CMD) == 0) {
            cout << "Received Message - " <<  HOST_READY_CMD << endl;
            sm.state = SM_WAIT_FOR_HOST_FLAG_CMD;
        } else {
            sm.ret = SM_OUT_OF_SYNC;
            sm.last_state = sm.state;
            sm.state = SM_TERMINATE;
        }
    }
    else if (sm.udp_ret == UDP_DATA_IS_NOT_AVAILABLE_WITHIN_TIMEOUT || sm.udp_ret == UDP_NON_UDP_DATA_IS_AVAILABLE) {
        sm.state = SM_WAIT_FOR_HOST_HANDSHAKE;
    }
    else {
        sm.ret = SM_UDP_ERROR;
        sm.last_state = sm.state;
        sm.state = SM_TERMINATE;
    }

    return sm;
}

SM wait_for_host_flag_cmd(SM sm){
    memset(recvd_cmd, 0, CMD_MAX_STRING_SIZE);
    sm.udp_ret = udp_nonblocking_receive(&udp_host, recvd_cmd, CMD_MAX_STRING_SIZE);

    if (sm.udp_ret > 0) {
        if (strcmp(recvd_cmd, HOST_FLAG_CMD) == 0) {
            cout << "Received Message - " << HOST_FLAG_CMD << endl;
            sm.state = SM_WAIT_FOR_HOST_FLAG_VALUE;
        } else {
            sm.ret = SM_OUT_OF_SYNC;
            sm.last_state = sm.state;
            sm.state = SM_TERMINATE;
        }
    }
    else if (sm.udp_ret == UDP_DATA_IS_NOT_AVAILABLE_WITHIN_TIMEOUT || sm.udp_ret == UDP_NON_UDP_DATA_IS_AVAILABLE) {
        sm.state = SM_WAIT_FOR_HOST_FLAG_CMD;
    }
    else {
        sm.ret = SM_UDP_ERROR;
        sm.last_state = sm.state;
        sm.state = SM_TERMINATE;
    }

    return sm;
}

SM wait_for_host_flag_value(SM sm){
    uint8_t flag_cmd = 0x00;
    sm.udp_ret = udp_nonblocking_receive(&udp_host, &flag_cmd, sizeof(flag_cmd));

    if (sm.udp_ret > 0) {
        use_ps_io_flag = (flag_cmd & ENABLE_PSIO_MSK);
        use_pot_flag = (flag_cmd & ENABLE_POT_MSK);
        useSampleRate = (flag_cmd & ENABLE_SAMPLE_RATE_MSK);

        cout << "Received Flag Byte: 0x" << std::hex << static_cast<int>(flag_cmd) << std::dec << endl;

        if (useSampleRate){
            sm.state = SM_WAIT_FOR_HOST_SAMPLE_RATE_CMD;
        } else {
            if (use_ps_io_flag || use_pot_flag){
                reset_double_buffer_info(&db, dvrk_controller.Board);
            }
            sm.state = SM_SEND_DATA_COLLECTION_METADATA;
        }
    }
    else if (sm.udp_ret == UDP_DATA_IS_NOT_AVAILABLE_WITHIN_TIMEOUT || sm.udp_ret == UDP_NON_UDP_DATA_IS_AVAILABLE) {
        sm.state = SM_WAIT_FOR_HOST_FLAG_VALUE;
    }
    else {
        sm.ret = SM_UDP_ERROR;
        sm.last_state = sm.state;
        sm.state = SM_TERMINATE;
    }

    return sm;
}

SM wait_for_host_sample_rate_cmd(SM sm){
    memset(recvd_cmd, 0, CMD_MAX_STRING_SIZE);
    sm.udp_ret = udp_nonblocking_receive(&udp_host, recvd_cmd, CMD_MAX_STRING_SIZE);

    if (sm.udp_ret > 0) {
        if (strcmp(recvd_cmd, HOST_SAMPLE_RATE_CMD) == 0){
            cout << "Received Message - " << HOST_SAMPLE_RATE_CMD << endl;
            sm.state = SM_WAIT_FOR_HOST_SAMPLE_RATE_VALUE;
        } else {
            sm.ret = SM_OUT_OF_SYNC;
            sm.last_state = sm.state;
            sm.state = SM_TERMINATE;
        }
    }
    else if (sm.udp_ret == UDP_DATA_IS_NOT_AVAILABLE_WITHIN_TIMEOUT || sm.udp_ret == UDP_NON_UDP_DATA_IS_AVAILABLE) {
        sm.state = SM_WAIT_FOR_HOST_SAMPLE_RATE_CMD;
    }
    else {
        sm.ret = SM_UDP_ERROR;
        sm.last_state = sm.state;
        sm.state = SM_TERMINATE;
    }

    return sm;
}

SM wait_for_host_sample_rate_value(SM sm){
    int host_sample_rate = 0;
    sm.udp_ret = udp_nonblocking_receive(&udp_host, &host_sample_rate, sizeof(host_sample_rate));

    if (sm.udp_ret > 0){
        SAMPLE_RATE = host_sample_rate;
        printf("NEW SAMPLE RATE: %d\n", SAMPLE_RATE);

        // Keep existing behavior when sample-rate mode is enabled.
        use_ps_io_flag = true;

        if (use_ps_io_flag){
            reset_double_buffer_info(&db, dvrk_controller.Board);
        }

        sm.state = SM_SEND_DATA_COLLECTION_METADATA;
    }
    else if (sm.udp_ret == UDP_DATA_IS_NOT_AVAILABLE_WITHIN_TIMEOUT || sm.udp_ret == UDP_NON_UDP_DATA_IS_AVAILABLE) {
        sm.state = SM_WAIT_FOR_HOST_SAMPLE_RATE_VALUE;
    }
    else {
        sm.ret = SM_UDP_ERROR;
        sm.last_state = sm.state;
        sm.state = SM_TERMINATE;
    }

    return sm;
}

SM send_data_collection_meta_data( SM sm ){
    package_meta_data(&data_collection_meta, dvrk_controller.Board);

    if (udp_transmit(&udp_host,  &data_collection_meta, sizeof(struct DataCollectionMeta )) < 1 ) {
        sm.ret = SM_UDP_INVALID_HOST_ADDR;
        sm.last_state = SM_TERMINATE;
    }
    else {
        sm.state = SM_WAIT_FOR_HOST_RECV_METADATA;
    }

    return sm;
}

SM wait_for_host_to_recv_metadata( SM sm ){
    memset(recvd_cmd, 0, CMD_MAX_STRING_SIZE);
    sm.udp_ret = udp_nonblocking_receive(&udp_host, recvd_cmd, CMD_MAX_STRING_SIZE);

    if (sm.udp_ret > 0) {
        if (strcmp(recvd_cmd, HOST_RECVD_METADATA) == 0) {
            cout << "Received Message: " << HOST_RECVD_METADATA << endl;
            cout << "Handshake Complete!" << endl;

            sm.state = SM_SEND_READY_STATE_TO_HOST;
        } else {
            sm.ret = SM_OUT_OF_SYNC;
            sm.last_state = sm.state;
            sm.state = SM_TERMINATE;
        }
    }
    else if (sm.udp_ret == UDP_DATA_IS_NOT_AVAILABLE_WITHIN_TIMEOUT || sm.udp_ret == UDP_NON_UDP_DATA_IS_AVAILABLE) {
        // Stay in same state
        sm.state = SM_WAIT_FOR_HOST_RECV_METADATA;
    } else {
        sm.ret = SM_UDP_ERROR;
        sm.last_state = sm.state;
        sm.state = SM_TERMINATE;
    }

    return sm;
}

SM send_ready_state_to_host( SM sm ){
    if (udp_transmit(&udp_host,  (char *) ZYNQ_READY_CMD, sizeof(ZYNQ_READY_CMD)) < 1 ) {
        sm.ret = SM_UDP_INVALID_HOST_ADDR;
        sm.state = SM_TERMINATE;
    }
    else {
        sm.state = SM_WAIT_FOR_HOST_START_CMD;
        cout << endl << "Waiting for Host to start data collection..." << endl << endl;
    }

    return sm;
}

SM wait_for_host_start_cmd( SM sm ){
    memset(recvd_cmd, 0, CMD_MAX_STRING_SIZE);
    sm.udp_ret = udp_nonblocking_receive(&udp_host, recvd_cmd, CMD_MAX_STRING_SIZE);

    if (sm.udp_ret > 0) {
        if (strcmp(recvd_cmd, HOST_START_DATA_COLLECTION) == 0) {
            cout << "Received Message: " <<  recvd_cmd << endl;
            sm.state = SM_START_DATA_COLLECTION;
        }
        else if (strcmp(recvd_cmd, HOST_TERMINATE_SERVER) == 0) {
            cout << "Received Message: " <<  recvd_cmd << endl;
            sm.ret = SM_SUCCESS;
            sm.state = SM_TERMINATE;
        }
        else {
            sm.ret = SM_OUT_OF_SYNC;
            sm.last_state = sm.state;
            sm.state = SM_TERMINATE;
        }
    }
    else if (sm.udp_ret == UDP_DATA_IS_NOT_AVAILABLE_WITHIN_TIMEOUT || sm.udp_ret == UDP_NON_UDP_DATA_IS_AVAILABLE) {
        sm.state = SM_WAIT_FOR_HOST_START_CMD;
    }
    else {
        sm.ret = SM_UDP_ERROR;
        sm.last_state = sm.state;
        sm.state = SM_TERMINATE;
    }

    return sm;
}

SM start_consumer_thread( SM sm , pthread_t *consumer_t){
    // Starting Consumer Thread: sends packets to host
    if (pthread_create(consumer_t, nullptr, consume_data, &db) != 0) {
        std::cerr << "Error creating consumer thread" << std::endl;
        sm.state = SM_EXIT;
        sm.ret = SM_FAILED_TO_CREATE_THREAD;
    }

    pthread_detach(*consumer_t);

    sm.state = SM_PRODUCE_DATA;

    return sm;
}

SM produce_data( SM sm ){

    if ( !load_data_packet(dvrk_controller, db.double_buffer[db.prod_buf], data_collection_meta.num_encoders, data_collection_meta.num_motors)) {
        cout << "[ERROR]load data buffer fail" << endl;
        sm.state = SM_EXIT;
        sm.ret = SM_BOARD_ERROR;
        return sm;
    }

    while (db.cons_busy) {}

    // Switch to the next buffer
    db.prod_buf = (db.prod_buf + 1) % 2;

    sm.state = SM_CHECK_FOR_STOP_DATA_COLLECTION_CMD;

    return sm;
}

SM check_for_stop_data_collection(SM sm, pthread_t consumer_t){

    sm.udp_ret = udp_nonblocking_receive(&udp_host, recvd_cmd, CMD_MAX_STRING_SIZE);

    if (sm.udp_ret > 0) {
        if (strcmp(recvd_cmd, HOST_STOP_DATA_COLLECTION) == 0) {
            cout << "Message from Host: STOP DATA COLLECTION" << endl;

            stop_data_collection_flag = true;

            pthread_join(consumer_t, nullptr);

            cout << "------------------------------------------------" << endl;
            cout << "UDP DATA PACKETS SENT TO HOST: " << data_packet_count << endl;
            cout << "SAMPLES SENT TO HOST: " << sample_count << endl;
            cout << "EMIO ERROR COUNT: " << emio_read_error_counter << endl;
            cout << "TIME ELAPSED: " << last_timestamp << endl;
            cout << "AVERAGE SAMPLE RATE: " << (float) (sample_count / last_timestamp) << "Hz" << endl;
            cout << "------------------------------------------------" << endl << endl;

            emio_read_error_counter = 0; 
            data_packet_count = 0;
            sample_count = 0;

            sm.state = SM_WAIT_FOR_HOST_START_CMD;
            cout << "Waiting for command from host..." << endl;
            
        } else {
            sm.ret = SM_OUT_OF_SYNC;
            sm.last_state = sm.state;
            sm.state = SM_TERMINATE;
        }
    } else if (sm.udp_ret == UDP_DATA_IS_NOT_AVAILABLE_WITHIN_TIMEOUT || sm.udp_ret == UDP_NON_UDP_DATA_IS_AVAILABLE){
        sm.state = SM_PRODUCE_DATA;
    } else {
        sm.ret = SM_UDP_ERROR;
        sm.last_state = sm.state;
        sm.state = SM_TERMINATE;
    }

    return sm;
}

SM start_data_collection(SM sm){

    stop_data_collection_flag = false;
    clock_gettime(CLOCK_MONOTONIC_RAW, &t_data_collection_start);

    if (useSampleRate){
        clock_gettime(CLOCK_MONOTONIC_RAW, &deadline);
        // compute the nanoseconds between samples
        period_ns = 1'000'000'000L / SAMPLE_RATE;
    }
   
    sm.state = SM_START_CONSUMER_THREAD;
    return sm;
}

SM terminate_data_collection(SM sm){

    if (sm.ret != SM_SUCCESS) {

        cout << "[ERROR] STATEMACHINE TERMINATING" << endl;

        cout << "At STATE " << sm.last_state << " ";

        switch (sm.ret){
            
            case SM_OUT_OF_SYNC:
                cout << "Zynq of sync with Host. Received unexpected command: " << recvd_cmd << endl;
                break;
            case SM_UDP_ERROR:
                cout << "Udp ERROR. Make sure host program is running." << endl;
                break;
            case SM_UDP_INVALID_HOST_ADDR:
                cout << "Udp ERROR. Invalid Host Address format" << endl;
                break;
            case SM_FAILED_TO_CREATE_THREAD:
                cout << "Failed to Create Thread" << endl;
            case SM_BOARD_ERROR:
                cout << "Board Error" << endl;
        }

    } else {
        cout << "STATE MACHINE SUCCESS !" << endl;
    }

    cout << endl << ZYNQ_TERMINATATION_SUCCESSFUL << endl;

    udp_transmit(&udp_host,  (void*) ZYNQ_TERMINATATION_SUCCESSFUL, sizeof(ZYNQ_TERMINATATION_SUCCESSFUL));

    close(udp_host.socket);
    sm.state = SM_EXIT;

    return sm;
}


static int dataCollectionStateMachine()
{
    SM sm;
    pthread_t consumer_t;
    
    cout << "Starting Handshake Routine..." << endl << endl;
    cout << "Start Data Collection Client on HOST to complete handshake..." << endl;

    reset_double_buffer_info(&db, dvrk_controller.Board);

    if (mio_mmap_init() != 0) {
        sm.state = SM_TERMINATE;
        sm.ret = SM_PS_IO_FAIL;
    }

    sm.state = SM_WAIT_FOR_HOST_HANDSHAKE;

    while (sm.state != SM_EXIT) {

        switch (sm.state) {
            case SM_WAIT_FOR_HOST_HANDSHAKE:
                sm = wait_for_host_handshake(sm);
                break;

            case SM_WAIT_FOR_HOST_FLAG_CMD:
                sm = wait_for_host_flag_cmd(sm);
                break;

            case SM_WAIT_FOR_HOST_FLAG_VALUE:
                sm = wait_for_host_flag_value(sm);
                break;

            case SM_WAIT_FOR_HOST_SAMPLE_RATE_CMD:
                sm = wait_for_host_sample_rate_cmd(sm);
                break;

            case SM_WAIT_FOR_HOST_SAMPLE_RATE_VALUE:
                sm = wait_for_host_sample_rate_value(sm);
                break;

            case SM_SEND_DATA_COLLECTION_METADATA:
                sm = send_data_collection_meta_data(sm);
                break;

            case SM_WAIT_FOR_HOST_RECV_METADATA:
                sm = wait_for_host_to_recv_metadata(sm);
                break;

            case SM_SEND_READY_STATE_TO_HOST:
                sm = send_ready_state_to_host(sm);
                break;

            case SM_WAIT_FOR_HOST_START_CMD:
                sm = wait_for_host_start_cmd(sm);
                break;

            case SM_START_DATA_COLLECTION:
                sm = start_data_collection(sm);
                break;

            case SM_START_CONSUMER_THREAD:
                sm = start_consumer_thread(sm, &consumer_t);
                break;

            case SM_PRODUCE_DATA:
                sm = produce_data(sm);
                break;

            case SM_CHECK_FOR_STOP_DATA_COLLECTION_CMD:
                sm = check_for_stop_data_collection(sm, consumer_t);     
                break;

            case SM_TERMINATE:
                sm = terminate_data_collection(sm);
                break;
        }
    }
    return sm.ret;
}



int main()
{
    string portDescription = BasePort::DefaultPort();
    dvrk_controller.Port = PortFactory(portDescription.c_str());

    if (!dvrk_controller.Port->IsOK()) {
        std::cerr << "Failed to initialize " << dvrk_controller.Port->GetPortTypeString() << std::endl;
        return -1;
    }

    if (dvrk_controller.Port->GetNumOfNodes() == 0) {
        std::cerr << "Failed to find any boards" << std::endl;
        return -1;
    }

    ZynqEmioPort *EmioPort = dynamic_cast<ZynqEmioPort *>(dvrk_controller.Port);
    if (EmioPort) {
        cout << "Verbose: " << EmioPort->GetVerbose() << std::endl;
        // EmioPort->SetVerbose(true);
        EmioPort->SetTimeout_us(80);
    }
    else {
      cout << "[warning] failed to dynamic cast to ZynqEmioPort" << endl;
    }

    dvrk_controller.Board = new AmpIO(dvrk_controller.Port->GetBoardId(0));

    dvrk_controller.Port->AddBoard(dvrk_controller.Board);

    bool isOK = initiate_socket_connection(udp_host.socket);

    if (!isOK) {
        cout << "[error] failed to establish socket connection !!" << endl;
        return -1;
    }

    dataCollectionStateMachine();

    return 0;
}