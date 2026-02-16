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
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <cstdlib>
#include <getopt.h>
#include <sys/select.h>
#include <chrono>
#include <ctime>
#include <cstdlib>
#include <fstream>
#include <string>
#include <ctype.h>
#include <climits>

#include "data_collection.h"

using namespace std;

static bool isInteger(const char* str)
{
    if (str == NULL || (strcmp(str, "") == 0)) {
        return false;
    }

    int strLength = strlen(str);

    for (int i = 0; i < strLength; i++) {
        if (!isdigit(str[i])) {
            return false;
        }
    }
    return true;
}

static bool isFloat(const char* str)
{
    if (str == NULL || (strcmp(str, "") == 0)) {
        return false;
    }

    int strLength = strlen(str);
    bool hasDecimalPoint = false;

    for (int i = 0; i < strLength; i++) {
        if (str[i] == '.') {

            if (hasDecimalPoint) {
                return false;
            }
            hasDecimalPoint = true;
        } else if (!isdigit(str[i])) {
            return false;
        }
    }

    return hasDecimalPoint && strLength > 1;
}

static void printUsage(const char *progName)
{
    cout << endl;
    cout << "                 dVRK Data Collection Program" << endl;
    cout << "|-----------------------------------------------------------------------" << endl;
    cout << "|Usage: " << progName << " <boardID> [-t <seconds>] [-s <Hz>] [-i] [-p]" << endl;
    cout << "|" << endl;
    cout << "|Arguments:" << endl;
    cout << "|  <boardID>          Required. ID of the board to connect to." << endl;
    cout << "|" << endl;
    cout << "|Options:" << endl;
    cout << "|  -t <seconds>       Optional. Duration for data capture in seconds (float)." << endl;
    cout << "|  -s <Hz>            Optional. Sample rate in Hz (integer)." << endl;
    cout << "|  -i                 Optional. Include PS IO in data packet." << endl;
    cout << "|  -p                 Optional. Include potentiometer readings in data packet." << endl;
    cout << "|  -h                 Show this help message." << endl;
    cout << "|" << endl;
    cout << "|[NOTE] Ensure the server is started before running the client." << endl;
    cout << "__________________________________________________________________________" << endl;
}

static bool isExitKeyPressed()
{
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(STDIN_FILENO, &readfds); // Monitor stdin for input

    struct timeval timeout = {0, 0}; // No wait time
    int ret = select(STDIN_FILENO + 1, &readfds, NULL, NULL, &timeout);

    if (ret > 0 && FD_ISSET(STDIN_FILENO, &readfds)) {
        char buf[256];
        fgets(buf, sizeof(buf), stdin); // Consume input
        return true;
    }
    return false;
}


int main(int argc, char *argv[])
{
    float data_collection_duration_s= 0;
    bool timedCaptureFlag = false;
    bool use_ps_io_flag = false;
    bool use_pot_flag = false;
    bool use_sample_rate = false;
    uint8_t options_mask = 0x00;
    uint8_t boardID = 0;
    int sample_rate = 0;

    if (argc == 1) {
        printUsage(argv[0]);
        return 0;
    }

    if (argc == 2 && (strcmp(argv[1], "-h") == 0 || strcmp(argv[1], "--help") == 0)) {
        printUsage(argv[0]);
        return 0;
    }

    if (!isInteger(argv[1])) {
        cout << "[ERROR] Invalid boardID arg: " << argv[1] << endl;
        printUsage(argv[0]);
        return -1;
    }

    long parsed_board_id = strtol(argv[1], nullptr, 10);
    if (parsed_board_id < 0 || parsed_board_id > UCHAR_MAX) {
        cout << "[ERROR] boardID out of range [0, " << UCHAR_MAX << "]: " << argv[1] << endl;
        return -1;
    }
    boardID = static_cast<uint8_t>(parsed_board_id);

    opterr = 0;
    optind = 1;
    int opt = 0;
    while ((opt = getopt(argc - 1, argv + 1, "t:s:iph")) != -1) {
        switch (opt) {
            case 't':
                if (!isFloat(optarg)) {
                    cout << "[ERROR] invalid time value " << optarg << " for timed capture. Pass in float" << endl;
                    return -1;
                }
                data_collection_duration_s = atof(optarg);
                timedCaptureFlag = true;
                cout << "Timed Capture Enabled!" << endl;
                break;

            case 's':
                if (!isInteger(optarg)) {
                    cout << "[ERROR] invalid sample rate value " << optarg << " for timed capture. Pass in Integer" << endl;
                    return -1;
                }
                use_sample_rate = true;
                sample_rate = atoi(optarg);
                cout << " Sample Rate set to " << sample_rate << "Hz" << endl;
                break;

            case 'i':
                use_ps_io_flag = true;
                cout << "PS IO pins will be included in data packet!" << endl;
                break;

            case 'p':
                use_pot_flag = true;
                cout << "Potentiometer readings will be included in data packet!" << endl;
                break;

            case 'h':
                printUsage(argv[0]);
                return 0;

            case '?':
                if (optopt == 't' || optopt == 's') {
                    cout << "[ERROR] Option -" << static_cast<char>(optopt) << " requires a value" << endl;
                } else {
                    cout << "[ERROR] Invalid arg: -" << static_cast<char>(optopt) << endl;
                }
                printUsage(argv[0]);
                return -1;

            default:
                printUsage(argv[0]);
                return -1;
        }
    }

    if (optind < argc - 1) {
        cout << "[ERROR] Unexpected extra positional argument: " << (argv + 1)[optind] << endl;
        printUsage(argv[0]);
        return -1;
    }

    if (use_ps_io_flag) {
        options_mask |= ENABLE_PSIO_MSK;
    }
    if (use_pot_flag) {
        options_mask |= ENABLE_POT_MSK;
    }
    if (use_sample_rate) {
        options_mask |= ENABLE_SAMPLE_RATE_MSK;
    }

    bool ret;

    DataCollection *DC = new DataCollection();
    bool stop_data_collection = false;

    if (!DC->init(boardID, options_mask, sample_rate)) {
        return -1;
    }

    int count = 1;

    while (!stop_data_collection) {

        cout << "Woud you like to start capture [" << count << "]? (y/n): ";

        char yn;
        cin >> yn;

        if (yn == 'y') {
            stop_data_collection = false;
        } else if (yn == 'n') {
            stop_data_collection = true;
            continue;
        } else {
            cout << "[error] Invalid character. Type either 'y' or 'n' and press enter: " << endl;
            stop_data_collection = false;
            continue;         
        }

        cout << endl;
    
        if (!DC->start()) {
            return -1;
        }
        cout << "...Press [ENTER] to terminate capture" << endl;

        if (timedCaptureFlag) {
            int data_collection_duration_us = data_collection_duration_s * 1000000;
            usleep(data_collection_duration_us);
        } else {
            while(1) {
                if (isExitKeyPressed()) {
                    break;
                } 
            }
        }

        if (!DC->stop()) {
            return -1;
        }

        count++;
    }

    ret = DC->terminate();

    return ret;
}
