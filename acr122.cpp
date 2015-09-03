/*-
 * ACR122U Near Field Communication (NFC)
 *
 * Copyright (C) 2015      Erwin Bejsta
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */

#include <fcntl.h>
#include <memory.h>
//#include <nfc/nfc.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <syslog.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include <iostream>

#include "acr122_usb.h"

using namespace std;

#define LOG_LEVEL_MAX 3
int logLevel = 0;   // 1 = error, 2 = info, 3 = debug
static int receivedSignal = 0;
static string execName;

void sigHandler(int signum)
{
    receivedSignal = signum;
    char signame[10];
    switch (signum) {
        case SIGTERM:
            strcpy(signame, "SIGTERM");
            break;
        case SIGHUP:
            strcpy(signame, "SIGHUP");
            break;
        case SIGINT:
            strcpy(signame, "SIGINT");
            break;
            
        default:
            break;
    }
    //cerr << endl << "Received " << signame << endl;
}

static void showUsage(void) {
    cout << "usage:" << endl;
    cout << execName << " -d[1-3] -h" << endl;
    //cout << "c = name of config file (.cfg is added automatically)" << endl;
    cout << "dx = debug level (x=1 to 3)" << endl;
    cout << "h = show help" << endl;
}

static bool parseArguments (int argc, const char *argv[])
{
    char buffer[64];
    int i, buflen;
    int retval = true;
    
    execName = std::string(basename(argv[0]));
    
    if (argc > 1) {
        for (i = 1; i < argc; i++) {
            strcpy(buffer, argv[i]);
            buflen = strlen(buffer);
            if ((buffer[0] == '-') && (buflen >=2)) {
                switch (buffer[1]) {
                    case 'd':
                        logLevel = buffer[2] - 0x30;
                        // enforce log level limits
                        logLevel = std::min(logLevel, LOG_LEVEL_MAX);
                        logLevel = std::max(logLevel, 0);
                        break;
                    case 'h':
                        showUsage();
                        retval = false;
                        break;
                    default:
                        std::cerr << "uknown parameter <" << &buffer[1] << ">" << endl;
                        showUsage();
                        retval = false;
                        break;
                }
            }
        }
    }
    return retval;
}

int main(int argc, const char *argv[])
{
    long x, y, z;
    ACR122_USB reader;
    uint16_t vendor_id, product_id;
    
    if (! parseArguments(argc,argv) )
        return EXIT_FAILURE;
    
    cout << "Loglevel = " << logLevel << endl;
    
    signal (SIGINT, sigHandler);
    signal (SIGHUP, sigHandler);
    signal (SIGTERM, sigHandler);
    
    if (! reader.usb_scan(&vendor_id, &product_id)) {
        goto exit_fail;
    }
    
    if (! reader.openDevice(vendor_id, product_id) ) {
        goto exit_fail;
    }
    
    printf("Opened %s (VendorID: %04x ProductID: %04x)\n", reader.getDeviceName() , vendor_id, product_id);

    reader.init(true);
    
    while (!receivedSignal) {
        // need to call process regularly so libusb can respond
        if (reader.process()) {
            printf("Target ID: %s\n", reader.getUID() );
        };
        usleep(250000);
    }
    
    // NOTE: reader object is destroyed (and thereby closed) automatically on return.
    
    return EXIT_SUCCESS;
    
exit_fail:
    return EXIT_FAILURE;
    
}


