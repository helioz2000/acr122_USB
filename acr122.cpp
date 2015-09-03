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

#include "acr122_usb.h"

int logLevel = 1;   // 1 = error, 2 = info, 3 = debug
static int receivedSignal = 0;

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


int main(int argc, const char *argv[])
{
    long x, y, z;
    ACR122_USB reader;
    uint16_t vendor_id, product_id;
    
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


