/*-
 * ACR122U Near Field Communication (NFC)
 *
 * During the development of this code much reference was made to code published by Ludovic Rousseau
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

/**
 * @file acr122_usb.c
 * @brief Driver for ACR122 using direct USB (without PCSC)
 */

/*
 * This implementation was written based on information provided by the
 * following documents:
 *
 * Smart Card CCID
 * Specification for Integrated Circuit(s) Cards Interface Devices
 * Revision 1.1
 * April 22rd, 2005
 * http://www.usb.org/developers/devclass_docs/DWG_Smart-Card_CCID_Rev110.pdf
 *
 * ACR122U NFC Reader
 * Application Programming Interface
 * Revision 2.02
 * http://acs.com.hk/drivers/eng/API_ACR122U.pdf
 */


#define __STDC_FORMAT_MACROS // required before inttypes.h
#include <inttypes.h>
#include <errno.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <unistd.h>

#include "nfc.h"
#include "nfc-internal.h"
#include "pn53x-internal.h"
#include "acr122_usb.h"

//#define LOG_GROUP     NFC_LOG_GROUP_DRIVER
//#define LOG_CATEGORY "libnfc.driver.acr122_usb"

#define USB_INFINITE_TIMEOUT 0

/*
USB activity trace for PN533, ACR122 and Touchatag

--------------------------------------------------------------------
PN533
                     0000ff02fe d402          2a00
                     0000ff00ff00
                     ACK
                     0000ff06fa d50333020707  e500

--------------------------------------------------------------------
Acr122U PICC    pseudo-APDU through PCSC Escape mechanism:
6b07000000000a000000 ff00000002 d402
PC_to_RDR_Escape     APDU
  Len.....           ClInP1P2Lc
          Slot=0     pseudo-APDU DirectTransmit
            Seq=0a
              RFU=000000
8308000000000a028100            d50332010407  9000
RDR_to_PC_Escape                              SW: OK
  Len.....
          Slot=0
            Seq=0a
              Slot Status=02  ??
                Slot Error=81 ??
                  RFU=00

--------------------------------------------------------------------
Touchatag (Acr122U SAM) pseudo-APDU mechanism:
6f07000000000e000000 ff00000002 d402
PC_to_RDR_XfrBlock   APDU
  Len.....           ClInP1P2Lc
          Slot=0     pseudo-APDU DirectTransmit
            Seq=0e
              BWI=00
                RFU=0000
8002000000000e000000                          6108
RDR_to_PC_DataBlock                           SW: more data: 8 bytes
          Slot=0
            Seq=0e
              Slot Status=00
                Slot Error=00
                  RFU=00
6f05000000000f000000 ffc0000008
                     pseudo-ADPU GetResponse
8008000000000f000000            d50332010407  9000
                                              SW: OK

--------------------------------------------------------------------
Apparently Acr122U PICC can also work without Escape (even if there is no card):
6f070000000000000000 ff00000002 d402
PC_to_RDR_XfrBlock   APDU
  Len.....           ClInP1P2Lc
          Slot=0     pseudo-APDU DirectTransmit
            Seq=00
              BWI=00
                RFU=0000
80080000000000008100            d50332010407  9000
                                              SW: OK
*/

// Internal data struct
//struct acr122_usb_data {
//  usb_dev_handle *pudh;
//  uint32_t uiEndPointIn;
//  uint32_t uiEndPointOut;
//  uint32_t uiMaxPacketSize;
//  volatile int abort_flag;
//  // Keep some buffers to reduce memcpy() usage
//  struct acr122_usb_tama_frame tama_frame;
//  struct acr122_usb_apdu_frame apdu_frame;
//};

/* CCID Standardised Bulk Out message types */
#define PC_RDR_SET_PARAMS      0x61
#define PC_RDR_ICC_ON          0x62
#define PC_RDR_ICC_OFF         0x63
#define PC_RDR_GET_SLOT_STATUS 0x65
#define PC_RDR_SECURE          0x69
#define PC_RDR_T0APDU          0x6A
#define PC_RDR_ESCAPE          0x6B
#define PC_RDR_GET_PARAMS      0x6C
#define PC_RDR_RESET_PARAMS    0x6D
#define PC_RDR_ICC_CLOCK       0x6E
#define PC_RDR_XFR_BLOCK       0x6F
#define PC_RDR_MECH            0x71
#define PC_RDR_ABORT           0x72
#define PC_RDR_DATA_CLOCK      0x73

/* CCID Standardised Bulk In message types */
#define RDR_PC_DATA_BLOCK      0x80
#define RDR_PC_SLOT_STATUS     0x81
#define RDR_PC_PARAMS          0x82
#define RDR_PC_ESCAPE          0x83
#define RDR_PC_DATA_CLOCK      0x84

// ISO 7816-4
#define SW1_More_Data_Available 0x61
#define SW1_Warning_with_NV_changed 0x63
#define PN53x_Specific_Application_Level_Error_Code 0x7f

#define RDR_SLOT1_INSERT    0x03
#define RDR_SLOT1_REMOVED   0x02

// This frame template is copied at init time
// Its designed for TAMA sending but is also used for simple ADPU frame: acr122_build_frame_from_apdu() will overwrite needed bytes
const uint8_t acr122_usb_frame_template[] = {
  PC_RDR_XFR_BLOCK, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // CCID header
  0xff, 0x00, 0x00, 0x00, 0x00, // ADPU header
  0xd4, // PN532 direction
};

// APDUs instructions
#define APDU_GetAdditionnalData 0xc0

// Internal io struct
//const struct pn53x_io acr122_usb_io;

// Prototypes
//static int acr122_usb_init(nfc_device *pnd);
//static int acr122_usb_ack(nfc_device *pnd);
//static int acr122_usb_send_apdu(nfc_device *pnd,
//                                const uint8_t ins, const uint8_t p1, const uint8_t p2, const uint8_t *const data, size_t data_len, const uint8_t le,
//                                uint8_t *out, const size_t out_size);
//

extern  int logLevel;

#    define LOG_HEX(group, pcTag, pbtData, szBytes) do { \
size_t	 __szPos; \
char	 __acBuf[1024]; \
size_t	 __szBuf = 0; \
if ((int)szBytes < 0) { \
fprintf (stderr, "%s:%d: Attempt to print %d bytes!\n", __FILE__, __LINE__, (int)szBytes); \
log_put (group, LOG_CATEGORY, ACR_LOG_PRIORITY_ERROR, "%s:%d: Attempt to print %d bytes!\n", __FILE__, __LINE__, (int)szBytes); \
abort(); \
break; \
} \
snprintf (__acBuf + __szBuf, sizeof(__acBuf) - __szBuf, "%s: ", pcTag); \
__szBuf += strlen (pcTag) + 2; \
for (__szPos=0; (__szPos < (size_t)(szBytes)) && (__szBuf < sizeof(__acBuf)); __szPos++) { \
snprintf (__acBuf + __szBuf, sizeof(__acBuf) - __szBuf, "%02x ",((uint8_t *)(pbtData))[__szPos]); \
__szBuf += 3; \
} \
log_put (group, LOG_CATEGORY, ACR_LOG_PRIORITY_DEBUG, "%s", __acBuf); \
} while (0);


#define ACR_LOG_PRIORITY_NONE   0
#define ACR_LOG_PRIORITY_ERROR  1
#define ACR_LOG_PRIORITY_INFO   2
#define ACR_LOG_PRIORITY_DEBUG  3

#define LOG_GROUP 0
#define LOG_CATEGORY 0

#define USB_TIMEOUT_PER_PASS 200
#define USB_WRITE_TIMEOUT 1000
#define USB_READ_TIMEOUT 1000

bool ACR122_USB::irqIsEnabled = false;
bool ACR122_USB::targetPresent = false;


#if !defined(htole32)

uint32_t htole32(uint32_t u32);

uint32_t
htole32(uint32_t u32)
{
    union {
        uint8_t arr[4];
        uint32_t u32;
    } u;
    for (int i = 0; i < 4; i++) {
        u.arr[i] = (u32 & 0xff);
        u32 >>= 8;
    }
    return u.u32;
}

#endif /* !defined(htole32) */

#pragma mark - Log function

void log_put(const uint8_t group, const char *category, const uint8_t priority, const char *format, ...)
{
//    char *env_log_level = NULL;
//#ifdef ENVVARS
//    env_log_level = getenv("LIBNFC_LOG_LEVEL");
//#endif
//    uint32_t log_level;
//    if (NULL == env_log_level) {
//        // LIBNFC_LOG_LEVEL is not set
//#ifdef DEBUG
//        log_level = 3;
//#else
//        log_level = 1;
//#endif
//    } else {
//        log_level = atoi(env_log_level);
//    }
//    //  printf("log_level = %"PRIu32" group = %"PRIu8" priority = %"PRIu8"\n", log_level, group, priority);
    
    if (!logLevel) {
        return;
    }
    
    if ((logLevel & 0x00000003) >= priority) {
        va_list va;
        va_start(va, format);
        vfprintf(stderr, format, va);
        fprintf(stderr, "\n");
        va_end(va);
    }
}

#pragma mark - ACR122 Class
#pragma mark Interrupt Callback

/*
 * Interrupt Callback Method
 * this method is called by libusb for ansync data transfer functions
 */
void LIBUSB_CALL ACR122_USB::irqCallback(struct libusb_transfer *transfer)
{
	unsigned char irqtype = transfer->buffer[0];
    const char *status;
    uint8_t stat = transfer->status;
    
    if ( (stat == LIBUSB_TRANSFER_COMPLETED) && (transfer->actual_length > 1) ) {
        if (transfer->buffer[1] == RDR_SLOT1_INSERT) {
            ACR122_USB::targetPresent = true;
            log_put(LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_DEBUG, "%s - Inserted", __PRETTY_FUNCTION__);
        } else {
            ACR122_USB::targetPresent = false;
            log_put(LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_DEBUG, "%s - Removed", __PRETTY_FUNCTION__);
        }
    }
    
    switch (transfer->status) {
        case LIBUSB_TRANSFER_COMPLETED:
            if (!ACR122_USB::irqIsEnabled) {
                ACR122_USB::irqIsEnabled = true;
            }
            status = "Completed";
            //log_put(LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_DEBUG, "%s - [%02x]", __PRETTY_FUNCTION__, transfer->buffer[1]);
            break;
        case LIBUSB_TRANSFER_ERROR:
            status = "Error";
            break;
        case LIBUSB_TRANSFER_TIMED_OUT:
            status = "Timed Out";
            break;
        case LIBUSB_TRANSFER_CANCELLED:
            ACR122_USB::irqIsEnabled = false;
            status = "Cancelled";
            break;
        case LIBUSB_TRANSFER_STALL:
            status = "Stalled";
            break;
        case LIBUSB_TRANSFER_NO_DEVICE:
            status = "No Device";
            break;
        case LIBUSB_TRANSFER_OVERFLOW:
            status = "Overflow";
            break;
            
        default:
            status = "Unknown";
            break;
    }
    
    //log_put(LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_DEBUG, "%s - got interrupt, length %d, status <Transfer %s>", __PRETTY_FUNCTION__, transfer->actual_length, status);
    
    //	if (transfer->status != LIBUSB_TRANSFER_COMPLETED) {
    //		log_put(LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_ERROR, "%s - failure, irq transfer status %d?", __PRETTY_FUNCTION__, transfer->status);
    //		libusb_free_transfer(transfer);
    //		//irqTransfer = NULL;
    //		return;
    //	}
    
    // re-submit interrupt transfer if enabled
    if (irqIsEnabled) {
        if (libusb_submit_transfer(transfer) < 0) {
            log_put(LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_ERROR, "%s - libusb_submit_transfer failed for irqTransfer", __PRETTY_FUNCTION__ );
            ACR122_USB::irqIsEnabled = false;
        } else {
            ACR122_USB::irqIsEnabled = true;
        }
    }
    //log_put(LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_DEBUG, "%s - irqEnabled = %d", __PRETTY_FUNCTION__, ACR122_USB::irqIsEnabled);
}

#pragma mark -

/*
 * returns number of bytes received or LIBUSB_ERROR_xxxxxx on failure
 */
int ACR122_USB::bulk_read(uint8_t abtRx[], const size_t szRx, const int timeout)
{
    int bytesTransferred;

    int res = libusb_bulk_transfer(devHandle, endpointIn, (unsigned char *) abtRx, szRx, &bytesTransferred, timeout);
    if (res != LIBUSB_SUCCESS) {
        log_put(LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_ERROR, "%s - transfer failed.", __PRETTY_FUNCTION__);
    }
    // Note: bytesTransferred indicates the number of bytes acturally received
    LOG_HEX(LOG_GROUP, "RX", abtRx, bytesTransferred);
    if (res < LIBUSB_SUCCESS) {
        return res;
    }
    return bytesTransferred;
}

/*
 * returns number of bytes transmitted or LIBUSB_ERROR_xxxxxx on failure
 */
int ACR122_USB::bulk_write(uint8_t abtTx[], const size_t szTx, const int timeout)
{
    int bytesTransferred;
    LOG_HEX(LOG_GROUP, "TX", abtTx, szTx);
    int res = libusb_bulk_transfer(devHandle, endpointOut, (unsigned char *) abtTx, szTx, &bytesTransferred, timeout);
    if (res != LIBUSB_SUCCESS) {
        log_put(LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_ERROR, "%s - transfer failed <%d>.", __PRETTY_FUNCTION__, res);
    }
    if ((unsigned int)bytesTransferred != szTx) {
        log_put(LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_ERROR, "%s - bytes transferred <> buffer (%d %d).", __PRETTY_FUNCTION__, bytesTransferred, szTx);
    }
    if (res == LIBUSB_SUCCESS) {
        return bytesTransferred;
    }
    return res;
}

struct acr122_usb_supported_device {
  uint16_t vendor_id;
  uint16_t product_id;
  const char *name;
};

const struct acr122_usb_supported_device acr122_usb_supported_devices[] = {
  { 0x072F, 0x2200, "ACS ACR122" },
  { 0x072F, 0x90CC, "Touchatag" },
};

ACR122_USB::ACR122_USB () {
    int r = libusb_init(&usbContext);
	if (r < 0) {
		log_put(LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_ERROR, "failed to initialise libusb.");
		exit(1);
	}
    deviceName = NULL;
    deviceIsOpen = false;
    devHandle = NULL;
    irqTransfer = NULL;
    irqbuf = NULL;
    irqbufSize=0;
    irqIsEnabled = false;
    restoreKernelDriver = false;
    targetDetectedOneShot = false;
}

ACR122_USB::~ACR122_USB () {
    
    close();
    libusb_exit (usbContext);
}

// Find transfer endpoints for bulk transfers
bool ACR122_USB::setEndpoints (struct libusb_config_descriptor *conf_desc)
{
    const struct libusb_endpoint_descriptor *endpointDesc;
    uint8_t epAttributes;
    int k;
    
    endpointInInt = 0;
    endpointIn = 0;
    endpointOut = 0;
    // interogate all endpoints
    // 3 Endpoints maximum: Interrupt In, Bulk In, Bulk Out
    for (k=0; k<conf_desc->interface[0].altsetting[0].bNumEndpoints; k++) {
        struct libusb_ss_endpoint_companion_descriptor *ep_comp = NULL;
        endpointDesc = &conf_desc->interface[0].altsetting[0].endpoint[k];
        
        // get the end point attributes
        epAttributes = endpointDesc->bmAttributes & LIBUSB_TRANSFER_TYPE_MASK;
        if (epAttributes & (LIBUSB_TRANSFER_TYPE_BULK | LIBUSB_TRANSFER_TYPE_INTERRUPT)) {
            if (endpointDesc->bEndpointAddress & LIBUSB_ENDPOINT_IN) {
                if ( epAttributes == LIBUSB_TRANSFER_TYPE_INTERRUPT ) {
                    //IN INT
                    if (irqbufSize) {
                        delete irqbuf;
                    }
                    endpointInInt = endpointDesc->bEndpointAddress;
                    endpointInIntMaxSize = endpointDesc->wMaxPacketSize;
                    irqbuf = new unsigned char[endpointInIntMaxSize];
                }
                if ( epAttributes == LIBUSB_TRANSFER_TYPE_BULK ) {
                    //IN BULK
                    endpointIn = endpointDesc->bEndpointAddress;
                    endpointInMaxSize = endpointDesc->wMaxPacketSize;

                }
            } else {
                //OUT BULK
                endpointOut = endpointDesc->bEndpointAddress;
                endpointOutMaxSize = endpointDesc->wMaxPacketSize;
            }
        }
    }
    if (endpointIn && endpointOut && endpointInInt) {
        return true;
    }
    log_put(LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_ERROR, "%s - unable to find all endpoints.", __PRETTY_FUNCTION__ );
    return false;
}


/*
 * scan connected USB devices an find the first compatible reader device
 * returns TRUE if a compatible device was found.
 * fills parameters vendor_id and product_id with for found device
 */
bool ACR122_USB::usb_scan(uint16_t *vendor_id, uint16_t *product_id)
{
    int deviceFound = false;
    libusb_device **devs;
    libusb_device *dev;
    ssize_t deviceCount, n;
    uint8_t string_index[3];	// indexes of the string descriptors
    
    struct libusb_config_descriptor *conf_desc;
	const struct libusb_endpoint_descriptor *endpoint;
    int iface, nb_ifaces, first_iface = -1;
    
    uint8_t endpoint_bulk_in = 0, endpoint_bulk_out = 0;	// default IN and OUT endpoints
    uint8_t endpoint_int_in = 0;    // interrupt IN endpoint
    uint8_t ep_attributes;
    
	int r = 1, d = 0, i,j,k;
    unsigned int x  ;
    
    *vendor_id = 0;
    *product_id = 0;
    
    deviceCount = libusb_get_device_list (usbContext, &devs);
    
    while ((dev = devs[d++]) != NULL) {
		struct libusb_device_descriptor desc;
		r = libusb_get_device_descriptor(dev, &desc);
		if (r < 0) {
			log_put(LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_ERROR, "failed to get device descriptor");
			return false;
		}
        
		//printf("%04x:%04x (bus %d, device %d)", desc.idVendor, desc.idProduct, libusb_get_bus_number(dev), libusb_get_device_address(dev));
        
        for (x = 0; x  < sizeof(acr122_usb_supported_devices) / sizeof(struct acr122_usb_supported_device); x++) {
            if ((acr122_usb_supported_devices[x].vendor_id == desc.idVendor) &&
                (acr122_usb_supported_devices[x].product_id == desc.idProduct)) {
                //printf("<%s>\n", acr122_usb_supported_devices[x].name);
                
                *vendor_id = desc.idVendor;
                *product_id = desc.idProduct;
                
                // Make sure there are 3 endpoints available
                r = libusb_get_config_descriptor(dev, 0, &conf_desc);
                if (r < 0) {
                    log_put(LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_ERROR, "failed to get config descriptor.");
                    return deviceFound;
                }
                if (conf_desc->bNumInterfaces < 1) {
                    log_put(LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_ERROR, "number of interfaces < 1.");
                }
                
                // interogate all endpoints
                for (k=0; k<conf_desc->interface[0].altsetting[0].bNumEndpoints; k++) {
                    struct libusb_ss_endpoint_companion_descriptor *ep_comp = NULL;
                    endpoint = &conf_desc->interface[0].altsetting[0].endpoint[k];
                    //printf("       endpoint[%d].address: %02X ", k, endpoint->bEndpointAddress);
                    
                    // get the end point attributes
                    ep_attributes = endpoint->bmAttributes & LIBUSB_TRANSFER_TYPE_MASK;
                    if (ep_attributes & (LIBUSB_TRANSFER_TYPE_BULK | LIBUSB_TRANSFER_TYPE_INTERRUPT)) {
                        if (endpoint->bEndpointAddress & LIBUSB_ENDPOINT_IN) {
                            if ( ep_attributes == LIBUSB_TRANSFER_TYPE_INTERRUPT ) {
                                // INT IN
                                endpoint_int_in = endpoint->bEndpointAddress;
                            }
                            if ( ep_attributes == LIBUSB_TRANSFER_TYPE_BULK ) {
                                // BULK IN
                                endpoint_bulk_in = endpoint->bEndpointAddress;
                            }
                        } else {
                            // BULK OUT
                            endpoint_bulk_out = endpoint->bEndpointAddress;
                        }
                    }
                }
                libusb_free_config_descriptor(conf_desc);
            }
        }
    }
    // if we have located all endpoints the device has been found
    if (endpoint_bulk_in && endpoint_bulk_out && endpoint_int_in) {
        deviceFound = true;
    }
    return deviceFound;
}

struct acr122_usb_descriptor {
  char *dirname;
  char *filename;
};

const char* ACR122_USB::getDeviceName(void)
{
  return deviceName;
}

const char* ACR122_USB::getUID(void) {
    return targetUID;
}

//static int acr122_build_frame_from_apdu(nfc_device *pnd, const uint8_t ins, const uint8_t p1, const uint8_t p2, const uint8_t *data, const size_t data_len, const uint8_t le)
int ACR122_USB::build_frame_from_apdu(const uint8_t ins, const uint8_t p1, const uint8_t p2, const uint8_t *data, const size_t data_len, const uint8_t le)
{
    if (data_len > sizeof(apdu_frame.apdu_payload))
        return NFC_EINVARG;
    if ((data == NULL) && (data_len != 0))
        return NFC_EINVARG;

    apdu_frame.ccid_header.dwLength = htole32(data_len + sizeof(struct apdu_header));
    apdu_frame.apdu_header.bIns = ins;
    apdu_frame.apdu_header.bP1 = p1;
    apdu_frame.apdu_header.bP2 = p2;
    if (data) {
        // bLen is Lc when data != NULL
        apdu_frame.apdu_header.bLen = data_len;
        memcpy(apdu_frame.apdu_payload, data, data_len);
    } else {
        // bLen is Le when no data.
        apdu_frame.apdu_header.bLen = le;
    }
    return (sizeof(struct ccid_header) + sizeof(struct apdu_header) + data_len);
}

//static int acr122_build_frame_from_tama(nfc_device *pnd, const uint8_t *tama, const size_t tama_len)
int ACR122_USB::build_frame_from_tama(const uint8_t *tama, const size_t tama_len)
{
    if (tama_len > sizeof(tama_frame.tama_payload))
        return NFC_EINVARG;

    tama_frame.ccid_header.dwLength = htole32(tama_len + sizeof(struct apdu_header) + 1);
    tama_frame.apdu_header.bLen = tama_len + 1;
    memcpy(tama_frame.tama_payload, tama, tama_len);
    return (sizeof(struct ccid_header) + sizeof(struct apdu_header) + 1 + tama_len);
}

//static int acr122_usb_send(nfc_device *pnd, const uint8_t *pbtData, const size_t szData, const int timeout)
int ACR122_USB::send(const uint8_t *pbtData, const size_t szData, const int timeout)
{
    int res;
    if ((res = build_frame_from_tama(pbtData, szData)) < 0) {
        last_error = NFC_EINVARG;
        return last_error;
    }

    if ((res = bulk_write((uint8_t*)&tama_frame, res, timeout)) < 0) {
        last_error = res;
        return last_error;
    }
    return NFC_SUCCESS;
}

/*
 * returns number of bytes in received reply of LIBUSB_ERROR_xxxxx in case of failure
 */
int ACR122_USB::send_apdu(const uint8_t ins, const uint8_t p1, const uint8_t p2, const uint8_t *const data, size_t data_len, const uint8_t le, uint8_t *out, const size_t out_size)
{
    int res = 0;
    size_t frame_len = build_frame_from_apdu(ins, p1, p2, data, data_len, le);
    if ( (res = bulk_write( (unsigned char *) &apdu_frame, frame_len, USB_WRITE_TIMEOUT) ) < LIBUSB_SUCCESS) {
        log_put (LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_ERROR, "%s - write failed %d", __PRETTY_FUNCTION__, res);
        return res;
    }
    if ( (res = bulk_read(out, out_size, USB_READ_TIMEOUT) ) < LIBUSB_SUCCESS) {
        log_put (LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_ERROR, "%s - read failed %d", __PRETTY_FUNCTION__, res);
    }
    return res;
}

//static int acr122_usb_receive(nfc_device *pnd, uint8_t *pbtData, const size_t szDataLen, const int timeout)
int ACR122_USB::receive(uint8_t *pbtData, const size_t szDataLen, const int timeout)
{
    off_t offset = 0;
    
    uint8_t  abtRxBuf[255 + sizeof(struct ccid_header)];
    int res = 0;
    
    /*
     * If no timeout is specified but the command is blocking, force a 200ms (USB_TIMEOUT_PER_PASS)
     * timeout to allow breaking the loop if the user wants to stop it.
     */
    int usb_timeout;
    int remaining_time = timeout;
read:
    if (timeout == USB_INFINITE_TIMEOUT) {
        usb_timeout = USB_TIMEOUT_PER_PASS;
    } else {
        // A user-provided timeout is set, we have to cut it in multiple chunk to be able to keep an nfc_abort_command() mecanism
        remaining_time -= USB_TIMEOUT_PER_PASS;
        if (remaining_time <= 0) {
            last_error = NFC_ETIMEOUT;
            last_error;
        } else {
            usb_timeout = MIN(remaining_time, USB_TIMEOUT_PER_PASS);
        }
    }

    res = bulk_read(abtRxBuf, sizeof(abtRxBuf), usb_timeout);

    uint8_t attempted_response = RDR_PC_DATA_BLOCK;
    size_t len = 0;

    if (res < 0) {
        if (abort_flag) {
            abort_flag = false;
            ack();
            last_error = NFC_EOPABORTED;
            last_error;
        } else {
            goto read;
        }
    }
    if (res < 12) {
        log_put(LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_ERROR, "%s - %s", __PRETTY_FUNCTION__, "Invalid RDR_to_PC_DataBlock frame");
        // try to interrupt current device state
        ack();
        last_error = NFC_EIO;
        return last_error;
    }
    if (abtRxBuf[offset] != attempted_response) {
        log_put(LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_ERROR, "%s", __PRETTY_FUNCTION__, "Frame header mismatch");
        last_error = NFC_EIO;
        return last_error;
    }
    offset++;

    len = abtRxBuf[offset++];
    if (!((len > 1) && (abtRxBuf[10] == 0xd5))) { // In case we didn't get an immediate answer:
        if (len != 2) {
            log_put(LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_ERROR, "%s - %s", __PRETTY_FUNCTION__, "Wrong reply");
            last_error = NFC_EIO;
            return last_error;
        }
        if (abtRxBuf[10] != SW1_More_Data_Available) {
            if ((abtRxBuf[10] == SW1_Warning_with_NV_changed) && (abtRxBuf[11] == PN53x_Specific_Application_Level_Error_Code)) {
                log_put(LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_ERROR, "%s - %s", __PRETTY_FUNCTION__, "PN532 has detected an error at the application level");
            } else if ((abtRxBuf[10] == SW1_Warning_with_NV_changed) && (abtRxBuf[11] == 0x00)) {
                log_put(LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_ERROR, "%s - %s", __PRETTY_FUNCTION__, "PN532 didn't reply");
            } else {
                log_put(LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_ERROR, "%s - Unexpected Status Word (SW1: %02x SW2: %02x)", __PRETTY_FUNCTION__, abtRxBuf[10], abtRxBuf[11]);
            }
            last_error = NFC_EIO;
            return last_error;
        }
        res = send_apdu(APDU_GetAdditionnalData, 0x00, 0x00, NULL, 0, abtRxBuf[11], abtRxBuf, sizeof(abtRxBuf));
        if (res == NFC_ETIMEOUT) {
            if (abort_flag) {
                abort_flag = false;
                ack();
                last_error = NFC_EOPABORTED;
                return last_error;
            } else {
                goto read; // FIXME May cause some trouble on Touchatag, right ?
            }
        }
        if (res < 12) {
            // try to interrupt current device state
            ack();
            last_error = NFC_EIO;
            return last_error;
        }
    }
    offset = 0;
    if (abtRxBuf[offset] != attempted_response) {
        log_put(LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_ERROR, "%s - %s", __PRETTY_FUNCTION__, "Frame header mismatch");
        last_error = NFC_EIO;
        return last_error;
    }
    offset++;

    // XXX In CCID specification, len is a 32-bits (dword), do we need to decode more than 1 byte ? (0-255 bytes for PN532 reply)
    len = abtRxBuf[offset++];
    if ((abtRxBuf[offset] != 0x00) && (abtRxBuf[offset + 1] != 0x00) && (abtRxBuf[offset + 2] != 0x00)) {
        log_put(LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_ERROR, "%s - %s", __PRETTY_FUNCTION__, "Not implemented: only 1-byte length is supported, please report this bug with a full trace.");
        last_error = NFC_EIO;
        return last_error;
    }
    offset += 3;

    if (len < 4) {
        log_put(LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_ERROR, "%s - %s", __PRETTY_FUNCTION__, "Too small reply");
        last_error = NFC_EIO;
        return last_error;
    }
    len -= 4; // We skip 2 bytes for PN532 direction byte (D5) and command byte (CMD+1), then 2 bytes for APDU status (90 00).

    if (len > szDataLen) {
        log_put(LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_ERROR, "%s - Unable to receive data: buffer too small. (szDataLen: %" PRIuPTR ", len: %" PRIuPTR ")", __PRETTY_FUNCTION__, szDataLen, len);
        last_error = NFC_EOVFLOW;
        return last_error;
    }

    // Skip CCID remaining bytes
    offset += 2; // bSlot and bSeq are not used
    offset += 2; // XXX bStatus and bError should maybe checked ?
    offset += 1; // bRFU should be 0x00

    // TFI + PD0 (CC+1)
    if (abtRxBuf[offset] != 0xD5) {
        log_put(LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_ERROR, "%s - %s", __PRETTY_FUNCTION__, "TFI Mismatch");
        last_error = NFC_EIO;
        return last_error;
    }
    offset += 1;

//    if (abtRxBuf[offset] != CHIP_DATA(pnd)->last_command + 1) {
//      log_put(LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_ERROR, "%s", "Command Code verification failed");
//      pnd->last_error = NFC_EIO;
//      return pnd->last_error;
//  }
    offset += 1;
    memcpy(pbtData, abtRxBuf + offset, len);
    return len;
}

//bool ACR122_USB::buzzerLED() {
//    int res = 0, x , uidLength;
//    uint8_t  abtRxBuf[255 + sizeof(struct ccid_header)];
//    bool intRestartRequired = false;
//    struct timeval t;
//    t.tv_sec = 0; t.tv_usec = 0;
//    //uint32_t *uidValue;
//    
//    // See ACR122 manual: "Bi-Color LED and Buzzer Control" section
//    uint8_t acr122u_led_duration[] = {
//        0x05, 0x05, 0x04, 0x00, // Blinking Buzzer control
//    };
//    
//    log_put(LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_DEBUG, "%s", __PRETTY_FUNCTION__ );
//    
//    // cancel interrupt
//    if (ACR122_USB::irqIsEnabled) {
//        intRestartRequired = true;
//        log_put(LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_DEBUG, "%s - Cancel Interrupt Transfer", __PRETTY_FUNCTION__ );
//        libusb_cancel_transfer(irqTransfer);
//    }
//    
//    // wait for interrupt cancellation to be confirmed
//    while (ACR122_USB::irqIsEnabled) {
//        libusb_handle_events_timeout (usbContext, &t);  // zero timeval = return immediately (non blocking)
//        usleep(100000);
//    }
//
//    //LED State Control: 10101100 - green LED blink and then left ON
////    log_put (LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_DEBUG, "%s - %s", __PRETTY_FUNCTION__, "LED and Buzzer Control");
////    if ((res = send_apdu(0x00, 0x40, 0xAC, (uint8_t *)acr122u_led_duration , sizeof(acr122u_led_duration), 0x04, abtRxBuf, sizeof(abtRxBuf))) < LIBUSB_SUCCESS) {
////        log_put (LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_ERROR, "%s - %s", __PRETTY_FUNCTION__, "ACR122 LED Control failed");
////    }
//
//    // restart interrupt;
//    if (intRestartRequired && !ACR122_USB::irqIsEnabled) {
//        log_put(LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_DEBUG, "%s - Start Interrupt Transfer", __PRETTY_FUNCTION__ );
//        res = libusb_submit_transfer(irqTransfer);
//        if (res < LIBUSB_SUCCESS) {
//            log_put(LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_ERROR, "%s - libusb_submit_transfer irqTransfer failed %d ", __PRETTY_FUNCTION__, res);
//        }
//    }
//    
//    return true;
//}

/*
 * reads the UID of the target into memory as ASCII representation of the hex number.
 @ returns true if the UUID was successfully retrieved and stored
 */
bool ACR122_USB::readUID () {
    int res = 0, x , uidLength;
    uint8_t  abtRxBuf[255 + sizeof(struct ccid_header)];
    bool intRestartRequired = false;
    struct timeval t;
    t.tv_sec = 0; t.tv_usec = 0;
    bool retval = false;
    
    // cancel interrupt
    if (ACR122_USB::irqIsEnabled) {
        intRestartRequired = true;
        log_put(LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_DEBUG, "%s - Cancel Interrupt Transfer", __PRETTY_FUNCTION__ );
        libusb_cancel_transfer(irqTransfer);
    }
    
    // wait for interrupt cancellation to be confirmed
    while (ACR122_USB::irqIsEnabled) {
        libusb_handle_events_timeout (usbContext, &t);  // zero timeval = return immediately (non blocking)
        usleep(100000);
    }

    targetUID[0] = 0;   // empty UID string
    
    // get UID via PICC command "Get Data"
    if ((res = send_apdu(0xCA, 0x00, 0x00, NULL, 0, 0, abtRxBuf, sizeof(abtRxBuf))) < LIBUSB_SUCCESS) {
        log_put (LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_ERROR, "%s - getUID error", __PRETTY_FUNCTION__);
        targetUID[0] = 0;
    } else { // check response code whcih is located at the end of the received data
        if ((abtRxBuf[res-2] != 0x90) || (abtRxBuf[res-1] != 0x00) ) {  // 0x90 0x00 = Operation completed successfully
            log_put (LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_ERROR, "%s - Get Data error SW1=%02x SW2=%02x", __PRETTY_FUNCTION__, abtRxBuf[res-2],abtRxBuf[res-1]);
            goto fail;
        }
        // retrieve the UID which is stored between offset 10 and the response code (2 bytes) at the end of the buffer
        uidLength = abtRxBuf[1] -2; // deduct the response code length
        if (uidLength > UID_LENGTH_MAX) {
            log_put (LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_ERROR, "%s - Max UID length exceeded (%d).", __PRETTY_FUNCTION__, uidLength);
            goto fail;
        }
        // print each byte to the UID string
        for (x = 0; x < uidLength; x++) {
            snprintf(&targetUID[x*2], UID_LENGTH_MAX - (x*2), "%02x", abtRxBuf[10 + x]);
        }
        targetUID[x*2] = 0; // set string end
        retval = true;
    }

fail:
    // restart interrupt
    // NOTE: until the card is removed from the reader there will be no response via the callback function
    if (intRestartRequired && !ACR122_USB::irqIsEnabled) {
        res = libusb_submit_transfer(irqTransfer);
        if (res < LIBUSB_SUCCESS) {
            log_put(LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_ERROR, "%s - libusb_submit_transfer irqTransfer failed %d ", __PRETTY_FUNCTION__, res);
        }
    }
    return retval;
}

/*
 * returns true if a target has been detected
 */
bool ACR122_USB::process() {
    struct timeval t;
    bool retval = false;
    // zero timeval = return immediately (non blocking)
    t.tv_sec = 0; t.tv_usec = 0;
    libusb_handle_events_timeout (usbContext, &t);
    
    if (ACR122_USB::targetPresent && !targetDetectedOneShot) {
        //log_put(LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_ERROR, "%s - calling getUID() ", __PRETTY_FUNCTION__);
        retval = readUID();     // true if successful
    }
    targetDetectedOneShot = ACR122_USB::targetPresent;
    
    // wait until target is removed from reader
    while (ACR122_USB::targetPresent) {
        libusb_handle_events_timeout (usbContext, &t);  // zero timeval = return immediately (non blocking)
        usleep(100000);
    }

    return retval;
}

//int acr122_usb_ack(nfc_device *pnd)
int ACR122_USB::ack()
{
//  (void) pnd;
    int res = 0;
    uint8_t acr122_ack_frame[] = { GetFirmwareVersion }; // We can't send a PN532's ACK frame, so we use a normal command to cancel current command
    log_put(LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_DEBUG, "%s - %s", __PRETTY_FUNCTION__, "ACK");
    if ((res = build_frame_from_tama(acr122_ack_frame, sizeof(acr122_ack_frame))) < 0) {
        log_put (LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_ERROR, "%s - build_frame_from_tama failed %d", __PRETTY_FUNCTION__, res);
        return res;
    }
    if ( (res = bulk_write( (uint8_t*) &tama_frame, res, 1000) ) < 0) {
        log_put (LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_ERROR, "%s - write failed %d", __PRETTY_FUNCTION__, res);
        return res;
    }
    
    uint8_t  abtRxBuf[255 + sizeof(struct ccid_header)];
    res = bulk_read (abtRxBuf, sizeof(abtRxBuf), 1000);
    return res;
}

bool ACR122_USB::openDevice(uint16_t vendor_id, uint16_t product_id)
{
    libusb_device **devs;
    libusb_device *dev;
    struct libusb_config_descriptor *conf_desc;
    ssize_t deviceCount, d, result ;
    unsigned int k, x ;
    
    deviceCount = libusb_get_device_list (usbContext, &devs);
    
    // find device in list
    d = 0;
    while ((dev = devs[d++]) != NULL) {
		struct libusb_device_descriptor desc;
		result = libusb_get_device_descriptor(dev, &desc);
		if (result < 0) {
			log_put(LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_ERROR, "%s - failed to get device descriptor", __PRETTY_FUNCTION__);
			return false;
		}
        if ((vendor_id == desc.idVendor) && (product_id == desc.idProduct)) {
            break;
        }
    }
    
    if (dev == NULL) {
        log_put(LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_ERROR, "%s - failed to find device", __PRETTY_FUNCTION__);
        goto fail;
    }
    
    // open device
    result = libusb_open(dev, &devHandle);
    if (!devHandle) {
        log_put(LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_ERROR, "%s - failed to open device ...", __PRETTY_FUNCTION__);
        switch (result) {
            case LIBUSB_ERROR_ACCESS:
                log_put(LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_ERROR, ".. Insufficient access");
                break;
            case LIBUSB_ERROR_NO_DEVICE:
                log_put(LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_ERROR, ".. Device has been disconnected");
                break;
            case LIBUSB_ERROR_NO_MEM:
                log_put(LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_ERROR, ".. Memory Allocation failure");
                break;
            default:
                log_put(LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_ERROR, ".. %d", result);
                break;
        }
        goto fail;
    }

    // Code below Does not detect kernel driver
//    // check if the USB interface is used by a kernel driver
//    result = libusb_kernel_driver_active(devHandle, 0); // returns 0 if no kernel driver
//    if (result < 0)
//    {
//        log_put(LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_ERROR, "%s - Error %i checking kernel driver.", __PRETTY_FUNCTION__, result);
//        goto fail;
//    }
//    if (result > 1)
//    {
//        log_put(LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_DEBUG, "%s - Detaching Kernel driver from USB interface.", __PRETTY_FUNCTION__, result);
//        result = libusb_detach_kernel_driver(devHandle, 0);
//        if (result < 0)
//        {
//            log_put(LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_ERROR, "Error %i detaching kernel driver.", __PRETTY_FUNCTION__, result);
//            goto fail;
//        }
//        restoreKernelDriver = true;
//    }
    
    // claim interface
    result = libusb_claim_interface(devHandle, 0);
	if (result < 0) {
		log_put(LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_ERROR, "%s - usb_claim_interface error ...", __PRETTY_FUNCTION__);
        switch (result) {
            case LIBUSB_ERROR_NOT_FOUND:
                log_put(LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_ERROR, ".. interface does not exist");
                break;
            case LIBUSB_ERROR_BUSY:
                log_put(LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_ERROR, ".. interface not available");
                break;
            case LIBUSB_ERROR_NO_DEVICE:
                log_put(LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_ERROR, ".. Device has been disconnected");
                break;
            default:
                log_put(LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_ERROR, ".. %d", result);
                break;
        }
		goto fail1;
	}
    
    // allocate transfer
    irqTransfer = libusb_alloc_transfer(0);
	if (!irqTransfer)
		goto fail2;
    
    // Make sure there are 3 endpoints available
    result = libusb_get_config_descriptor(dev, 0, &conf_desc);
    if (result < 0) {
        log_put(LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_ERROR, "%s - failed to get config descriptor", __PRETTY_FUNCTION__);
        goto fail2;
    }
    
    if (conf_desc->bNumInterfaces < 1) {
        log_put(LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_ERROR, "%s - number of interfaces < 1", __PRETTY_FUNCTION__);
        goto fail3;
    }
    
    if (! setEndpoints(conf_desc) ) {
        log_put(LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_ERROR, "%s - setEndoints failed", __PRETTY_FUNCTION__);
        goto fail3;
    }
    
    irqTransfer = libusb_alloc_transfer(0);
    if (!irqTransfer) {
        log_put(LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_ERROR, "%s - libusb_alloc_transfer failed", __PRETTY_FUNCTION__);
        goto fail3;
    }
    
    libusb_free_config_descriptor(conf_desc);
    
    libusb_free_device_list(devs, 0);
    
    // set the device name
    for (x = 0; x  < sizeof(acr122_usb_supported_devices) / sizeof(struct acr122_usb_supported_device); x++) {
        if ((acr122_usb_supported_devices[x].vendor_id == vendor_id) && (acr122_usb_supported_devices[x].product_id == product_id)) {
            deviceName = new char[strlen(acr122_usb_supported_devices[x].name)];
            strcpy(deviceName, acr122_usb_supported_devices[x].name);
            //deviceName[sizeof(acr122_usb_supported_devices[x].name)] = 0;
        }
    }
    
    deviceIsOpen = true;
    abort_flag = false;
    
    memcpy(&tama_frame, acr122_usb_frame_template, sizeof(acr122_usb_frame_template));
    memcpy(&apdu_frame, acr122_usb_frame_template, sizeof(acr122_usb_frame_template));
    
    return true;
    
fail3:
    libusb_free_config_descriptor(conf_desc);
fail2:
    libusb_release_interface(devHandle, 0);
fail1:
    libusb_close(devHandle);
fail:
    libusb_free_device_list(devs, 0);
    return false;
}

bool ACR122_USB::init(bool buzzerEnable)
{
    int res = 0;
    int i;
    uint8_t buzzerCmd;
    uint8_t abtRxBuf[255 + sizeof(struct ccid_header)];
    
    log_put (LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_DEBUG, "%s - %s", __PRETTY_FUNCTION__, "Get ACR122 Firmware Version");
    if ((res = send_apdu(0x00, 0x48, 0x00, NULL, 0, 0, abtRxBuf, sizeof(abtRxBuf))) >= LIBUSB_SUCCESS) {
        abtRxBuf[res] = 0;
        log_put (LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_DEBUG, "%s - Firmware: %s", __PRETTY_FUNCTION__, abtRxBuf + 10);
    } // in case of failure send_apdu will generate a log message
    
    
    // Power On ICC
    uint8_t ccid_frame[] = {
        PC_RDR_ICC_ON, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00
    };
    log_put (LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_DEBUG, "%s - %s", __PRETTY_FUNCTION__, "Power On ICC");
    if ((res = bulk_write (ccid_frame, sizeof(struct ccid_header), 1000)) < LIBUSB_SUCCESS)
        goto fail;
    if ((res = bulk_read (abtRxBuf, sizeof(abtRxBuf), 1000)) < LIBUSB_SUCCESS)
        goto fail;

    // Set PICC parameters
    log_put(LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_DEBUG, "%s - %s", __PRETTY_FUNCTION__, "ACR122 PICC Operating Parameters");
    if ((res = send_apdu(0x00, 0x51, 0x83, NULL, 0, 0, abtRxBuf, sizeof(abtRxBuf))) < LIBUSB_SUCCESS)
        goto fail;

    if (buzzerEnable) {
        buzzerCmd = 0xFF;
    } else {
        buzzerCmd = 0x00;
    }
    // Set buzzer on/off for card detection
    log_put(LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_DEBUG, "%s - %s", __PRETTY_FUNCTION__, "Buzzer disable for card detection");
    if ((res = send_apdu(0x00, 0x52, buzzerCmd, NULL, 0, 0, abtRxBuf, sizeof(abtRxBuf))) < LIBUSB_SUCCESS)
        goto fail;
    
    // activate interrupt
    libusb_fill_interrupt_transfer(irqTransfer, devHandle, endpointInInt, irqbuf, sizeof(irqbuf), ACR122_USB::irqCallback, NULL, 0);

    log_put(LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_DEBUG, "%s - Start Interrupt Transfer", __PRETTY_FUNCTION__ );

    if ((res = libusb_submit_transfer(irqTransfer)) < LIBUSB_SUCCESS) {
        log_put(LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_ERROR, "%s - libusb_submit_transfer irqTransfer failed %d ", __PRETTY_FUNCTION__, res);
    	goto fail;
    }
//    res = 0;
//    for (i = 0; i < 3; i++) {
//        if (res < 0){
//            log_put(LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_ERROR, "%s - %s", __PRETTY_FUNCTION__, "PN532 init failed, trying again...");
//        }
//        if ( (res = pn53x_init(pnd) ) >= 0)
//            break;
//    }
//    if (res < 0)
//        return res;
    
    return true;
fail:
    return false;
}

//static void acr122_usb_close(nfc_device *pnd)
void ACR122_USB::close()
{
    int res;
    uint8_t  abtRxBuf[255 + sizeof(struct ccid_header)];
    
    // cancel interrupt
    if (irqTransfer) {
        log_put(LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_DEBUG, "%s - Cancel Interrupt Transfer", __PRETTY_FUNCTION__ );
        libusb_cancel_transfer(irqTransfer);
    }
    // wait for transfer to complete
    while (ACR122_USB::irqIsEnabled) {
        process();
        usleep(100000);
    };
    
    // PICC operating parameter
    log_put(LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_DEBUG, "%s - %s", __PRETTY_FUNCTION__, "ACR122 PICC Operating Parameters");
    if ( (res = send_apdu(0x00, 0x51, 0x00, NULL, 0, 0, abtRxBuf, sizeof(abtRxBuf) ) ) < LIBUSB_SUCCESS) {
        log_put(LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_DEBUG, "%s - send_apdu failed <%d>", __PRETTY_FUNCTION__, res );
    }
    
    // See ACR122 manual: "Bi-Color LED and Buzzer Control" section
    uint8_t acr122u_led_duration[] = {
        0x00, 0x00, 0x00, 0x00, // Blinking Buzzer control
    };
    
    log_put (LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_DEBUG, "%s - %s", __PRETTY_FUNCTION__, "LED and Buzzer Control");
    // LED State Control: 00001100 - green LED blink and then left ON
    send_apdu (0x00, 0x40, 0x0C, (uint8_t *)acr122u_led_duration , sizeof(acr122u_led_duration), 0x04, abtRxBuf, sizeof(abtRxBuf));
    
    //ack();
    //  pn53x_idle(pnd);
    //
    //  int res;
    //  if ((res = usb_release_interface(DRIVER_DATA(pnd)->pudh, 0)) < 0) {
    //    log_put(LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_ERROR, "Unable to release USB interface (%s)", _usb_strerror(res));
    //  }
    //
    //  if ((res = usb_close(DRIVER_DATA(pnd)->pudh)) < 0) {
    //    log_put(LOG_GROUP, LOG_CATEGORY, ACR_LOG_PRIORITY_ERROR, "Unable to close USB connection (%s)", _usb_strerror(res));
    //  }
    //  pn53x_data_free(pnd);
    //  nfc_device_free(pnd);
    
    // clean up interrupt
    if (irqTransfer) {
        libusb_free_transfer(irqTransfer);
		irqTransfer = NULL;
    }
    if (irqbufSize) {
        delete irqbuf;
        irqbufSize = 0;
    }

    if (devHandle) {
        libusb_release_interface(devHandle, 0);
        libusb_close (devHandle);
    }
}

//static int acr122_usb_abort_command(nfc_device *pnd)
int ACR122_USB::abortCommand()
{
    abort_flag = true;
    return NFC_SUCCESS;
}

//const struct pn53x_io acr122_usb_io = {
//  .send       = acr122_usb_send,
//  .receive    = acr122_usb_receive,
//};
//
//const struct nfc_driver acr122_usb_driver = {
//  .name                             = ACR122_USB_DRIVER_NAME,
//  .scan_type                        = NOT_INTRUSIVE,
//  .scan                             = acr122_usb_scan,
//  .open                             = acr122_usb_open,
//  .close                            = acr122_usb_close,
//  .strerror                         = pn53x_strerror,
//
//  .initiator_init                   = pn53x_initiator_init,
//  .initiator_init_secure_element    = NULL, // No secure-element support
//  .initiator_select_passive_target  = pn53x_initiator_select_passive_target,
//  .initiator_poll_target            = pn53x_initiator_poll_target,
//  .initiator_select_dep_target      = pn53x_initiator_select_dep_target,
//  .initiator_deselect_target        = pn53x_initiator_deselect_target,
//  .initiator_transceive_bytes       = pn53x_initiator_transceive_bytes,
//  .initiator_transceive_bits        = pn53x_initiator_transceive_bits,
//  .initiator_transceive_bytes_timed = pn53x_initiator_transceive_bytes_timed,
//  .initiator_transceive_bits_timed  = pn53x_initiator_transceive_bits_timed,
//  .initiator_target_is_present      = pn53x_initiator_target_is_present,
//
//  .target_init           = pn53x_target_init,
//  .target_send_bytes     = pn53x_target_send_bytes,
//  .target_receive_bytes  = pn53x_target_receive_bytes,
//  .target_send_bits      = pn53x_target_send_bits,
//  .target_receive_bits   = pn53x_target_receive_bits,
//
//  .device_set_property_bool     = pn53x_set_property_bool,
//  .device_set_property_int      = pn53x_set_property_int,
//  .get_supported_modulation     = pn53x_get_supported_modulation,
//  .get_supported_baud_rate      = pn53x_get_supported_baud_rate,
//  .device_get_information_about = pn53x_get_information_about,
//
//  .abort_command  = acr122_usb_abort_command,
//  .idle           = pn53x_idle,
//  /* Even if PN532, PowerDown is not recommended on those devices */
//  .powerdown      = NULL,
//};
