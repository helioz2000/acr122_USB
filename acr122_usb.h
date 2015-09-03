/*-
  */

/**
 * @file acr122_usb.h
 * @brief
 */

#ifndef _ACR122_USB_H_
#define _ACR122_USB_H_

#include <libusb-1.0/libusb.h>

#define UID_LENGTH_MAX 20

#pragma pack(1)
struct ccid_header {
    uint8_t bMessageType;
    uint32_t dwLength;
    uint8_t bSlot;
    uint8_t bSeq;
    uint8_t bMessageSpecific[3];
};

struct apdu_header {
    uint8_t bClass;
    uint8_t bIns;
    uint8_t bP1;
    uint8_t bP2;
    uint8_t bLen;
};

struct tama_frame_struc {
    struct ccid_header ccid_header;
    struct apdu_header apdu_header;
    uint8_t tama_header;
    uint8_t tama_payload[254]; // According to ACR122U manual: Pseudo APDUs (Section 6.0), Lc is 1-byte long (Data In: 255-bytes).
};

struct apdu_frame_struc {
    struct ccid_header ccid_header;
    struct apdu_header apdu_header;
    uint8_t apdu_payload[255]; // APDU Lc is 1-byte long
};
#pragma pack()


class ACR122_USB {
public:
    ACR122_USB ();
    ~ACR122_USB ();
    bool usb_scan (uint16_t *vendor_id, uint16_t *product_id);
    bool openDevice(uint16_t vendor_id, uint16_t product_id);
    bool init(bool buzzerEnable);
    void close();
    bool process();
    const char* getDeviceName(void);
    const char* getUID(void);

private:
    libusb_context *usbContext;
    libusb_device_handle *devHandle;
    unsigned char endpointIn;
    unsigned char endpointInInt;
    unsigned char endpointOut;
    volatile int abort_flag;
    int last_error;
    char targetUID[UID_LENGTH_MAX];
    
    int endpointInMaxSize;
    int endpointInIntMaxSize;
    int endpointOutMaxSize;
    
    uint16_t vendor_id;
    uint16_t product_id;
    struct libusb_transfer *irqTransfer;
    unsigned char *irqbuf;
    unsigned int irqbufSize;
    bool deviceIsOpen;
    static bool irqIsEnabled;
    static bool targetPresent;
    bool restoreKernelDriver;
    char *deviceName;
    bool targetDetectedOneShot;
    
    struct apdu_frame_struc apdu_frame;
    struct tama_frame_struc tama_frame;
    
    static void LIBUSB_CALL irqCallback(struct libusb_transfer *transfer);
    int bulk_write(uint8_t abtTx[], const size_t szTx, const int timeout);
    int bulk_read(uint8_t abtRx[], const size_t szRx, const int timeout);
    int build_frame_from_apdu(const uint8_t ins, const uint8_t p1, const uint8_t p2, const uint8_t *data, const size_t data_len, const uint8_t le);
    int build_frame_from_tama(const uint8_t *tama, const size_t tama_len);
    bool setEndpoints (struct libusb_config_descriptor *conf_desc);
    int send_apdu(const uint8_t ins, const uint8_t p1, const uint8_t p2, const uint8_t *const data, size_t data_len, const uint8_t le, uint8_t *out, const size_t out_size);
    int send(const uint8_t *pbtData, const size_t szData, const int timeout);
    int receive(uint8_t *pbtData, const size_t szDataLen, const int timeout);
    
    int ack();
    int abortCommand();
    bool readUID ();
};

#endif // ! _ACR122_USB_H__
