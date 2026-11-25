#pragma once

#include <Arduino.h>
#include <LittleFS.h>
#include <time.h>

#include <amis.h> // libamis: M-Bus, E5, AES-128, Datensaetze

// TODO(anyone): Refactor this global vars
extern uint8_t dow;
extern uint8_t mon, myyear;


#ifndef AMISREADER_SIMULATE_SERIAL
// If enabled fixed values are returned
// Use AmisKey: 1234567890abcdef1234567890abcdef
#define AMISREADER_SIMULATE_SERIAL      0
#endif


typedef struct {
    bool isSet;
    struct tm time;
    char timeCp48Hex[11];    // Wie vom Zäler bekommen (MBUS CP48 Format) als lesbarer Hexstring - aber nur 5 Bytes
    uint32_t results_u32[8]; // Zählerwerte
    int32_t  results_i32[1]; // Sonderfall Inkasso 1.128.0: Ist als int32 definiert
} AmisReaderNumResult_t;


enum AmisReadSerialnumberMode_t {
    disabled,       // Serialnummer nicht lesen. Nur Zählerdaten lesen
    tryRead,        // Versuche Serialnummer zu lesen, wenns nicht funktioniert (Timeout) dann nur Zählerdaten lesen
    mustRead,       // Serialnummer muss gelesen werden, erst dann werden Zählerdaten gelesen
};


// Maximale Länge der Seriennummer
// siehe AMIS TD-351x Benutzerhandbuch // 5.2.5 und 5.2.8.1. Anforderungstelegramm (ev gesamt 5.2)
// ( https://wiki.volkszaehler.org/_media/hardware/channels/meters/power/siemens/amis_td-351x_bhbk.pdf )
// Wird auch als Geräteadresse, Sachnummer oder Identifikation bezeichnet
#define AMISREADER_MAX_SERIALNUMER 32

class AmisReaderClass {
public:
    void init(uint8_t serialNo);
    void loop();
    void setKey(const char *key);
    void setKey(const uint8_t *key);
    const char *getSerialNumber();
    void enable();
    void disable();
    void end();
    // int getState() { return (int) ((_state == msDelay) ?_statePostDelay :_state);}
    String statusText();
    // void enableRestart(); // enables and restart fresh readings

    void readBillingDataset();

    void serialEnableDump(bool enable);
    void serialClearDumpFile();

    void fetchSerialData(size_t nbytes);

    size_t clearSerialRx();

private:
    enum AmisReaderState_t {
        initReadSerial,         // serielle: baudrate usw einstellen
        requestReaderSerial,    // anfrage
        waitForReaderSerial,
        decodeReaderSerial,

        initReadCounters,
        // requestReaderCounter, // there is nothing to do as the counter sends the data each second
        readReaderCounters,
        decodeReaderCounter,

        initReadBillingDataset,
        requestBillingDataset,
        requestSpecialCounterValue,
        waitForBillingDatasetStart,
        waitForBillingDatasetStartSingleValue,
        waitForBillingDatasetEnd,
        waitForBillingDatasetEndSingleValue,
        decodeBillingDataset,
        endBillingDataset,

        msDelay,

        //stateUndefined,
    };

    size_t serialWrite(const char *str);
    size_t serialWrite(uint8_t byte);
    size_t pollSerial(size_t nbytes);

    void eatSerialReadBuffer(size_t n);

    void processStateSerialnumber(const uint32_t msNow);
    void processStateCounters(const uint32_t msNow);
    void processStateReadBillingDataset(const uint32_t msNow);

    int convertReading(const amis::Reading &reading, AmisReaderNumResult_t &result);
    void onCounterResult(uint8_t status, const uint32_t msNow);
    void handleIecActions(uint8_t act);

#if (AMISREADER_SIMULATE_SERIAL)
    size_t pollSerialSimulateSerialInput();
#endif

    HardwareSerial *_serial = nullptr;

    char _baudRateIdentifier;
    char _serialNumber[AMISREADER_MAX_SERIALNUMER + 1]; // auch als "Geräteadresse" oder "Identifikation" bezeichnet

    AmisReaderState_t _state = initReadSerial;
    AmisReaderState_t _statePostDelay = initReadSerial;
    uint32_t _stateTimeoutMsPostDelay;
    //AmisReaderState_t _prevState = stateUndefined;
    uint32_t _stateLastSetMs;

    uint32_t _stateTimoutMs;
    uint32_t _stateErrorCnt;
    uint32_t _stateErrorMax;

    uint8_t _serialReadBuffer[128];
    size_t _serialReadBufferIdx=0;
    //unsigned long _lastSerialEvent;
    uint8_t _bytesInBufferExpectd;

    uint8_t _key[16];
    amis::Reader _mbus; // libamis Lesekern fuer die Zaehlerdaten (M-Bus)
    amis::IecReader _iec; // libamis IEC 62056-21: Seriennummer und Verrechnungsdatensatz
    bool _logSerialPending = false;

    AmisReadSerialnumberMode_t _readSerialNumberMode = tryRead;

    bool _isEnabled = false;
    bool _readerIsOnline = false;

    File _dumpSerialRxData;
    size_t _rxSerailRead=0;

    uint32_t _validFrameCnt=0; // Gültige Zählerdaten-Frames hintereinander

    uint32_t _lastTimeSync;

    File _billingDataset;
    size_t _billingDatasetRead=0;
    size_t _billingDatasetReadPrev=0;
    bool _billingDatasetComplete=false; // nur dann <ident>_vd.txt ersetzen
};

extern AmisReaderClass AmisReader;

/* vim:set ts=4 et: */
