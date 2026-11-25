#include "AmisReader.h"

#include "Databroker.h"
#include "Log.h"
#define LOGMODULE   LOGMODULE_AMISREADER
#include "MaxDemand.h"
#include "ModbusSmartmeterEmulation.h"
#include "RebootAtMidnight.h"
#include "RemoteOnOff.h"
#include "ShellySmartmeterEmulation.h"
#include "SystemMonitor.h"
#include "ThingSpeak.h"
#include "UA.h"
#include "Utils.h"


// TODO(anyone): Refactor this global vars and external function
extern bool new_data_for_websocket;
extern unsigned first_frame;
uint8_t dow;
uint8_t mon, myyear;


// Filename fürs Mitloggen der Seriellen
#define AMISREADER_SERIAL_RX_DUMPFILENAME   "/serialRxDump.bin"

// Filename während des Einlesens des Verrechungsdatensatzes
#define AMISREADER_BILLINGDATA_FILENAME     "/billingdata_tmp.txt"


// Ein paar Steuerzeichendefines, die immer wieder in der Doku auftauchen
#define SOH     "\x01"
#define SOH0x   0x01
#define STX     "\x02"
#define STX0x   0x02
#define ETX     "\x03"
#define ETX0x   0x03
#define EOT     "\x04"
#define ACK     "\x06"
#define CR      "\r"    // 0x0d
#define LF      "\n"    // 0x0a
#define CRLF    CR LF



// ---------------------------------------------------------------------------------------------------------------------------

static void setTime(time_t ts_now) {
    time_t tv_sec_old;
    tv_sec_old = time(NULL);
    if (ts_now == tv_sec_old) {
        LOG_DP("Time already in sync");
        return; // skip if we're already in sync!
    }
    // Transfer of enryptedMBUSTelegram_SND_UD (101 bytes) needs ~105ms at 9600 8N1
    struct timeval ti;
    ti.tv_sec = ts_now;
    ti.tv_usec = 105000;
    settimeofday(&ti, NULL);

    if (tv_sec_old - ts_now > 1 || tv_sec_old - ts_now < -1) {
        // only log if time diff is more than 1 sec
        LOGF_IP("Time sync (counter). (ts-old=%llu, ts-now=%llu, millis=%u)", tv_sec_old, ts_now, millis());
    }
    RebootAtMidnight.adjustTicker();
}

int AmisReaderClass::convertReading(const amis::Reading &reading, AmisReaderNumResult_t &result)
{
    // libamis hat den Frame schon geprueft (Pruefsumme, Kopf), entschluesselt und die Datensaetze gelesen.
    // Hier nur noch in das bisherige Format (Databroker) umwandeln.
    result.isSet = false;

    // Alle 10 Werte muessen im Frame sein: prueft libamis (set_required_fields(kAll), sonst kIncomplete)
    if ((reading.present & amis::Reading::kAll) != amis::Reading::kAll) {
        return -10;
    }

    // Die 6 Bytes Datum/Uhrzeit (M-Bus CP48) wie vom Zaehler gesendet
    const uint8_t *valueDT = reading.time.raw;

    AmisReaderNumResult_t numresult;
    if (!Utils::MbusCP48IToTm(numresult.time, valueDT)) {
        return -20;
    }
    // timeCp48Hex: 10 stelliges hexformat (= 5 bytes) wie bisher (fuer den Webclient)
    snprintf(numresult.timeCp48Hex, sizeof(numresult.timeCp48Hex),
             "%02x%02x%02x%02x%02x",
             valueDT[4], valueDT[3], valueDT[2], valueDT[1], valueDT[0]);

    numresult.results_u32[0] = reading.energy_import_wh;            // 1.8.0
    numresult.results_u32[1] = reading.energy_export_wh;            // 2.8.0
    numresult.results_u32[2] = reading.reactive_energy_import_varh; // 3.8.1
    numresult.results_u32[3] = reading.reactive_energy_export_varh; // 4.8.1
    numresult.results_u32[4] = reading.power_import_w;              // 1.7.0
    numresult.results_u32[5] = reading.power_export_w;              // 2.7.0
    numresult.results_u32[6] = reading.reactive_power_import_var;   // 3.7.0
    numresult.results_u32[7] = reading.reactive_power_export_var;   // 4.7.0
    numresult.results_i32[0] = reading.inkasso;                     // 1.128.0

    numresult.isSet = true;
    result = numresult;
    return 0;
}

const char *AmisReaderClass::getSerialNumber()
{
    if (_serialNumber[0]) {
        return _serialNumber;
    }
    return "unbekannt";
}

size_t AmisReaderClass::serialWrite(const char *str)
{
    size_t r;
    if (_serial) {
        r = _serial->write(str);
        _serial->flush();
    } else {
        r = strlen(str);
    }
    /* if (_dumpSerialRxData) {
        _dumpSerialRxData->printf("Write %d bytes\n", r);
    }*/
    return r;
}

size_t AmisReaderClass::serialWrite(uint8_t byte)
{
    size_t r;
    if (_serial) {
        r = _serial->write(byte);
        _serial->flush();
    } else {
        r = sizeof(byte);
    }
    /*if (_dumpSerialRxData) {
        _dumpSerialRxData->write("Write 1 byte\n");
    }*/
    return r;
}

void AmisReaderClass::serialEnableDump(bool enable) {
    if (enable && !_dumpSerialRxData) {
        _dumpSerialRxData = LittleFS.open(AMISREADER_SERIAL_RX_DUMPFILENAME, "a");
    } else if (!enable && _dumpSerialRxData) {
        _dumpSerialRxData.close();
    }
}

void AmisReaderClass::serialClearDumpFile() {
    bool wasEnabled = (_dumpSerialRxData) ?true :false;
    serialEnableDump(false);
    LittleFS.remove(AMISREADER_SERIAL_RX_DUMPFILENAME);
    serialEnableDump(wasEnabled);
}

#if (AMISREADER_SIMULATE_SERIAL)
size_t AmisReaderClass::pollSerialSimulateSerialInput()
{
    static uint32_t lastSent = 0;

    uint32_t now = millis();
    if (now - lastSent < 1000) {
        return 0;
    }

    if (!_serialReadBufferIdx) {
        if (_state == waitForReaderSerial) {
            strlcpy((char*)_serialReadBuffer, "/SAT63511D-SiMuLaToR-\r\n", sizeof(_serialReadBuffer));
            _serialReadBufferIdx = strlen((const char*)_serialReadBuffer);
            lastSent = now;
        } else if (_state == readReaderCounters) {
            uint8_t demodata[106] = { // Demodaten für den Key '1234567890abcdef1234567890abcdef' (2025/12/18 18:34:41)
                0x10, 0x40, 0xF0, 0x30, 0x16,
                0x68, 0x5f, 0x5f, 0x68, 0x53, 0xf0, 0x5b, 0x00, 0x00, 0x00, 0x00, 0x2d, 0x4c, 0x01, 0x0e, 0x00,
                0x50, 0x05, 0x00, 0xf3, 0xd2, 0xd5, 0xaf, 0x0d, 0xf9, 0xe4, 0x3e, 0xcd, 0x57, 0xc5, 0x1e, 0xf1,
                0x57, 0x93, 0x62, 0x5c, 0x98, 0x7f, 0x71, 0x63, 0xc9, 0x5a, 0x4a, 0xec, 0xf4, 0xfc, 0x1d, 0xf1,
                0x93, 0x68, 0x27, 0xd8, 0xf0, 0x01, 0x42, 0x84, 0x15, 0xa2, 0x98, 0xda, 0x6d, 0xe5, 0x27, 0x9f,
                0xa8, 0x2b, 0x7c, 0x84, 0xa6, 0x05, 0xd2, 0x5d, 0x92, 0x6e, 0xf3, 0x93, 0x29, 0x9a, 0x67, 0x4b,
                0x16, 0xd2, 0x6f, 0x68, 0x8d, 0x0b, 0x33, 0xbe, 0x5f, 0xc4, 0xc5, 0x1c, 0x79, 0x2a, 0x6a, 0x7b,
                0x5a, 0x29, 0x1e, 0xdd, 0x16
            };
            memcpy(_serialReadBuffer, demodata, sizeof(demodata));
            _serialReadBufferIdx = sizeof(demodata);
            lastSent = now;
        }
    }

    return _serialReadBufferIdx;
}
#endif // (AMISREADER_SIMULATE_SERIAL)


void AmisReaderClass::fetchSerialData(size_t nbytes)
{
    if (_isEnabled) {
        // read some bytes from the serial interface preventing overflow errors
        _rxSerailRead += pollSerial(nbytes);
    }
}

size_t AmisReaderClass::pollSerial(size_t nbytes)
{
#if (AMISREADER_SIMULATE_SERIAL)
    return pollSerialSimulateSerialInput();
#else
    if (_serial == nullptr) {
        return 0;
    }

    size_t bytesReadTotal = 0;
    size_t serialBufferLeft = sizeof(_serialReadBuffer) - _serialReadBufferIdx;
    size_t serialAvail = _serial->available();
    size_t bytesRead;
    size_t nBytesLeft = nbytes;
    size_t serialReadBufferIdxStarted = _serialReadBufferIdx;
    // Fehlerflags hier bewusst ungeprueft loeschen: Der ESP8266-Core (uart.cpp) setzt rx_error nicht nur bei
    // Paritaets- und Framing-Fehlern, sondern auch beim RX-Timeout-Interrupt (UITO), also nach fast jedem
    // empfangenen Block. Ausgewertet verwirft das gueltige Daten (die IEC-Identifikation kam am echten Zaehler
    // minutenlang nicht durch). Paritaetsfehler lassen sich damit nicht erkennen.
    bool rxError = false & _serial->hasRxError(); // INFO: This call clears the pending error flag
    bool overrun = false & _serial->hasOverrun();

    while (serialAvail > 0 && serialBufferLeft > 0 && nBytesLeft > 0 && !rxError && !overrun) {
        bytesRead = _serial->readBytes(&_serialReadBuffer[_serialReadBufferIdx], std::min(nBytesLeft, std::min(serialAvail, serialBufferLeft)));
        bytesReadTotal += bytesRead;
        nBytesLeft -= bytesRead;
        serialBufferLeft -= bytesRead;
        _serialReadBufferIdx += bytesRead;
        serialAvail = _serial->available();
        rxError = _serial->hasRxError();
        overrun = _serial->hasOverrun();
    }

    if ((serialBufferLeft == 0 && serialAvail > 0 && nBytesLeft > 0) || rxError || overrun) {
        // No space in buffer left and still something available or a rx error occured
        // Eat all from the searial port and discard everything as we had an overflow
        // ERROR: Serial input overflowed!

        if (overrun) {
            LOG_EP("Serial overrun error");
        }
        if (rxError) {
            LOG_EP("Serial rx error");
        }
        if (serialBufferLeft == 0 && serialAvail > 0 && nBytesLeft > 0) {
            LOG_EP("Serial read buffer overflow");
        }
        _serialReadBufferIdx = 0;
        bytesReadTotal += clearSerialRx();
        _mbus.reset(); // angefangenen M-Bus-Frame verwerfen, Bytes fehlen
    }

    if (_dumpSerialRxData) {
        if (_serialReadBufferIdx > serialReadBufferIdxStarted) {
            _dumpSerialRxData.write(&_serialReadBuffer[serialReadBufferIdxStarted], _serialReadBufferIdx - serialReadBufferIdxStarted);
            _dumpSerialRxData.flush();
        }
    }

    return bytesReadTotal;
#endif // AMISREADER_SIMULATE_SERIAL
}

size_t AmisReaderClass::clearSerialRx()
{
    /* Alles aus dem internen Puffer der Seriellen Schnittstelle auslesen und die Anzahl der Bytes zurückgeben */
    size_t bytesRead = 0;
    if (_serial == nullptr) {
        return 0;
    }

    size_t avail = _serial->available();
    while (avail > 0) {
        char b[128];
        bytesRead += _serial->readBytes(&b[0], std::min(avail, (size_t)sizeof(b)));
        avail = _serial->available();
    }
    // Clear the pending error flags
    _serial->hasRxError();
    _serial->hasOverrun();
    return bytesRead;
}


void AmisReaderClass::init(uint8_t serialNo)
{
    _baudRateIdentifier = 0; _serialNumber[0] = 0;
    _serialReadBufferIdx = 0;

    memset(&Databroker, 0, sizeof(Databroker));

    if (serialNo == 1) {
        _serial = &Serial;
    } else if (serialNo == 2) {
        _serial = &Serial1;
    } else {
        _serial = nullptr;
    }

    clearSerialRx();
}

void AmisReaderClass::end()
{
    disable();
    if (_serial != nullptr) {
        _serial->end();
        _serial = nullptr;
    }

    Databroker.valid = 0;
    new_data_for_websocket = false;

    _readerIsOnline = false;

    ModbusSmartmeterEmulation.setCurrentValues(false);
    ThingSpeak.onNewData(false);
}

static void sendBCC(const char *cmd, HardwareSerial *s) {
    const uint8_t *v = (const uint8_t *) cmd;

    if (s == nullptr || !v[0]) {
        return;
    }
    s->write(cmd);

    /*
    char msg[50]; strcpy(msg, "Send:");
    for (size_t i=0; i< strlen(cmd); i++) {
        sprintf(msg + strlen(msg), " 0x%02x", cmd[i]);
    }
    */

    size_t bcc = Utils::calculateChecksumISO1155(cmd, true);
    if (bcc < 0x80) {
        s->write((uint8_t)bcc);
        //sprintf(msg + strlen(msg), " bcc=0x%02x", bcc);
    }
    s->flush();

    //LOGF_I(msg);
}

void AmisReaderClass::readBillingDataset() {
    if (_billingDataset) {
        return;
    }

    _billingDatasetComplete = false;
    _billingDataset = LittleFS.open(AMISREADER_BILLINGDATA_FILENAME, "w");
    if (!_billingDataset) {
        LOG_EP("Error opening billing dataset file");
        return;
    }

    LOGF_IP("Stopping counter read. %u data frames received.", _validFrameCnt);
    _state = initReadSerial;
    _readerIsOnline = false;
    _stateLastSetMs = millis();
    _stateTimoutMs = 5000;
    _stateErrorCnt = 0;

    _validFrameCnt = 0;
    first_frame = 0;
    Databroker.valid = 0;
    ModbusSmartmeterEmulation.setCurrentValues(false);
    ThingSpeak.onNewData(false);

    _bytesInBufferExpectd = 0;
    _serialReadBufferIdx = 0;
    clearSerialRx();
    LOG_IP("Start reading billing dataset (Verrechnungsdatensatz). Needs about 5-6 minutes!");
}

void AmisReaderClass::handleIecActions(uint8_t act)
{
    // Was libamis (IecReader) verlangt: senden, dann ggf. Baudrate umstellen (7E1)
    if ((act & amis::IecReader::kSend) && _serial != nullptr) {
        size_t n = 0;
        const uint8_t *d = _iec.tx(&n);
        _serial->write(d, n);
        _serial->flush();
    }
    if ((act & amis::IecReader::kSetSerial) && _serial != nullptr) {
        _serial->begin(_iec.baud(), SERIAL_7E1);
        _serial->clearWriteError();
    }
}

void AmisReaderClass::processStateReadBillingDataset(const uint32_t msNow)
{
    // Das Lesen selbst macht libamis (IecReader, Mode kReadout) in processStateSerialnumber().
    // Hier nur noch der Abschluss.
    if (_state == endBillingDataset) {
        char filename[33];

        // Senden wir noch gleich ein "Break" (siehe 5.2.8.12. Break B0) - das sollte das aktuelle Protokoll beenden
        // SOH B0 ETX BCC // BCC müsste hier 113 (0x71) sein
        _iec.abort();
        sendBCC(SOH "B0" ETX, _serial);

        if (_billingDataset) {
            _billingDataset.close();
            if (!_serialNumber[0] || _billingDatasetRead == 0) {
                LOG_WP("End read billing dataset: No data read!");
            } else if (_billingDatasetComplete) {
                snprintf_P(filename, sizeof(filename), "%s_vd.txt", _serialNumber); // _vd = VerrechnungsDatensatz
                LittleFS.remove(filename);
                LittleFS.rename(AMISREADER_BILLINGDATA_FILENAME, filename);
                LOGF_IP("End read billing dataset: %" PRIu32 " bytes read, saved as '%s'.", _billingDatasetRead, filename);
            } else {
                // Abgebrochen: den letzten vollstaendigen Datensatz (<ident>_vd.txt) nicht ueberschreiben,
                // den Rest als <ident>_vd_partial.txt behalten
                snprintf_P(filename, sizeof(filename), "%s_vd_partial.txt", _serialNumber);
                LittleFS.remove(filename);
                if (!LittleFS.rename(AMISREADER_BILLINGDATA_FILENAME, filename)) {
                    filename[0] = 0; // z. B. Name zu lang fuer LittleFS: Rest verwerfen
                }
                LOGF_WP("Read billing dataset incomplete (%s %s): %" PRIu32 " bytes saved as '%s'.",
                        _iec.error() != amis::IecReader::Error::kNone ? amis::error_str(_iec.error()) : "state timeout",
                        _iec.meter_error(), _billingDatasetRead, filename);
            }
        } else {
            LOG_WP("End read billing dataset: Internal error!");
        }
        LittleFS.remove(AMISREADER_BILLINGDATA_FILENAME);

        _state = initReadCounters;
        _stateLastSetMs = msNow;
        return;
    }
}

void AmisReaderClass::processStateSerialnumber(const uint32_t msNow)
{
    if (_state == initReadSerial) {
        LOG_DP("Initalizing reading serialnumber");
        _baudRateIdentifier = 0; _serialNumber[0] = 0;
        _iec.abort();

        if (_readSerialNumberMode == disabled && !_billingDataset) {
            _state = initReadCounters;
            return;
        }
        _serialReadBufferIdx = 0;
        clearSerialRx();
        _readerIsOnline = false;
        _state = requestReaderSerial;
        _stateLastSetMs = msNow;
        // libamis sendet die Anfrage bis zu 3x im Abstand von 5 s und gibt nach ca. 12,5 s auf (kFailed);
        // der State-Timeout ist nur noch eine Sicherung. 5 Versuche = ca. 65 Sekunden wie bisher.
        _stateTimoutMs = 20000;
        _stateErrorCnt = 0;
        _stateErrorMax = 5;
        _validFrameCnt = 0;
    } else if (_state == requestReaderSerial) {
        // Geräteadresse lesen (IEC 62056-21 Mode "C", libamis IecReader):
        // "/?!" mit 300 Bd 7E1, Antwort "/SATx<Identifikation>", dann Break (nur Seriennummer)
        // oder ACK 0 0 0 und Verrechnungsdatensatz mit 300 Bd (wie bisher: geht nur mit 300 Bd)
        _iec.abort();
        _serialReadBufferIdx = 0;
        clearSerialRx();
        amis::IecReader::Mode mode = amis::IecReader::Mode::kIdentOnly;
        if (_billingDataset) {
            mode = amis::IecReader::Mode::kReadout;
            _iec.set_max_baud_char('0');
            _iec.set_total_timeout_ms(15u * 60u * 1000u); // dauert mit 300 Bd etwa 5-6 Minuten
            _billingDatasetRead = 0;
            _billingDatasetReadPrev = 0;
        }
        LOG_DP("Requesting serialnumber");
        handleIecActions(_iec.begin(mode, msNow));
        _state = waitForReaderSerial;
        _stateLastSetMs = msNow;
    } else if (_state == waitForReaderSerial || _state == waitForBillingDatasetEnd) {
        uint8_t act = 0;
        for (size_t i = 0; i < _serialReadBufferIdx; i++) {
            const uint8_t a = _iec.push(_serialReadBuffer[i], msNow);
            if ((a & amis::IecReader::kRawLine) && _billingDataset) {
                const char *line = _iec.raw_line();
                _billingDatasetRead += _billingDataset.write((const uint8_t *)line, strlen(line));
                _billingDatasetRead += _billingDataset.write((const uint8_t *)CRLF, 2);
            }
            act |= a;
        }
        _serialReadBufferIdx = 0;
        act |= _iec.poll(msNow);

        if (!_serialNumber[0] && _iec.identification()[0]) {
            // Wir haben eine "gültige" Zähler-Antwort bekommen!
            _baudRateIdentifier = _iec.baud_char();
            strlcpy(_serialNumber, _iec.identification(), sizeof(_serialNumber));
            // Nicht jetzt ins Log schreiben: das dauert auf LittleFS mitunter ueber 1 s, und der Zaehler
            // erwartet ACK bzw. Break innerhalb von 1,5 s. Gelogged wird nach dem Senden (siehe unten).
            _logSerialPending = true;
            if (_billingDataset) {
                _state = waitForBillingDatasetEnd;
                _stateTimoutMs = 16u * 60u * 1000u; // Zeitueberwachung macht ab jetzt libamis (15 min)
                _stateErrorCnt = 0;
                _stateLastSetMs = msNow;
            }
        }
        if (_billingDataset && _billingDatasetRead > _billingDatasetReadPrev + 20) {
            _billingDataset.flush();
            _billingDatasetReadPrev = _billingDatasetRead;
            _stateLastSetMs = msNow;
        }

        handleIecActions(act);
        if (_logSerialPending && (act & (amis::IecReader::kSend | amis::IecReader::kFailed))) {
            _logSerialPending = false;
            LOGF_IP("Serialnumber %s found", _serialNumber);
            if (_billingDataset) {
                LOG_IP("Requesting whole billing dataset with 300 baud.");
            }
        }

        if (act & amis::IecReader::kDone) {
            if (_billingDataset) {
                _billingDatasetComplete = true;
                _state = endBillingDataset;
            } else {
                _state = initReadCounters; // Break ist gesendet, weiter zu den Online Zählerdaten
            }
            _stateLastSetMs = msNow;
        } else if (act & amis::IecReader::kFailed) {
            LOGF_IP("IEC readout failed: %s %s", amis::error_str(_iec.error()), _iec.meter_error());
            if (_state == waitForBillingDatasetEnd) {
                _state = endBillingDataset;
                _stateLastSetMs = msNow;
            } else {
                // keine Identifikation: gleich neuer Versuch, zaehlt als Fehler (nach _stateErrorMax aufgeben)
                _stateErrorCnt++;
                _state = requestReaderSerial;
                _stateLastSetMs = msNow;
            }
        }
    }
}

void AmisReaderClass::eatSerialReadBuffer(size_t n)
{
    // Entfernt n Bytes aus dem internen Serial-Lese Puffer
    size_t i;
    if (n >= _serialReadBufferIdx || n >= sizeof(_serialReadBuffer)) {
        _serialReadBufferIdx = 0;
        return;
    }

    for (i=n; i<_serialReadBufferIdx; i++) {
        _serialReadBuffer[i-n] = _serialReadBuffer[i];
    }
    _serialReadBufferIdx -= n;
}

void AmisReaderClass::processStateCounters(const uint32_t msNow)
{
    if (_state == initReadCounters) {
        LOG_DP("Initalizing reading counter values");
        _iec.abort();
        if (_billingDataset) {
            // Verrechnungsdatensatz wurde nicht fertig gelesen (z. B. keine Seriennummer bekommen)
            _billingDataset.close();
            LittleFS.remove(AMISREADER_BILLINGDATA_FILENAME);
        }
        if (_serial) {
            _serial->begin(9600, SERIAL_8E1); /* Zählerdaten kommen angeblich mit 9600 8E1 */
            _serial->clearWriteError();
        }
        _serialReadBufferIdx = 0;
        clearSerialRx();
        _bytesInBufferExpectd = 0;
        _mbus.reset();
        // Bytes kommen gesammelt pro loop()-Durchlauf; bei laengeren Pausen der loop() soll ein Frame nicht
        // gleich verworfen werden (libamis-Standard waere 200 ms)
        _mbus.set_interbyte_timeout_ms(500);
        _mbus.set_required_fields(amis::Reading::kAll); // wie bisher: nur vollstaendige Frames zaehlen
        _readerIsOnline = false;
        _state = readReaderCounters;
        _stateLastSetMs = msNow;
        _stateTimoutMs = 4000; // Timeout für das Lesen: 16 * 4 (=64 Sekunden)
        _stateErrorCnt = 0;
        _stateErrorMax = 16;
        _validFrameCnt = 0;
    } else if (_state == readReaderCounters) {
        // Alle empfangenen Bytes einzeln an libamis geben. libamis erkennt SND_NKE und SND_UD,
        // sagt wann E5 zu senden ist (genau eins pro Frame), entschluesselt und liest die Werte.
        for (size_t i = 0; i < _serialReadBufferIdx; i++) {
            const uint8_t st = _mbus.push(_serialReadBuffer[i], msNow);
            if (st & amis::Reader::kSendAck) {
                serialWrite(amis::Reader::kAck);
            }
            if (st & (amis::Reader::kReading | amis::Reader::kError)) {
                onCounterResult(st, msNow);
            }
        }
        _serialReadBufferIdx = 0;
    }
}

void AmisReaderClass::onCounterResult(uint8_t status, const uint32_t msNow)
{
    if (!(status & amis::Reader::kReading) && _mbus.last_error() == amis::Error::kReplay) {
        // libamis: Zaehlerzeit nicht neuer als beim letzten Datensatz (wiederholter oder alter Frame).
        // Keine neuen Daten, die bisherigen bleiben gueltig.
        LOGF_DP("Frame ignored: %s", amis::error_str(amis::Error::kReplay));
        return;
    }
    {
        Databroker.valid = 0;

        AmisReaderNumResult_t l_result;
        int r = -1;
        if (status & amis::Reader::kReading) {
            r = convertReading(_mbus.reading(), l_result);
        }
        const amis::Error mbusError = _mbus.last_error();
        if (r == 0) {
            // OK, we got new valid data ...
            _validFrameCnt++;

            // Move them to Databroker
            static_assert(std::size(Databroker.results_u32) == std::size(l_result.results_u32));
            for (size_t i=0;i < std::size(Databroker.results_u32); i++) {
                Databroker.results_u32[i] = l_result.results_u32[i];
            }
            static_assert(std::size(Databroker.results_i32) == std::size(l_result.results_i32));
            for (size_t i=0;i < std::size(Databroker.results_i32); i++) {
                Databroker.results_i32[i] = l_result.results_i32[i];
            }

            strlcpy(Databroker.timeCp48Hex, l_result.timeCp48Hex, sizeof(Databroker.timeCp48Hex));  // Hextsring der MBUS-CP48 Bytes[aber NUR 5 Bytes] (wie vom Zähler bekommen)
            memcpy(&Databroker.time, &l_result.time, sizeof(Databroker.time));  // struct tm (gefüllt - wie vom Zähler bekommen)
            Databroker.ts = mktime(&l_result.time); // Seconds since epoch in UTC (umgewandelt von l_result.time)

            Databroker.valid = 5;

            // Now all new data should be fetched only from Databroker!

            new_data_for_websocket = true;
            if (first_frame == 0) {
                first_frame = 3; // Erster Datensatz nach Reboot oder verlorenem Sync
            }

            RemoteOnOff.onNewValidData(Databroker.results_u32[4]/* 1.7.0 */, Databroker.results_u32[5] /* 2.7.0 */);
            MaxDemand.onNewValidData(Databroker.time, Databroker.ts, Databroker.results_u32[0] /* 1.8.0 */);

            // TODO(anyone): Refactor that "special values" from main.cpp
            dow = l_result.time.tm_wday; // dow: 1..7 (1=MON...7=SUN)
            if (dow == 0) {              // tm_wday: days since Sunday (0...6)
                dow = 7;
            }
            mon = l_result.time.tm_mon + 1; // mon: 1...12  -   tm_mon: 0...11 (months since January)
            myyear = l_result.time.tm_year - 100; // myyear: Jahr nur 2 stellig benötigt (seit 2000)

            if (!_readerIsOnline || msNow - _lastTimeSync > 1800000u ) {
                if (!_readerIsOnline) {
                    LOG_IP("Data synced with counter");
                    _readerIsOnline = true;
                }
                // Sync time afer sync with reader or each 30 minutes avoid internal clock drift
                setTime(Databroker.ts);
                _lastTimeSync = msNow;
            }

            _stateLastSetMs = msNow;
            _stateTimoutMs = 5000;  // ab jetzt: Timeout mit 10 Sekunden (2 * 5000)
            _stateErrorCnt = 0;
            _stateErrorMax = 2;
        } else if (r < -7 || mbusError == amis::Error::kWrongKey || mbusError == amis::Error::kNoKey ||
                   mbusError == amis::Error::kRecordFormat || mbusError == amis::Error::kNoData ||
                   mbusError == amis::Error::kIncomplete) {
            // das Zählertelegramm sah prinzipiell gut aus aber nach dem Entschlüsseln passten einige Werte nicht
            // ==> d.h.: vermutlich stimmt der AMIS-Key nicht !
            Databroker.valid = 1;
            LOGF_DP("Decrypting data failed: %s (%d)", amis::error_str(mbusError), r);
        } else {
            // Beim Zählertelegram stimmten schon die Eckdaten (Header, Checksumme, usw nicht)
            // ==> d.h.: vermutlich ungültige Daten empfangen
            Databroker.valid = 0;
            LOGF_DP("Invalid data received: %s", amis::error_str(mbusError));
        }

        // TODO(anyone): Refactor - Create events on changed data
        bool isValid = (Databroker.valid == 5) ?true :false;
        ModbusSmartmeterEmulation.setCurrentValues(isValid,
                                                   Databroker.results_u32[4], Databroker.results_u32[5],
                                                   Databroker.results_u32[0], Databroker.results_u32[1]);
        ShellySmartmeterEmulation.setCurrentValues(isValid,
                                                   Databroker.results_u32[4], Databroker.results_u32[5],
                                                   Databroker.results_u32[0], Databroker.results_u32[1]);
        ThingSpeak.onNewData(isValid, &Databroker.results_u32[0], Databroker.ts);
        SYSTEMMONITOR_STAT();
     }
}

void AmisReaderClass::loop()
{
    uint32_t msNow;

    if (!_isEnabled) {
        return;
    }

    // always read some bytes from the serial interface
    _rxSerailRead += pollSerial(120);

    msNow = millis();

    if (_state == msDelay) {
        if (msNow - _stateLastSetMs <= _stateTimoutMs) {
            return;
        }
        _state = _statePostDelay;
        _stateTimoutMs = _stateTimeoutMsPostDelay;
        _stateLastSetMs = msNow;
    }

    // Do all releated things querying serialnumber
    processStateSerialnumber(msNow);

    // Do all releated things querying billing dataset (Verrechnungsdatensatz)
    processStateReadBillingDataset(msNow);

    // Do all releated steps to read the counters
    processStateCounters(msNow);

    if (_state == msDelay) {
        // Status wurd nun soeben auf delay gesetzt (ansonsten hätten wir das oben behandlet!)
        _stateLastSetMs = millis(); // wirklich die aktuelle Zeit fürs delay verwenden
        return;
    }

    if (msNow - _stateLastSetMs >= _stateTimoutMs) {
        // Timeout: seems something gone at the actual state wrong and needed too long
        LOGF_DP("State timeout occured! current state=%d _stateErrorCnt=%u _stateErrorMax=%u",
                 (int)_state, _stateErrorCnt, _stateErrorMax);

        _stateLastSetMs = msNow;
        _stateErrorCnt++;

        // Be sure to cleanup the serial interface
        _serialReadBufferIdx = 0;
        clearSerialRx();
        _bytesInBufferExpectd = 0;

        if (_state == waitForReaderSerial) {
            _state = requestReaderSerial;
        } else if (_state == waitForBillingDatasetStart || _state == waitForBillingDatasetEnd ||
                   _state == waitForBillingDatasetStartSingleValue || _state == waitForBillingDatasetEndSingleValue) {
            _state = endBillingDataset; // libamis hat schon selbst abgebrochen bzw. nichts mehr gekommen
        }
    }

    if (_stateErrorCnt >= _stateErrorMax) {
        LOGF_DP("State max errors reached! Switching state. current state=%d", (int)_state);

        if (_readerIsOnline) {
            LOGF_WP("Sync lost. %u data frames received. Starting resync...", _validFrameCnt);
            _readerIsOnline = false;
        }
        _validFrameCnt = 0;
        Databroker.valid = 0;
        ModbusSmartmeterEmulation.setCurrentValues(false);
        ThingSpeak.onNewData(false);

        _stateErrorCnt = 0;

        _bytesInBufferExpectd = 0;

        _serialReadBufferIdx = 0;
        clearSerialRx();

        if (_state == initReadBillingDataset || _state == requestBillingDataset || _state == requestSpecialCounterValue ||
            _state == waitForBillingDatasetStart || _state == waitForBillingDatasetEnd ||
            _state == decodeBillingDataset) {
            _state = endBillingDataset;
            return;
        }

        if (_state == initReadSerial || _state == requestReaderSerial ||
            _state == waitForReaderSerial || _state == decodeReaderSerial ) {
            if (_readSerialNumberMode == tryRead || _readSerialNumberMode == disabled ) {
                // we tried to get the serialnumber ... skip that - get the counter numbers
                // btw: "disabled" should never happend here
                _state = initReadCounters;
            } else {
                _state = initReadSerial;
            }
        } else {
            // we tried to get the counter numbers ... do a fresh start by first querying serial number
            if (_readSerialNumberMode == tryRead || _readSerialNumberMode == mustRead ) {
                _state = initReadSerial;
            } else {
                _state = initReadCounters;
            }
        }
    }
}

void AmisReaderClass::setKey(const char *key)
{
    // 32 Hex-Zeichen (Leerzeichen, ':' und '-' erlaubt). Ungueltig oder leer: Schluessel aus Nullen
    // wie bisher, dann kann nichts entschluesselt werden.
    if (key == nullptr || !_mbus.set_key_hex(key)) {
        if (key != nullptr && key[0]) {
            LOG_EP("Invalid AMIS key (expected 32 hex characters)");
        }
        memset(_key, 0, sizeof(_key));
        _mbus.set_key(_key);
    }
}

void AmisReaderClass::setKey(const uint8_t *key)
{
    memcpy(_key, key, sizeof(_key));
    _mbus.set_key(_key);
}

void AmisReaderClass::enable() {
    _isEnabled = true;
}

#if 0
void AmisReaderClass::enableRestart() {
    if (_isEnabled) {
        return;
    }
    _isEnabled = true;
    _state = initReadSerial;
}
#endif

void AmisReaderClass::disable() {
    _isEnabled = false;
}


String AmisReaderClass::statusText() {
    static const __FlashStringHelper* dots[] = { F(""), F("."), F(".."), F("...") };
    String r;
    AmisReaderState_t s = (_state == msDelay) ?_statePostDelay :_state;

    if (!_isEnabled) {
        r = F("Zählerabfrage disabled");
    } else {
        size_t dots_num = time(NULL) & 3;
        const __FlashStringHelper* dots_repeat = dots[dots_num];
        if (s == readReaderCounters || s == decodeReaderCounter) {
            if (Databroker.valid == 5) {
                r = F("Lese Zählerdaten");
                r += dots_repeat; // Alles OK ... wir lesen
            } else if (Databroker.valid == 0) {
                r = F("Warte auf Zählerdaten");
                r += dots_repeat;
            } else {
                r = F("Amis-Key ungültig?");
            }
        } else {
            if (s == initReadCounters) {
                r = F("Warte auf Zählerdaten");
                r += dots_repeat;
            } else if (s == initReadSerial || s == requestReaderSerial || s == waitForReaderSerial || s == decodeReaderSerial) {
                r = F("Lese Gerätenummer");
                r += dots_repeat;
            } else {
                r = F("Lese interne Zählerdaten (") + String(_billingDatasetRead)  + " Bytes)";
                r += dots_repeat;
            }
        }
    }
    return r;
}

AmisReaderClass AmisReader;

/* vim:set ts=4 et: */
