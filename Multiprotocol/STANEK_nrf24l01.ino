/*
***********************************************************************************************
Simple RF protocol with frequency hopping (FHSS) and two-way telemetry (Auto-ACK Payload).
***********************************************************************************************
Rapid hopping between 4 strong RF channels (12, 40, 76, 88) outside the main WiFi frequencies.
Transmission rate of 1 Mbps (increased for full support of the newer nRF52 generation).
Constant latency / period of 4.5 ms (222 Hz).
The received telemetry displays the A1(A2) voltage and RSSI.

* The bind "Bnd" function saves the Fail-Safe positions for the servos into the receiver.
* In the "Option" menu, enter the number of RC channels from 2 to 14.

Support for DIY receivers from this repository: https://github.com/stanekTM/RX_nRF24_FHSS
***********************************************************************************************
*/

#if defined(STANEK_NRF24L01_INO)

#include "iface_nrf24l01.h"

// Unikátní RF adresa max. 5 bajtů (číslo nebo znak). Musí být stejná i v přijímači
const uint8_t STANEK_RF_ADDRESS[] = "jirka";

// 4 silné RF kanály (mimo hlavní WiFi frekvence)
const uint8_t STANEK_RF_CHANNELS[] = {12, 40, 76, 88};
uint8_t fhss_index = 0; // Ukazatel na aktuálně používaný RF kanál z tabulky

// Délka jednoho vysílacího / přijímacího okna v mikrosekundách
#define STANEK_PACKET_PERIOD  4500 // Pevná odesílací / přijímací perioda 4.5 ms (222 Hz)

// Prototypy všech funkcí pro správné sestavení kompilátorem (mimo STANEK_callback a STANEK_init)
static void STANEK_RF_init();
static void STANEK_send_packet();
static void STANEK_get_telemetry();

//**********************************************************************************************************************************
// Kompletní inicializace registrů nRF24L01+
//**********************************************************************************************************************************
static void __attribute__((unused)) STANEK_RF_init()
{
  // 1. Hardwarový reset (uvede čip do předem známého výchozího stavu)
  NRF24L01_Initialize();
  
  // 2. Vyčistit případné staré nebo poškozené pakety z FIFO front po resetu
  NRF24L01_FlushTx(); // Vyčistit vysílací vyrovnávací paměť (TX FIFO)
  NRF24L01_FlushRx(); // Vyčistit přijímací vyrovnávací paměť (RX FIFO)
  
  // 3. Zápis adres a aktivace datových kanálů (Pipe 0 je nutná pro zachycení zpětné telemetrie)
  NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR,    (uint8_t*)(&STANEK_RF_ADDRESS), 5);
  NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0, (uint8_t*)(&STANEK_RF_ADDRESS), 5);
  NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, 0x01); // Povolit pouze datový kanál Pipe 0 (pro příjem dat)
  NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, 0x03); // Šířka adresy 5 bajtů
  
  // 4. Protokolární konfigurace (řízení přenosových vlastností)
  NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x01); // Zapnuto hardwarové Auto-ACK na Pipe 0 pro příjem potvrzení a telemetrie
  NRF24L01_WriteReg(NRF24L01_04_SETUP_RETR, 0x00); // Vypnuto automatické opakování paketů (zajišťuje nulovou latenci)
  NRF24L01_SetBitrate(NRF24L01_BR_1M); // Nastavení střední rychlosti 1 Mbps (250 kbps nRF52832 nepodporuje)
  NRF24L01_WriteReg(NRF24L01_05_RF_CH, STANEK_RF_CHANNELS[fhss_index]); // Nastavení startovního RF kanálu z FHSS tabulky
  
  // 5. Odemčení a zápis pokročilých funkcí (dynamická délka dat a ACK payload)
  //NRF24L01_Activate(0x73); // 1. Odemknout přístup k pokročilým registrům (DYNPD, FEATURE)
  NRF24L01_WriteReg(NRF24L01_1C_DYNPD, 0x01); // Povolení příjmu paketů s proměnlivou (dynamickou) délkou pouze pro Pipe 0
  NRF24L01_WriteReg(NRF24L01_1D_FEATURE, 0x06); // Bit 0x04 zapíná Dynamic Payload, bit 0x02 zapíná funkci ACK Payload (přibalování telemetrie)
  //NRF24L01_Activate(0x73); // 2. Zamknout registry pro zajištění stability (DYNPD, FEATURE). Zakomentovat v případě klonu
  
  // 6. Vyčistit všechny stavové flagy (RX_DR, TX_DS, MAX_RT)
  NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
  
  // 7. Finální spuštění modulu
  NRF24L01_SetPower(); // Zapnout napájení a zápis RF výkonu (nastaví RF_SETUP)
  NRF24L01_SetTxRxMode(TX_EN); // Zapnout vysílací režim
  
  delayMilliseconds(10); // Čas pro nastavení registrů
}

//**********************************************************************************************************************************
// Asynchronní příjem zpětných dat z modelu a vyhodnocování kvality signálu linky
//**********************************************************************************************************************************
static void __attribute__((unused)) STANEK_get_telemetry()
{  
  // Měření TSSI: Okno pro 100 očekávaných paketů jednou za 250 cyklů
  if (++state > 250)
  {
    // Pojistka proti přetečení
    //if (telemetry_counter > 100) telemetry_counter = 100;
    // Počet paketů (0 až 100) je přímo výsledné procento
    //TX_RSSI = telemetry_counter; // Zápis TSSI (kvality odesílání) do telemetrie zpětné vazby OpenTX / EdgeTX
    
    // Přenásobením koeficientem 1.6 získáme při plném signálu hodnotu 200 (125 * 1.6 = 200)
    uint16_t tssi_scaled = ((uint16_t)telemetry_counter * 16) / 10;
    
    // Pojistka proti přetečení
    if (tssi_scaled > 200) tssi_scaled = 200;
    
    // Počet paketů (0 až 200) se v OpenTX / EdgeTX zobrazí jako 0 až 100 %
    TX_RSSI = tssi_scaled; // Zápis TSSI (kvality odesílání) do telemetrie zpětné vazby OpenTX / EdgeTX
    
    state = 0; // Vynulování děliče cyklů
    telemetry_counter = 0; // Vynulování počítadla úspěšně přijatých paketů
    telemetry_lost = 0; // Vynulování ztráty telemetrického spojení
  }
  
  // Vyčteme aktuální stav registru STATUS
  //uint8_t status_reg = NRF24L01_ReadReg(NRF24L01_07_STATUS);
  
  // Pokud je nastaven bit 6 (RX_DR), znamená, že v rámci Auto-ACK hardwarově dorazil telemetrický paket od přijímače
  //if (status_reg & (1 << 6))
  if (NRF24L01_ReadReg(NRF24L01_07_STATUS) & _BV(NRF24L01_07_RX_DR))
  {
    NRF24L01_ReadPayload(packet, 3); // Vyčteme 3 bajty telemetrie z RX FIFO paměti čipu
    
    RX_RSSI = packet[0]; // 1.bajt: Síla signálu (RSSI) na straně přijímače (0 až 255)
    v_lipo1 = packet[1]; // 2.bajt: Analogový senzor A1 napětí baterie (0 až 255)
    v_lipo2 = packet[2]; // 3.bajt: Analogový senzor A2 napětí baterie (0 až 255)
    
    telemetry_counter++; // Navýšení počtu přijatých paketů do statistiky TSSI
    
    // Ztráta telemetrického spojení
    if (telemetry_lost == 0) telemetry_link = 1; // Aktivujeme příznak, že telemetrické spojení je plně aktivní
  }
  
  NRF24L01_SetTxRxMode(TX_EN); // Okamžitě vypni příjem a zapni vysílání
  NRF24L01_FlushRx(); // Vyčistit přijímací vyrovnávací paměť (RX FIFO) pro čistý start příštího kola
}

//**********************************************************************************************************************************
// Sestavení dat a odeslání paketu
//**********************************************************************************************************************************
static void __attribute__((unused)) STANEK_send_packet()
{
  // Okamžitě na začátku cyklu stáhneme telemetrická data z minulého kola
  STANEK_get_telemetry();
  
  // Určení počtu RC kanálů v menu nastavení modelu "Možnosti"
  int8_t signed_option = (int8_t)option; // Přetypováním na (int8_t) zajistíme, že procesor uvidí záporné hodnoty správně
  
  if (signed_option < 2) num_ch = 2; // Cokoli pod hodnotu 2 zafixujeme
  else if (signed_option > 14) num_ch = 14; // Cokoli nad hodnotu 14 zafixujeme
  else num_ch = signed_option; // Počet RC kanálů v přesném rozsahu 2 až 14
  
  // Pokud aktivujeme párování "Bnd" (příznak Fail-Safe)
  if (IS_BIND_IN_PROGRESS)
  {
    packet[0] = 1; // Zapneme příznak Fail-Safe v paketu na indexu 0
    
    // Po 3 sekundách (667 průchodů při 4.5 ms periodě) ukončete párování "Bnd"
    if (bind_counter++ == 667) BIND_DONE;
  }
  // Párování "Bnd" neběží
  else 
  {
    packet[0] = 0; // Vypneme příznak Fail-Safe v paketu na indexu 0
    bind_counter = 0; // Vynulujeme počítadlo
  }
  
  // Sestavení payloadu (2 bajty na jeden RC kanál). Na indexu 0 je příznak Fail-Safe
  uint8_t payload_index = 1; // Plnění RC kanálů začíná přesně od indexu 1
  
  for (uint8_t i = 0; i < num_ch; i++)
  {
    uint16_t hold_value = convert_channel_16b_limit(i, 1000, 2000); // Konverze výchylek páček na limitovaný rozsah (1000 až 2000 us)
    
    packet[payload_index++] = hold_value & 0x00FF; // Spodní (Low) bajt
    packet[payload_index++] = hold_value >> 8;     // Horní (High) bajt
  }
  
  // Velikost paketu: 1 bajt (příznak Fail-Safe) na indexu 0 + 2 bajty na RC kanál
  uint8_t packet_size = 1 + (num_ch * 2);
  
  // 1. Synchronní skok na další RF kanál
  fhss_index = (fhss_index + 1) % 4; // Posun na další index v tabulce
  NRF24L01_WriteReg(NRF24L01_05_RF_CH, STANEK_RF_CHANNELS[fhss_index]); // Nastavení RF kanálu na novou frekvenci
  
  // 2. Vyčistit stavové flagy a vyprázdnit paměť před novým zápisem
  NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70); // Vyčistit všechny předchozí stavové flagy (TX_DS, MAX_RT, RX_DR)
  NRF24L01_FlushTx(); // Vyčistit vysílací vyrovnávací paměť (TX FIFO)
  
  // 3. Zapnout napájení a zápis RF výkonu (nastaví RF_SETUP)
  NRF24L01_SetPower();
  
  // 4. Zápis sestavených dat
  NRF24L01_WritePayload(packet, packet_size);
  
  // 5. Trvalé vynucení vysílacího režimu (PWR_UP, PRIM_TX, 2B CRC)
  NRF24L01_WriteReg(NRF24L01_00_CONFIG, 0x7E);
}

//**********************************************************************************************************************************
// Pravidelný hardwarový callback volaný operačním systémem RC vysílače
//**********************************************************************************************************************************
uint16_t STANEK_callback()
{
 #ifdef MULTI_SYNC
  telemetry_set_input_sync(STANEK_PACKET_PERIOD); // Synchronizuje hodiny OpenTX / EdgeTX s rádiem před odesláním dat
 #endif
  
  STANEK_send_packet(); // Odeslání aktuálního paketu k modelu
  
  return STANEK_PACKET_PERIOD; // Vrácení pevné periody 4500 us (4.5 ms)
}

//**********************************************************************************************************************************
// Inicializace protokolu STANEK volaná jádrem Multiprotokolu při startu / přepnutí modelu
//**********************************************************************************************************************************
void STANEK_init(void)
{
  fhss_index = 0; // Nastavení startovního RF kanálu z FHSS tabulky
  STANEK_RF_init(); // Kompletní inicializace registrů nRF24L01+
}

#endif // Konec STANEK_NRF24L01_INO
