/* 
   METAMORE - FIRMWARE DEL TEST UTENTE, Versione finale
*/

#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "Adafruit_DRV2605.h"

/* 
   1. LE MANOPOLE
*/

/* --- PNEUMATICA ---
   livello raggiunto 28 s · sfiato spontaneo 43 s · svuotamento 44,7 s        */
const unsigned long GONFIAGGIO_POS3_MS = 14000UL;   // meta' del livello
const unsigned long GONFIAGGIO_POS2_MS =  9000UL;   // sequenza ridotta
const unsigned long IMPULSO_GONFIAGGIO_MS = 350UL;
const unsigned long TETTO_POMPAGGIO_MS = 40000UL;   // D29 / D34

/* Il ritmo (D30). Sul capo lo sfiato e' PIU' LENTO del riempimento (1,6x),
   quindi lo sfiato dev'essere 1,90 volte il soffio: cosi' ogni battito toglie
   circa il 19 % in piu' di quanto mette e la pressione non puo' salire.      */
const unsigned long RITMO_POMPA_MIN_MS   = 120;
const unsigned long RITMO_POMPA_MAX_MS   = 200;
const float         RITMO_SFIATO_FATTORE = 1.90;
const unsigned long RITMO_PAUSA_MIN_MS   = 2200;
const unsigned long RITMO_PAUSA_MAX_MS   = 3400;

/* --- TERMICA (D32) --- */
const float T_PARTENZA = 32.0, T_ARRIVO = 37.0, T_BLOCCO = 40.0;
const float ISTERESI_T = 0.15;
const float PLAUS_MIN = 5.0, PLAUS_MAX = 60.0;
const int   STRIKE_SENSORE = 3;          // letture cattive di fila prima di spegnere
const unsigned long ATTESA_PRIMA_LETTURA_MS = 15000UL;
/* LA RAMPA E' DEFINITA DALLA SUA PENDENZA, NON DALLA SUA DURATA.
   Il sensore sta fra la pelle e il riscaldatore: addosso parte gia' a
   33-34 gradi, non a temperatura ambiente. Se la rampa partisse comunque
   da 32 gradi, i primi minuti sarebbero a riscaldatori spenti - visti da
   chi indossa il capo, minuti in cui non succede niente.
   Quindi la rampa parte dalla temperatura VERA letta in quel momento
*/
float GRADI_AL_MINUTO = 1.20;    // si cambia da tastiera con f
                                // e g, anche a sessione in corso
const unsigned long RAMPA_MS = 500000UL;   // solo come riferimento/tetto
const unsigned long DISCESA_TERMICA_MS =  60000UL;
const unsigned long DISCESA_PNEUM_MS   =  45000UL;

/* --- SLITTA (D3) ---  */
int SOGLIA_12 = 1400, SOGLIA_23 = 2800, ISTERESI = 200;
const unsigned long SLITTA_STABILE_MS = 250;
int zonaForzata = 0;                     // 0 = slitta vera; 1,2,3 = da tastiera

/* --- LRA (D15) --- */
uint8_t INTENSITA_SX = 0x7F;             // massimo
uint8_t INTENSITA_DX = 0x7F;
/* FREQUENZA
*/
const uint8_t PERIODO_COMUNE  = 0x3A;    // circa 175 Hz
const uint8_t PERIODO_MIN     = 0x36;    // finestra di misure accettabili
const uint8_t PERIODO_MAX     = 0x3E;

/* FORZA DI SPINTA per lato (registro 0x17).
   Il driver destro si riavviava sui picchi di corrente: abbassata la
   spinta a 0x6C i riavvii sono andati a zero e l'intensita' percepita
   e' pari a quella del sinistro. */
const uint8_t CLAMP_SX = 0x8C;
const uint8_t CLAMP_DX = 0x6C;
const int M1_COLPO = 240, M1_PAUSA = 120;
const int M2_COLPO = 120, M2_PAUSA = 120, M2_TRA = 500;
const int M3_COLPO = 300, M3_PAUSA = 150, M3_TRA = 400, M3_RIPET = 3;

/* 
   2. I PIN
*/
const int RISC_A  = 13;
const int RISC_B  = 14;
const int POMPA   = 16;
const int VALVOLA = 17;
const int NON_USATO_1 = 27;
const int NON_USATO_2 = 32;
const int ONEWIRE = 4;
const int SLITTA  = 34;
const int LED_SX  = 25;
const int LED_DX  = 26;

const int COMANDI[6] = { RISC_A, RISC_B, POMPA, VALVOLA, NON_USATO_1, NON_USATO_2 };

inline void accendi(int p) { pinMode(p, INPUT); }
inline void spegni (int p) { pinMode(p, OUTPUT); digitalWrite(p, LOW); }
void spegniTutto() { for (int i = 0; i < 6; i++) spegni(COMANDI[i]); }

/* 
   3. SENSORI
    */
OneWire filo(ONEWIRE);
DallasTemperature sensori(&filo);
DeviceAddress ROM_A = {0x28, 0x88, 0x53, 0xC4, 0x00, 0x00, 0x00, 0xB9};
DeviceAddress ROM_B = {0x28, 0x16, 0x5C, 0x25, 0x00, 0x00, 0x00, 0xB2};

float tempA = NAN, tempB = NAN;
bool  zonaA_viva = true, zonaB_viva = true;
int   strikeA = 0, strikeB = 0;
bool  primaLetturaFatta = false;
unsigned long tAccensione = 0;
unsigned long ultimaRichiesta = 0;
bool inAttesaConversione = false;

bool plausibile(float t) { return (t > PLAUS_MIN && t < PLAUS_MAX); }

void aggiornaTemperature() {
  unsigned long ora = millis();
  if (!inAttesaConversione && ora - ultimaRichiesta > 1000) {
    sensori.requestTemperatures();
    ultimaRichiesta = ora;
    inAttesaConversione = true;
    return;
  }
  if (inAttesaConversione && ora - ultimaRichiesta > 300) {
    tempA = sensori.getTempC(ROM_A);
    tempB = sensori.getTempC(ROM_B);
    inAttesaConversione = false;
    if (plausibile(tempA) || plausibile(tempB)) primaLetturaFatta = true;
  }
}

/* 
   4. I DUE DRIVER LRA
*/
Adafruit_DRV2605 drvSX, drvDX;
bool cSX = false, cDX = false;
uint8_t periodoSX = PERIODO_COMUNE, periodoDX = PERIODO_COMUNE;
int rimessiAPosto = 0;

/* Ogni lato ha la sua forza di spinta. */
uint8_t clampDi(Adafruit_DRV2605 &d) { return (&d == &drvSX) ? CLAMP_SX : CLAMP_DX; }

void configuraLRA(Adafruit_DRV2605 &d, uint8_t periodo) {
  d.writeRegister8(0x01, 0x00);
  d.writeRegister8(0x16, 0x3E);
  d.writeRegister8(0x17, clampDi(d));
  d.writeRegister8(0x1A, 0xB6);
  d.writeRegister8(0x1D, 0xA1);     // ANELLO APERTO
  d.writeRegister8(0x20, periodo);
  d.writeRegister8(0x03, 0x06);
  d.writeRegister8(0x01, 0x05);     // tempo reale
}

/* RIPRISTINO VELOCE. Se il chip si e' riavviato non serve rifare tutto:
   bastano i registri che contano, senza passare dallo standby. Il buco di
   silenzio dura pochi millisecondi invece di trenta, e anche un riavvio
   residuo non si sente.                                                     */
void ripristinaVeloce(Adafruit_DRV2605 &d, uint8_t periodo) {
  d.writeRegister8(0x1A, 0xB6);
  d.writeRegister8(0x1D, 0xA1);
  d.writeRegister8(0x20, periodo);
  d.writeRegister8(0x17, clampDi(d));
  d.writeRegister8(0x01, 0x05);
}

/* I DRV2605L si riavviano sui picchi di corrente: tornano ai valori di
   fabbrica e vanno in standby restando pero' sul bus. Si controlla anche
   0x1D, che e' il registro dell'anello aperto, e 0x20, la frequenza.        */
void assicuraLRA(Adafruit_DRV2605 &d, uint8_t periodo) {
  if (d.readRegister8(0x01) != 0x05 ||
      d.readRegister8(0x1A) != 0xB6 ||
      d.readRegister8(0x1D) != 0xA1 ||
      d.readRegister8(0x20) != periodo) {
    ripristinaVeloce(d, periodo);
    rimessiAPosto++;
  }
}

/* Calibrazione dolce: serve solo a leggere la frequenza di risonanza. */
bool misuraFrequenza(Adafruit_DRV2605 &d, uint8_t &periodo) {
  for (int tent = 0; tent < 3; tent++) {
    d.writeRegister8(0x01, 0x00);
    d.writeRegister8(0x1A, 0xB6);
    d.writeRegister8(0x16, 0x20);
    d.writeRegister8(0x17, 0x50);
    d.writeRegister8(0x1E, 0x20);
    d.writeRegister8(0x01, 0x07);
    d.writeRegister8(0x0C, 0x01);
    unsigned long t = millis();
    while (d.readRegister8(0x0C) & 0x01) { if (millis() - t > 2000) break; delay(10); }
    if (!(d.readRegister8(0x00) & 0x08)) {
      uint8_t p = d.readRegister8(0x22);
      if (p >= PERIODO_MIN && p <= PERIODO_MAX) { periodo = p; return true; }
    }
    delay(300);
  }
  return false;
}

void colpoLRA(Adafruit_DRV2605 &d, uint8_t periodo, uint8_t forza,
              int led, bool presente, int durata) {
  if (!presente) { delay(durata); return; }
  digitalWrite(led, HIGH);
  /* Il comando viene riscritto ogni 20 ms per tutta la durata del colpo  */
  unsigned long t0 = millis();
  while (millis() - t0 < (unsigned long)durata) {
    assicuraLRA(d, periodo);
    d.writeRegister8(0x02, forza);
    delay(20);
  }
  d.writeRegister8(0x02, 0x00);
  digitalWrite(led, LOW);
}

void sospendiRiscaldatori() { spegni(RISC_A); spegni(RISC_B); }

/* verso 'D' = sinistra->destra (lato dominante); 'S' = destra->sinistra */
void spazzata(char verso, int colpoMs, int pausaMs) {
  if (verso == 'D') {
    colpoLRA(drvSX, periodoSX, INTENSITA_SX, LED_SX, cSX, colpoMs);
    delay(pausaMs);
    colpoLRA(drvDX, periodoDX, INTENSITA_DX, LED_DX, cDX, colpoMs);
  } else {
    colpoLRA(drvDX, periodoDX, INTENSITA_DX, LED_DX, cDX, colpoMs);
    delay(pausaMs);
    colpoLRA(drvSX, periodoSX, INTENSITA_SX, LED_SX, cSX, colpoMs);
  }
}

void messaggioSoglia() {                      // una spazzata verso il dominante
  sospendiRiscaldatori();
  spazzata('D', M1_COLPO, M1_PAUSA);
}
void messaggioBatteria() {                    // due spazzate separate da pausa
  sospendiRiscaldatori();
  spazzata('D', M2_COLPO, M2_PAUSA);
  delay(M2_TRA);
  spazzata('D', M2_COLPO, M2_PAUSA);
}
void messaggioArresto() {                     // impulsi lunghi, verso opposto
  sospendiRiscaldatori();
  for (int r = 0; r < M3_RIPET; r++) {
    spazzata('S', M3_COLPO, M3_PAUSA);
    if (r < M3_RIPET - 1) delay(M3_TRA);
  }
}

void avviaDriver() {
  cSX = drvSX.begin(&Wire);
  cDX = drvDX.begin(&Wire1);
  Serial.print(F("  LRA SX: ")); Serial.println(cSX ? F("trovato") : F("NON TROVATO"));
  Serial.print(F("  LRA DX: ")); Serial.println(cDX ? F("trovato") : F("NON TROVATO"));
  if (cSX) {
    periodoSX = PERIODO_COMUNE;
    bool ok = misuraFrequenza(drvSX, periodoSX);   // accetta solo dentro la finestra
    Serial.print(F("    SX periodo 0x")); Serial.print(periodoSX, HEX);
    Serial.println(ok ? F("  (misurato)") : F("  (misura scartata, uso il comune)"));
    configuraLRA(drvSX, periodoSX);
  }
  if (cDX) {
    periodoDX = PERIODO_COMUNE;
    bool ok = misuraFrequenza(drvDX, periodoDX);
    Serial.print(F("    DX periodo 0x")); Serial.print(periodoDX, HEX);
    Serial.println(ok ? F("  (misurato)") : F("  (misura scartata, uso il comune)"));
    configuraLRA(drvDX, periodoDX);
  }
  
  if (cSX && cDX) {
    int diff = (int)periodoDX - (int)periodoSX;
    if (diff > 4 || diff < -4) {
      Serial.println(F("    >>> Le due frequenze misurate sono troppo diverse."));
      Serial.print  (F("        Uso per tutti e due il periodo 0x"));
      Serial.println(PERIODO_COMUNE, HEX);
      periodoSX = PERIODO_COMUNE;
      periodoDX = PERIODO_COMUNE;
      configuraLRA(drvSX, periodoSX);
      configuraLRA(drvDX, periodoDX);
    }
  }
}

/* 
   5. STATO
*/
enum Stato { BLOCCATO, PRONTO, SESSIONE, DISCESA, ARRESTO };
Stato stato = BLOCCATO;
enum Fase { PRERISCALDO, RAMPA, REGIME };
Fase fase = PRERISCALDO;

int  posizioneSlitta = 1, zona = 1;
int  posizioneDiSessione = 2;
unsigned long tSessione = 0, tRampa = 0, tDiscesa = 0;
float obiettivo = T_PARTENZA;
float tInizioRampa = T_PARTENZA;           // da dove parte davvero la rampa
unsigned long rampaDurataMs = RAMPA_MS;    // calcolata all'inizio della rampa
bool sogliaAnnunciata = false, tettoAnnunciato = false;
unsigned long pompato = 0, prossimoEvento = 0;
char motivoArresto[52] = "";

/* 
   6. SLITTA
*/
int leggiGrezzo() {
  long s = 0;
  for (int i = 0; i < 8; i++) { s += analogRead(SLITTA); delayMicroseconds(200); }
  return s / 8;
}

int posizioneDaValore(int v) {
  if (v < SOGLIA_12) return 1;
  if (v < SOGLIA_23) return 2;
  return 3;
}

void aggiornaSlitta() {
  if (zonaForzata > 0) { posizioneSlitta = zonaForzata; return; }
  static int candidata = 1;
  static unsigned long da = 0;
  int p = posizioneDaValore(leggiGrezzo());
  if (p != candidata) { candidata = p; da = millis(); return; }
  if (p != posizioneSlitta && millis() - da > SLITTA_STABILE_MS) posizioneSlitta = p;
}

/* 
   7. SICUREZZA
*/
void vaiInArresto(const char* motivo) {
  spegniTutto();                     // valvola diseccitata = camera in sfiato
  strncpy(motivoArresto, motivo, sizeof(motivoArresto) - 1);
  stato = ARRESTO;
  Serial.print(F("### ARRESTO DI SICUREZZA: "));
  Serial.println(motivoArresto);
  messaggioArresto();
}

void controlliDiSicurezza() {
  if (stato == BLOCCATO || stato == ARRESTO) return;

  if (!primaLetturaFatta) {
    if (millis() - tAccensione > ATTESA_PRIMA_LETTURA_MS)
      vaiInArresto("nessuna lettura dai sensori in 15 s");
    return;
  }

  if (!plausibile(tempA)) { if (++strikeA >= STRIKE_SENSORE && zonaA_viva) {
        zonaA_viva = false; spegni(RISC_A);
        Serial.println(F("!! zona A: sensore non risponde -> zona spenta")); } }
  else strikeA = 0;

  if (!plausibile(tempB)) { if (++strikeB >= STRIKE_SENSORE && zonaB_viva) {
        zonaB_viva = false; spegni(RISC_B);
        Serial.println(F("!! zona B: sensore non risponde -> zona spenta")); } }
  else strikeB = 0;

  if (plausibile(tempA) && tempA >= T_BLOCCO) { vaiInArresto("zona A oltre 40 C"); return; }
  if (plausibile(tempB) && tempB >= T_BLOCCO) { vaiInArresto("zona B oltre 40 C"); return; }
  if (!zonaA_viva && !zonaB_viva)             { vaiInArresto("nessun sensore valido"); return; }
}

/* 
   8. CONTROLLO TERMICO a due posizioni
*/
void controlloTermico(bool consentito) {
  if (!consentito) { spegni(RISC_A); spegni(RISC_B); return; }

  if (zonaA_viva && plausibile(tempA)) {
    if (tempA < obiettivo - ISTERESI_T) accendi(RISC_A);
    else if (tempA > obiettivo)         spegni(RISC_A);
  } else spegni(RISC_A);

  if (zonaB_viva && plausibile(tempB)) {
    if (tempB < obiettivo - ISTERESI_T) accendi(RISC_B);
    else if (tempB > obiettivo)         spegni(RISC_B);
  } else spegni(RISC_B);
}

/* 
   9. PNEUMATICA
*/
void valvolaChiusa() { accendi(VALVOLA); }    // alimentata = tiene
void valvolaAperta() { spegni(VALVOLA);  }    // diseccitata = sfiata

unsigned long budgetGonfiaggio() {
  return (posizioneDiSessione == 3) ? GONFIAGGIO_POS3_MS : GONFIAGGIO_POS2_MS;
}

void soffio(unsigned long durata) {
  spegni(RISC_A); spegni(RISC_B);             // riscaldatori sospesi (D35)
  valvolaChiusa();
  accendi(POMPA);
  delay(durata);
  spegni(POMPA);
  pompato += durata;
}

void sfiato(unsigned long durata) {
  valvolaAperta();
  delay(durata);
  valvolaChiusa();
}

unsigned long fraDue(unsigned long a, unsigned long b) {
  return a + (unsigned long)random(0, (long)(b - a + 1));
}

/* 
   10. LOG
*/
const char* nomeStato() {
  switch (stato) {
    case BLOCCATO: return "BLOCCATO";
    case PRONTO:   return "PRONTO";
    case SESSIONE: return (fase == PRERISCALDO) ? "PRERISCALDO"
                        : (fase == RAMPA)       ? "RAMPA" : "REGIME";
    case DISCESA:  return "DISCESA";
    default:       return "ARRESTO";
  }
}

void log() {
  static unsigned long ultimo = 0;
  if (millis() - ultimo < 2000) return;
  ultimo = millis();
  Serial.print(millis() / 1000);        Serial.print(F("s  "));
  Serial.print(nomeStato());            Serial.print(F("  slitta="));
  Serial.print(posizioneSlitta);
  if (zonaForzata > 0) Serial.print(F("(tastiera)"));
  Serial.print(F("  obiettivo="));      Serial.print(obiettivo, 2);
  Serial.print(F("  A="));              Serial.print(tempA, 2);
  Serial.print(F("  B="));              Serial.print(tempB, 2);
  Serial.print(F("  pompato="));        Serial.print(pompato / 1000);
  Serial.print(F("s  LRA-rip="));       Serial.println(rimessiAPosto);
}

/* 
   11. TASTIERA
*/
void taraturaSlitta() {
  Serial.println(F("Taratura slitta: 12 secondi, muovila fra le tre posizioni."));
  int minimo = 4095, massimo = 0;
  unsigned long fine = millis() + 12000;
  while (millis() < fine) {
    int v = leggiGrezzo();
    if (v < minimo)  minimo = v;
    if (v > massimo) massimo = v;
    Serial.print(F("  grezzo = ")); Serial.println(v);
    delay(400);
  }
  Serial.print(F("corsa: da ")); Serial.print(minimo);
  Serial.print(F(" a "));        Serial.println(massimo);
  Serial.print(F("soglie consigliate:  SOGLIA_12 = "));
  Serial.print(minimo + (massimo - minimo) / 3);
  Serial.print(F("   SOGLIA_23 = "));
  Serial.println(minimo + 2 * (massimo - minimo) / 3);
}

void verificaCanali() {
  const char* nomi[4] = { "fascia A (13)", "fascia B (14)",
                          "pompa (16)",    "valvola (17)" };
  const int pin[4] = { RISC_A, RISC_B, POMPA, VALVOLA };
  if (stato == SESSIONE || stato == DISCESA) {
    Serial.println(F("Non durante una sessione.")); return;
  }
  Serial.println(F("\n=== VERIFICA DEI QUATTRO CANALI - capo sul tavolo ==="));
  spegniTutto(); delay(1200);
  for (int i = 0; i < 4; i++) {
    Serial.print(F("  -> ")); Serial.println(nomi[i]);
    accendi(pin[i]); delay(2000); spegni(pin[i]); delay(800);
  }
  Serial.println(F("=== fine, tutto spento ===\n"));
}

void menu() {
  Serial.println();
  Serial.println(F("  1  2  3 = i tre messaggi"));
  Serial.println(F("  s / d   = un colpo sul singolo LRA"));
  Serial.println(F("  + / -   = alza / abbassa l'intensita' del DESTRO"));
  Serial.println(F("  c       = taratura della slitta"));
  Serial.println(F("  v       = verifica dei quattro canali"));
  Serial.println(F("  r       = ricerca i driver LRA da capo"));
  Serial.println(F("  z x k   = forza zona 1 / 2 / 3 da tastiera"));
  Serial.println(F("  u       = torna alla slitta vera"));
  Serial.println(F("  f / g   = rampa piu' veloce / piu' lenta"));
  Serial.println(F("  q       = ARRESTO immediato"));
  Serial.println(F("  ?       = rivedi questo elenco"));
  Serial.println();
}

void tastiera() {
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c == '\n' || c == '\r' || c == ' ') return;

  if (c == '1') { Serial.println(F("-> messaggio SOGLIA"));   messaggioSoglia(); }
  if (c == '2') { Serial.println(F("-> messaggio BATTERIA")); messaggioBatteria(); }
  if (c == '3') { Serial.println(F("-> messaggio ARRESTO"));  messaggioArresto(); }
  if (c == 's') colpoLRA(drvSX, periodoSX, INTENSITA_SX, LED_SX, cSX, 400);
  if (c == 'd') colpoLRA(drvDX, periodoDX, INTENSITA_DX, LED_DX, cDX, 400);
  if (c == '+') { if (INTENSITA_DX < 0x7F) INTENSITA_DX += 0x10;
                  Serial.print(F("intensita' DX = 0x")); Serial.println(INTENSITA_DX, HEX); }
  if (c == '-') { if (INTENSITA_DX > 0x10) INTENSITA_DX -= 0x10;
                  Serial.print(F("intensita' DX = 0x")); Serial.println(INTENSITA_DX, HEX); }
  if (c == 'c') taraturaSlitta();
  if (c == 'v') verificaCanali();
  if (c == 'r') avviaDriver();
  if (c == 'z') { zonaForzata = 1; Serial.println(F(">>> zona 1 da tastiera")); }
  if (c == 'x') { zonaForzata = 2; Serial.println(F(">>> zona 2 da tastiera")); }
  if (c == 'k') { zonaForzata = 3; Serial.println(F(">>> zona 3 da tastiera")); }
  if (c == 'u') { zonaForzata = 0; Serial.println(F(">>> torno alla slitta vera")); }
  if (c == 'f' || c == 'g') {
    if (c == 'f' && GRADI_AL_MINUTO < 3.0) GRADI_AL_MINUTO += 0.20;
    if (c == 'g' && GRADI_AL_MINUTO > 0.20) GRADI_AL_MINUTO -= 0.20;
    Serial.print(F("pendenza = ")); Serial.print(GRADI_AL_MINUTO, 2);
    Serial.print(F(" C/min   ("));
    Serial.print((T_ARRIVO - T_PARTENZA) / GRADI_AL_MINUTO * 60.0, 0);
    Serial.println(F(" s per 5 gradi)"));
    if (fase == RAMPA) {      // ricalcola la rampa in corso
      float fatto = (float)(millis() - tRampa) / (float)rampaDurataMs;
      if (fatto > 1.0) fatto = 1.0;
      float adesso = tInizioRampa + (T_ARRIVO - tInizioRampa) * fatto;
      tInizioRampa = adesso;
      tRampa = millis();
      rampaDurataMs = (unsigned long)
          ((T_ARRIVO - tInizioRampa) / GRADI_AL_MINUTO * 60000.0);
      if (rampaDurataMs < 30000UL) rampaDurataMs = 30000UL;
      Serial.print(F("rampa ricalcolata: mancano "));
      Serial.print(rampaDurataMs / 1000); Serial.println(F(" s"));
    }
  }
  if (c == 'q') vaiInArresto("arresto chiesto da tastiera");
  if (c == '?') menu();
}

/* 
   12. MACCHINA A STATI
*/
void macchinaStati() {
  unsigned long ora = millis();

  switch (stato) {

    case BLOCCATO:
      spegniTutto();
      if (posizioneSlitta == 1) {
        stato = PRONTO;
        Serial.println(F(">>> SBLOCCATO. Pronto: muovi la slitta per cominciare."));
      }
      break;

    case PRONTO:
      spegniTutto();
      if (posizioneSlitta >= 2) {
        posizioneDiSessione = posizioneSlitta;
        tSessione = ora;
        fase = PRERISCALDO;
        obiettivo = T_PARTENZA;
        pompato = 0;
        sogliaAnnunciata = false;
        tettoAnnunciato = false;
        prossimoEvento = ora + 2000;
        stato = SESSIONE;
        valvolaChiusa();
        Serial.print(F(">>> SESSIONE avviata, posizione "));
        Serial.print(posizioneDiSessione);
        Serial.print(F(", gonfiaggio "));
        Serial.print(budgetGonfiaggio() / 1000);
        Serial.println(F(" s"));
      }
      break;

    case SESSIONE: {
      if (posizioneSlitta == 1) {
        stato = DISCESA; tDiscesa = ora;
        Serial.println(F(">>> DISCESA: rilascio progressivo"));
        break;
      }
      if (posizioneSlitta != posizioneDiSessione) posizioneDiSessione = posizioneSlitta;

      if (fase == PRERISCALDO) {
        obiettivo = T_PARTENZA;
        bool a = (!zonaA_viva) || (plausibile(tempA) && tempA >= T_PARTENZA - 0.2);
        bool b = (!zonaB_viva) || (plausibile(tempB) && tempB >= T_PARTENZA - 0.2);
        if (a && b) {
          fase = RAMPA; tRampa = ora;
          /* la rampa parte da dove si trova davvero la pelle, adesso */
          float base = T_PARTENZA;
          if (zonaA_viva && plausibile(tempA) && tempA > base) base = tempA;
          if (zonaB_viva && plausibile(tempB) && tempB > base) base = tempB;
          if (base > T_ARRIVO - 0.3) base = T_ARRIVO - 0.3;
          tInizioRampa = base;
          rampaDurataMs = (unsigned long)
              ((T_ARRIVO - tInizioRampa) / GRADI_AL_MINUTO * 60000.0);
          if (rampaDurataMs < 30000UL) rampaDurataMs = 30000UL;
          Serial.print(F(">>> comincia la RAMPA da "));
          Serial.print(tInizioRampa, 2);
          Serial.print(F(" C, durata "));
          Serial.print(rampaDurataMs / 1000);
          Serial.println(F(" s"));
        }
      } else if (fase == RAMPA) {
        float f = (float)(ora - tRampa) / (float)rampaDurataMs;
        if (f >= 1.0) { f = 1.0; fase = REGIME; Serial.println(F(">>> REGIME")); }
        obiettivo = tInizioRampa + (T_ARRIVO - tInizioRampa) * f;
      } else {
        obiettivo = T_ARRIVO;
        if (!sogliaAnnunciata) {
          sogliaAnnunciata = true;
          Serial.println(F(">>> messaggio SOGLIA RAGGIUNTA"));
          messaggioSoglia();
        }
      }

      if (ora >= prossimoEvento) {
        unsigned long budget = budgetGonfiaggio();
        if (fase != REGIME) {
          if (pompato < budget) {
            soffio(IMPULSO_GONFIAGGIO_MS);
            unsigned long n = budget / IMPULSO_GONFIAGGIO_MS;
            if (n < 1) n = 1;
            prossimoEvento = ora + (rampaDurataMs / n);
          } else prossimoEvento = ora + 2000;
        } else {
          if (pompato + RITMO_POMPA_MAX_MS < TETTO_POMPAGGIO_MS) {
            unsigned long p = fraDue(RITMO_POMPA_MIN_MS, RITMO_POMPA_MAX_MS);
            soffio(p);
            sfiato((unsigned long)(p * RITMO_SFIATO_FATTORE));
            prossimoEvento = ora + fraDue(RITMO_PAUSA_MIN_MS, RITMO_PAUSA_MAX_MS);
          } else {
            if (!tettoAnnunciato) {
              tettoAnnunciato = true;
              Serial.println(F(">>> tetto di pompaggio: il ritmo finisce, la sessione prosegue"));
            }
            prossimoEvento = ora + 5000;
          }
        }
      }

      valvolaChiusa();
      controlloTermico(true);
      break;
    }

    case DISCESA: {
      unsigned long dt = ora - tDiscesa;
      float f = (float)dt / (float)DISCESA_TERMICA_MS;
      if (f > 1.0) f = 1.0;
      obiettivo = T_ARRIVO - (T_ARRIVO - T_PARTENZA) * f;
      controlloTermico(f < 1.0);

      if (dt < DISCESA_PNEUM_MS) {
        unsigned long ciclo = 2000;
        unsigned long apertura = 200 + (unsigned long)(1700.0 * dt / DISCESA_PNEUM_MS);
        if ((dt % ciclo) < apertura) valvolaAperta(); else valvolaChiusa();
      } else valvolaAperta();

      if (dt > DISCESA_PNEUM_MS && dt > DISCESA_TERMICA_MS) {
        spegniTutto();
        stato = PRONTO;
        Serial.println(F(">>> sessione conclusa. PRONTO."));
      }
      break;
    }

    case ARRESTO: {
      spegniTutto();
      static unsigned long ultimoMessaggio = 0;
      if (ora - ultimoMessaggio > 6000) { ultimoMessaggio = ora; messaggioArresto(); }
      break;
    }
  }
}

/* 
   13. SETUP E LOOP
*/
void setup() {
  spegniTutto();                        // prima di tutto
  pinMode(LED_SX, OUTPUT); digitalWrite(LED_SX, LOW);
  pinMode(LED_DX, OUTPUT); digitalWrite(LED_DX, LOW);
  analogReadResolution(12);

  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println(F("=========================================="));
  Serial.println(F("  METAMORE - firmware del test utente"));
  Serial.println(F("=========================================="));

  sensori.begin();
  sensori.setResolution(10);
  sensori.setWaitForConversion(false);

  Wire.begin(21, 22);
  Wire.setClock(10000);
  Wire1.begin(18, 19);
  Wire1.setClock(10000);

  avviaDriver();

  tAccensione = millis();
  randomSeed(esp_random());

  Serial.print(F("  slitta, valore ora: ")); Serial.println(leggiGrezzo());
  Serial.println();
  Serial.println(F(">>> SISTEMA BLOCCATO. Porta la slitta in prima posizione."));
  Serial.println(F("    (premi ? per l'elenco dei comandi)"));
  Serial.println();
}

void loop() {
  aggiornaSlitta();
  aggiornaTemperature();
  controlliDiSicurezza();
  macchinaStati();
  tastiera();
  log();
}
