/*
 * M17 ESP8266 — firmware autonome, identique pour les deux cartes.
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Arduino IDE / CLI : ESP8266 core 3.1.2, carte LOLIN(WEMOS) D1 R2 & mini,
 * CPU 160 MHz, flash 4 Mo, MMU «16KB cache + 48KB IRAM».
 * FQBN: esp8266:esp8266:d1_mini:xtal=160,eesz=4M,mmu=4816
 * Enregistrer ce fichier sous M17Modem/M17Modem.ino puis compiler.
 * Aucune bibliothèque externe à installer : Arduino.h, ESP8266WiFi.h et
 * user_interface.h sont fournis par le core ESP8266 installé.
 *
 * Ce fichier regroupe les sources finales testées du projet M17_ESP8266.
 * Les algorithmes, commandes, données et réglages radio sont conservés.
 * Les ports USB, indicatifs et réglages de chaque carte viennent du terminal.
 * RX reste désactivée au démarrage jusqu'à HELLO / CONFIG / RX_START.
 * Version 0.2.0 : gain RX AUTO par défaut (CONFIG gain=65535), profils
 * manuels 0..59 conservés. AUTO restaure le contrôle PBUS normal du PHY,
 * suivant RH_ESP8266FSK::applyReceiveGain() dans le dépôt source ci-dessous,
 * ESP8266TDMAdaptive4FSK-0.6.0-beta.2-final/src/RH_ESP8266FSK.cpp:48-59.
 * STATUS confirme le mode PBUS ; il ne mesure pas un gain RF en dB.
 * Version 0.3.0 : correction grossiere PLL avant PTT, suivi fin borne
 * apres RRC pendant PTT, mesures CFO explicitement modulo38400Hz.
 * Aucune calibration RF absolue n'est deduite de ces seuls residus CFO.
 *
 * Primitives RF et estimateur I/Q : Guillaume / kaboom748,
 * https://github.com/kaboom748/esp8266_WLAN_PHY
 * Base étudiée : 5e67a9f150d578cce000749b9fa8db27e9f4f98d.
 * Génération fractionnaire : principe ditherBurst de SP8ESA,
 * https://github.com/SP8ESA/ESP8266_2.4GHz_SSB_TRX
 * Référence : a5bc0f3dcf3f2ef9fabe61e0bdf4ad8158166b3e, sous MIT.
 * Les notices MIT et le texte GPLv3 sont conservés à la fin de ce fichier.
 * Arduino ESP8266 et son SDK conservent leurs licences amont.
 */
#include <Arduino.h>
#include <ESP8266WiFi.h>
#if !defined(ARDUINO_ARCH_ESP8266)
#error "Installer le core ESP8266 3.1.2 et choisir la carte D1 mini."
#endif
#if !defined(F_CPU) || F_CPU != 160000000L
#error "Regler CPU Frequency sur 160 MHz."
#endif
#if !defined(MMU_IRAM_SIZE) || MMU_IRAM_SIZE != 0xC000
#error "Regler MMU sur 16KB cache + 48KB IRAM (mmu=4816)."
#endif

// ===== BEGIN ORIGINAL COMPONENT: DspCore.h =====
#include <stdint.h>
#include <string.h>
#include <math.h>

// Bounded, allocation-free M17 pulse shaping and symbol acquisition.
// This file deliberately has no Arduino dependency so the RF-independent
// timing, filter state and synchronizer are exercised on a host as well.
namespace M17Radio {
#if defined(ARDUINO_ARCH_ESP8266)
#define M17_DSP_OPT IRAM_ATTR __attribute__((optimize("O2"),noinline))
#else
#define M17_DSP_OPT
#endif
inline uint32_t dspCyclesNow();
inline int symbol(uint8_t);
inline int32_t wrapHz(int32_t);
inline uint32_t dspCyclesNow(){
#if defined(ARDUINO_ARCH_ESP8266)
  uint32_t n;__asm__ volatile("rsr %0, ccount":"=a"(n));return n;
#else
  return 0;
#endif
}
static constexpr unsigned SPS = 8, FS = 38400, SYMBOLS = 192, TAPS = 81;
static constexpr uint16_t SYNCS[5] = {0x55f7, 0xff5d, 0x75ff, 0xdf55, 0x555d};
inline int symbol(uint8_t dibit) {
  static const int8_t map[4] = {1,3,-1,-3};
  return map[dibit & 3];
}
inline int32_t wrapHz(int32_t x) {
  while (x > 19200) x -= 38400;
  while (x < -19200) x += 38400;
  return x;
}
struct PulseShape {
  int16_t rx[TAPS];
  int32_t tx[TAPS]; // Q16 tone-K units, one K = 78125 Hz.
  void begin() {
    double taps[TAPS], sum = 0;
    const double pi = 3.14159265358979323846, beta = .5;
    for (unsigned k=0; k<TAPS; ++k) {
      const double t = (int(k)-40)/8.0;
      double v;
      if (fabs(t)<1e-10) v=1+beta*(4/pi-1);
      else if (fabs(fabs(4*beta*t)-1)<1e-10)
        v=beta/sqrt(2.0)*((1+2/pi)*sin(pi/(4*beta))+(1-2/pi)*cos(pi/(4*beta)));
      else v=(sin(pi*t*(1-beta))+4*beta*t*cos(pi*t*(1+beta))) /
        (pi*t*(1-16*beta*beta*t*t));
      taps[k]=v; sum+=v;
    }
    for (unsigned k=0;k<TAPS;++k) {
      rx[k]=int16_t(lround(taps[k]*32768.0/sum));
      // Do not inherit the extra factor 1024 in the historical modem's
      // deviation conversion: q is Q16 K, not a Q26 frequency quantity.
      tx[k]=int32_t(lround(taps[k]*SPS/sum*800.0*65536.0/78125.0));
    }
  }
};
class TxShape {
public:
  void clear() { memset(history,0,sizeof(history)); head=0; phase=0; }
  bool needsSymbol() const { return phase==0; }
  int32_t M17_DSP_OPT sample(const PulseShape& f,int nextSymbol) {
    if (!phase) { head=(head+1)%11; history[head]=int8_t(nextSymbol); }
    int32_t sum=0;
    for(unsigned k=0;k<11;++k) {
      unsigned tap=phase+8*k;
      if(tap<TAPS) sum+=f.tx[tap]*history[(head+11-k)%11];
    }
    phase=(phase+1)&7;
    return sum;
  }
private:
  int8_t history[11] = {};
  uint8_t head=0,phase=0;
};
struct SoftFrame {
  uint32_t sampleCounter;
  uint8_t kind;
  int8_t symbols[SYMBOLS];
  // SYNC fit snapshot, before software AFC. Frequency remains moduloFS;
  // a narrowband estimator alone cannot resolve an absolute RF alias.
  int32_t totalCfoHz=0,afcHz=0,syncResidualHz=0,unitHz=800;
  uint32_t syncError=0;
};
class RxDsp {
public:
  typedef void (*FrameCallback)(void*,const SoftFrame&);
  struct Stats {
    uint32_t samples=0,invalid=0,frames=0,acquisitions=0,syncLost=0;
    int32_t cfoHz=0,unitHz=800,syncError=0,lastFiltered=0;
    uint32_t stageMax[8]={},stageCount[8]={},cycleHistogram[6]={};
    uint64_t stageCycles[8]={};
    uint32_t filterMax=0;
  } stats;
  void begin(const PulseShape* taps,FrameCallback callback,void* arg) {
    filter=taps; cb=callback; context=arg; reset();
  }
  void reset() {
    memset(history,0,sizeof(history)); memset(filtered,0,sizeof(filtered));
    memset(afcHistory,0,sizeof(afcHistory));cfoPending=false;
    write=0; validRun=0; locked=false; candidate=false; waitingSync=false;
    framePos=0; syncCount=0; syncScan=0; syncCopyAt=8; cfoQ8=0; cfoInitialized=false;
    circularContrast=0;circularSamples=0;
  }
  bool frameLocked() const { return locked; }
  void M17_DSP_OPT feed(int32_t hz,bool valid=true) {
    uint32_t began=dspCyclesNow();
    uint8_t stage=feedImpl(hz,valid);
#if defined(ARDUINO_ARCH_ESP8266)
    uint32_t cost=dspCyclesNow()-began;
    if(cost>stats.stageMax[stage])stats.stageMax[stage]=cost;
    ++stats.stageCount[stage];stats.stageCycles[stage]+=cost;
    unsigned bin=cost<2000?0:cost<3000?1:cost<4000?2:cost<6000?3:cost<10000?4:5;
    ++stats.cycleHistogram[bin];
#else
    (void)began;(void)stage;
#endif
  }
private:
  uint8_t M17_DSP_OPT feedImpl(int32_t hz,bool valid) {
    uint32_t filterAt=dspCyclesNow();
    const uint32_t n=stats.samples++;
    if(!valid) { ++stats.invalid; validRun=0; locked=false; candidate=false; waitingSync=false; syncCopyAt=8;cfoPending=false;circularContrast=0;circularSamples=0; }
    if(!cfoInitialized && valid) { cfoQ8=hz*256; cfoInitialized=true; }
    if(valid){
      int32_t error=wrapHz(hz-cfoQ8/256);
      // A cold receiver may see noise near0, then FSK straddling +/-Fs/2.
      // Linear averaging can remain at the antipode. Compare circular
      // residual energy at two opposite centers; one always avoids that
      // branch cut for a narrow M17 signal. No trigonometry or PLL retune.
      if((!locked&&!candidate) || error<=-9600 || error>=9600){
        const int32_t opposite=wrapHz(error+19200);
        circularContrast+=(error*error-opposite*opposite-circularContrast)/64;
        if(circularSamples<64)++circularSamples;
        if(circularSamples>=32 && circularContrast>23040000){ // (Fs/8)^2
          cfoQ8+=(cfoQ8>=0?-19200:19200)*256;
          if(locked)++stats.syncLost;
          locked=candidate=waitingSync=false;syncCopyAt=8;cfoPending=false;validRun=0;
          circularContrast=0;circularSamples=0;
          error=wrapHz(hz-cfoQ8/256);
        }
      }else if(circularSamples){circularContrast=0;circularSamples=0;}
      if(!locked && !candidate)cfoQ8+=error;
      // Continuous tracking has a213ms time constant, preserving payload
      // while following thermal drift. Reject near-Nyquist phase outliers.
      // The exact time-varying AFC/FIR baseline is retained for telemetry.
      else if(error> -12000 && error<12000)cfoQ8+=error/32;
    }
    stats.cfoHz=cfoQ8/256;
    afcHistory[n&255]=stats.cfoHz;
    const int32_t centered=wrapHz(hz-stats.cfoHz);
    history[write]=history[write+TAPS]=int16_t(valid?centered:0);
    // Duplicating the tiny history trades162bytes for a contiguous dot
    // product. Four independent accumulators remove both the per-tap wrap
    // branch and the multiply/add dependency chain on the LX106.
    int32_t a=0,b=0,c=0,d=0;
    const int16_t* x=history+write+TAPS;
    const int16_t* h=filter->rx;
    for(unsigned k=0;k<20;++k){
      a+=int32_t(h[0])*x[0];b+=int32_t(h[1])*x[-1];
      c+=int32_t(h[2])*x[-2];d+=int32_t(h[3])*x[-3];h+=4;x-=4;
    }
    int32_t sum=a+b+c+d+int32_t(h[0])*x[0];
    if(++write==TAPS)write=0;
    const int16_t y=int16_t(sum/32768);
    filtered[n&255]=y; stats.lastFiltered=y;
    uint32_t filterCost=dspCyclesNow()-filterAt;if(filterCost>stats.filterMax)stats.filterMax=filterCost;
    if(validRun<TAPS+64 && valid) ++validRun;
    if(validRun<TAPS+64) return valid?0:7;
    if(cfoPending)stepCfo();
    if(syncCopyAt<8){
      for(unsigned k=0;k<2 && syncCopyAt<8;++k,++syncCopyAt)
        frame.symbols[syncCopyAt]=soft(filtered[(syncCopyEnd-56+8*syncCopyAt)&255],syncCopyEnd-56+8*syncCopyAt);
    }

    if(waitingSync) {
      if(int32_t(n-syncEnd)>=1) {
        // LSF/Stream and Packet/BERT are opposite-polarity basis pairs.
        // Evaluate these and EOT on three successive samples, then refine
        // timing. No per-type acquisition phase is omitted.
        if(syncScan<3){
          Fit fit=matchBasis(syncEnd,syncScan);
          if(better(fit,trackedSync))trackedSync=fit;
        }else if(trackedSync.valid){
          Fit fit=matchBasis(syncEnd+(syncScan==3?-1:1),trackedSync.kind==4?2:trackedSync.kind>>1);
          if(better(fit,trackedSync))trackedSync=fit;
        }
        if(++syncScan==5){
          waitingSync=false;
          if(!trackedSync.valid) { locked=false; ++stats.syncLost; candidate=false; return 5; }
          acceptSync(trackedSync); syncCount=0;
          return 5;
        }
      }
      return 4;
    }
    if(locked) {
      if(int32_t(n-nextSymbol)<0) return 3;
      if(framePos<8) {
        ++syncCount; ++framePos; nextSymbol+=SPS;
        if(syncCount==8) { syncEnd=n; waitingSync=true; syncScan=0; trackedSync=Fit(); }
        return 3;
      }
      frame.symbols[framePos++]=soft(y,n);
      nextSymbol+=SPS;
      if(framePos==SYMBOLS) {
        ++stats.frames; if(cb)cb(context,frame);
        framePos=0; syncCount=0;
        if(frame.kind==4){locked=false;candidate=false;validRun=0;}
        return 6;
      }
      return 3;
    }
    // Both independent sync bases are tested on every sample phase. A
    // rotating type schedule can otherwise acquire two samples off-center.
    Fit fit=matchBasis(n,0),second=matchBasis(n,1);
    if(better(second,fit))fit=second;
    if(fit.valid) {
      if(!candidate) { candidate=true; decisionAt=n+7; bestCandidate=fit; }
      else if(better(fit,bestCandidate))bestCandidate=fit;
    }
    if(candidate && int32_t(n-decisionAt)>=0) {
      candidate=false; ++stats.acquisitions; locked=true; acceptSync(bestCandidate);
      return 2;
    }
    return 1;
  }
private:
  struct Fit {
    uint32_t end=0,error=0xffffffffu;
    int32_t offset=0,unit=800;
    uint8_t kind=0;
    bool valid=false;
  };
  const PulseShape* filter=nullptr;
  FrameCallback cb=nullptr;
  void* context=nullptr;
  int16_t history[TAPS*2]={},filtered[256]={};
  int32_t afcHistory[256]={};
  bool cfoPending=false;
  uint8_t cfoSymbol=0,cfoTap=0;
  int32_t cfoOrigin=0,cfoAccumulator=0,cfoSum=0,cfoCross=0,cfoLevelSum=0;
  unsigned write=0,validRun=0;
  bool locked=false,candidate=false,waitingSync=false,cfoInitialized=false;
  uint8_t framePos=0,syncCount=0,syncScan=0,syncCopyAt=8;
  uint32_t syncCopyEnd=0;
  uint32_t nextSymbol=0,decisionAt=0,syncEnd=0;
  int32_t cfoQ8=0,offset=0,unit=800,softReciprocal=2539,softBaseline=0;
  int32_t circularContrast=0;
  uint8_t circularSamples=0;
  Fit bestCandidate,trackedSync;
  SoftFrame frame{};
  void M17_DSP_OPT stepCfo(){
    // The acquisition AFC changes before lock. A current AFC plus an old
    // filtered residual has a transient bias. Reconstruct the intercept
    // of that exact AFC/FIR/SYNC epoch, with only8MACs per RF sample.
    for(unsigned work=0;work<8 && cfoPending;++work){
      const uint32_t at=syncCopyEnd-56+8*cfoSymbol;
      cfoAccumulator+=int32_t(filter->rx[cfoTap])*(afcHistory[(at-cfoTap)&255]-cfoOrigin);
      if(++cfoTap==TAPS){
        const int32_t baseline=cfoAccumulator/32768;
        const int level=symbol(uint8_t(SYNCS[frame.kind]>>(14-2*cfoSymbol)));
        cfoSum+=baseline;cfoCross+=baseline*level;cfoLevelSum+=level;
        cfoTap=0;cfoAccumulator=0;
        if(++cfoSymbol==8){
          const int32_t denominator=576-cfoLevelSum*cfoLevelSum;
          const int32_t scale=(8*cfoCross-cfoLevelSum*cfoSum)/denominator;
          frame.afcHz=cfoOrigin+(cfoSum-scale*cfoLevelSum)/8;
          frame.totalCfoHz=wrapHz(frame.afcHz+frame.syncResidualHz);
          cfoPending=false;
        }
      }
    }
  }
  int8_t M17_DSP_OPT soft(int32_t y,uint32_t at) const {
    // The expensive variable division is evaluated once at SYNC, not nine
    // times in acceptSync or once for every payload symbol.
    // The SYNC offset belongs to its own AFC epoch. AFC continues to move
    // during the payload: restore that baseline before slicing each symbol.
    // Symmetry of the81-tap RRC gives a40-sample baseline delay; the slow
    // AFC curvature error is small compared with the800Hz symbol unit.
    // Use the symbol's historical index for deferred SYNC copies as well.
    int32_t correction=wrapHz(afcHistory[(at-40)&255]-softBaseline);
    int32_t v=((y+correction-offset)*softReciprocal)/65536;
    return int8_t(v>127?127:v< -127?-127:v);
  }
  static bool M17_DSP_OPT better(const Fit& a,const Fit& b){
    return a.valid && (!b.valid || uint64_t(a.error)*uint32_t(b.unit*b.unit)<uint64_t(b.error)*uint32_t(a.unit*a.unit));
  }
  Fit M17_DSP_OPT matchBasis(uint32_t end,uint8_t basis) const {
    int32_t values[8],sum=0;
    for(unsigned i=0;i<8;++i) { values[i]=filtered[(end-56+8*i)&255]; sum+=values[i]; }
    Fit fit;
      uint8_t kind=basis==2?4:uint8_t(basis*2);
      int lsum=basis==0?6:basis==1?-6:18;
      int crossSign=basis==0?sum-2*(values[4]+values[5]+values[7]):
        basis==1?-sum+2*(values[0]+values[2]+values[3]):sum-2*values[6];
      // The two paired bases have sum(levels)=+/-6; EOT has sum18.
      const int32_t scale=basis==2?(8*crossSign-sum*6)/84:
        (8*crossSign-sum*(basis==0?2:-2))/180;
      // Preserve the discriminator's polarity. The browser resolves the
      // I/Q inversion with decoded CRC/FEC, rather than changing polarity
      // independently for every frame with a sign-ambiguous syncword.
      const int32_t magnitude=scale<0?-scale:scale;
      if(magnitude<180 || magnitude>2000)return fit;
      const int32_t center=(sum-scale*lsum)/8;
      const uint8_t negative=basis==0?0xb0:basis==1?0xf2:0x40;
      const int32_t high=center+3*scale,low=center-3*scale;
      // Measured valid-frame ratios remained below0.448. A tighter common
      // acquisition/tracking guard rejects noisy false SYNC reservations.
      uint32_t error=0,limit=uint32_t(magnitude*magnitude)*2;
      for(unsigned i=0;i<8;++i) {
        int32_t e=values[i]-(((negative>>i)&1)?low:high);
        if(e>6000 || e< -6000)return fit;
        error+=uint32_t(e*e);
        if(error>limit)return fit;
      }
      if(scale<0 && kind!=4)kind^=1;
      fit.valid=true;fit.end=end;fit.error=error;fit.offset=center;fit.unit=magnitude;fit.kind=kind;
    return fit;
  }
  void M17_DSP_OPT acceptSync(const Fit& fit) {
    offset=fit.offset; unit=fit.unit;
    // Regress the SAME eight delayed AFC samples used by the SYNC fit.
    // Keep this approximate reference fixed for the whole frame: the exact
    // telemetry baseline completes later and must not step soft decisions.
    const int32_t origin=afcHistory[(fit.end-40)&255];
    int32_t baselineSum=0,baselineCross=0,levelSum=0;
    for(unsigned i=0;i<8;++i){
      const int32_t value=wrapHz(afcHistory[(fit.end-56+8*i-40)&255]-origin);
      const int level=symbol(uint8_t(SYNCS[fit.kind]>>(14-2*i)));
      baselineSum+=value;baselineCross+=value*level;levelSum+=level;
    }
    const int32_t baselineScale=(8*baselineCross-levelSum*baselineSum)/(576-levelSum*levelSum);
    softBaseline=wrapHz(origin+(baselineSum-baselineScale*levelSum)/8);
    softReciprocal=(31*65536+unit/2)/unit;
    // Keep raw SSE here; normalize only while serializing a status record.
    stats.unitHz=unit;stats.syncError=int32_t(fit.error);
    frame.sampleCounter=fit.end-56;
    frame.kind=fit.kind;
    frame.afcHz=stats.cfoHz;frame.syncResidualHz=fit.offset;
    frame.totalCfoHz=wrapHz(stats.cfoHz+fit.offset);
    frame.unitHz=unit;frame.syncError=fit.error;
    cfoOrigin=stats.cfoHz;cfoSymbol=cfoTap=0;cfoAccumulator=cfoSum=cfoCross=cfoLevelSum=0;cfoPending=true;
    syncCopyEnd=fit.end;syncCopyAt=0;
    framePos=8; nextSymbol=fit.end+SPS;
  }
};
}
// ===== END ORIGINAL COMPONENT: DspCore.h =====

// ===== BEGIN ORIGINAL COMPONENT: PhaseLut.h =====
#include <stdint.h>
// Phase atan lookup derived from the pinned AirModem source, GPL-3.0.
static const int16_t phaseTable[257] = {
  0,41,81,122,163,204,244,285,326,367,407,448,489,529,570,610,
  651,692,732,773,813,854,894,935,975,1015,1056,1096,1136,1177,1217,1257,
  1297,1337,1377,1417,1457,1497,1537,1577,1617,1656,1696,1736,1775,1815,1854,1894,
  1933,1973,2012,2051,2090,2129,2168,2207,2246,2285,2324,2363,2401,2440,2478,2517,
  2555,2594,2632,2670,2708,2746,2784,2822,2860,2897,2935,2973,3010,3047,3085,3122,
  3159,3196,3233,3270,3307,3344,3380,3417,3453,3490,3526,3562,3599,3635,3670,3706,
  3742,3778,3813,3849,3884,3920,3955,3990,4025,4060,4095,4129,4164,4199,4233,4267,
  4302,4336,4370,4404,4438,4471,4505,4539,4572,4605,4639,4672,4705,4738,4771,4803,
  4836,4869,4901,4933,4966,4998,5030,5062,5094,5125,5157,5188,5220,5251,5282,5313,
  5344,5375,5406,5437,5467,5498,5528,5559,5589,5619,5649,5679,5708,5738,5768,5797,
  5826,5856,5885,5914,5943,5972,6000,6029,6058,6086,6114,6142,6171,6199,6227,6254,
  6282,6310,6337,6365,6392,6419,6446,6473,6500,6527,6554,6580,6607,6633,6660,6686,
  6712,6738,6764,6790,6815,6841,6867,6892,6917,6943,6968,6993,7018,7043,7068,7092,
  7117,7141,7166,7190,7214,7238,7262,7286,7310,7334,7358,7381,7405,7428,7451,7475,
  7498,7521,7544,7566,7589,7612,7635,7657,7679,7702,7724,7746,7768,7790,7812,7834,
  7856,7877,7899,7920,7942,7963,7984,8005,8026,8047,8068,8089,8110,8131,8151,8172,
  8192
};
// ===== END ORIGINAL COMPONENT: PhaseLut.h =====

// ===== BEGIN ORIGINAL COMPONENT: CarrierControl.h =====
#include <stdint.h>

#if defined(ARDUINO_ARCH_ESP8266)
#define M17_CARRIER_HOT IRAM_ATTR __attribute__((noinline))
#else
#define M17_CARRIER_HOT
#endif

namespace M17Radio {
// Local actuator only: this class does not estimate an absolute RF frequency.
// Q8 here means 1/256 of a Q16 tone-K unit (one K nominally78125Hz).
// Thus every sample uses bounded32-bit adds and comparisons, no division.
class CarrierControl {
public:
  static constexpr int32_t MAX_COARSE=2000000,MAX_FINE=12000,MAX_RATE=20000;
  static constexpr int32_t DENOMINATOR=11718750; // 78125*38400/256
  struct Command {
    uint32_t version=0,expectedBurst=0;
    int32_t coarseHz=0,fineHz=0,slopeHzPerSecond=0;
    uint32_t validForMs=0,slewHzPerSecond=0;
  };
  void M17_CARRIER_HOT clear(){
    command=Command();configured=expired=limited=active=armed=false;
    applied=target=targetCarry=slewCarry=0;age=0;
    slopeWhole=slopeRemainder=slewWhole=slewRemainder=0;
  }
  bool M17_CARRIER_HOT set(const Command& c,bool transmitting,uint32_t burst){
    if(!c.version || (command.version && int32_t(c.version-command.version)<=0) ||
       c.coarseHz< -MAX_COARSE || c.coarseHz>MAX_COARSE ||
       c.fineHz< -MAX_FINE || c.fineHz>MAX_FINE ||
       c.slopeHzPerSecond< -MAX_RATE || c.slopeHzPerSecond>MAX_RATE ||
       !c.validForMs || c.validForMs>30000 ||
       !c.slewHzPerSecond || c.slewHzPerSecond>uint32_t(MAX_RATE))return false;
    if(transmitting){
      if(!c.expectedBurst || c.expectedBurst!=burst || c.coarseHz!=command.coarseHz)return false;
    }else if(c.expectedBurst)return false;
    command=c;configured=true;expired=limited=false;active=transmitting;armed=!transmitting;
    target=fromHz(c.fineHz);if(!transmitting)applied=target;
    const int32_t slopeNumerator=c.slopeHzPerSecond*65536;
    slopeWhole=slopeNumerator/DENOMINATOR;slopeRemainder=slopeNumerator%DENOMINATOR;
    const int32_t slewNumerator=int32_t(c.slewHzPerSecond)*65536;
    slewWhole=slewNumerator/DENOMINATOR;slewRemainder=slewNumerator%DENOMINATOR;
    targetCarry=slewCarry=0;age=0;
    validSamples=c.validForMs*192/5;return true;
  }
  void M17_CARRIER_HOT start(){
    active=true;age=0;targetCarry=slewCarry=0;
    if(armed){target=applied=fromHz(command.fineHz);expired=limited=false;armed=false;}
    else if(configured){expired=true;target=applied;}
  }
  void M17_CARRIER_HOT stop(){active=false;}
  int32_t M17_CARRIER_HOT sample(){
    if(!active || !configured)return applied/256;
    if(expired)return applied/256;
    if(age++>=validSamples){expired=true;target=applied;return applied/256;}
    target+=slopeWhole;targetCarry+=slopeRemainder;
    if(targetCarry>=DENOMINATOR){++target;targetCarry-=DENOMINATOR;}
    else if(targetCarry<=-DENOMINATOR){--target;targetCarry+=DENOMINATOR;}
    constexpr int32_t limit=MAX_FINE*214+(MAX_FINE*58466)/78125;
    if(target>limit){target=limit;limited=true;}
    else if(target< -limit){target=-limit;limited=true;}
    int32_t step=slewWhole;slewCarry+=slewRemainder;
    if(slewCarry>=DENOMINATOR){++step;slewCarry-=DENOMINATOR;}
    int32_t difference=target-applied;
    if(difference>step)difference=step;else if(difference< -step)difference=-step;
    applied+=difference;return applied/256;
  }
  uint32_t version() const{return command.version;}
  int32_t coarseHz() const{return command.coarseHz;}
  int32_t M17_CARRIER_HOT fineHz() const{return int32_t(int64_t(applied)*78125/16777216);}
  uint32_t flags() const{return (configured?1u:0u)|(expired?2u:0u)|(limited?4u:0u)|(active?8u:0u);}
  int32_t fineQ8() const{return applied;} // exact state for native rate/expiry tests
private:
  static int32_t fromHz(int32_t hz){return hz*214+(hz*58466)/78125;}
  Command command;
  int32_t applied=0,target=0,targetCarry=0,slewCarry=0;
  int32_t slopeWhole=0,slopeRemainder=0,slewWhole=0,slewRemainder=0;
  uint32_t age=0,validSamples=0;
  bool configured=false,expired=false,limited=false,active=false,armed=false;
};
}
// ===== END ORIGINAL COMPONENT: CarrierControl.h =====

// ===== BEGIN ORIGINAL COMPONENT: TxBufferPolicy.h =====
namespace M17Radio {
constexpr unsigned txPrefillFrames(unsigned declared,unsigned capacity);
// Finite Stream/BERT data already exists at the host: buffer enough before
// RF starts to absorb USB/VirtualHere round-trip jitter. Open-ended live
// voice retains its short prefill. Two entries remain free for the producer.
constexpr unsigned txPrefillFrames(unsigned declared,unsigned capacity){
  const unsigned available=capacity>2?capacity-2:capacity;
  return declared?(declared<available?declared:available):(available<4?available:4);
}
}
// ===== END ORIGINAL COMPONENT: TxBufferPolicy.h =====

// ===== BEGIN ORIGINAL COMPONENT: RadioEngine.h =====
#include <Arduino.h>
#ifndef M17_REALTIME_IRQ_LOCK
#define M17_REALTIME_IRQ_LOCK 1
#endif

namespace M17Radio {
static constexpr uint16_t AUTO_GAIN=0xffffu;
struct Configuration {
  uint32_t centerHz=2437000000UL;
  int32_t rxOffsetHz=0,txOffsetHz=0;
  uint16_t gain=AUTO_GAIN;
  uint8_t attenuation=0;
};
class RadioEngine {
public:
  typedef void (*IoCallback)();
  struct Stats {
    uint32_t rxMisses=0,iqTimeouts=0,maxLateCycles=0,txFrames=0,txUnderruns=0;
    uint32_t txAborts=0,txSamples=0,lastE4=0,maxWorkCycles=0;
    uint32_t maxDspCycles=0,maxIqCycles=0,lastIqAmplitude=0;
    uint32_t maxPhaseCycles=0,irqLevelDrops=0;
    int32_t lastRawHz=0;
  } stats;
  bool begin(RxDsp::FrameCallback received,void* arg,IoCallback io);
  bool configure(const Configuration& next);
  void IRAM_ATTR listen(bool enable);
  bool IRAM_ATTR beginTransmission(uint8_t mode,uint16_t declared,uint32_t leaseMs);
  bool IRAM_ATTR enqueue(const uint8_t* normal,const uint8_t* terminal=nullptr);
  bool IRAM_ATTR endTransmission();
  void abort();
  void renewLease() { leaseAt=millis(); }
  void IRAM_ATTR serviceControl();
  void IRAM_ATTR step();
  bool carrier(int32_t hz,uint16_t durationMs);
  bool IRAM_ATTR setCarrierTrim(const CarrierControl::Command& command);
  void clearCarrierTrim(){trim.clear();trackDirty=true;}
  uint32_t txBurstId() const{return burstId;}
  uint32_t configurationId() const{return configId;}
  int32_t actualRxPllOffsetHz() const{return rxPllOffsetHz;}
  const CarrierControl& carrierTrim() const{return trim;}
  bool takeTrackDirty(){bool v=trackDirty;trackDirty=false;return v;}
  uint16_t freeEntries() const { return QUEUE-count; }
  uint16_t queuedEntries() const { return count; }
  uint8_t txState() const { return transmitting?2:preloading?1:0; }
  bool receiving() const { return listening && !transmitting; }
  bool frameLocked() const { return dsp.frameLocked(); }
  bool takeCreditsDirty() { bool b=creditsDirty; creditsDirty=false; return b; }
  bool takeTxFinished() { bool b=txFinished; txFinished=false; return b; }
  const Configuration& configuration() const { return config; }
  bool IRAM_ATTR pbusManualControl() const;
  const RxDsp::Stats& rxStats() const { return dsp.stats; }
  int16_t lastFilteredHz() const { return int16_t(dsp.stats.lastFiltered); }
  int16_t lastRawHz() const { return int16_t(stats.lastRawHz); }
private:
  static constexpr unsigned QUEUE=34;
  struct Entry { uint8_t normal[48],terminal[48]; bool paired; } queue[QUEUE];
  uint8_t read=0,write=0,count=0,mode=0,stage=0;
  uint16_t declared=0,symbolAt=0,frameSymbol=192,tailSymbols=0;
  uint8_t current[48]={};
  bool ready=false,listening=false,preloading=false,transmitting=false;
  bool endRequested=false,startPending=false,finishPending=false,terminalCommitted=false;
  bool creditsDirty=false,txFinished=false,carrierMode=false,dcReady=false;
  bool txClockStarted=false;
  bool trackDirty=false;
  uint32_t burstId=0,configId=0;
  int32_t rxPllOffsetHz=0;
  uint32_t deadline=0,fraction=0,leaseAt=0,leaseMs=3000,carrierAt=0,carrierCycles=0;
  uint32_t toneBase=0,sampleNumber=0,iqConfig=0,toneAt=0;
  int32_t code=0,error=0,previousQ=0,qOffset=0,carrierQ=0,dcI=0,dcQ=0,lastI=0,lastQ=0;
  bool previousValid=false;
  bool saved=false;
  bool realtimeLocked=false;
  uint32_t realtimePs=0;
  uint16_t savedPbus[4]={};
  Configuration config;
  CarrierControl trim;
  PulseShape pulse;
  TxShape shape;
  RxDsp dsp;
  IoCallback io=nullptr;
  bool tune(int32_t rxOffset,bool tx);
  void applyGain();
  bool setTxPath();
  bool setRxPath();
  bool start();
  void finish();
  int IRAM_ATTR nextTxSymbol();
  void IRAM_ATTR transmitSample(int32_t q,uint32_t sampleBegin);
  void IRAM_ATTR receiveSample();
  void resetDeadline();
  void enterRealtime();
  void leaveRealtime();
};
}
// ===== END ORIGINAL COMPONENT: RadioEngine.h =====

// ===== BEGIN ORIGINAL COMPONENT: SerialWire.h =====
#include <Arduino.h>
#include <string.h>

// The UART is serviced directly while the RF sampler/dither is running.
// No Arduino RX ISR, dynamic allocation or unbounded Serial.write occurs.
class SerialWire {
public:
  uint32_t crcErrors=0,formatErrors=0,uartErrors=0,droppedRecords=0;
  bool peerPaused=false;
  struct Record {
    uint8_t type,flags;
    uint16_t session,sequence,length;
    const uint8_t* payload;
  };
  void begin(){
    Serial.begin(115200,SERIAL_8N1);Serial.setDebugOutput(false);delay(10);Serial.flush();
    reg(0x6000000cu)=0;reg(0x60000010u)=0xffffffffu;
  }
  void setSession(uint16_t s){
    session=s;outRead=outWrite=outCount=0;outStage=0;pendingEscape=false;peerPaused=false;
  }
  bool IRAM_ATTR command(Record& r){
    if(!ready)return false;
    r.type=in[1];r.flags=in[2];r.session=get16(in+4);r.sequence=get16(in+6);
    r.length=get16(in+8);r.payload=in+10;return true;
  }
  void IRAM_ATTR consumed(){ready=false;inAt=0;escaped=false;inCrc=0xffffffffu;inExpected=0;}
  bool IRAM_ATTR send(uint8_t type,uint16_t seq,const uint8_t* data,uint16_t length,uint8_t flags=0,bool bulk=false){
    if(length>MAXOUT || outCount>=OUT || (bulk && outCount>=OUT-2)){++droppedRecords;return false;}
    Out& r=out[outWrite];r.type=type;r.flags=flags;r.seq=seq;r.length=length;r.session=session;
    r.bytes=r.payload;if(length)memcpy(r.payload,data,length);
    outWrite=(outWrite+1)%OUT;++outCount;return true;
  }
  // STATUS owns one fixed buffer and keeps it immutable until queueIdle().
  // This internal zero-copy path changes no wire format or normal record.
  bool IRAM_ATTR sendBorrowedStatus(const uint8_t* data,uint16_t length){
    if(length>MAXOUT || outCount>=OUT)return false;
    Out& r=out[outWrite];r.type=13;r.flags=0;r.seq=65535;r.length=length;r.session=session;r.bytes=data;
    outWrite=(outWrite+1)%OUT;++outCount;return true;
  }
  bool queueIdle() const{return outCount==0;}
  bool IRAM_ATTR pending(uint8_t type,uint16_t sequence,uint16_t wantedSession) const{
    unsigned at=outRead;
    for(unsigned remaining=outCount;remaining;--remaining){
      const Out& record=out[at];
      if(record.type==type && record.seq==sequence && record.session==wantedSession)return true;
      at=(at+1)%OUT;
    }
    return false;
  }
  bool sendText(uint8_t type,uint16_t seq,const char* text){return send(type,seq,reinterpret_cast<const uint8_t*>(text),uint16_t(strlen(text)));}
  void IRAM_ATTR service(){
    uint32_t status=reg(0x6000001cu);
    if(!ready && (status&255u))accept(uint8_t(reg(0x60000000u)));
    uint32_t errors=reg(0x60000004u)&0x1cu;
    if(errors){++uartErrors;reg(0x60000010u)=errors;}
    if(((reg(0x6000001cu)>>16)&255u)>=120 || peerPaused)return;
    if(pendingEscape){putByte(escapedByte);pendingEscape=false;return;}
    if(!outCount)return;
    Out& r=out[outRead];
    if(!outStage){
      header[0]=1;header[1]=r.type;header[2]=r.flags;header[3]=0;
      put16(header+4,r.session);put16(header+6,r.seq);put16(header+8,r.length);
      outAt=0;outCrc=0xffffffffu;outStage=1;putByte(0x7e);return;
    }
    if(outStage==3){putByte(0x7e);outRead=(outRead+1)%OUT;--outCount;outStage=0;return;}
    uint8_t b;
    if(outAt<10+r.length){b=outAt<10?header[outAt]:r.bytes[outAt-10];outCrc=crcByte(outCrc,b);}
    else b=uint8_t((outCrc^0xffffffffu)>>(8*(outAt-10-r.length)));
    ++outAt;if(outAt==14+r.length)outStage=3;
    if(needsEscape(b)){putByte(0x7d);pendingEscape=true;escapedByte=b^0x20;}
    else putByte(b);
  }
  static uint16_t get16(const uint8_t* p){return p[0]|uint16_t(p[1])<<8;}
  static uint32_t get32(const uint8_t* p){return uint32_t(get16(p))|uint32_t(get16(p+2))<<16;}
  static void put16(uint8_t* p,uint16_t v){p[0]=uint8_t(v);p[1]=uint8_t(v>>8);}
  static void put32(uint8_t* p,uint32_t v){put16(p,uint16_t(v));put16(p+2,uint16_t(v>>16));}
private:
  static constexpr unsigned OUT=8,MAXOUT=1024;
  struct Out {uint8_t type,flags;uint16_t seq,length,session;const uint8_t* bytes;uint8_t payload[MAXOUT];}out[OUT];
  uint8_t outRead=0,outWrite=0,outCount=0,outStage=0;
  uint16_t session=0,outAt=0,inAt=0,inExpected=0;
  uint32_t inCrc=0xffffffffu,outCrc=0xffffffffu;
  uint8_t in[1038]={},header[10]={},escapedByte=0;
  bool ready=false,escaped=false,pendingEscape=false;
  static inline volatile uint32_t& reg(uint32_t a){return *reinterpret_cast<volatile uint32_t*>(a);}
  static inline IRAM_ATTR __attribute__((always_inline)) void putByte(uint8_t b){
    reg(0x60000000u)=b;__asm__ volatile("memw" ::: "memory");
  }
  static bool needsEscape(uint8_t b){return b==0x7e||b==0x7d||b==0x11||b==0x13;}
  static inline __attribute__((always_inline)) uint32_t crcByte(uint32_t c,uint8_t b){c^=b;for(unsigned i=0;i<8;++i)c=(c>>1)^((0u-(c&1))&0xedb88320u);return c;}
  void IRAM_ATTR accept(uint8_t b){
    if(b==0x11){peerPaused=false;return;}if(b==0x13){peerPaused=true;return;}
    if(b==0x7e){
      if(inAt){
        if(!escaped && inAt>=14 && inExpected && inAt==inExpected && in[0]==1 && in[3]==0){
          if((inCrc^0xffffffffu)==get32(in+inAt-4))ready=true;else ++crcErrors;
        }else ++formatErrors;
      }
      if(!ready){inAt=0;inExpected=0;inCrc=0xffffffffu;}
      escaped=false;return;
    }
    if(b==0x7d && !escaped){escaped=true;return;}
    if(escaped){b^=0x20;escaped=false;}
    if(inAt>=sizeof(in)){inAt=0;inExpected=0;inCrc=0xffffffffu;++formatErrors;return;}
    in[inAt++]=b;
    if(inAt==10){uint16_t len=get16(in+8);if(len>1024){inAt=0;inExpected=0;inCrc=0xffffffffu;++formatErrors;return;}inExpected=14+len;}
    if(inAt<=10 || !inExpected || inAt<=inExpected-4)inCrc=crcByte(inCrc,b);
  }
};
// ===== END ORIGINAL COMPONENT: SerialWire.h =====

// ===== BEGIN ORIGINAL COMPONENT: StatusJson.h =====
#include <stdint.h>
#include <stddef.h>

#if defined(ARDUINO_ARCH_ESP8266)
#define M17_STATUS_HOT IRAM_ATTR __attribute__((noinline))
#define M17_STATUS_INLINE inline __attribute__((always_inline))
#else
#define M17_STATUS_HOT
#define M17_STATUS_INLINE inline
#endif

// Incremental status serialization. A tick emits at most one character and
// decimal conversion uses at most nine subtractions, with no division,
// allocation, printf or library call in the active radio foreground.
namespace M17Status {
enum Scalar : uint8_t {
  RX,TX_STATE,LOCKED,RX_FRAMES,TX_FRAMES,SAMPLES,RX_MISSES,IQ_TIMEOUTS,
  LATE_CYCLES,WORK_CYCLES,DSP_CYCLES,IQ_CYCLES,PHASE_CYCLES,FILTER_CYCLES,
  IRQ_DROPS,RAW_HZ,IQ_AMPLITUDE,CFO_HZ,UNIT_HZ,SYNC_SCORE,E4,UNDERRUNS,
  TX_ABORTS,UART_ERRORS,SERIAL_DROPS,CRC_ERRORS,RX_GAIN_PROFILE,RX_GAIN_AUTO,
  RX_PBUS_MANUAL,RX_AGC_ACTIVE,FREE_HEAP,SCALAR_COUNT
};
struct Snapshot {
  uint32_t scalar[SCALAR_COUNT]={};
  uint32_t dspMax[8]={},dspN[8]={},dspHist[6]={};
};
struct Property {const char* name;uint8_t kind;};
// kind: unsigned0, signed1, boolean2, arrays3/4/5.
static const Property PROPERTIES[] = {
  {"rx",2},{"txState",0},{"locked",2},{"rxFrames",0},{"txFrames",0},
  {"samples",0},{"rxMisses",0},{"iqTimeouts",0},{"lateCycles",0},
  {"workCycles",0},{"dspCycles",0},{"iqCycles",0},{"phaseCycles",0},
  {"filterCycles",0},{"irqDrops",0},{"rawHz",1},{"iqAmplitude",0},
  {"cfoHz",1},{"unitHz",1},{"syncScore",1},{"e4",0},{"underruns",0},
  {"txAborts",0},{"uartErrors",0},{"serialDrops",0},{"crcErrors",0},
  {"rxGainProfile",0},{"rxGainAuto",2},{"rxPbusManual",2},{"rxAgcActive",2},
  {"freeHeap",0},{"dspMax",3},{"dspN",4},{"dspHist",5}
};
static const uint32_t POWERS[10]={1000000000UL,100000000UL,10000000UL,
  1000000UL,100000UL,10000UL,1000UL,100UL,10UL,1UL};

class UnsignedDivide {
public:
  void M17_STATUS_HOT begin(uint32_t n,uint32_t d){numerator=n;denominator=d?d:1;remainder=quotient=0;bit=32;}
  bool M17_STATUS_HOT step(){
    if(!bit)return true;
    --bit;
    // The status denominator is <=3907; the remainder cannot overflow.
    remainder=(remainder<<1)|((numerator>>bit)&1);
    if(remainder>=denominator){remainder-=denominator;quotient|=uint32_t(1)<<bit;}
    return !bit;
  }
  uint32_t value() const{return quotient;}
private:
  uint32_t numerator=0,denominator=1,remainder=0,quotient=0;
  uint8_t bit=0;
};

class JsonBuilder {
public:
  static constexpr size_t CAPACITY=1024;
  Snapshot values;
  void M17_STATUS_HOT begin(bool withHeap){
    includeHeap=withHeap;position=0;buffer[0]=0;property=0;state=OPEN;
    first=true;active=true;finished=false;failed=false;array=false;
  }
  void M17_STATUS_HOT cancel(){active=false;finished=false;failed=false;position=0;buffer[0]=0;}
  bool busy() const{return active;}
  bool ready() const{return finished && !failed;}
  bool overflow() const{return failed;}
  const uint8_t* data() const{return reinterpret_cast<const uint8_t*>(buffer);}
  const char* text() const{return buffer;}
  uint16_t length() const{return position;}
  void M17_STATUS_HOT step(){
    if(!active)return;
    switch(state){
      case OPEN:append('{');state=NEXT;break;
      case NEXT:
        if(property==FREE_HEAP && !includeHeap){++property;break;}
        if(property>=sizeof(PROPERTIES)/sizeof(PROPERTIES[0])){append('}');active=false;finished=!failed;break;}
        cursor=0;state=first?KEY_QUOTE:COMMA;first=false;break;
      case COMMA:append(',');state=KEY_QUOTE;break;
      case KEY_QUOTE:append('"');state=KEY;break;
      case KEY:
        if(PROPERTIES[property].name[cursor])append(PROPERTIES[property].name[cursor++]);
        else state=KEY_END;
        break;
      case KEY_END:append('"');state=COLON;break;
      case COLON:append(':');state=VALUE;break;
      case VALUE:{
        uint8_t kind=PROPERTIES[property].kind;
        if(kind>=3){array=true;arrayAt=0;append('[');state=ARRAY_NEXT;}
        else if(kind==2){literal=values.scalar[property]?"true":"false";cursor=0;state=LITERAL;}
        else startNumber(values.scalar[property],kind==1);
        break;
      }
      case LITERAL:
        if(literal[cursor])append(literal[cursor++]);else completeValue();
        break;
      case NUMBER_SIGN:append('-');state=NUMBER;break;
      case NUMBER:{
        uint8_t digit=0;uint32_t power=POWERS[powerAt];
        while(number>=power){number-=power;++digit;}
        if(digit || !leading || powerAt==9){append(char('0'+digit));leading=false;}
        if(++powerAt==10)completeValue();
        break;
      }
      case ARRAY_NEXT:{
        uint8_t count=property==SCALAR_COUNT+2?6:8;
        if(arrayAt==count){append(']');array=false;++property;state=NEXT;}
        else if(arrayAt)state=ARRAY_COMMA;
        else startArrayNumber();
        break;
      }
      case ARRAY_COMMA:append(',');startArrayNumber();break;
    }
  }
private:
  enum State:uint8_t {OPEN,NEXT,COMMA,KEY_QUOTE,KEY,KEY_END,COLON,VALUE,
    LITERAL,NUMBER_SIGN,NUMBER,ARRAY_NEXT,ARRAY_COMMA};
  char buffer[CAPACITY]={};
  uint16_t position=0;
  uint8_t property=0,state=OPEN,cursor=0,arrayAt=0,powerAt=0;
  uint32_t number=0;
  const char* literal=nullptr;
  bool includeHeap=false,first=true,active=false,finished=false,failed=false,array=false,leading=true;
  M17_STATUS_INLINE void append(char c){
    if(position>=CAPACITY-1){failed=true;active=false;return;}
    buffer[position++]=c;buffer[position]=0;
  }
  M17_STATUS_INLINE void startNumber(uint32_t raw,bool signedValue){
    bool negative=signedValue && (raw&0x80000000u);
    number=negative?0u-raw:raw;powerAt=0;leading=true;
    state=negative?NUMBER_SIGN:NUMBER;
  }
  M17_STATUS_INLINE void startArrayNumber(){
    uint32_t n=property==SCALAR_COUNT?values.dspMax[arrayAt]:
      property==SCALAR_COUNT+1?values.dspN[arrayAt]:values.dspHist[arrayAt];
    startNumber(n,false);
  }
  M17_STATUS_INLINE void completeValue(){
    if(array){++arrayAt;state=ARRAY_NEXT;}
    else{++property;state=NEXT;}
  }
};
}
// ===== END ORIGINAL COMPONENT: StatusJson.h =====

// ===== BEGIN ORIGINAL COMPONENT: HelloReply.h =====
#include <stdint.h>
#include <string.h>
#if defined(ARDUINO_ARCH_ESP8266)
#define M17_HELLO_HOT IRAM_ATTR __attribute__((noinline))
#else
#define M17_HELLO_HOT
#endif

// HELLO is retried before capabilities are known, so its reply cache cannot
// depend on negotiated controlRetry. Preserve a partially emitted record.
class HelloReply {
public:
  static constexpr unsigned CAPACITY=960;
  void remember(uint16_t session,uint16_t sequence,const char* bytes,uint16_t length){
    cachedSession=session;cachedSequence=sequence;cachedLength=length;
    valid=session && length && length<=CAPACITY;
    if(valid)memcpy(payload,bytes,length);
  }
  template<class Wire>
  bool M17_HELLO_HOT repeat(uint16_t session,uint16_t sequence,Wire& wire) const {
    if(!valid || session!=cachedSession || sequence!=cachedSequence)return false;
    // A response still queued/in flight must finish unchanged. After a
    // lost response, replay cached bytes without resetting radio/session
    // or mutating the partial output record. Queue full is retriable.
    if(!wire.pending(2,sequence,session))wire.send(2,sequence,payload,cachedLength);
    return true;
  }
private:
  uint8_t payload[CAPACITY]={};
  uint16_t cachedSession=0,cachedSequence=0,cachedLength=0;
  bool valid=false;
};
// ===== END ORIGINAL COMPONENT: HelloReply.h =====

// ===== BEGIN ORIGINAL COMPONENT: RadioEngine.cpp =====
#include <ESP8266WiFi.h>
extern "C" {
#include "user_interface.h"
extern uint8_t chip6_phy_init_ctrl[];
void set_rf_freq_offset(int,int,int);
#ifdef NONOSDK22x_190703
void pp_disable_noise_timer(void);
#endif
}
namespace M17Radio {
namespace {
constexpr uint32_t TONE=0x600005b8u,IQCTRL=0x6000057cu,IREG=0x600005dcu,QREG=0x600005e0u;
constexpr uint32_t E4REG=0x600005e4u,RXCTRL=0x60009b08u,PCMD=0x60000594u,PSTAT=0x600005a0u;
constexpr uint32_t GATE=0x40000u,FIELDS=0x7ffffu,RXSTOP=0x08000000u;
inline volatile uint32_t& reg(uint32_t);
inline void fence();
bool pbusWrite(uint8_t,uint8_t,uint16_t);
inline uint32_t magnitude(int32_t);
int32_t IRAM_ATTR phaseHz(int32_t,int32_t,int32_t,int32_t);
inline int16_t shrink(int32_t);
inline volatile uint32_t& reg(uint32_t a){return *reinterpret_cast<volatile uint32_t*>(a);}
inline void fence(){__asm__ volatile("memw" ::: "memory");}
typedef uint32_t(*PbusRead)(uint32_t,uint32_t);
typedef void(*PbusForce)(uint32_t,uint32_t,uint32_t);
typedef void(*PbusMode)();
typedef void(*SetTxClk)(int);
typedef uint8_t(*SetScale)(uint8_t);
const PbusRead pbusRead=reinterpret_cast<PbusRead>(0x400074d8u);
const PbusForce pbusForce=reinterpret_cast<PbusForce>(0x4000747cu);
const PbusMode pbusDebug=reinterpret_cast<PbusMode>(0x4000737cu);
const PbusMode pbusWork=reinterpret_cast<PbusMode>(0x40007648u);
const SetTxClk setTxClk=reinterpret_cast<SetTxClk>(0x4000650cu);
const SetScale setScale=reinterpret_cast<SetScale>(0x4000678cu);
const uint8_t selectors[4]={2,7,1,6};
bool pbusWrite(uint8_t sel,uint8_t bank,uint16_t value){
  uint32_t v=reg(PCMD)&0xffff0001u;
  v|=uint32_t(bank&3)<<14;v|=uint32_t(value&511)<<5;v|=uint32_t(sel&7)<<2;v|=2;
  reg(PCMD)=v;fence();uint32_t began=micros();
  while(reg(PSTAT)&0x80000000u){if(uint32_t(micros()-began)>2500){reg(PCMD)&=~2u;return false;}delay(0);}
  reg(PCMD)&=~2u;fence();return true;
}
inline uint32_t magnitude(int32_t x){return x<0?0u-uint32_t(x):uint32_t(x);}
int32_t IRAM_ATTR phaseHz(int32_t ai,int32_t aq,int32_t bi,int32_t bq){
  uint32_t peak=magnitude(ai)|magnitude(aq)|magnitude(bi)|magnitude(bq);
  int shift=peak?19-__builtin_clz(peak):0;
  if(shift>0){ai>>=shift;aq>>=shift;bi>>=shift;bq>>=shift;}
  int32_t cross=ai*bq-aq*bi,dot=ai*bi+aq*bq;
  uint32_t x=magnitude(dot),y=magnitude(cross),hi=x>y?x:y,lo=x>y?y:x;
  if(!hi)return 0;
  if(hi>65535){unsigned s=16-__builtin_clz(hi);lo>>=s;hi>>=s;}
  uint32_t ratio=(lo<<16)/hi;unsigned index=ratio>>8;
  int32_t angle=phaseTable[index];
  if(index<256)angle+=((phaseTable[index+1]-angle)*int32_t(ratio&255)+128)/256;
  if(y>x)angle=16384-angle;if(dot<0)angle=32768-angle;if(cross<0)angle=-angle;
  return angle*75/128;
}
inline int16_t shrink(int32_t v){v/=1024;return int16_t(v>32767?32767:v< -32768?-32768:v);}
}
bool RadioEngine::begin(RxDsp::FrameCallback callback,void* arg,IoCallback service){
  if(ESP.getCpuFreqMHz()!=160)return false;
  io=service;pulse.begin();dsp.begin(&pulse,callback,arg);shape.clear();
  WiFi.persistent(false);wifi_station_set_auto_connect(0);
  if(!WiFi.mode(WIFI_STA))return false;
  wifi_station_disconnect();if(!wifi_set_sleep_type(NONE_SLEEP_T))return false;
  if(!wifi_set_channel(6))return false;delay(100);
#ifdef NONOSDK22x_190703
  pp_disable_noise_timer();
#endif
  ready=tune(config.rxOffsetHz,false)&&setRxPath();listening=false;resetDeadline();return ready;
}
bool RadioEngine::configure(const Configuration& c){
  if(transmitting || preloading || c.centerHz<2412000000UL || c.centerHz>2462000000UL ||
      c.rxOffsetHz< -2000000 || c.rxOffsetHz>2000000 || c.txOffsetHz< -12000 ||
      c.txOffsetHz>12000 || (c.gain!=AUTO_GAIN && c.gain>59) || c.attenuation>127)return false;
  leaveRealtime();config=c;trim.clear();++configId;trackDirty=true;previousValid=false;dcReady=false;dsp.reset();
  ready=tune(config.rxOffsetHz,false)&&setRxPath();resetDeadline();if(ready&&listening)enterRealtime();return ready;
}
bool RadioEngine::tune(int32_t offset,bool tx){
  const int64_t hz=int64_t(config.centerHz)+offset;
  int channel=int((int64_t(config.centerHz)-2407000000LL+2500000)/5000000);
  if(channel<1)channel=1;if(channel>11)channel=11;
  const int baseMHz=2407+5*channel;
  const int64_t difference=hz-int64_t(baseMHz)*1000000;
  const int units=int((difference*1024+(difference<0?-500000:500000))/1000000);
  if(units< -4608 || units>4608 || chip6_phy_init_ctrl[1]>2)return false;
  if(!wifi_set_channel(uint8_t(channel)))return false;
  set_rf_freq_offset(int(chip6_phy_init_ctrl[1]),baseMHz,units);delay(5);
  if(tx){
    // The desired PLL center includes coarse TX calibration. Refer the
    // quantization residual to that center, never cancel the coarse trim.
    const int32_t residual=int32_t(hz-int64_t(baseMHz)*1000000-int64_t(units)*1000000/1024);
    qOffset=int32_t(int64_t(config.txOffsetHz+residual)*65536/78125);
  }else rxPllOffsetHz=int32_t(int64_t(baseMHz)*1000000+int64_t(units)*1000000/1024-int64_t(config.centerHz));
  return true;
}
void RadioEngine::applyGain(){
  // Restore PHY-controlled gain exactly as the upstream /gain agc path:
  // kaboom748/esp8266_WLAN_PHY @5e67a9f150d578cce000749b9fa8db27e9f4f98d,
  // ESP8266TDMAdaptive4FSK-0.6.0-beta.2-final/src/RH_ESP8266FSK.cpp:48-59.
  // This is PBUS work mode, distinct from the misleading CCA wrappers
  // historically named phy_enable_agc()/phy_disable_agc().
  if(config.gain==AUTO_GAIN){pbusWork();return;}
  unsigned bb=config.gain%30;
  uint16_t coarse=bb<6?0:bb<12?0x40:bb<18?0x60:bb<24?0x70:0x78;
  pbusDebug();pbusForce(2,1,config.gain<30?0x1c6:0x1fe);
  pbusForce(3,1,coarse);pbusForce(3,2,6u|((bb%6)<<3));
}
bool IRAM_ATTR RadioEngine::pbusManualControl() const{return (reg(PCMD)&1u)!=0;}
bool RadioEngine::setTxPath(){
  if(!saved){for(unsigned i=0;i<4;++i)savedPbus[i]=uint16_t(pbusRead(selectors[i],1));saved=true;}
  reg(RXCTRL)|=RXSTOP;reg(PCMD)|=1;fence();
  if(!(pbusWrite(2,1,1)&&pbusWrite(7,1,95)&&pbusWrite(1,1,127)&&pbusWrite(6,1,127)))return false;
  setScale(0);setTxClk(1);
  reg(TONE)&=~FIELDS;reg(0x600005bcu)&=~GATE;reg(0x600005c4u)&=~GATE;fence();
  toneBase=reg(TONE)&~FIELDS;return true;
}
bool RadioEngine::setRxPath(){
  reg(TONE)&=~GATE;reg(0x600005bcu)&=~GATE;reg(0x600005c4u)&=~GATE;fence();setTxClk(0);
  if(saved){for(unsigned i=0;i<4;++i)if(!pbusWrite(selectors[i],1,savedPbus[i]))return false;saved=false;}
  if(!pbusWrite(2,1,0x1fe))return false;pbusWork();applyGain();
  reg(RXCTRL)&=~RXSTOP;
  iqConfig=(reg(IQCTRL)&0x7ff80000u)|(127u<<2)|0x40000u;
  reg(IQCTRL)=iqConfig;fence();return true;
}
void RadioEngine::resetDeadline(){deadline=ESP.getCycleCount()+2000;fraction=0;}
void RadioEngine::enterRealtime(){
#if M17_REALTIME_IRQ_LOCK
  if(!realtimeLocked){__asm__ volatile("rsil %0, 15":"=a"(realtimePs)::"memory");realtimeLocked=true;}
#endif
}
void RadioEngine::leaveRealtime(){
#if M17_REALTIME_IRQ_LOCK
  if(realtimeLocked){realtimeLocked=false;__asm__ volatile("wsr %0, ps; rsync"::"a"(realtimePs):"memory");}
#endif
}
void IRAM_ATTR RadioEngine::listen(bool on){
  if(!ready)return;
  // A local receive preference must never reset the active TX clock.
  if(transmitting){listening=on;return;}
  if(!on)leaveRealtime();
  listening=on;previousValid=false;dcReady=false;dsp.reset();resetDeadline();
  if(on)enterRealtime();
}
bool IRAM_ATTR RadioEngine::beginTransmission(uint8_t m,uint16_t frames,uint32_t lease){
  if(!ready || transmitting || m>2 || frames>65534 || (m==0 && (!frames || frames>QUEUE)))return false;
  read=write=count=0;mode=m;declared=frames;leaseMs=lease<1000?1000:lease>30000?30000:lease;
  leaseAt=millis();preloading=true;endRequested=false;terminalCommitted=false;startPending=false;
  creditsDirty=true;return true;
}
bool IRAM_ATTR RadioEngine::enqueue(const uint8_t* normal,const uint8_t* terminal){
  if((!preloading&&!transmitting)||terminalCommitted||count==QUEUE)return false;
  memcpy(queue[write].normal,normal,48);queue[write].paired=terminal!=nullptr;
  if(terminal)memcpy(queue[write].terminal,terminal,48);
  write=(write+1)%QUEUE;++count;creditsDirty=true;
  if(preloading && mode!=0 && count>=txPrefillFrames(declared,QUEUE))startPending=true;
  return true;
}
bool IRAM_ATTR RadioEngine::endTransmission(){
  if(!preloading&&!transmitting)return false;
  if(mode==0 && preloading && count!=declared)return false;
  endRequested=true;if(preloading && count)startPending=true;return true;
}
bool RadioEngine::start(){
  leaveRealtime();if(!tune(trim.coarseHz(),true)||!setTxPath()){if(listening)enterRealtime();return false;}
  if(!++burstId)++burstId;trim.start();trackDirty=true;
  preloading=false;transmitting=true;carrierMode=false;stage=0;symbolAt=0;frameSymbol=192;
  tailSymbols=0;sampleNumber=0;txClockStarted=false;code=error=previousQ=0;shape.clear();finishPending=false;resetDeadline();toneAt=ESP.getCycleCount();enterRealtime();return true;
}
void RadioEngine::finish(){
  reg(TONE)&=~GATE;fence();leaveRealtime();transmitting=false;preloading=false;carrierMode=false;
  trim.stop();trackDirty=true;
  count=read=write=0;finishPending=false;startPending=false;txFinished=true;creditsDirty=true;
  ready=tune(config.rxOffsetHz,false)&&setRxPath();previousValid=false;dcReady=false;dsp.reset();resetDeadline();if(ready&&listening)enterRealtime();
}
void RadioEngine::abort(){
  ++stats.txAborts;if(transmitting)finish();
  else{preloading=false;read=write=count=0;startPending=false;creditsDirty=true;}
}
void IRAM_ATTR RadioEngine::serviceControl(){
  if((transmitting||preloading) && uint32_t(millis()-leaseAt)>leaseMs){abort();return;}
  if(finishPending){finish();return;}
  if(startPending){startPending=false;if(!start()){++stats.txAborts;preloading=false;count=0;creditsDirty=true;}}
}
bool RadioEngine::carrier(int32_t hz,uint16_t duration){
  if(!ready||transmitting||preloading||hz< -12000||hz>12000||!duration||duration>2000)return false;
  leaveRealtime();if(!tune(trim.coarseHz(),true)||!setTxPath()){if(listening)enterRealtime();return false;}
  if(!++burstId)++burstId;trim.start();trackDirty=true;
  transmitting=true;carrierMode=true;carrierQ=qOffset+int32_t(int64_t(hz)*65536/78125);
  carrierAt=ESP.getCycleCount();carrierCycles=uint32_t(duration)*160000;
  code=error=previousQ=0;sampleNumber=0;txClockStarted=false;leaseAt=millis();leaseMs=3000;resetDeadline();toneAt=ESP.getCycleCount();enterRealtime();return true;
}
bool IRAM_ATTR RadioEngine::setCarrierTrim(const CarrierControl::Command& command){
  if(preloading)return false;
  if(!trim.set(command,transmitting,burstId))return false;
  trackDirty=true;return true;
}
int IRAM_ATTR RadioEngine::nextTxSymbol(){
  if(stage==0){
    if(symbolAt<192){int sign=((symbolAt++&1)?-3:3);return mode==2?-sign:sign;}
    stage=1;frameSymbol=192;
  }
  if(stage==1){
    if(frameSymbol==192){
      if(!count){if(!endRequested)++stats.txUnderruns;stage=2;symbolAt=0;}
      else{
        Entry& e=queue[read];
        bool terminal=e.paired && count==1 && (endRequested || mode==1);
        memcpy(current,terminal?e.terminal:e.normal,48);
        if(terminal){endRequested=true;terminalCommitted=true;}
        read=(read+1)%QUEUE;--count;frameSymbol=0;creditsDirty=true;
      }
    }
    if(stage==1){
      unsigned i=frameSymbol++;int s=symbol((current[i>>2]>>(6-2*(i&3)))&3);
      if(frameSymbol==192)++stats.txFrames;
      return s;
    }
  }
  if(stage==2){
    if(symbolAt<192){unsigned i=symbolAt++;return symbol((0x555du>>(14-2*(i&7)))&3);}
    stage=3;tailSymbols=0;
  }
  if(stage==3){if(++tailSymbols>12)finishPending=true;return 0;}
  return 0;
}
void IRAM_ATTR RadioEngine::transmitSample(int32_t q,uint32_t sampleBegin){
  uint32_t ps;__asm__ volatile("rsil %0, 15":"=a"(ps)::"memory");
  uint32_t last=toneAt;
  if(!txClockStarted){
    // No RF phase exists while the gate was off during SDK/setup/cache work.
    last=ESP.getCycleCount();previousQ=q;
  }else{
    // Split the desired phase integral at the fixed sample boundary. This
    // also corrects any final previous-sample write which overhung it.
    error+=(previousQ-q)*int32_t(sampleBegin-last);
  }
  const int32_t nearest=(q+32768)>>16;
  unsigned ask=config.attenuation;
  if(!carrierMode && sampleNumber<32){unsigned edgeAsk=(32-sampleNumber)*4;if(edgeAsk>ask)ask=edgeAsk>127?127:edgeAsk;}
  if(!carrierMode && stage==3 && tailSymbols>10){unsigned edgeAsk=(tailSymbols-10)*63;if(edgeAsk>ask)ask=edgeAsk>127?127:edgeAsk;}
  const uint32_t amplitude=GATE|(uint32_t(uint8_t(0u-ask))<<10);
  do{
    uint32_t now=ESP.getCycleCount(),dt=now-last;
    if(dt>8192){++stats.txUnderruns;finishPending=true;break;}
    int32_t accumulated=error+(q-code*65536)*int32_t(dt);
    if(accumulated>(20<<16)||accumulated<-(20<<16)){
      bool up=accumulated>0;int32_t residual=q-nearest*65536;
      int32_t rate=up?65536-residual:65536+residual;
      int32_t length=int32_t((up?accumulated:-accumulated)/rate);
      int32_t remaining=int32_t(deadline-now);if(length>remaining)length=remaining;if(length>4000)length=4000;
      if(length>0){
        reg(TONE)=toneBase|(uint32_t(nearest+(up?1:-1))&1023)|amplitude;fence();
        uint32_t pulseAt=ESP.getCycleCount();
        if(pulseAt-last>8192){++stats.txUnderruns;finishPending=true;break;}
        error+=(q-code*65536)*int32_t(pulseAt-last);code=nearest+(up?1:-1);last=pulseAt;
        while(int32_t(ESP.getCycleCount()-pulseAt)<length){}
      }
    }
    reg(TONE)=toneBase|(uint32_t(nearest)&1023)|amplitude;fence();
    uint32_t written=ESP.getCycleCount();
    if(written-last>8192){++stats.txUnderruns;finishPending=true;break;}
    int32_t next=error+(q-code*65536)*int32_t(written-last);
    error=next>(1<<29)?(1<<29):next<-(1<<29)?-(1<<29):next;
    code=nearest;last=written;
    // UART is polled between compensated tone updates, never during a pulse.
    if(io)io();
  }while(int32_t(ESP.getCycleCount()-deadline)<0);
  toneAt=last;
  previousQ=q;
  __asm__ volatile("wsr %0, ps; rsync"::"a"(ps):"memory");
  txClockStarted=true;++sampleNumber;++stats.txSamples;
}
void IRAM_ATTR RadioEngine::receiveSample(){
  uint32_t iqAt=ESP.getCycleCount();
  uint32_t ps;__asm__ volatile("rsil %0, 15":"=a"(ps)::"memory");
  reg(IQCTRL)=iqConfig;reg(IQCTRL)=iqConfig|1;fence();
  bool fresh=!(reg(IQCTRL)&0x80000000u);
  uint32_t began=ESP.getCycleCount();reg(IQCTRL)=iqConfig|3;fence();
  while(!(reg(IQCTRL)&0x80000000u) && ESP.getCycleCount()-began<3200){}
  bool valid=fresh && (reg(IQCTRL)&0x80000000u);
  int32_t i=shrink(int32_t(reg(IREG))),q=shrink(int32_t(reg(QREG)));
  stats.lastE4=reg(E4REG);reg(IQCTRL)=iqConfig;fence();
  __asm__ volatile("wsr %0, ps; rsync"::"a"(ps):"memory");
  uint32_t iqSpent=ESP.getCycleCount()-iqAt;if(iqSpent>stats.maxIqCycles)stats.maxIqCycles=iqSpent;
  uint32_t dspAt=ESP.getCycleCount();
  if(!valid){++stats.iqTimeouts;previousValid=false;dsp.feed(0,false);return;}
  if(!dcReady){dcI=i*4096;dcQ=q*4096;dcReady=true;}
  dcI+=i-dcI/4096;dcQ+=q-dcQ/4096;
  i-=dcI/4096;q-=dcQ/4096;
  stats.lastIqAmplitude=magnitude(i)+magnitude(q);
  if(previousValid){
    uint32_t phaseAt=ESP.getCycleCount();stats.lastRawHz=phaseHz(lastI,lastQ,i,q);
    uint32_t phaseSpent=ESP.getCycleCount()-phaseAt;if(phaseSpent>stats.maxPhaseCycles)stats.maxPhaseCycles=phaseSpent;
    dsp.feed(stats.lastRawHz,true);
  }else dsp.feed(0,false);
  lastI=i;lastQ=q;previousValid=true;
  uint32_t dspSpent=ESP.getCycleCount()-dspAt;if(dspSpent>stats.maxDspCycles)stats.maxDspCycles=dspSpent;
}
void IRAM_ATTR RadioEngine::step(){
#if M17_REALTIME_IRQ_LOCK
  if(realtimeLocked){
    uint32_t ps;__asm__ volatile("rsr %0, ps":"=a"(ps));
    if((ps&15u)!=15u){++stats.irqLevelDrops;__asm__ volatile("rsil %0, 15":"=a"(ps)::"memory");}
  }
#endif
  if(!ready){if(io)io();return;}
  if(!transmitting&&!listening){if(io)io();return;}
  uint32_t now=ESP.getCycleCount();
  if(transmitting && !txClockStarted){
    // SDK setup, control replies and metadata run with the RF gate off.
    // Their duration is not a missed RF sample. Start the grid here once;
    // every later sample retains the strict overrun/abort guard below.
    deadline=now;fraction=0;if(carrierMode)carrierAt=now;
  }
  if(!transmitting){while(int32_t(ESP.getCycleCount()-deadline)<0){if(io)io();}now=ESP.getCycleCount();}
  int32_t late=int32_t(now-deadline);
  if(late>int32_t(stats.maxLateCycles))stats.maxLateCycles=uint32_t(late);
  if(late>4166){
    unsigned missed=unsigned(late)/4166;stats.rxMisses+=missed;
    if(transmitting){
      // Keep the sample grid honest: TX cannot silently stretch its frame
      // when an entire deadline is missed.
      ++stats.txUnderruns;finishPending=true;reg(TONE)&=~GATE;fence();return;
    }
    if(!transmitting){unsigned mark=missed>4?4:missed;for(unsigned i=0;i<mark;++i)dsp.feed(0,false);dsp.stats.samples+=missed-mark;}
    deadline=now;previousValid=false;
  }
  const uint32_t sampleBegin=deadline;
  deadline+=4166;fraction+=25600;if(fraction>=38400){fraction-=38400;++deadline;}
  uint32_t workAt=ESP.getCycleCount();
  if(transmitting){
    const int32_t fine=trim.sample();
    if(carrierMode){transmitSample(carrierQ+fine,sampleBegin);if(ESP.getCycleCount()-carrierAt>=carrierCycles)finishPending=true;}
    else {int next=shape.needsSymbol()?nextTxSymbol():0;transmitSample(shape.sample(pulse,next)+qOffset+fine,sampleBegin);}
  }else receiveSample();
  uint32_t work=ESP.getCycleCount()-workAt;if(work>stats.maxWorkCycles)stats.maxWorkCycles=work;
  if(io)io();
}
}
// ===== END ORIGINAL COMPONENT: RadioEngine.cpp =====

// ===== BEGIN ORIGINAL COMPONENT: M17Modem.ino =====
#include <Arduino.h>
#include <ESP8266WiFi.h>
using M17Radio::RadioEngine;
using M17Radio::Configuration;
static RadioEngine radio;
static SerialWire wire;
static HelloReply helloReply;
static uint16_t session=0;
static uint16_t rxSequence=0;
struct CachedRxFrame {uint16_t sequence;bool valid;uint8_t payload[197],track[72];};
static CachedRxFrame rxCache[8];
static uint8_t rxCacheNext=0;
struct CachedReply {
  uint16_t session,sequence,length;
  uint8_t requestType,responseType;
  bool valid;
  uint8_t payload[160];
};
static CachedReply replyCache[8];
static uint8_t replyCacheNext=0;
static bool radioReady=false;
static uint32_t serialFrameDrops=0,lastStatusAt=0,lastFeed=0;
static uint32_t rxBurstId=0,rxFrameIndex=0,lastRxSample=0,lastTrackAt=0;
static bool rxNeedsBurst=true;
static uint16_t diagnosticCount=0,diagnosticAt=0,diagnosticSequence=0,diagnosticWarmup=0;
static bool diagnosticRestore=false;
static bool diagnosticRaw=false;
static uint8_t diagnostic[519];
static M17Status::JsonBuilder statusBuilder;
static M17Status::UnsignedDivide statusScoreDivide;
static bool statusRequested=false,statusBorrowed=false,statusCapturing=false,statusNormalizing=false,statusIncludeHeap=false;
static uint8_t statusCaptureAt=0;
static uint32_t statusIdleHeap=0;


// Explicit declarations keep Arduino's automatic prototype pass type-safe.
static void IRAM_ATTR serviceUart();
static void IRAM_ATTR rememberReply(uint8_t,uint16_t,uint8_t,const uint8_t*,uint16_t);
static bool IRAM_ATTR replayReply(const SerialWire::Record&);
static void IRAM_ATTR result(uint16_t,uint8_t,bool,const char*);
static void IRAM_ATTR credits(uint16_t,uint8_t);
static void IRAM_ATTR buildTrack(uint8_t*,const M17Radio::SoftFrame*,bool);
static void IRAM_ATTR serviceTrack();
static void IRAM_ATTR received(void*,const M17Radio::SoftFrame&);
static void capabilities(uint16_t);
static void IRAM_ATTR status();
static void IRAM_ATTR resetStatus();
static uint32_t IRAM_ATTR statusScalar(uint8_t);
static void IRAM_ATTR serviceStatus();
static void IRAM_ATTR handle(const SerialWire::Record&);
void setup();
void IRAM_ATTR loop();

static void IRAM_ATTR serviceUart(){wire.service();}
static void IRAM_ATTR rememberReply(uint8_t request,uint16_t sequence,uint8_t response,const uint8_t* payload,uint16_t length){
  if(request==1 || request==17 || length>160)return;
  CachedReply& cached=replyCache[replyCacheNext];
  cached.session=session;cached.sequence=sequence;cached.requestType=request;cached.responseType=response;
  cached.length=length;cached.valid=true;if(length)memcpy(cached.payload,payload,length);
  replyCacheNext=(replyCacheNext+1)&7;
}
static bool IRAM_ATTR replayReply(const SerialWire::Record& r){
  if(r.type==17)return false;
  for(const auto& cached:replyCache){
    if(cached.valid && cached.session==r.session && cached.sequence==r.sequence && cached.requestType==r.type){
      wire.send(cached.responseType,cached.sequence,cached.payload,cached.length);
      return true;
    }
  }
  return false;
}
static void IRAM_ATTR result(uint16_t sequence,uint8_t request,bool success,const char* detail=""){
  uint8_t payload[160];payload[0]=request;payload[1]=success?0:1;
  size_t length=strlen(detail);if(length>158)length=158;
  memcpy(payload+2,detail,length);
  rememberReply(request,sequence,4,payload,uint16_t(length+2));
  wire.send(4,sequence,payload,uint16_t(length+2));
}
static void IRAM_ATTR credits(uint16_t sequence,uint8_t request=0){
  uint8_t b[6];SerialWire::put16(b,radio.freeEntries());SerialWire::put16(b+2,radio.queuedEntries());
  b[4]=radio.txState();b[5]=0;
  if(request==8)rememberReply(8,sequence,12,b,6);
  wire.send(12,sequence,b,6);
}
static void IRAM_ATTR buildTrack(uint8_t* b,const M17Radio::SoftFrame* f,bool first=false){
  // Fixed binary snapshots avoid printf, allocation and variable64-bit
  // division in the RF foreground. All CFO fields remain modulo38400Hz.
  b[0]=1;b[1]=f?uint8_t(7|(first?8:0)):128;b[2]=f?f->kind:255;b[3]=0;
  SerialWire::put32(b+4,f?f->sampleCounter:0);
  SerialWire::put32(b+8,f?rxBurstId:0);SerialWire::put32(b+12,f?rxFrameIndex:0);
  SerialWire::put32(b+16,millis());
  SerialWire::put32(b+20,uint32_t(f?f->totalCfoHz:0));
  SerialWire::put32(b+24,uint32_t(f?f->afcHz:0));
  SerialWire::put32(b+28,uint32_t(f?f->syncResidualHz:0));
  SerialWire::put32(b+32,uint32_t(f?f->unitHz:0));
  SerialWire::put32(b+36,f?f->syncError:0);
  SerialWire::put32(b+40,uint32_t(radio.actualRxPllOffsetHz()));
  SerialWire::put32(b+44,radio.configurationId());
  SerialWire::put32(b+48,radio.carrierTrim().version());
  SerialWire::put32(b+52,radio.txBurstId());
  SerialWire::put32(b+56,uint32_t(radio.carrierTrim().fineHz()));
  SerialWire::put32(b+60,radio.carrierTrim().flags());
  SerialWire::put32(b+64,uint32_t(radio.configuration().rxOffsetHz));
  SerialWire::put32(b+68,uint32_t(f?M17Radio::wrapHz(f->totalCfoHz+radio.actualRxPllOffsetHz()):0));
}
static void IRAM_ATTR serviceTrack(){
  const bool changed=radio.takeTrackDirty();
  const uint32_t now=millis();
  if(!changed && (radio.txState()!=2 || uint32_t(now-lastTrackAt)<200))return;
  uint8_t payload[72];buildTrack(payload,nullptr);wire.send(18,65535,payload,72,0,true);lastTrackAt=now;
}
static void IRAM_ATTR received(void*,const M17Radio::SoftFrame& f){
  CachedRxFrame& cached=rxCache[rxCacheNext];
  cached.sequence=rxSequence++;cached.valid=true;
  SerialWire::put32(cached.payload,f.sampleCounter);cached.payload[4]=f.kind;
  memcpy(cached.payload+5,f.symbols,192);
  // Native I/Q inversion swaps LSF/Stream sync types; kind alone cannot
  // identify a new burst before the host resolves polarity with CRC/FEC.
  bool first=rxNeedsBurst || uint32_t(f.sampleCounter-lastRxSample)>3072;
  if(first){if(!++rxBurstId)++rxBurstId;rxFrameIndex=0;}
  buildTrack(cached.track,&f,first);++rxFrameIndex;lastRxSample=f.sampleCounter;rxNeedsBurst=f.kind==4;
  if(!wire.send(11,cached.sequence,cached.payload,sizeof(cached.payload),0,true))++serialFrameDrops;
  wire.send(18,cached.sequence,cached.track,sizeof(cached.track),0,true);
  rxCacheNext=(rxCacheNext+1)&7;
}
static void capabilities(uint16_t sequence){
  char text[960];
  snprintf(text,sizeof(text),"{\"version\":\"M17-ESP8266-0.3.0\",\"radio\":\"IQ_EST/4FSK\",\"ready\":%s,\"frequency\":2437000000,\"baud\":115200,\"txQueue\":34,\"sampleRate\":38400,\"symbolRate\":4800,\"softScale\":31,\"rxGainMax\":59,\"rxAgc\":true,\"rxAgcGain\":65535,\"frequencyMin\":2412000000,\"frequencyMax\":2462000000,\"rxRetry\":true,\"rxRetryDepth\":8,\"controlRetry\":true,\"controlRetryDepth\":8,\"rxTrack\":true,\"rxTrackSchema\":1,\"rxTrackLength\":72,\"txTrim\":true,\"txTrimSchema\":1,\"txTrimFineLimit\":12000,\"txTrimCoarseLimit\":2000000,\"txTrimRateLimit\":20000,\"modes\":[\"Packet\",\"Stream\",\"BERT\"],\"mac\":\"%s\",\"chip\":%u}",radioReady?"true":"false",WiFi.macAddress().c_str(),ESP.getChipId());
  helloReply.remember(session,sequence,text,uint16_t(strlen(text)));
  wire.sendText(2,sequence,text);
}
static void IRAM_ATTR status(){statusRequested=true;lastStatusAt=millis();}
static void IRAM_ATTR resetStatus(){
  statusRequested=statusBorrowed=statusCapturing=statusNormalizing=false;
  statusCaptureAt=0;statusBuilder.cancel();lastStatusAt=millis();
}
static uint32_t IRAM_ATTR statusScalar(uint8_t field){
  const auto& r=radio.rxStats();const auto& t=radio.stats;
  switch(field){
    case M17Status::RX:return radio.receiving();
    case M17Status::TX_STATE:return radio.txState();
    case M17Status::LOCKED:return radio.frameLocked();
    case M17Status::RX_FRAMES:return r.frames;
    case M17Status::TX_FRAMES:return t.txFrames;
    case M17Status::SAMPLES:return r.samples;
    case M17Status::RX_MISSES:return t.rxMisses;
    case M17Status::IQ_TIMEOUTS:return t.iqTimeouts;
    case M17Status::LATE_CYCLES:return t.maxLateCycles;
    case M17Status::WORK_CYCLES:return t.maxWorkCycles;
    case M17Status::DSP_CYCLES:return t.maxDspCycles;
    case M17Status::IQ_CYCLES:return t.maxIqCycles;
    case M17Status::PHASE_CYCLES:return t.maxPhaseCycles;
    case M17Status::FILTER_CYCLES:return r.filterMax;
    case M17Status::IRQ_DROPS:return t.irqLevelDrops;
    case M17Status::RAW_HZ:return uint32_t(t.lastRawHz);
    case M17Status::IQ_AMPLITUDE:return t.lastIqAmplitude;
    case M17Status::CFO_HZ:return uint32_t(r.cfoHz);
    case M17Status::UNIT_HZ:return uint32_t(r.unitHz);
    case M17Status::E4:return t.lastE4;
    case M17Status::UNDERRUNS:return t.txUnderruns;
    case M17Status::TX_ABORTS:return t.txAborts;
    case M17Status::UART_ERRORS:return wire.uartErrors;
    case M17Status::SERIAL_DROPS:return serialFrameDrops;
    case M17Status::CRC_ERRORS:return wire.crcErrors;
    case M17Status::RX_GAIN_PROFILE:return radio.configuration().gain;
    case M17Status::RX_GAIN_AUTO:return radio.configuration().gain==M17Radio::AUTO_GAIN;
    case M17Status::RX_PBUS_MANUAL:return radio.pbusManualControl();
    case M17Status::RX_AGC_ACTIVE:
      // This describes released PHY control, not a calibrated RF gain or
      // an assertion that a particular gain change occurred on this sample.
      return statusBuilder.values.scalar[M17Status::RX]
        && statusBuilder.values.scalar[M17Status::RX_GAIN_AUTO]
        && !statusBuilder.values.scalar[M17Status::RX_PBUS_MANUAL];
    case M17Status::FREE_HEAP:return statusIdleHeap;
    default:return 0;
  }
}
static void IRAM_ATTR serviceStatus(){
  // The borrowed buffer cannot be reused while any queued record can still
  // refer to it. Normal UART records continue to own their own payloads.
  if(statusBorrowed){
    if(!wire.queueIdle())return;
    statusBorrowed=false;statusBuilder.cancel();
  }
  if(statusNormalizing){
    if(statusScoreDivide.step()){
      statusBuilder.values.scalar[M17Status::SYNC_SCORE]=statusScoreDivide.value();
      statusNormalizing=false;++statusCaptureAt;
    }
    return;
  }
  if(statusCapturing){
    const auto& r=radio.rxStats();
    if(statusCaptureAt==M17Status::SYNC_SCORE){
      uint32_t denominator=uint32_t(r.unitHz*r.unitHz)/1024+1;
      statusScoreDivide.begin(uint32_t(r.syncError),denominator);statusNormalizing=true;return;
    }
    if(statusCaptureAt<M17Status::SCALAR_COUNT)
      statusBuilder.values.scalar[statusCaptureAt]=statusScalar(statusCaptureAt);
    else{
      unsigned i=statusCaptureAt-M17Status::SCALAR_COUNT;
      if(i<8)statusBuilder.values.dspMax[i]=r.stageMax[i];
      else if(i<16)statusBuilder.values.dspN[i-8]=r.stageCount[i-8];
      else statusBuilder.values.dspHist[i-16]=r.cycleHistogram[i-16];
    }
    if(++statusCaptureAt==M17Status::SCALAR_COUNT+22){
      statusCapturing=false;statusBuilder.begin(statusIncludeHeap);
    }
    return;
  }
  if(statusBuilder.busy()){statusBuilder.step();return;}
  if(statusBuilder.ready()){
    if(wire.sendBorrowedStatus(statusBuilder.data(),statusBuilder.length()))statusBorrowed=true;
    return;
  }
  if(statusRequested){
    statusRequested=false;statusCaptureAt=0;statusCapturing=true;
    statusIncludeHeap=!radio.receiving() && radio.txState()==0;
    // Never invoke allocator/SDK telemetry while an RF acquisition or TX is
    // active. Heap is omitted from those live status snapshots.
    statusIdleHeap=statusIncludeHeap?ESP.getFreeHeap():0;
  }
}
static void IRAM_ATTR handle(const SerialWire::Record& r){
  if(r.type==1){
    if(!r.session || r.length){result(r.sequence,r.type,false,"HELLO requires nonzero session");return;}
    if(helloReply.repeat(r.session,r.sequence,wire))return;
    if(radio.txState())radio.abort();radio.listen(false);session=r.session;wire.setSession(session);
    radio.clearCarrierTrim();rxBurstId=rxFrameIndex=lastRxSample=0;rxNeedsBurst=true;lastTrackAt=millis();
    resetStatus();
    rxSequence=0;rxCacheNext=0;for(auto& cached:rxCache)cached.valid=false;
    replyCacheNext=0;for(auto& cached:replyCache)cached.valid=false;
    capabilities(r.sequence);return;
  }
  if(!session || r.session!=session)return;
  radio.renewLease();
  if(replayReply(r)){if(r.type==15)credits(65535);return;}
  switch(r.type){
    case 3:{
      if(r.length!=12 && r.length!=16){result(r.sequence,r.type,false,"CONFIG length");break;}
      Configuration c;c.centerHz=SerialWire::get32(r.payload);c.rxOffsetHz=int32_t(SerialWire::get32(r.payload+4));
      c.gain=SerialWire::get16(r.payload+8);c.attenuation=r.payload[10];
      if(r.length==16)c.txOffsetHz=int32_t(SerialWire::get32(r.payload+12));
      bool ok=radio.configure(c);result(r.sequence,r.type,ok,ok?"":"Invalid configuration or RF busy");break;
    }
    case 5:radio.listen(true);result(r.sequence,r.type,radioReady);break;
    case 6:radio.listen(false);result(r.sequence,r.type,true);break;
    case 7:{
      bool ok=r.length==8 && radio.beginTransmission(r.payload[0],SerialWire::get16(r.payload+2),SerialWire::get32(r.payload+4));
      result(r.sequence,r.type,ok,ok?"":"TX_BEGIN rejected");if(ok)credits(r.sequence);break;
    }
    case 8:{
      bool pair=(r.flags&1)!=0;
      bool ok=r.length==(pair?96:48) && radio.enqueue(r.payload,pair?r.payload+48:nullptr);
      if(ok)credits(r.sequence,8);else result(r.sequence,r.type,false,"TX_FRAME rejected, queue full or TX closed");break;
    }
    case 9:{bool ok=radio.endTransmission();result(r.sequence,r.type,ok,ok?"":"TX_END rejected");break;}
    case 10:radio.abort();result(r.sequence,r.type,true);credits(r.sequence);break;
    case 15:result(r.sequence,r.type,true);credits(65535);break;
    case 16:{
      if(r.length==3 && (r.payload[0]==1 || r.payload[0]==3)){
        uint16_t n=SerialWire::get16(r.payload+1);
        if(!n || n>256 || diagnosticCount || radio.txState())result(r.sequence,r.type,false,"Capture unavailable");
        else{
          diagnosticRestore=radio.receiving();if(!diagnosticRestore)radio.listen(true);
          diagnosticCount=n;diagnosticAt=0;diagnosticWarmup=160;diagnosticSequence=r.sequence;
          diagnosticRaw=r.payload[0]==3;diagnostic[0]=r.payload[0];SerialWire::put16(diagnostic+1,n);SerialWire::put32(diagnostic+3,38400);
        }
      }else if(r.length==7 && r.payload[0]==2){
        bool ok=radio.carrier(int32_t(SerialWire::get32(r.payload+1)),SerialWire::get16(r.payload+5));result(r.sequence,r.type,ok,ok?"":"Carrier unavailable");
      }else result(r.sequence,r.type,false,"Unknown diagnostic");
      break;
    }
    case 17:{
      if(r.length!=2){result(r.sequence,r.type,false,"RX_RETRY length");break;}
      uint16_t wanted=SerialWire::get16(r.payload);bool found=false;
      for(const auto& cached:rxCache){
        if(cached.valid && cached.sequence==wanted){
          found=true;
          if(!wire.send(11,cached.sequence,cached.payload,sizeof(cached.payload)))
            result(r.sequence,r.type,false,"USB output queue full");
          else wire.send(18,cached.sequence,cached.track,sizeof(cached.track),0,true);
          break;
        }
      }
      if(!found)result(r.sequence,r.type,false,"RX cache miss");
      break;
    }
    case 19:{
      bool ok=r.length==32 && r.payload[0]==1 && !r.payload[1] && !SerialWire::get16(r.payload+2);
      if(ok){
        M17Radio::CarrierControl::Command command;
        command.version=SerialWire::get32(r.payload+4);command.expectedBurst=SerialWire::get32(r.payload+8);
        command.coarseHz=int32_t(SerialWire::get32(r.payload+12));command.fineHz=int32_t(SerialWire::get32(r.payload+16));
        command.slopeHzPerSecond=int32_t(SerialWire::get32(r.payload+20));command.validForMs=SerialWire::get32(r.payload+24);
        command.slewHzPerSecond=SerialWire::get32(r.payload+28);ok=radio.setCarrierTrim(command);
      }
      result(r.sequence,r.type,ok,ok?"":"TX_TRIM invalid, stale version/burst, busy or coarse change during TX");break;
    }
    default:result(r.sequence,r.type,false,"Unknown command");break;
  }
}
void setup(){
  wire.begin();radioReady=radio.begin(received,nullptr,serviceUart);
  wire.begin(); // RF SDK initialization may have reset UART interrupt enables.
  lastFeed=millis();
}
void IRAM_ATTR loop(){
  // Stay in foreground across samples; Arduino loop scheduling is not the RF
  // clock. The watchdog is fed without yielding the RF timing to background IO.
  for(;;){
  for(unsigned n=0;n<384;++n){
    radio.step();
    SerialWire::Record r;
    if(wire.command(r)){handle(r);wire.consumed();}
    radio.serviceControl();
    if(radio.takeCreditsDirty() && session)credits(65535);
    if(session)serviceTrack();
    if(diagnosticCount && radio.receiving()){
      if(diagnosticWarmup)--diagnosticWarmup;
      else{
        SerialWire::put16(diagnostic+7+diagnosticAt*2,uint16_t(diagnosticRaw?radio.lastRawHz():radio.lastFilteredHz()));
        if(++diagnosticAt==diagnosticCount){
          wire.send(16,diagnosticSequence,diagnostic,uint16_t(7+diagnosticCount*2));diagnosticCount=0;
          if(!diagnosticRestore)radio.listen(false);
        }
      }
    }
    if(radio.takeTxFinished() && session)status();
    if(uint32_t(millis()-lastFeed)>100){ESP.wdtFeed();lastFeed=millis();}
    if(session)serviceStatus();
  }
  if(session && uint32_t(millis()-lastStatusAt)>2000 && radio.txState()!=2)status();
  }
}
// ===== END ORIGINAL COMPONENT: M17Modem.ino =====

/* ===== LICENSE NOTICE: SP8ESA-MIT.txt =====
MIT License

Copyright (c) 2026 SP8ESA

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

===== END LICENSE NOTICE ===== */

/* ===== LICENSE NOTICE: BASE-MIT.txt =====
MIT License

Copyright (c) 2026 kaboom748

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

===== END LICENSE NOTICE ===== */

/* ===== LICENSE NOTICE: GPL-3.0 =====
                    GNU GENERAL PUBLIC LICENSE
                       Version 3, 29 June 2007

 Copyright (C) 2007 Free Software Foundation, Inc. <http://fsf.org/>
 Everyone is permitted to copy and distribute verbatim copies
 of this license document, but changing it is not allowed.

                            Preamble

  The GNU General Public License is a free, copyleft license for
software and other kinds of works.

  The licenses for most software and other practical works are designed
to take away your freedom to share and change the works.  By contrast,
the GNU General Public License is intended to guarantee your freedom to
share and change all versions of a program--to make sure it remains free
software for all its users.  We, the Free Software Foundation, use the
GNU General Public License for most of our software; it applies also to
any other work released this way by its authors.  You can apply it to
your programs, too.

  When we speak of free software, we are referring to freedom, not
price.  Our General Public Licenses are designed to make sure that you
have the freedom to distribute copies of free software (and charge for
them if you wish), that you receive source code or can get it if you
want it, that you can change the software or use pieces of it in new
free programs, and that you know you can do these things.

  To protect your rights, we need to prevent others from denying you
these rights or asking you to surrender the rights.  Therefore, you have
certain responsibilities if you distribute copies of the software, or if
you modify it: responsibilities to respect the freedom of others.

  For example, if you distribute copies of such a program, whether
gratis or for a fee, you must pass on to the recipients the same
freedoms that you received.  You must make sure that they, too, receive
or can get the source code.  And you must show them these terms so they
know their rights.

  Developers that use the GNU GPL protect your rights with two steps:
(1) assert copyright on the software, and (2) offer you this License
giving you legal permission to copy, distribute and/or modify it.

  For the developers' and authors' protection, the GPL clearly explains
that there is no warranty for this free software.  For both users' and
authors' sake, the GPL requires that modified versions be marked as
changed, so that their problems will not be attributed erroneously to
authors of previous versions.

  Some devices are designed to deny users access to install or run
modified versions of the software inside them, although the manufacturer
can do so.  This is fundamentally incompatible with the aim of
protecting users' freedom to change the software.  The systematic
pattern of such abuse occurs in the area of products for individuals to
use, which is precisely where it is most unacceptable.  Therefore, we
have designed this version of the GPL to prohibit the practice for those
products.  If such problems arise substantially in other domains, we
stand ready to extend this provision to those domains in future versions
of the GPL, as needed to protect the freedom of users.

  Finally, every program is threatened constantly by software patents.
States should not allow patents to restrict development and use of
software on general-purpose computers, but in those that do, we wish to
avoid the special danger that patents applied to a free program could
make it effectively proprietary.  To prevent this, the GPL assures that
patents cannot be used to render the program non-free.

  The precise terms and conditions for copying, distribution and
modification follow.

                       TERMS AND CONDITIONS

  0. Definitions.

  "This License" refers to version 3 of the GNU General Public License.

  "Copyright" also means copyright-like laws that apply to other kinds of
works, such as semiconductor masks.

  "The Program" refers to any copyrightable work licensed under this
License.  Each licensee is addressed as "you".  "Licensees" and
"recipients" may be individuals or organizations.

  To "modify" a work means to copy from or adapt all or part of the work
in a fashion requiring copyright permission, other than the making of an
exact copy.  The resulting work is called a "modified version" of the
earlier work or a work "based on" the earlier work.

  A "covered work" means either the unmodified Program or a work based
on the Program.

  To "propagate" a work means to do anything with it that, without
permission, would make you directly or secondarily liable for
infringement under applicable copyright law, except executing it on a
computer or modifying a private copy.  Propagation includes copying,
distribution (with or without modification), making available to the
public, and in some countries other activities as well.

  To "convey" a work means any kind of propagation that enables other
parties to make or receive copies.  Mere interaction with a user through
a computer network, with no transfer of a copy, is not conveying.

  An interactive user interface displays "Appropriate Legal Notices"
to the extent that it includes a convenient and prominently visible
feature that (1) displays an appropriate copyright notice, and (2)
tells the user that there is no warranty for the work (except to the
extent that warranties are provided), that licensees may convey the
work under this License, and how to view a copy of this License.  If
the interface presents a list of user commands or options, such as a
menu, a prominent item in the list meets this criterion.

  1. Source Code.

  The "source code" for a work means the preferred form of the work
for making modifications to it.  "Object code" means any non-source
form of a work.

  A "Standard Interface" means an interface that either is an official
standard defined by a recognized standards body, or, in the case of
interfaces specified for a particular programming language, one that
is widely used among developers working in that language.

  The "System Libraries" of an executable work include anything, other
than the work as a whole, that (a) is included in the normal form of
packaging a Major Component, but which is not part of that Major
Component, and (b) serves only to enable use of the work with that
Major Component, or to implement a Standard Interface for which an
implementation is available to the public in source code form.  A
"Major Component", in this context, means a major essential component
(kernel, window system, and so on) of the specific operating system
(if any) on which the executable work runs, or a compiler used to
produce the work, or an object code interpreter used to run it.

  The "Corresponding Source" for a work in object code form means all
the source code needed to generate, install, and (for an executable
work) run the object code and to modify the work, including scripts to
control those activities.  However, it does not include the work's
System Libraries, or general-purpose tools or generally available free
programs which are used unmodified in performing those activities but
which are not part of the work.  For example, Corresponding Source
includes interface definition files associated with source files for
the work, and the source code for shared libraries and dynamically
linked subprograms that the work is specifically designed to require,
such as by intimate data communication or control flow between those
subprograms and other parts of the work.

  The Corresponding Source need not include anything that users
can regenerate automatically from other parts of the Corresponding
Source.

  The Corresponding Source for a work in source code form is that
same work.

  2. Basic Permissions.

  All rights granted under this License are granted for the term of
copyright on the Program, and are irrevocable provided the stated
conditions are met.  This License explicitly affirms your unlimited
permission to run the unmodified Program.  The output from running a
covered work is covered by this License only if the output, given its
content, constitutes a covered work.  This License acknowledges your
rights of fair use or other equivalent, as provided by copyright law.

  You may make, run and propagate covered works that you do not
convey, without conditions so long as your license otherwise remains
in force.  You may convey covered works to others for the sole purpose
of having them make modifications exclusively for you, or provide you
with facilities for running those works, provided that you comply with
the terms of this License in conveying all material for which you do
not control copyright.  Those thus making or running the covered works
for you must do so exclusively on your behalf, under your direction
and control, on terms that prohibit them from making any copies of
your copyrighted material outside their relationship with you.

  Conveying under any other circumstances is permitted solely under
the conditions stated below.  Sublicensing is not allowed; section 10
makes it unnecessary.

  3. Protecting Users' Legal Rights From Anti-Circumvention Law.

  No covered work shall be deemed part of an effective technological
measure under any applicable law fulfilling obligations under article
11 of the WIPO copyright treaty adopted on 20 December 1996, or
similar laws prohibiting or restricting circumvention of such
measures.

  When you convey a covered work, you waive any legal power to forbid
circumvention of technological measures to the extent such circumvention
is effected by exercising rights under this License with respect to
the covered work, and you disclaim any intention to limit operation or
modification of the work as a means of enforcing, against the work's
users, your or third parties' legal rights to forbid circumvention of
technological measures.

  4. Conveying Verbatim Copies.

  You may convey verbatim copies of the Program's source code as you
receive it, in any medium, provided that you conspicuously and
appropriately publish on each copy an appropriate copyright notice;
keep intact all notices stating that this License and any
non-permissive terms added in accord with section 7 apply to the code;
keep intact all notices of the absence of any warranty; and give all
recipients a copy of this License along with the Program.

  You may charge any price or no price for each copy that you convey,
and you may offer support or warranty protection for a fee.

  5. Conveying Modified Source Versions.

  You may convey a work based on the Program, or the modifications to
produce it from the Program, in the form of source code under the
terms of section 4, provided that you also meet all of these conditions:

    a) The work must carry prominent notices stating that you modified
    it, and giving a relevant date.

    b) The work must carry prominent notices stating that it is
    released under this License and any conditions added under section
    7.  This requirement modifies the requirement in section 4 to
    "keep intact all notices".

    c) You must license the entire work, as a whole, under this
    License to anyone who comes into possession of a copy.  This
    License will therefore apply, along with any applicable section 7
    additional terms, to the whole of the work, and all its parts,
    regardless of how they are packaged.  This License gives no
    permission to license the work in any other way, but it does not
    invalidate such permission if you have separately received it.

    d) If the work has interactive user interfaces, each must display
    Appropriate Legal Notices; however, if the Program has interactive
    interfaces that do not display Appropriate Legal Notices, your
    work need not make them do so.

  A compilation of a covered work with other separate and independent
works, which are not by their nature extensions of the covered work,
and which are not combined with it such as to form a larger program,
in or on a volume of a storage or distribution medium, is called an
"aggregate" if the compilation and its resulting copyright are not
used to limit the access or legal rights of the compilation's users
beyond what the individual works permit.  Inclusion of a covered work
in an aggregate does not cause this License to apply to the other
parts of the aggregate.

  6. Conveying Non-Source Forms.

  You may convey a covered work in object code form under the terms
of sections 4 and 5, provided that you also convey the
machine-readable Corresponding Source under the terms of this License,
in one of these ways:

    a) Convey the object code in, or embodied in, a physical product
    (including a physical distribution medium), accompanied by the
    Corresponding Source fixed on a durable physical medium
    customarily used for software interchange.

    b) Convey the object code in, or embodied in, a physical product
    (including a physical distribution medium), accompanied by a
    written offer, valid for at least three years and valid for as
    long as you offer spare parts or customer support for that product
    model, to give anyone who possesses the object code either (1) a
    copy of the Corresponding Source for all the software in the
    product that is covered by this License, on a durable physical
    medium customarily used for software interchange, for a price no
    more than your reasonable cost of physically performing this
    conveying of source, or (2) access to copy the
    Corresponding Source from a network server at no charge.

    c) Convey individual copies of the object code with a copy of the
    written offer to provide the Corresponding Source.  This
    alternative is allowed only occasionally and noncommercially, and
    only if you received the object code with such an offer, in accord
    with subsection 6b.

    d) Convey the object code by offering access from a designated
    place (gratis or for a charge), and offer equivalent access to the
    Corresponding Source in the same way through the same place at no
    further charge.  You need not require recipients to copy the
    Corresponding Source along with the object code.  If the place to
    copy the object code is a network server, the Corresponding Source
    may be on a different server (operated by you or a third party)
    that supports equivalent copying facilities, provided you maintain
    clear directions next to the object code saying where to find the
    Corresponding Source.  Regardless of what server hosts the
    Corresponding Source, you remain obligated to ensure that it is
    available for as long as needed to satisfy these requirements.

    e) Convey the object code using peer-to-peer transmission, provided
    you inform other peers where the object code and Corresponding
    Source of the work are being offered to the general public at no
    charge under subsection 6d.

  A separable portion of the object code, whose source code is excluded
from the Corresponding Source as a System Library, need not be
included in conveying the object code work.

  A "User Product" is either (1) a "consumer product", which means any
tangible personal property which is normally used for personal, family,
or household purposes, or (2) anything designed or sold for incorporation
into a dwelling.  In determining whether a product is a consumer product,
doubtful cases shall be resolved in favor of coverage.  For a particular
product received by a particular user, "normally used" refers to a
typical or common use of that class of product, regardless of the status
of the particular user or of the way in which the particular user
actually uses, or expects or is expected to use, the product.  A product
is a consumer product regardless of whether the product has substantial
commercial, industrial or non-consumer uses, unless such uses represent
the only significant mode of use of the product.

  "Installation Information" for a User Product means any methods,
procedures, authorization keys, or other information required to install
and execute modified versions of a covered work in that User Product from
a modified version of its Corresponding Source.  The information must
suffice to ensure that the continued functioning of the modified object
code is in no case prevented or interfered with solely because
modification has been made.

  If you convey an object code work under this section in, or with, or
specifically for use in, a User Product, and the conveying occurs as
part of a transaction in which the right of possession and use of the
User Product is transferred to the recipient in perpetuity or for a
fixed term (regardless of how the transaction is characterized), the
Corresponding Source conveyed under this section must be accompanied
by the Installation Information.  But this requirement does not apply
if neither you nor any third party retains the ability to install
modified object code on the User Product (for example, the work has
been installed in ROM).

  The requirement to provide Installation Information does not include a
requirement to continue to provide support service, warranty, or updates
for a work that has been modified or installed by the recipient, or for
the User Product in which it has been modified or installed.  Access to a
network may be denied when the modification itself materially and
adversely affects the operation of the network or violates the rules and
protocols for communication across the network.

  Corresponding Source conveyed, and Installation Information provided,
in accord with this section must be in a format that is publicly
documented (and with an implementation available to the public in
source code form), and must require no special password or key for
unpacking, reading or copying.

  7. Additional Terms.

  "Additional permissions" are terms that supplement the terms of this
License by making exceptions from one or more of its conditions.
Additional permissions that are applicable to the entire Program shall
be treated as though they were included in this License, to the extent
that they are valid under applicable law.  If additional permissions
apply only to part of the Program, that part may be used separately
under those permissions, but the entire Program remains governed by
this License without regard to the additional permissions.

  When you convey a copy of a covered work, you may at your option
remove any additional permissions from that copy, or from any part of
it.  (Additional permissions may be written to require their own
removal in certain cases when you modify the work.)  You may place
additional permissions on material, added by you to a covered work,
for which you have or can give appropriate copyright permission.

  Notwithstanding any other provision of this License, for material you
add to a covered work, you may (if authorized by the copyright holders of
that material) supplement the terms of this License with terms:

    a) Disclaiming warranty or limiting liability differently from the
    terms of sections 15 and 16 of this License; or

    b) Requiring preservation of specified reasonable legal notices or
    author attributions in that material or in the Appropriate Legal
    Notices displayed by works containing it; or

    c) Prohibiting misrepresentation of the origin of that material, or
    requiring that modified versions of such material be marked in
    reasonable ways as different from the original version; or

    d) Limiting the use for publicity purposes of names of licensors or
    authors of the material; or

    e) Declining to grant rights under trademark law for use of some
    trade names, trademarks, or service marks; or

    f) Requiring indemnification of licensors and authors of that
    material by anyone who conveys the material (or modified versions of
    it) with contractual assumptions of liability to the recipient, for
    any liability that these contractual assumptions directly impose on
    those licensors and authors.

  All other non-permissive additional terms are considered "further
restrictions" within the meaning of section 10.  If the Program as you
received it, or any part of it, contains a notice stating that it is
governed by this License along with a term that is a further
restriction, you may remove that term.  If a license document contains
a further restriction but permits relicensing or conveying under this
License, you may add to a covered work material governed by the terms
of that license document, provided that the further restriction does
not survive such relicensing or conveying.

  If you add terms to a covered work in accord with this section, you
must place, in the relevant source files, a statement of the
additional terms that apply to those files, or a notice indicating
where to find the applicable terms.

  Additional terms, permissive or non-permissive, may be stated in the
form of a separately written license, or stated as exceptions;
the above requirements apply either way.

  8. Termination.

  You may not propagate or modify a covered work except as expressly
provided under this License.  Any attempt otherwise to propagate or
modify it is void, and will automatically terminate your rights under
this License (including any patent licenses granted under the third
paragraph of section 11).

  However, if you cease all violation of this License, then your
license from a particular copyright holder is reinstated (a)
provisionally, unless and until the copyright holder explicitly and
finally terminates your license, and (b) permanently, if the copyright
holder fails to notify you of the violation by some reasonable means
prior to 60 days after the cessation.

  Moreover, your license from a particular copyright holder is
reinstated permanently if the copyright holder notifies you of the
violation by some reasonable means, this is the first time you have
received notice of violation of this License (for any work) from that
copyright holder, and you cure the violation prior to 30 days after
your receipt of the notice.

  Termination of your rights under this section does not terminate the
licenses of parties who have received copies or rights from you under
this License.  If your rights have been terminated and not permanently
reinstated, you do not qualify to receive new licenses for the same
material under section 10.

  9. Acceptance Not Required for Having Copies.

  You are not required to accept this License in order to receive or
run a copy of the Program.  Ancillary propagation of a covered work
occurring solely as a consequence of using peer-to-peer transmission
to receive a copy likewise does not require acceptance.  However,
nothing other than this License grants you permission to propagate or
modify any covered work.  These actions infringe copyright if you do
not accept this License.  Therefore, by modifying or propagating a
covered work, you indicate your acceptance of this License to do so.

  10. Automatic Licensing of Downstream Recipients.

  Each time you convey a covered work, the recipient automatically
receives a license from the original licensors, to run, modify and
propagate that work, subject to this License.  You are not responsible
for enforcing compliance by third parties with this License.

  An "entity transaction" is a transaction transferring control of an
organization, or substantially all assets of one, or subdividing an
organization, or merging organizations.  If propagation of a covered
work results from an entity transaction, each party to that
transaction who receives a copy of the work also receives whatever
licenses to the work the party's predecessor in interest had or could
give under the previous paragraph, plus a right to possession of the
Corresponding Source of the work from the predecessor in interest, if
the predecessor has it or can get it with reasonable efforts.

  You may not impose any further restrictions on the exercise of the
rights granted or affirmed under this License.  For example, you may
not impose a license fee, royalty, or other charge for exercise of
rights granted under this License, and you may not initiate litigation
(including a cross-claim or counterclaim in a lawsuit) alleging that
any patent claim is infringed by making, using, selling, offering for
sale, or importing the Program or any portion of it.

  11. Patents.

  A "contributor" is a copyright holder who authorizes use under this
License of the Program or a work on which the Program is based.  The
work thus licensed is called the contributor's "contributor version".

  A contributor's "essential patent claims" are all patent claims
owned or controlled by the contributor, whether already acquired or
hereafter acquired, that would be infringed by some manner, permitted
by this License, of making, using, or selling its contributor version,
but do not include claims that would be infringed only as a
consequence of further modification of the contributor version.  For
purposes of this definition, "control" includes the right to grant
patent sublicenses in a manner consistent with the requirements of
this License.

  Each contributor grants you a non-exclusive, worldwide, royalty-free
patent license under the contributor's essential patent claims, to
make, use, sell, offer for sale, import and otherwise run, modify and
propagate the contents of its contributor version.

  In the following three paragraphs, a "patent license" is any express
agreement or commitment, however denominated, not to enforce a patent
(such as an express permission to practice a patent or covenant not to
sue for patent infringement).  To "grant" such a patent license to a
party means to make such an agreement or commitment not to enforce a
patent against the party.

  If you convey a covered work, knowingly relying on a patent license,
and the Corresponding Source of the work is not available for anyone
to copy, free of charge and under the terms of this License, through a
publicly available network server or other readily accessible means,
then you must either (1) cause the Corresponding Source to be so
available, or (2) arrange to deprive yourself of the benefit of the
patent license for this particular work, or (3) arrange, in a manner
consistent with the requirements of this License, to extend the patent
license to downstream recipients.  "Knowingly relying" means you have
actual knowledge that, but for the patent license, your conveying the
covered work in a country, or your recipient's use of the covered work
in a country, would infringe one or more identifiable patents in that
country that you have reason to believe are valid.

  If, pursuant to or in connection with a single transaction or
arrangement, you convey, or propagate by procuring conveyance of, a
covered work, and grant a patent license to some of the parties
receiving the covered work authorizing them to use, propagate, modify
or convey a specific copy of the covered work, then the patent license
you grant is automatically extended to all recipients of the covered
work and works based on it.

  A patent license is "discriminatory" if it does not include within
the scope of its coverage, prohibits the exercise of, or is
conditioned on the non-exercise of one or more of the rights that are
specifically granted under this License.  You may not convey a covered
work if you are a party to an arrangement with a third party that is
in the business of distributing software, under which you make payment
to the third party based on the extent of your activity of conveying
the work, and under which the third party grants, to any of the
parties who would receive the covered work from you, a discriminatory
patent license (a) in connection with copies of the covered work
conveyed by you (or copies made from those copies), or (b) primarily
for and in connection with specific products or compilations that
contain the covered work, unless you entered into that arrangement,
or that patent license was granted, prior to 28 March 2007.

  Nothing in this License shall be construed as excluding or limiting
any implied license or other defenses to infringement that may
otherwise be available to you under applicable patent law.

  12. No Surrender of Others' Freedom.

  If conditions are imposed on you (whether by court order, agreement or
otherwise) that contradict the conditions of this License, they do not
excuse you from the conditions of this License.  If you cannot convey a
covered work so as to satisfy simultaneously your obligations under this
License and any other pertinent obligations, then as a consequence you may
not convey it at all.  For example, if you agree to terms that obligate you
to collect a royalty for further conveying from those to whom you convey
the Program, the only way you could satisfy both those terms and this
License would be to refrain entirely from conveying the Program.

  13. Use with the GNU Affero General Public License.

  Notwithstanding any other provision of this License, you have
permission to link or combine any covered work with a work licensed
under version 3 of the GNU Affero General Public License into a single
combined work, and to convey the resulting work.  The terms of this
License will continue to apply to the part which is the covered work,
but the special requirements of the GNU Affero General Public License,
section 13, concerning interaction through a network will apply to the
combination as such.

  14. Revised Versions of this License.

  The Free Software Foundation may publish revised and/or new versions of
the GNU General Public License from time to time.  Such new versions will
be similar in spirit to the present version, but may differ in detail to
address new problems or concerns.

  Each version is given a distinguishing version number.  If the
Program specifies that a certain numbered version of the GNU General
Public License "or any later version" applies to it, you have the
option of following the terms and conditions either of that numbered
version or of any later version published by the Free Software
Foundation.  If the Program does not specify a version number of the
GNU General Public License, you may choose any version ever published
by the Free Software Foundation.

  If the Program specifies that a proxy can decide which future
versions of the GNU General Public License can be used, that proxy's
public statement of acceptance of a version permanently authorizes you
to choose that version for the Program.

  Later license versions may give you additional or different
permissions.  However, no additional obligations are imposed on any
author or copyright holder as a result of your choosing to follow a
later version.

  15. Disclaimer of Warranty.

  THERE IS NO WARRANTY FOR THE PROGRAM, TO THE EXTENT PERMITTED BY
APPLICABLE LAW.  EXCEPT WHEN OTHERWISE STATED IN WRITING THE COPYRIGHT
HOLDERS AND/OR OTHER PARTIES PROVIDE THE PROGRAM "AS IS" WITHOUT WARRANTY
OF ANY KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE.  THE ENTIRE RISK AS TO THE QUALITY AND PERFORMANCE OF THE PROGRAM
IS WITH YOU.  SHOULD THE PROGRAM PROVE DEFECTIVE, YOU ASSUME THE COST OF
ALL NECESSARY SERVICING, REPAIR OR CORRECTION.

  16. Limitation of Liability.

  IN NO EVENT UNLESS REQUIRED BY APPLICABLE LAW OR AGREED TO IN WRITING
WILL ANY COPYRIGHT HOLDER, OR ANY OTHER PARTY WHO MODIFIES AND/OR CONVEYS
THE PROGRAM AS PERMITTED ABOVE, BE LIABLE TO YOU FOR DAMAGES, INCLUDING ANY
GENERAL, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES ARISING OUT OF THE
USE OR INABILITY TO USE THE PROGRAM (INCLUDING BUT NOT LIMITED TO LOSS OF
DATA OR DATA BEING RENDERED INACCURATE OR LOSSES SUSTAINED BY YOU OR THIRD
PARTIES OR A FAILURE OF THE PROGRAM TO OPERATE WITH ANY OTHER PROGRAMS),
EVEN IF SUCH HOLDER OR OTHER PARTY HAS BEEN ADVISED OF THE POSSIBILITY OF
SUCH DAMAGES.

  17. Interpretation of Sections 15 and 16.

  If the disclaimer of warranty and limitation of liability provided
above cannot be given local legal effect according to their terms,
reviewing courts shall apply local law that most closely approximates
an absolute waiver of all civil liability in connection with the
Program, unless a warranty or assumption of liability accompanies a
copy of the Program in return for a fee.

                     END OF TERMS AND CONDITIONS

            How to Apply These Terms to Your New Programs

  If you develop a new program, and you want it to be of the greatest
possible use to the public, the best way to achieve this is to make it
free software which everyone can redistribute and change under these terms.

  To do so, attach the following notices to the program.  It is safest
to attach them to the start of each source file to most effectively
state the exclusion of warranty; and each file should have at least
the "copyright" line and a pointer to where the full notice is found.

    {one line to give the program's name and a brief idea of what it does.}
    Copyright (C) {year}  {name of author}

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

Also add information on how to contact you by electronic and paper mail.

  If the program does terminal interaction, make it output a short
notice like this when it starts in an interactive mode:

    {project}  Copyright (C) {year}  {fullname}
    This program comes with ABSOLUTELY NO WARRANTY; for details type `show w'.
    This is free software, and you are welcome to redistribute it
    under certain conditions; type `show c' for details.

The hypothetical commands `show w' and `show c' should show the appropriate
parts of the General Public License.  Of course, your program's commands
might be different; for a GUI interface, you would use an "about box".

  You should also get your employer (if you work as a programmer) or school,
if any, to sign a "copyright disclaimer" for the program, if necessary.
For more information on this, and how to apply and follow the GNU GPL, see
<http://www.gnu.org/licenses/>.

  The GNU General Public License does not permit incorporating your program
into proprietary programs.  If your program is a subroutine library, you
may consider it more useful to permit linking proprietary applications with
the library.  If this is what you want to do, use the GNU Lesser General
Public License instead of this License.  But first, please read
<http://www.gnu.org/philosophy/why-not-lgpl.html>.

===== END LICENSE NOTICE ===== */
