#include <chrono>
#include <thread>
#include <mutex>
#include <cstdio>
#include <cstring>
#include <array>
#include <queue>
#include <atomic>
#include <vector>
#ifdef _MSC_VER
#include <intrin.h>
#include <winsock2.h>
#pragma comment (lib, "ws2_32.lib")

#define ROL64(a, offset) _rotl64(a, offset)
#else
#include <signal.h>
#include <immintrin.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>


#if defined(__GNUC__) && !defined(__clang__)
#define _andn_u64(a, b) __andn_u64(a, b)
#endif

#define ROL64(a, offset) ((((unsigned long long)a) << offset) ^ (((unsigned long long)a) >> (64 - offset)))
#endif

#if ENABLE_AVX512 || defined(__AVX512F__)
#define AVX512 1
#endif

#if AVX512
const static __m512i zero = _mm512_maskz_set1_epi64(0, 0);
const static __m512i moveThetaPrev = _mm512_setr_epi64(4, 0, 1, 2, 3, 5, 6, 7);
const static __m512i moveThetaNext = _mm512_setr_epi64(1, 2, 3, 4, 0, 5, 6, 7);
const static __m512i rhoB = _mm512_setr_epi64(0, 1, 62, 28, 27, 0, 0, 0);
const static __m512i rhoG = _mm512_setr_epi64(36, 44, 6, 55, 20, 0, 0, 0);
const static __m512i rhoK = _mm512_setr_epi64(3, 10, 43, 25, 39, 0, 0, 0);
const static __m512i rhoM = _mm512_setr_epi64(41, 45, 15, 21, 8, 0, 0, 0);
const static __m512i rhoS = _mm512_setr_epi64(18, 2, 61, 56, 14, 0, 0, 0);
const static __m512i pi1B = _mm512_setr_epi64(0, 3, 1, 4, 2, 5, 6, 7);
const static __m512i pi1G = _mm512_setr_epi64(1, 4, 2, 0, 3, 5, 6, 7);
const static __m512i pi1K = _mm512_setr_epi64(2, 0, 3, 1, 4, 5, 6, 7);
const static __m512i pi1M = _mm512_setr_epi64(3, 1, 4, 2, 0, 5, 6, 7);
const static __m512i pi1S = _mm512_setr_epi64(4, 2, 0, 3, 1, 5, 6, 7);
const static __m512i pi2S1 = _mm512_setr_epi64(0, 1, 2, 3, 4, 5, 8, 10);
const static __m512i pi2S2 = _mm512_setr_epi64(0, 1, 2, 3, 4, 5, 9, 11);
const static __m512i pi2BG = _mm512_setr_epi64(0, 1, 8, 9, 6, 5, 6, 7);
const static __m512i pi2KM = _mm512_setr_epi64(2, 3, 10, 11, 7, 5, 6, 7);
const static __m512i pi2S3 = _mm512_setr_epi64(4, 5, 12, 13, 4, 5, 6, 7);
const static __m512i padding = _mm512_maskz_set1_epi64(1, 0x8000000000000000);

const static __m512i K12RoundConst0 = _mm512_maskz_set1_epi64(1, 0x000000008000808bULL);
const static __m512i K12RoundConst1 = _mm512_maskz_set1_epi64(1, 0x800000000000008bULL);
const static __m512i K12RoundConst2 = _mm512_maskz_set1_epi64(1, 0x8000000000008089ULL);
const static __m512i K12RoundConst3 = _mm512_maskz_set1_epi64(1, 0x8000000000008003ULL);
const static __m512i K12RoundConst4 = _mm512_maskz_set1_epi64(1, 0x8000000000008002ULL);
const static __m512i K12RoundConst5 = _mm512_maskz_set1_epi64(1, 0x8000000000000080ULL);
const static __m512i K12RoundConst6 = _mm512_maskz_set1_epi64(1, 0x000000000000800aULL);
const static __m512i K12RoundConst7 = _mm512_maskz_set1_epi64(1, 0x800000008000000aULL);
const static __m512i K12RoundConst8 = _mm512_maskz_set1_epi64(1, 0x8000000080008081ULL);
const static __m512i K12RoundConst9 = _mm512_maskz_set1_epi64(1, 0x8000000000008080ULL);
const static __m512i K12RoundConst10 = _mm512_maskz_set1_epi64(1, 0x0000000080000001ULL);
const static __m512i K12RoundConst11 = _mm512_maskz_set1_epi64(1, 0x8000000080008008ULL);

#else

#define KeccakF1600RoundConstant0   0x000000008000808bULL
#define KeccakF1600RoundConstant1   0x800000000000008bULL
#define KeccakF1600RoundConstant2   0x8000000000008089ULL
#define KeccakF1600RoundConstant3   0x8000000000008003ULL
#define KeccakF1600RoundConstant4   0x8000000000008002ULL
#define KeccakF1600RoundConstant5   0x8000000000000080ULL
#define KeccakF1600RoundConstant6   0x000000000000800aULL
#define KeccakF1600RoundConstant7   0x800000008000000aULL
#define KeccakF1600RoundConstant8   0x8000000080008081ULL
#define KeccakF1600RoundConstant9   0x8000000000008080ULL
#define KeccakF1600RoundConstant10  0x0000000080000001ULL

#define declareABCDE \
    unsigned long long Aba, Abe, Abi, Abo, Abu; \
    unsigned long long Aga, Age, Agi, Ago, Agu; \
    unsigned long long Aka, Ake, Aki, Ako, Aku; \
    unsigned long long Ama, Ame, Ami, Amo, Amu; \
    unsigned long long Asa, Ase, Asi, Aso, Asu; \
    unsigned long long Bba, Bbe, Bbi, Bbo, Bbu; \
    unsigned long long Bga, Bge, Bgi, Bgo, Bgu; \
    unsigned long long Bka, Bke, Bki, Bko, Bku; \
    unsigned long long Bma, Bme, Bmi, Bmo, Bmu; \
    unsigned long long Bsa, Bse, Bsi, Bso, Bsu; \
    unsigned long long Ca, Ce, Ci, Co, Cu; \
    unsigned long long Da, De, Di, Do, Du; \
    unsigned long long Eba, Ebe, Ebi, Ebo, Ebu; \
    unsigned long long Ega, Ege, Egi, Ego, Egu; \
    unsigned long long Eka, Eke, Eki, Eko, Eku; \
    unsigned long long Ema, Eme, Emi, Emo, Emu; \
    unsigned long long Esa, Ese, Esi, Eso, Esu; \

#define thetaRhoPiChiIotaPrepareTheta(i, A, E) \
    Da = Cu^ROL64(Ce, 1); \
    De = Ca^ROL64(Ci, 1); \
    Di = Ce^ROL64(Co, 1); \
    Do = Ci^ROL64(Cu, 1); \
    Du = Co^ROL64(Ca, 1); \
    A##ba ^= Da; \
    Bba = A##ba; \
    A##ge ^= De; \
    Bbe = ROL64(A##ge, 44); \
    A##ki ^= Di; \
    Bbi = ROL64(A##ki, 43); \
    A##mo ^= Do; \
    Bbo = ROL64(A##mo, 21); \
    A##su ^= Du; \
    Bbu = ROL64(A##su, 14); \
    E##ba =   Bba ^((~Bbe)&  Bbi ); \
    E##ba ^= KeccakF1600RoundConstant##i; \
    Ca = E##ba; \
    E##be =   Bbe ^((~Bbi)&  Bbo ); \
    Ce = E##be; \
    E##bi =   Bbi ^((~Bbo)&  Bbu ); \
    Ci = E##bi; \
    E##bo =   Bbo ^((~Bbu)&  Bba ); \
    Co = E##bo; \
    E##bu =   Bbu ^((~Bba)&  Bbe ); \
    Cu = E##bu; \
    A##bo ^= Do; \
    Bga = ROL64(A##bo, 28); \
    A##gu ^= Du; \
    Bge = ROL64(A##gu, 20); \
    A##ka ^= Da; \
    Bgi = ROL64(A##ka, 3); \
    A##me ^= De; \
    Bgo = ROL64(A##me, 45); \
    A##si ^= Di; \
    Bgu = ROL64(A##si, 61); \
    E##ga =   Bga ^((~Bge)&  Bgi ); \
    Ca ^= E##ga; \
    E##ge =   Bge ^((~Bgi)&  Bgo ); \
    Ce ^= E##ge; \
    E##gi =   Bgi ^((~Bgo)&  Bgu ); \
    Ci ^= E##gi; \
    E##go =   Bgo ^((~Bgu)&  Bga ); \
    Co ^= E##go; \
    E##gu =   Bgu ^((~Bga)&  Bge ); \
    Cu ^= E##gu; \
    A##be ^= De; \
    Bka = ROL64(A##be, 1); \
    A##gi ^= Di; \
    Bke = ROL64(A##gi, 6); \
    A##ko ^= Do; \
    Bki = ROL64(A##ko, 25); \
    A##mu ^= Du; \
    Bko = ROL64(A##mu, 8); \
    A##sa ^= Da; \
    Bku = ROL64(A##sa, 18); \
    E##ka =   Bka ^((~Bke)&  Bki ); \
    Ca ^= E##ka; \
    E##ke =   Bke ^((~Bki)&  Bko ); \
    Ce ^= E##ke; \
    E##ki =   Bki ^((~Bko)&  Bku ); \
    Ci ^= E##ki; \
    E##ko =   Bko ^((~Bku)&  Bka ); \
    Co ^= E##ko; \
    E##ku =   Bku ^((~Bka)&  Bke ); \
    Cu ^= E##ku; \
    A##bu ^= Du; \
    Bma = ROL64(A##bu, 27); \
    A##ga ^= Da; \
    Bme = ROL64(A##ga, 36); \
    A##ke ^= De; \
    Bmi = ROL64(A##ke, 10); \
    A##mi ^= Di; \
    Bmo = ROL64(A##mi, 15); \
    A##so ^= Do; \
    Bmu = ROL64(A##so, 56); \
    E##ma =   Bma ^((~Bme)&  Bmi ); \
    Ca ^= E##ma; \
    E##me =   Bme ^((~Bmi)&  Bmo ); \
    Ce ^= E##me; \
    E##mi =   Bmi ^((~Bmo)&  Bmu ); \
    Ci ^= E##mi; \
    E##mo =   Bmo ^((~Bmu)&  Bma ); \
    Co ^= E##mo; \
    E##mu =   Bmu ^((~Bma)&  Bme ); \
    Cu ^= E##mu; \
    A##bi ^= Di; \
    Bsa = ROL64(A##bi, 62); \
    A##go ^= Do; \
    Bse = ROL64(A##go, 55); \
    A##ku ^= Du; \
    Bsi = ROL64(A##ku, 39); \
    A##ma ^= Da; \
    Bso = ROL64(A##ma, 41); \
    A##se ^= De; \
    Bsu = ROL64(A##se, 2); \
    E##sa =   Bsa ^((~Bse)&  Bsi ); \
    Ca ^= E##sa; \
    E##se =   Bse ^((~Bsi)&  Bso ); \
    Ce ^= E##se; \
    E##si =   Bsi ^((~Bso)&  Bsu ); \
    Ci ^= E##si; \
    E##so =   Bso ^((~Bsu)&  Bsa ); \
    Co ^= E##so; \
    E##su =   Bsu ^((~Bsa)&  Bse ); \
    Cu ^= E##su;

#define copyFromState(state) \
    Aba = state[ 0]; \
    Abe = state[ 1]; \
    Abi = state[ 2]; \
    Abo = state[ 3]; \
    Abu = state[ 4]; \
    Aga = state[ 5]; \
    Age = state[ 6]; \
    Agi = state[ 7]; \
    Ago = state[ 8]; \
    Agu = state[ 9]; \
    Aka = state[10]; \
    Ake = state[11]; \
    Aki = state[12]; \
    Ako = state[13]; \
    Aku = state[14]; \
    Ama = state[15]; \
    Ame = state[16]; \
    Ami = state[17]; \
    Amo = state[18]; \
    Amu = state[19]; \
    Asa = state[20]; \
    Ase = state[21]; \
    Asi = state[22]; \
    Aso = state[23]; \
    Asu = state[24];

#define copyToState(state) \
    state[ 0] = Aba; \
    state[ 1] = Abe; \
    state[ 2] = Abi; \
    state[ 3] = Abo; \
    state[ 4] = Abu; \
    state[ 5] = Aga; \
    state[ 6] = Age; \
    state[ 7] = Agi; \
    state[ 8] = Ago; \
    state[ 9] = Agu; \
    state[10] = Aka; \
    state[11] = Ake; \
    state[12] = Aki; \
    state[13] = Ako; \
    state[14] = Aku; \
    state[15] = Ama; \
    state[16] = Ame; \
    state[17] = Ami; \
    state[18] = Amo; \
    state[19] = Amu; \
    state[20] = Asa; \
    state[21] = Ase; \
    state[22] = Asi; \
    state[23] = Aso; \
    state[24] = Asu;

#define rounds12 \
    Ca = Aba^Aga^Aka^Ama^Asa; \
    Ce = Abe^Age^Ake^Ame^Ase; \
    Ci = Abi^Agi^Aki^Ami^Asi; \
    Co = Abo^Ago^Ako^Amo^Aso; \
    Cu = Abu^Agu^Aku^Amu^Asu; \
    thetaRhoPiChiIotaPrepareTheta(0, A, E) \
    thetaRhoPiChiIotaPrepareTheta(1, E, A) \
    thetaRhoPiChiIotaPrepareTheta(2, A, E) \
    thetaRhoPiChiIotaPrepareTheta(3, E, A) \
    thetaRhoPiChiIotaPrepareTheta(4, A, E) \
    thetaRhoPiChiIotaPrepareTheta(5, E, A) \
    thetaRhoPiChiIotaPrepareTheta(6, A, E) \
    thetaRhoPiChiIotaPrepareTheta(7, E, A) \
    thetaRhoPiChiIotaPrepareTheta(8, A, E) \
    thetaRhoPiChiIotaPrepareTheta(9, E, A) \
    thetaRhoPiChiIotaPrepareTheta(10, A, E) \
    Da = Cu^ROL64(Ce, 1); \
    De = Ca^ROL64(Ci, 1); \
    Di = Ce^ROL64(Co, 1); \
    Do = Ci^ROL64(Cu, 1); \
    Du = Co^ROL64(Ca, 1); \
    Eba ^= Da; \
    Bba = Eba; \
    Ege ^= De; \
    Bbe = ROL64(Ege, 44); \
    Eki ^= Di; \
    Bbi = ROL64(Eki, 43); \
    Emo ^= Do; \
    Bbo = ROL64(Emo, 21); \
    Esu ^= Du; \
    Bbu = ROL64(Esu, 14); \
    Aba =   Bba ^((~Bbe)&  Bbi ); \
    Aba ^= 0x8000000080008008ULL; \
    Abe =   Bbe ^((~Bbi)&  Bbo ); \
    Abi =   Bbi ^((~Bbo)&  Bbu ); \
    Abo =   Bbo ^((~Bbu)&  Bba ); \
    Abu =   Bbu ^((~Bba)&  Bbe ); \
    Ebo ^= Do; \
    Bga = ROL64(Ebo, 28); \
    Egu ^= Du; \
    Bge = ROL64(Egu, 20); \
    Eka ^= Da; \
    Bgi = ROL64(Eka, 3); \
    Eme ^= De; \
    Bgo = ROL64(Eme, 45); \
    Esi ^= Di; \
    Bgu = ROL64(Esi, 61); \
    Aga =   Bga ^((~Bge)&  Bgi ); \
    Age =   Bge ^((~Bgi)&  Bgo ); \
    Agi =   Bgi ^((~Bgo)&  Bgu ); \
    Ago =   Bgo ^((~Bgu)&  Bga ); \
    Agu =   Bgu ^((~Bga)&  Bge ); \
    Ebe ^= De; \
    Bka = ROL64(Ebe, 1); \
    Egi ^= Di; \
    Bke = ROL64(Egi, 6); \
    Eko ^= Do; \
    Bki = ROL64(Eko, 25); \
    Emu ^= Du; \
    Bko = ROL64(Emu, 8); \
    Esa ^= Da; \
    Bku = ROL64(Esa, 18); \
    Aka =   Bka ^((~Bke)&  Bki ); \
    Ake =   Bke ^((~Bki)&  Bko ); \
    Aki =   Bki ^((~Bko)&  Bku ); \
    Ako =   Bko ^((~Bku)&  Bka ); \
    Aku =   Bku ^((~Bka)&  Bke ); \
    Ebu ^= Du; \
    Bma = ROL64(Ebu, 27); \
    Ega ^= Da; \
    Bme = ROL64(Ega, 36); \
    Eke ^= De; \
    Bmi = ROL64(Eke, 10); \
    Emi ^= Di; \
    Bmo = ROL64(Emi, 15); \
    Eso ^= Do; \
    Bmu = ROL64(Eso, 56); \
    Ama =   Bma ^((~Bme)&  Bmi ); \
    Ame =   Bme ^((~Bmi)&  Bmo ); \
    Ami =   Bmi ^((~Bmo)&  Bmu ); \
    Amo =   Bmo ^((~Bmu)&  Bma ); \
    Amu =   Bmu ^((~Bma)&  Bme ); \
    Ebi ^= Di; \
    Bsa = ROL64(Ebi, 62); \
    Ego ^= Do; \
    Bse = ROL64(Ego, 55); \
    Eku ^= Du; \
    Bsi = ROL64(Eku, 39); \
    Ema ^= Da; \
    Bso = ROL64(Ema, 41); \
    Ese ^= De; \
    Bsu = ROL64(Ese, 2); \
    Asa =   Bsa ^((~Bse)&  Bsi ); \
    Ase =   Bse ^((~Bsi)&  Bso ); \
    Asi =   Bsi ^((~Bso)&  Bsu ); \
    Aso =   Bso ^((~Bsu)&  Bsa ); \
    Asu =   Bsu ^((~Bsa)&  Bse );
#endif

#define K12_security        128
#define K12_capacity        (2 * K12_security)
#define K12_capacityInBytes (K12_capacity / 8)
#define K12_rateInBytes     ((1600 - K12_capacity) / 8)
#define K12_chunkSize       8192
#define K12_suffixLeaf      0x0B

typedef struct
{
    unsigned char state[200];
    unsigned char byteIOIndex;
} KangarooTwelve_F;

static void KeccakP1600_Permute_12rounds(unsigned char* state)
{
#if AVX512
    __m512i Baeiou = _mm512_maskz_loadu_epi64(0x1F, state);
    __m512i Gaeiou = _mm512_maskz_loadu_epi64(0x1F, state + 40);
    __m512i Kaeiou = _mm512_maskz_loadu_epi64(0x1F, state + 80);
    __m512i Maeiou = _mm512_maskz_loadu_epi64(0x1F, state + 120);
    __m512i Saeiou = _mm512_maskz_loadu_epi64(0x1F, state + 160);
    __m512i b0, b1, b2, b3, b4, b5;

    b0 = _mm512_ternarylogic_epi64(_mm512_ternarylogic_epi64(Baeiou, Gaeiou, Kaeiou, 0x96), Maeiou, Saeiou, 0x96);
    b1 = _mm512_permutexvar_epi64(moveThetaPrev, b0);
    b0 = _mm512_rol_epi64(_mm512_permutexvar_epi64(moveThetaNext, b0), 1);
    b2 = _mm512_permutexvar_epi64(pi1K, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Kaeiou, b0, b1, 0x96), rhoK));
    b3 = _mm512_permutexvar_epi64(pi1M, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Maeiou, b0, b1, 0x96), rhoM));
    b4 = _mm512_permutexvar_epi64(pi1S, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Saeiou, b0, b1, 0x96), rhoS));
    b5 = _mm512_permutexvar_epi64(pi1G, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Gaeiou, b0, b1, 0x96), rhoG));
    b0 = _mm512_permutexvar_epi64(pi1B, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Baeiou, b0, b1, 0x96), rhoB));
    Baeiou = _mm512_xor_si512(_mm512_ternarylogic_epi64(b0, b5, b2, 0xD2), K12RoundConst0);
    Gaeiou = _mm512_ternarylogic_epi64(b5, b2, b3, 0xD2);
    Kaeiou = _mm512_ternarylogic_epi64(b2, b3, b4, 0xD2);
    Maeiou = _mm512_ternarylogic_epi64(b3, b4, b0, 0xD2);
    Saeiou = _mm512_ternarylogic_epi64(b4, b0, b5, 0xD2);
    b0 = _mm512_permutex2var_epi64(_mm512_unpacklo_epi64(Baeiou, Gaeiou), pi2S1, Saeiou);
    b2 = _mm512_permutex2var_epi64(_mm512_unpackhi_epi64(Baeiou, Gaeiou), pi2S2, Saeiou);
    b1 = _mm512_unpacklo_epi64(Kaeiou, Maeiou);
    b3 = _mm512_unpackhi_epi64(Kaeiou, Maeiou);
    Baeiou = _mm512_permutex2var_epi64(b0, pi2BG, b1);
    Gaeiou = _mm512_permutex2var_epi64(b2, pi2BG, b3);
    Kaeiou = _mm512_permutex2var_epi64(b0, pi2KM, b1);
    Maeiou = _mm512_permutex2var_epi64(b2, pi2KM, b3);
    Saeiou = _mm512_mask_blend_epi64(0x10, _mm512_permutex2var_epi64(b0, pi2S3, b1), Saeiou);

    b0 = _mm512_ternarylogic_epi64(_mm512_ternarylogic_epi64(Baeiou, Gaeiou, Kaeiou, 0x96), Maeiou, Saeiou, 0x96);
    b1 = _mm512_permutexvar_epi64(moveThetaPrev, b0);
    b0 = _mm512_rol_epi64(_mm512_permutexvar_epi64(moveThetaNext, b0), 1);
    b2 = _mm512_permutexvar_epi64(pi1K, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Kaeiou, b0, b1, 0x96), rhoK));
    b3 = _mm512_permutexvar_epi64(pi1M, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Maeiou, b0, b1, 0x96), rhoM));
    b4 = _mm512_permutexvar_epi64(pi1S, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Saeiou, b0, b1, 0x96), rhoS));
    b5 = _mm512_permutexvar_epi64(pi1G, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Gaeiou, b0, b1, 0x96), rhoG));
    b0 = _mm512_permutexvar_epi64(pi1B, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Baeiou, b0, b1, 0x96), rhoB));
    Baeiou = _mm512_xor_si512(_mm512_ternarylogic_epi64(b0, b5, b2, 0xD2), K12RoundConst1);
    Gaeiou = _mm512_ternarylogic_epi64(b5, b2, b3, 0xD2);
    Kaeiou = _mm512_ternarylogic_epi64(b2, b3, b4, 0xD2);
    Maeiou = _mm512_ternarylogic_epi64(b3, b4, b0, 0xD2);
    Saeiou = _mm512_ternarylogic_epi64(b4, b0, b5, 0xD2);
    b0 = _mm512_permutex2var_epi64(_mm512_unpacklo_epi64(Baeiou, Gaeiou), pi2S1, Saeiou);
    b2 = _mm512_permutex2var_epi64(_mm512_unpackhi_epi64(Baeiou, Gaeiou), pi2S2, Saeiou);
    b1 = _mm512_unpacklo_epi64(Kaeiou, Maeiou);
    b3 = _mm512_unpackhi_epi64(Kaeiou, Maeiou);
    Baeiou = _mm512_permutex2var_epi64(b0, pi2BG, b1);
    Gaeiou = _mm512_permutex2var_epi64(b2, pi2BG, b3);
    Kaeiou = _mm512_permutex2var_epi64(b0, pi2KM, b1);
    Maeiou = _mm512_permutex2var_epi64(b2, pi2KM, b3);
    Saeiou = _mm512_mask_blend_epi64(0x10, _mm512_permutex2var_epi64(b0, pi2S3, b1), Saeiou);

    b0 = _mm512_ternarylogic_epi64(_mm512_ternarylogic_epi64(Baeiou, Gaeiou, Kaeiou, 0x96), Maeiou, Saeiou, 0x96);
    b1 = _mm512_permutexvar_epi64(moveThetaPrev, b0);
    b0 = _mm512_rol_epi64(_mm512_permutexvar_epi64(moveThetaNext, b0), 1);
    b2 = _mm512_permutexvar_epi64(pi1K, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Kaeiou, b0, b1, 0x96), rhoK));
    b3 = _mm512_permutexvar_epi64(pi1M, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Maeiou, b0, b1, 0x96), rhoM));
    b4 = _mm512_permutexvar_epi64(pi1S, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Saeiou, b0, b1, 0x96), rhoS));
    b5 = _mm512_permutexvar_epi64(pi1G, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Gaeiou, b0, b1, 0x96), rhoG));
    b0 = _mm512_permutexvar_epi64(pi1B, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Baeiou, b0, b1, 0x96), rhoB));
    Baeiou = _mm512_xor_si512(_mm512_ternarylogic_epi64(b0, b5, b2, 0xD2), K12RoundConst2);
    Gaeiou = _mm512_ternarylogic_epi64(b5, b2, b3, 0xD2);
    Kaeiou = _mm512_ternarylogic_epi64(b2, b3, b4, 0xD2);
    Maeiou = _mm512_ternarylogic_epi64(b3, b4, b0, 0xD2);
    Saeiou = _mm512_ternarylogic_epi64(b4, b0, b5, 0xD2);
    b0 = _mm512_permutex2var_epi64(_mm512_unpacklo_epi64(Baeiou, Gaeiou), pi2S1, Saeiou);
    b2 = _mm512_permutex2var_epi64(_mm512_unpackhi_epi64(Baeiou, Gaeiou), pi2S2, Saeiou);
    b1 = _mm512_unpacklo_epi64(Kaeiou, Maeiou);
    b3 = _mm512_unpackhi_epi64(Kaeiou, Maeiou);
    Baeiou = _mm512_permutex2var_epi64(b0, pi2BG, b1);
    Gaeiou = _mm512_permutex2var_epi64(b2, pi2BG, b3);
    Kaeiou = _mm512_permutex2var_epi64(b0, pi2KM, b1);
    Maeiou = _mm512_permutex2var_epi64(b2, pi2KM, b3);
    Saeiou = _mm512_mask_blend_epi64(0x10, _mm512_permutex2var_epi64(b0, pi2S3, b1), Saeiou);

    b0 = _mm512_ternarylogic_epi64(_mm512_ternarylogic_epi64(Baeiou, Gaeiou, Kaeiou, 0x96), Maeiou, Saeiou, 0x96);
    b1 = _mm512_permutexvar_epi64(moveThetaPrev, b0);
    b0 = _mm512_rol_epi64(_mm512_permutexvar_epi64(moveThetaNext, b0), 1);
    b2 = _mm512_permutexvar_epi64(pi1K, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Kaeiou, b0, b1, 0x96), rhoK));
    b3 = _mm512_permutexvar_epi64(pi1M, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Maeiou, b0, b1, 0x96), rhoM));
    b4 = _mm512_permutexvar_epi64(pi1S, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Saeiou, b0, b1, 0x96), rhoS));
    b5 = _mm512_permutexvar_epi64(pi1G, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Gaeiou, b0, b1, 0x96), rhoG));
    b0 = _mm512_permutexvar_epi64(pi1B, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Baeiou, b0, b1, 0x96), rhoB));
    Baeiou = _mm512_xor_si512(_mm512_ternarylogic_epi64(b0, b5, b2, 0xD2), K12RoundConst3);
    Gaeiou = _mm512_ternarylogic_epi64(b5, b2, b3, 0xD2);
    Kaeiou = _mm512_ternarylogic_epi64(b2, b3, b4, 0xD2);
    Maeiou = _mm512_ternarylogic_epi64(b3, b4, b0, 0xD2);
    Saeiou = _mm512_ternarylogic_epi64(b4, b0, b5, 0xD2);
    b0 = _mm512_permutex2var_epi64(_mm512_unpacklo_epi64(Baeiou, Gaeiou), pi2S1, Saeiou);
    b2 = _mm512_permutex2var_epi64(_mm512_unpackhi_epi64(Baeiou, Gaeiou), pi2S2, Saeiou);
    b1 = _mm512_unpacklo_epi64(Kaeiou, Maeiou);
    b3 = _mm512_unpackhi_epi64(Kaeiou, Maeiou);
    Baeiou = _mm512_permutex2var_epi64(b0, pi2BG, b1);
    Gaeiou = _mm512_permutex2var_epi64(b2, pi2BG, b3);
    Kaeiou = _mm512_permutex2var_epi64(b0, pi2KM, b1);
    Maeiou = _mm512_permutex2var_epi64(b2, pi2KM, b3);
    Saeiou = _mm512_mask_blend_epi64(0x10, _mm512_permutex2var_epi64(b0, pi2S3, b1), Saeiou);

    b0 = _mm512_ternarylogic_epi64(_mm512_ternarylogic_epi64(Baeiou, Gaeiou, Kaeiou, 0x96), Maeiou, Saeiou, 0x96);
    b1 = _mm512_permutexvar_epi64(moveThetaPrev, b0);
    b0 = _mm512_rol_epi64(_mm512_permutexvar_epi64(moveThetaNext, b0), 1);
    b2 = _mm512_permutexvar_epi64(pi1K, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Kaeiou, b0, b1, 0x96), rhoK));
    b3 = _mm512_permutexvar_epi64(pi1M, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Maeiou, b0, b1, 0x96), rhoM));
    b4 = _mm512_permutexvar_epi64(pi1S, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Saeiou, b0, b1, 0x96), rhoS));
    b5 = _mm512_permutexvar_epi64(pi1G, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Gaeiou, b0, b1, 0x96), rhoG));
    b0 = _mm512_permutexvar_epi64(pi1B, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Baeiou, b0, b1, 0x96), rhoB));
    Baeiou = _mm512_xor_si512(_mm512_ternarylogic_epi64(b0, b5, b2, 0xD2), K12RoundConst4);
    Gaeiou = _mm512_ternarylogic_epi64(b5, b2, b3, 0xD2);
    Kaeiou = _mm512_ternarylogic_epi64(b2, b3, b4, 0xD2);
    Maeiou = _mm512_ternarylogic_epi64(b3, b4, b0, 0xD2);
    Saeiou = _mm512_ternarylogic_epi64(b4, b0, b5, 0xD2);
    b0 = _mm512_permutex2var_epi64(_mm512_unpacklo_epi64(Baeiou, Gaeiou), pi2S1, Saeiou);
    b2 = _mm512_permutex2var_epi64(_mm512_unpackhi_epi64(Baeiou, Gaeiou), pi2S2, Saeiou);
    b1 = _mm512_unpacklo_epi64(Kaeiou, Maeiou);
    b3 = _mm512_unpackhi_epi64(Kaeiou, Maeiou);
    Baeiou = _mm512_permutex2var_epi64(b0, pi2BG, b1);
    Gaeiou = _mm512_permutex2var_epi64(b2, pi2BG, b3);
    Kaeiou = _mm512_permutex2var_epi64(b0, pi2KM, b1);
    Maeiou = _mm512_permutex2var_epi64(b2, pi2KM, b3);
    Saeiou = _mm512_mask_blend_epi64(0x10, _mm512_permutex2var_epi64(b0, pi2S3, b1), Saeiou);

    b0 = _mm512_ternarylogic_epi64(_mm512_ternarylogic_epi64(Baeiou, Gaeiou, Kaeiou, 0x96), Maeiou, Saeiou, 0x96);
    b1 = _mm512_permutexvar_epi64(moveThetaPrev, b0);
    b0 = _mm512_rol_epi64(_mm512_permutexvar_epi64(moveThetaNext, b0), 1);
    b2 = _mm512_permutexvar_epi64(pi1K, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Kaeiou, b0, b1, 0x96), rhoK));
    b3 = _mm512_permutexvar_epi64(pi1M, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Maeiou, b0, b1, 0x96), rhoM));
    b4 = _mm512_permutexvar_epi64(pi1S, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Saeiou, b0, b1, 0x96), rhoS));
    b5 = _mm512_permutexvar_epi64(pi1G, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Gaeiou, b0, b1, 0x96), rhoG));
    b0 = _mm512_permutexvar_epi64(pi1B, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Baeiou, b0, b1, 0x96), rhoB));
    Baeiou = _mm512_xor_si512(_mm512_ternarylogic_epi64(b0, b5, b2, 0xD2), K12RoundConst5);
    Gaeiou = _mm512_ternarylogic_epi64(b5, b2, b3, 0xD2);
    Kaeiou = _mm512_ternarylogic_epi64(b2, b3, b4, 0xD2);
    Maeiou = _mm512_ternarylogic_epi64(b3, b4, b0, 0xD2);
    Saeiou = _mm512_ternarylogic_epi64(b4, b0, b5, 0xD2);
    b0 = _mm512_permutex2var_epi64(_mm512_unpacklo_epi64(Baeiou, Gaeiou), pi2S1, Saeiou);
    b2 = _mm512_permutex2var_epi64(_mm512_unpackhi_epi64(Baeiou, Gaeiou), pi2S2, Saeiou);
    b1 = _mm512_unpacklo_epi64(Kaeiou, Maeiou);
    b3 = _mm512_unpackhi_epi64(Kaeiou, Maeiou);
    Baeiou = _mm512_permutex2var_epi64(b0, pi2BG, b1);
    Gaeiou = _mm512_permutex2var_epi64(b2, pi2BG, b3);
    Kaeiou = _mm512_permutex2var_epi64(b0, pi2KM, b1);
    Maeiou = _mm512_permutex2var_epi64(b2, pi2KM, b3);
    Saeiou = _mm512_mask_blend_epi64(0x10, _mm512_permutex2var_epi64(b0, pi2S3, b1), Saeiou);

    b0 = _mm512_ternarylogic_epi64(_mm512_ternarylogic_epi64(Baeiou, Gaeiou, Kaeiou, 0x96), Maeiou, Saeiou, 0x96);
    b1 = _mm512_permutexvar_epi64(moveThetaPrev, b0);
    b0 = _mm512_rol_epi64(_mm512_permutexvar_epi64(moveThetaNext, b0), 1);
    b2 = _mm512_permutexvar_epi64(pi1K, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Kaeiou, b0, b1, 0x96), rhoK));
    b3 = _mm512_permutexvar_epi64(pi1M, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Maeiou, b0, b1, 0x96), rhoM));
    b4 = _mm512_permutexvar_epi64(pi1S, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Saeiou, b0, b1, 0x96), rhoS));
    b5 = _mm512_permutexvar_epi64(pi1G, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Gaeiou, b0, b1, 0x96), rhoG));
    b0 = _mm512_permutexvar_epi64(pi1B, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Baeiou, b0, b1, 0x96), rhoB));
    Baeiou = _mm512_xor_si512(_mm512_ternarylogic_epi64(b0, b5, b2, 0xD2), K12RoundConst6);
    Gaeiou = _mm512_ternarylogic_epi64(b5, b2, b3, 0xD2);
    Kaeiou = _mm512_ternarylogic_epi64(b2, b3, b4, 0xD2);
    Maeiou = _mm512_ternarylogic_epi64(b3, b4, b0, 0xD2);
    Saeiou = _mm512_ternarylogic_epi64(b4, b0, b5, 0xD2);
    b0 = _mm512_permutex2var_epi64(_mm512_unpacklo_epi64(Baeiou, Gaeiou), pi2S1, Saeiou);
    b2 = _mm512_permutex2var_epi64(_mm512_unpackhi_epi64(Baeiou, Gaeiou), pi2S2, Saeiou);
    b1 = _mm512_unpacklo_epi64(Kaeiou, Maeiou);
    b3 = _mm512_unpackhi_epi64(Kaeiou, Maeiou);
    Baeiou = _mm512_permutex2var_epi64(b0, pi2BG, b1);
    Gaeiou = _mm512_permutex2var_epi64(b2, pi2BG, b3);
    Kaeiou = _mm512_permutex2var_epi64(b0, pi2KM, b1);
    Maeiou = _mm512_permutex2var_epi64(b2, pi2KM, b3);
    Saeiou = _mm512_mask_blend_epi64(0x10, _mm512_permutex2var_epi64(b0, pi2S3, b1), Saeiou);

    b0 = _mm512_ternarylogic_epi64(_mm512_ternarylogic_epi64(Baeiou, Gaeiou, Kaeiou, 0x96), Maeiou, Saeiou, 0x96);
    b1 = _mm512_permutexvar_epi64(moveThetaPrev, b0);
    b0 = _mm512_rol_epi64(_mm512_permutexvar_epi64(moveThetaNext, b0), 1);
    b2 = _mm512_permutexvar_epi64(pi1K, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Kaeiou, b0, b1, 0x96), rhoK));
    b3 = _mm512_permutexvar_epi64(pi1M, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Maeiou, b0, b1, 0x96), rhoM));
    b4 = _mm512_permutexvar_epi64(pi1S, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Saeiou, b0, b1, 0x96), rhoS));
    b5 = _mm512_permutexvar_epi64(pi1G, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Gaeiou, b0, b1, 0x96), rhoG));
    b0 = _mm512_permutexvar_epi64(pi1B, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Baeiou, b0, b1, 0x96), rhoB));
    Baeiou = _mm512_xor_si512(_mm512_ternarylogic_epi64(b0, b5, b2, 0xD2), K12RoundConst7);
    Gaeiou = _mm512_ternarylogic_epi64(b5, b2, b3, 0xD2);
    Kaeiou = _mm512_ternarylogic_epi64(b2, b3, b4, 0xD2);
    Maeiou = _mm512_ternarylogic_epi64(b3, b4, b0, 0xD2);
    Saeiou = _mm512_ternarylogic_epi64(b4, b0, b5, 0xD2);
    b0 = _mm512_permutex2var_epi64(_mm512_unpacklo_epi64(Baeiou, Gaeiou), pi2S1, Saeiou);
    b2 = _mm512_permutex2var_epi64(_mm512_unpackhi_epi64(Baeiou, Gaeiou), pi2S2, Saeiou);
    b1 = _mm512_unpacklo_epi64(Kaeiou, Maeiou);
    b3 = _mm512_unpackhi_epi64(Kaeiou, Maeiou);
    Baeiou = _mm512_permutex2var_epi64(b0, pi2BG, b1);
    Gaeiou = _mm512_permutex2var_epi64(b2, pi2BG, b3);
    Kaeiou = _mm512_permutex2var_epi64(b0, pi2KM, b1);
    Maeiou = _mm512_permutex2var_epi64(b2, pi2KM, b3);
    Saeiou = _mm512_mask_blend_epi64(0x10, _mm512_permutex2var_epi64(b0, pi2S3, b1), Saeiou);

    b0 = _mm512_ternarylogic_epi64(_mm512_ternarylogic_epi64(Baeiou, Gaeiou, Kaeiou, 0x96), Maeiou, Saeiou, 0x96);
    b1 = _mm512_permutexvar_epi64(moveThetaPrev, b0);
    b0 = _mm512_rol_epi64(_mm512_permutexvar_epi64(moveThetaNext, b0), 1);
    b2 = _mm512_permutexvar_epi64(pi1K, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Kaeiou, b0, b1, 0x96), rhoK));
    b3 = _mm512_permutexvar_epi64(pi1M, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Maeiou, b0, b1, 0x96), rhoM));
    b4 = _mm512_permutexvar_epi64(pi1S, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Saeiou, b0, b1, 0x96), rhoS));
    b5 = _mm512_permutexvar_epi64(pi1G, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Gaeiou, b0, b1, 0x96), rhoG));
    b0 = _mm512_permutexvar_epi64(pi1B, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Baeiou, b0, b1, 0x96), rhoB));
    Baeiou = _mm512_xor_si512(_mm512_ternarylogic_epi64(b0, b5, b2, 0xD2), K12RoundConst8);
    Gaeiou = _mm512_ternarylogic_epi64(b5, b2, b3, 0xD2);
    Kaeiou = _mm512_ternarylogic_epi64(b2, b3, b4, 0xD2);
    Maeiou = _mm512_ternarylogic_epi64(b3, b4, b0, 0xD2);
    Saeiou = _mm512_ternarylogic_epi64(b4, b0, b5, 0xD2);
    b0 = _mm512_permutex2var_epi64(_mm512_unpacklo_epi64(Baeiou, Gaeiou), pi2S1, Saeiou);
    b2 = _mm512_permutex2var_epi64(_mm512_unpackhi_epi64(Baeiou, Gaeiou), pi2S2, Saeiou);
    b1 = _mm512_unpacklo_epi64(Kaeiou, Maeiou);
    b3 = _mm512_unpackhi_epi64(Kaeiou, Maeiou);
    Baeiou = _mm512_permutex2var_epi64(b0, pi2BG, b1);
    Gaeiou = _mm512_permutex2var_epi64(b2, pi2BG, b3);
    Kaeiou = _mm512_permutex2var_epi64(b0, pi2KM, b1);
    Maeiou = _mm512_permutex2var_epi64(b2, pi2KM, b3);
    Saeiou = _mm512_mask_blend_epi64(0x10, _mm512_permutex2var_epi64(b0, pi2S3, b1), Saeiou);

    b0 = _mm512_ternarylogic_epi64(_mm512_ternarylogic_epi64(Baeiou, Gaeiou, Kaeiou, 0x96), Maeiou, Saeiou, 0x96);
    b1 = _mm512_permutexvar_epi64(moveThetaPrev, b0);
    b0 = _mm512_rol_epi64(_mm512_permutexvar_epi64(moveThetaNext, b0), 1);
    b2 = _mm512_permutexvar_epi64(pi1K, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Kaeiou, b0, b1, 0x96), rhoK));
    b3 = _mm512_permutexvar_epi64(pi1M, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Maeiou, b0, b1, 0x96), rhoM));
    b4 = _mm512_permutexvar_epi64(pi1S, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Saeiou, b0, b1, 0x96), rhoS));
    b5 = _mm512_permutexvar_epi64(pi1G, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Gaeiou, b0, b1, 0x96), rhoG));
    b0 = _mm512_permutexvar_epi64(pi1B, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Baeiou, b0, b1, 0x96), rhoB));
    Baeiou = _mm512_xor_si512(_mm512_ternarylogic_epi64(b0, b5, b2, 0xD2), K12RoundConst9);
    Gaeiou = _mm512_ternarylogic_epi64(b5, b2, b3, 0xD2);
    Kaeiou = _mm512_ternarylogic_epi64(b2, b3, b4, 0xD2);
    Maeiou = _mm512_ternarylogic_epi64(b3, b4, b0, 0xD2);
    Saeiou = _mm512_ternarylogic_epi64(b4, b0, b5, 0xD2);
    b0 = _mm512_permutex2var_epi64(_mm512_unpacklo_epi64(Baeiou, Gaeiou), pi2S1, Saeiou);
    b2 = _mm512_permutex2var_epi64(_mm512_unpackhi_epi64(Baeiou, Gaeiou), pi2S2, Saeiou);
    b1 = _mm512_unpacklo_epi64(Kaeiou, Maeiou);
    b3 = _mm512_unpackhi_epi64(Kaeiou, Maeiou);
    Baeiou = _mm512_permutex2var_epi64(b0, pi2BG, b1);
    Gaeiou = _mm512_permutex2var_epi64(b2, pi2BG, b3);
    Kaeiou = _mm512_permutex2var_epi64(b0, pi2KM, b1);
    Maeiou = _mm512_permutex2var_epi64(b2, pi2KM, b3);
    Saeiou = _mm512_mask_blend_epi64(0x10, _mm512_permutex2var_epi64(b0, pi2S3, b1), Saeiou);

    b0 = _mm512_ternarylogic_epi64(_mm512_ternarylogic_epi64(Baeiou, Gaeiou, Kaeiou, 0x96), Maeiou, Saeiou, 0x96);
    b1 = _mm512_permutexvar_epi64(moveThetaPrev, b0);
    b0 = _mm512_rol_epi64(_mm512_permutexvar_epi64(moveThetaNext, b0), 1);
    b2 = _mm512_permutexvar_epi64(pi1K, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Kaeiou, b0, b1, 0x96), rhoK));
    b3 = _mm512_permutexvar_epi64(pi1M, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Maeiou, b0, b1, 0x96), rhoM));
    b4 = _mm512_permutexvar_epi64(pi1S, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Saeiou, b0, b1, 0x96), rhoS));
    b5 = _mm512_permutexvar_epi64(pi1G, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Gaeiou, b0, b1, 0x96), rhoG));
    b0 = _mm512_permutexvar_epi64(pi1B, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Baeiou, b0, b1, 0x96), rhoB));
    Baeiou = _mm512_xor_si512(_mm512_ternarylogic_epi64(b0, b5, b2, 0xD2), K12RoundConst10);
    Gaeiou = _mm512_ternarylogic_epi64(b5, b2, b3, 0xD2);
    Kaeiou = _mm512_ternarylogic_epi64(b2, b3, b4, 0xD2);
    Maeiou = _mm512_ternarylogic_epi64(b3, b4, b0, 0xD2);
    Saeiou = _mm512_ternarylogic_epi64(b4, b0, b5, 0xD2);
    b0 = _mm512_permutex2var_epi64(_mm512_unpacklo_epi64(Baeiou, Gaeiou), pi2S1, Saeiou);
    b2 = _mm512_permutex2var_epi64(_mm512_unpackhi_epi64(Baeiou, Gaeiou), pi2S2, Saeiou);
    b1 = _mm512_unpacklo_epi64(Kaeiou, Maeiou);
    b3 = _mm512_unpackhi_epi64(Kaeiou, Maeiou);
    Baeiou = _mm512_permutex2var_epi64(b0, pi2BG, b1);
    Gaeiou = _mm512_permutex2var_epi64(b2, pi2BG, b3);
    Kaeiou = _mm512_permutex2var_epi64(b0, pi2KM, b1);
    Maeiou = _mm512_permutex2var_epi64(b2, pi2KM, b3);
    Saeiou = _mm512_mask_blend_epi64(0x10, _mm512_permutex2var_epi64(b0, pi2S3, b1), Saeiou);

    b0 = _mm512_ternarylogic_epi64(_mm512_ternarylogic_epi64(Baeiou, Gaeiou, Kaeiou, 0x96), Maeiou, Saeiou, 0x96);
    b1 = _mm512_permutexvar_epi64(moveThetaPrev, b0);
    b0 = _mm512_rol_epi64(_mm512_permutexvar_epi64(moveThetaNext, b0), 1);
    b2 = _mm512_permutexvar_epi64(pi1K, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Kaeiou, b0, b1, 0x96), rhoK));
    b3 = _mm512_permutexvar_epi64(pi1M, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Maeiou, b0, b1, 0x96), rhoM));
    b4 = _mm512_permutexvar_epi64(pi1S, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Saeiou, b0, b1, 0x96), rhoS));
    b5 = _mm512_permutexvar_epi64(pi1G, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Gaeiou, b0, b1, 0x96), rhoG));
    b0 = _mm512_permutexvar_epi64(pi1B, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Baeiou, b0, b1, 0x96), rhoB));
    Baeiou = _mm512_xor_si512(_mm512_ternarylogic_epi64(b0, b5, b2, 0xD2), K12RoundConst11);
    Gaeiou = _mm512_ternarylogic_epi64(b5, b2, b3, 0xD2);
    Kaeiou = _mm512_ternarylogic_epi64(b2, b3, b4, 0xD2);
    Maeiou = _mm512_ternarylogic_epi64(b3, b4, b0, 0xD2);
    Saeiou = _mm512_ternarylogic_epi64(b4, b0, b5, 0xD2);
    b0 = _mm512_permutex2var_epi64(_mm512_unpacklo_epi64(Baeiou, Gaeiou), pi2S1, Saeiou);
    b2 = _mm512_permutex2var_epi64(_mm512_unpackhi_epi64(Baeiou, Gaeiou), pi2S2, Saeiou);
    b1 = _mm512_unpacklo_epi64(Kaeiou, Maeiou);
    b3 = _mm512_unpackhi_epi64(Kaeiou, Maeiou);
    Baeiou = _mm512_permutex2var_epi64(b0, pi2BG, b1);
    Gaeiou = _mm512_permutex2var_epi64(b2, pi2BG, b3);
    Kaeiou = _mm512_permutex2var_epi64(b0, pi2KM, b1);
    Maeiou = _mm512_permutex2var_epi64(b2, pi2KM, b3);
    Saeiou = _mm512_mask_blend_epi64(0x10, _mm512_permutex2var_epi64(b0, pi2S3, b1), Saeiou);

    _mm512_mask_storeu_epi64(state, 0x1F, Baeiou);
    _mm512_mask_storeu_epi64(state + 40, 0x1F, Gaeiou);
    _mm512_mask_storeu_epi64(state + 80, 0x1F, Kaeiou);
    _mm512_mask_storeu_epi64(state + 120, 0x1F, Maeiou);
    _mm512_mask_storeu_epi64(state + 160, 0x1F, Saeiou);
#else
    declareABCDE
        unsigned long long* stateAsLanes = (unsigned long long*)state;
    copyFromState(stateAsLanes)
        rounds12
        copyToState(stateAsLanes)
#endif
}

static void KangarooTwelve_F_Absorb(KangarooTwelve_F* instance, unsigned char* data, unsigned long long dataByteLen)
{
    unsigned long long i = 0;
    while (i < dataByteLen)
    {
        if (!instance->byteIOIndex && dataByteLen >= i + K12_rateInBytes)
        {
#if AVX512
            __m512i Baeiou = _mm512_maskz_loadu_epi64(0x1F, instance->state);
            __m512i Gaeiou = _mm512_maskz_loadu_epi64(0x1F, instance->state + 40);
            __m512i Kaeiou = _mm512_maskz_loadu_epi64(0x1F, instance->state + 80);
            __m512i Maeiou = _mm512_maskz_loadu_epi64(0x1F, instance->state + 120);
            __m512i Saeiou = _mm512_maskz_loadu_epi64(0x1F, instance->state + 160);
#else
            declareABCDE
                unsigned long long* stateAsLanes = (unsigned long long*)instance->state;
            copyFromState(stateAsLanes)
#endif
                unsigned long long modifiedDataByteLen = dataByteLen - i;
            while (modifiedDataByteLen >= K12_rateInBytes)
            {
#if AVX512
                Baeiou = _mm512_xor_si512(Baeiou, _mm512_maskz_loadu_epi64(0x1F, data));
                Gaeiou = _mm512_xor_si512(Gaeiou, _mm512_maskz_loadu_epi64(0x1F, data + 40));
                Kaeiou = _mm512_xor_si512(Kaeiou, _mm512_maskz_loadu_epi64(0x1F, data + 80));
                Maeiou = _mm512_xor_si512(Maeiou, _mm512_maskz_loadu_epi64(0x1F, data + 120));
                Saeiou = _mm512_xor_si512(Saeiou, _mm512_maskz_loadu_epi64(0x01, data + 160));
                __m512i b0, b1, b2, b3, b4, b5;

                b0 = _mm512_ternarylogic_epi64(_mm512_ternarylogic_epi64(Baeiou, Gaeiou, Kaeiou, 0x96), Maeiou, Saeiou, 0x96);
                b1 = _mm512_permutexvar_epi64(moveThetaPrev, b0);
                b0 = _mm512_rol_epi64(_mm512_permutexvar_epi64(moveThetaNext, b0), 1);
                b2 = _mm512_permutexvar_epi64(pi1K, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Kaeiou, b0, b1, 0x96), rhoK));
                b3 = _mm512_permutexvar_epi64(pi1M, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Maeiou, b0, b1, 0x96), rhoM));
                b4 = _mm512_permutexvar_epi64(pi1S, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Saeiou, b0, b1, 0x96), rhoS));
                b5 = _mm512_permutexvar_epi64(pi1G, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Gaeiou, b0, b1, 0x96), rhoG));
                b0 = _mm512_permutexvar_epi64(pi1B, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Baeiou, b0, b1, 0x96), rhoB));
                Baeiou = _mm512_xor_si512(_mm512_ternarylogic_epi64(b0, b5, b2, 0xD2), K12RoundConst0);
                Gaeiou = _mm512_ternarylogic_epi64(b5, b2, b3, 0xD2);
                Kaeiou = _mm512_ternarylogic_epi64(b2, b3, b4, 0xD2);
                Maeiou = _mm512_ternarylogic_epi64(b3, b4, b0, 0xD2);
                Saeiou = _mm512_ternarylogic_epi64(b4, b0, b5, 0xD2);
                b0 = _mm512_permutex2var_epi64(_mm512_unpacklo_epi64(Baeiou, Gaeiou), pi2S1, Saeiou);
                b2 = _mm512_permutex2var_epi64(_mm512_unpackhi_epi64(Baeiou, Gaeiou), pi2S2, Saeiou);
                b1 = _mm512_unpacklo_epi64(Kaeiou, Maeiou);
                b3 = _mm512_unpackhi_epi64(Kaeiou, Maeiou);
                Baeiou = _mm512_permutex2var_epi64(b0, pi2BG, b1);
                Gaeiou = _mm512_permutex2var_epi64(b2, pi2BG, b3);
                Kaeiou = _mm512_permutex2var_epi64(b0, pi2KM, b1);
                Maeiou = _mm512_permutex2var_epi64(b2, pi2KM, b3);
                Saeiou = _mm512_mask_blend_epi64(0x10, _mm512_permutex2var_epi64(b0, pi2S3, b1), Saeiou);

                b0 = _mm512_ternarylogic_epi64(_mm512_ternarylogic_epi64(Baeiou, Gaeiou, Kaeiou, 0x96), Maeiou, Saeiou, 0x96);
                b1 = _mm512_permutexvar_epi64(moveThetaPrev, b0);
                b0 = _mm512_rol_epi64(_mm512_permutexvar_epi64(moveThetaNext, b0), 1);
                b2 = _mm512_permutexvar_epi64(pi1K, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Kaeiou, b0, b1, 0x96), rhoK));
                b3 = _mm512_permutexvar_epi64(pi1M, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Maeiou, b0, b1, 0x96), rhoM));
                b4 = _mm512_permutexvar_epi64(pi1S, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Saeiou, b0, b1, 0x96), rhoS));
                b5 = _mm512_permutexvar_epi64(pi1G, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Gaeiou, b0, b1, 0x96), rhoG));
                b0 = _mm512_permutexvar_epi64(pi1B, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Baeiou, b0, b1, 0x96), rhoB));
                Baeiou = _mm512_xor_si512(_mm512_ternarylogic_epi64(b0, b5, b2, 0xD2), K12RoundConst1);
                Gaeiou = _mm512_ternarylogic_epi64(b5, b2, b3, 0xD2);
                Kaeiou = _mm512_ternarylogic_epi64(b2, b3, b4, 0xD2);
                Maeiou = _mm512_ternarylogic_epi64(b3, b4, b0, 0xD2);
                Saeiou = _mm512_ternarylogic_epi64(b4, b0, b5, 0xD2);
                b0 = _mm512_permutex2var_epi64(_mm512_unpacklo_epi64(Baeiou, Gaeiou), pi2S1, Saeiou);
                b2 = _mm512_permutex2var_epi64(_mm512_unpackhi_epi64(Baeiou, Gaeiou), pi2S2, Saeiou);
                b1 = _mm512_unpacklo_epi64(Kaeiou, Maeiou);
                b3 = _mm512_unpackhi_epi64(Kaeiou, Maeiou);
                Baeiou = _mm512_permutex2var_epi64(b0, pi2BG, b1);
                Gaeiou = _mm512_permutex2var_epi64(b2, pi2BG, b3);
                Kaeiou = _mm512_permutex2var_epi64(b0, pi2KM, b1);
                Maeiou = _mm512_permutex2var_epi64(b2, pi2KM, b3);
                Saeiou = _mm512_mask_blend_epi64(0x10, _mm512_permutex2var_epi64(b0, pi2S3, b1), Saeiou);

                b0 = _mm512_ternarylogic_epi64(_mm512_ternarylogic_epi64(Baeiou, Gaeiou, Kaeiou, 0x96), Maeiou, Saeiou, 0x96);
                b1 = _mm512_permutexvar_epi64(moveThetaPrev, b0);
                b0 = _mm512_rol_epi64(_mm512_permutexvar_epi64(moveThetaNext, b0), 1);
                b2 = _mm512_permutexvar_epi64(pi1K, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Kaeiou, b0, b1, 0x96), rhoK));
                b3 = _mm512_permutexvar_epi64(pi1M, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Maeiou, b0, b1, 0x96), rhoM));
                b4 = _mm512_permutexvar_epi64(pi1S, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Saeiou, b0, b1, 0x96), rhoS));
                b5 = _mm512_permutexvar_epi64(pi1G, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Gaeiou, b0, b1, 0x96), rhoG));
                b0 = _mm512_permutexvar_epi64(pi1B, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Baeiou, b0, b1, 0x96), rhoB));
                Baeiou = _mm512_xor_si512(_mm512_ternarylogic_epi64(b0, b5, b2, 0xD2), K12RoundConst2);
                Gaeiou = _mm512_ternarylogic_epi64(b5, b2, b3, 0xD2);
                Kaeiou = _mm512_ternarylogic_epi64(b2, b3, b4, 0xD2);
                Maeiou = _mm512_ternarylogic_epi64(b3, b4, b0, 0xD2);
                Saeiou = _mm512_ternarylogic_epi64(b4, b0, b5, 0xD2);
                b0 = _mm512_permutex2var_epi64(_mm512_unpacklo_epi64(Baeiou, Gaeiou), pi2S1, Saeiou);
                b2 = _mm512_permutex2var_epi64(_mm512_unpackhi_epi64(Baeiou, Gaeiou), pi2S2, Saeiou);
                b1 = _mm512_unpacklo_epi64(Kaeiou, Maeiou);
                b3 = _mm512_unpackhi_epi64(Kaeiou, Maeiou);
                Baeiou = _mm512_permutex2var_epi64(b0, pi2BG, b1);
                Gaeiou = _mm512_permutex2var_epi64(b2, pi2BG, b3);
                Kaeiou = _mm512_permutex2var_epi64(b0, pi2KM, b1);
                Maeiou = _mm512_permutex2var_epi64(b2, pi2KM, b3);
                Saeiou = _mm512_mask_blend_epi64(0x10, _mm512_permutex2var_epi64(b0, pi2S3, b1), Saeiou);

                b0 = _mm512_ternarylogic_epi64(_mm512_ternarylogic_epi64(Baeiou, Gaeiou, Kaeiou, 0x96), Maeiou, Saeiou, 0x96);
                b1 = _mm512_permutexvar_epi64(moveThetaPrev, b0);
                b0 = _mm512_rol_epi64(_mm512_permutexvar_epi64(moveThetaNext, b0), 1);
                b2 = _mm512_permutexvar_epi64(pi1K, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Kaeiou, b0, b1, 0x96), rhoK));
                b3 = _mm512_permutexvar_epi64(pi1M, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Maeiou, b0, b1, 0x96), rhoM));
                b4 = _mm512_permutexvar_epi64(pi1S, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Saeiou, b0, b1, 0x96), rhoS));
                b5 = _mm512_permutexvar_epi64(pi1G, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Gaeiou, b0, b1, 0x96), rhoG));
                b0 = _mm512_permutexvar_epi64(pi1B, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Baeiou, b0, b1, 0x96), rhoB));
                Baeiou = _mm512_xor_si512(_mm512_ternarylogic_epi64(b0, b5, b2, 0xD2), K12RoundConst3);
                Gaeiou = _mm512_ternarylogic_epi64(b5, b2, b3, 0xD2);
                Kaeiou = _mm512_ternarylogic_epi64(b2, b3, b4, 0xD2);
                Maeiou = _mm512_ternarylogic_epi64(b3, b4, b0, 0xD2);
                Saeiou = _mm512_ternarylogic_epi64(b4, b0, b5, 0xD2);
                b0 = _mm512_permutex2var_epi64(_mm512_unpacklo_epi64(Baeiou, Gaeiou), pi2S1, Saeiou);
                b2 = _mm512_permutex2var_epi64(_mm512_unpackhi_epi64(Baeiou, Gaeiou), pi2S2, Saeiou);
                b1 = _mm512_unpacklo_epi64(Kaeiou, Maeiou);
                b3 = _mm512_unpackhi_epi64(Kaeiou, Maeiou);
                Baeiou = _mm512_permutex2var_epi64(b0, pi2BG, b1);
                Gaeiou = _mm512_permutex2var_epi64(b2, pi2BG, b3);
                Kaeiou = _mm512_permutex2var_epi64(b0, pi2KM, b1);
                Maeiou = _mm512_permutex2var_epi64(b2, pi2KM, b3);
                Saeiou = _mm512_mask_blend_epi64(0x10, _mm512_permutex2var_epi64(b0, pi2S3, b1), Saeiou);

                b0 = _mm512_ternarylogic_epi64(_mm512_ternarylogic_epi64(Baeiou, Gaeiou, Kaeiou, 0x96), Maeiou, Saeiou, 0x96);
                b1 = _mm512_permutexvar_epi64(moveThetaPrev, b0);
                b0 = _mm512_rol_epi64(_mm512_permutexvar_epi64(moveThetaNext, b0), 1);
                b2 = _mm512_permutexvar_epi64(pi1K, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Kaeiou, b0, b1, 0x96), rhoK));
                b3 = _mm512_permutexvar_epi64(pi1M, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Maeiou, b0, b1, 0x96), rhoM));
                b4 = _mm512_permutexvar_epi64(pi1S, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Saeiou, b0, b1, 0x96), rhoS));
                b5 = _mm512_permutexvar_epi64(pi1G, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Gaeiou, b0, b1, 0x96), rhoG));
                b0 = _mm512_permutexvar_epi64(pi1B, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Baeiou, b0, b1, 0x96), rhoB));
                Baeiou = _mm512_xor_si512(_mm512_ternarylogic_epi64(b0, b5, b2, 0xD2), K12RoundConst4);
                Gaeiou = _mm512_ternarylogic_epi64(b5, b2, b3, 0xD2);
                Kaeiou = _mm512_ternarylogic_epi64(b2, b3, b4, 0xD2);
                Maeiou = _mm512_ternarylogic_epi64(b3, b4, b0, 0xD2);
                Saeiou = _mm512_ternarylogic_epi64(b4, b0, b5, 0xD2);
                b0 = _mm512_permutex2var_epi64(_mm512_unpacklo_epi64(Baeiou, Gaeiou), pi2S1, Saeiou);
                b2 = _mm512_permutex2var_epi64(_mm512_unpackhi_epi64(Baeiou, Gaeiou), pi2S2, Saeiou);
                b1 = _mm512_unpacklo_epi64(Kaeiou, Maeiou);
                b3 = _mm512_unpackhi_epi64(Kaeiou, Maeiou);
                Baeiou = _mm512_permutex2var_epi64(b0, pi2BG, b1);
                Gaeiou = _mm512_permutex2var_epi64(b2, pi2BG, b3);
                Kaeiou = _mm512_permutex2var_epi64(b0, pi2KM, b1);
                Maeiou = _mm512_permutex2var_epi64(b2, pi2KM, b3);
                Saeiou = _mm512_mask_blend_epi64(0x10, _mm512_permutex2var_epi64(b0, pi2S3, b1), Saeiou);

                b0 = _mm512_ternarylogic_epi64(_mm512_ternarylogic_epi64(Baeiou, Gaeiou, Kaeiou, 0x96), Maeiou, Saeiou, 0x96);
                b1 = _mm512_permutexvar_epi64(moveThetaPrev, b0);
                b0 = _mm512_rol_epi64(_mm512_permutexvar_epi64(moveThetaNext, b0), 1);
                b2 = _mm512_permutexvar_epi64(pi1K, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Kaeiou, b0, b1, 0x96), rhoK));
                b3 = _mm512_permutexvar_epi64(pi1M, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Maeiou, b0, b1, 0x96), rhoM));
                b4 = _mm512_permutexvar_epi64(pi1S, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Saeiou, b0, b1, 0x96), rhoS));
                b5 = _mm512_permutexvar_epi64(pi1G, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Gaeiou, b0, b1, 0x96), rhoG));
                b0 = _mm512_permutexvar_epi64(pi1B, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Baeiou, b0, b1, 0x96), rhoB));
                Baeiou = _mm512_xor_si512(_mm512_ternarylogic_epi64(b0, b5, b2, 0xD2), K12RoundConst5);
                Gaeiou = _mm512_ternarylogic_epi64(b5, b2, b3, 0xD2);
                Kaeiou = _mm512_ternarylogic_epi64(b2, b3, b4, 0xD2);
                Maeiou = _mm512_ternarylogic_epi64(b3, b4, b0, 0xD2);
                Saeiou = _mm512_ternarylogic_epi64(b4, b0, b5, 0xD2);
                b0 = _mm512_permutex2var_epi64(_mm512_unpacklo_epi64(Baeiou, Gaeiou), pi2S1, Saeiou);
                b2 = _mm512_permutex2var_epi64(_mm512_unpackhi_epi64(Baeiou, Gaeiou), pi2S2, Saeiou);
                b1 = _mm512_unpacklo_epi64(Kaeiou, Maeiou);
                b3 = _mm512_unpackhi_epi64(Kaeiou, Maeiou);
                Baeiou = _mm512_permutex2var_epi64(b0, pi2BG, b1);
                Gaeiou = _mm512_permutex2var_epi64(b2, pi2BG, b3);
                Kaeiou = _mm512_permutex2var_epi64(b0, pi2KM, b1);
                Maeiou = _mm512_permutex2var_epi64(b2, pi2KM, b3);
                Saeiou = _mm512_mask_blend_epi64(0x10, _mm512_permutex2var_epi64(b0, pi2S3, b1), Saeiou);

                b0 = _mm512_ternarylogic_epi64(_mm512_ternarylogic_epi64(Baeiou, Gaeiou, Kaeiou, 0x96), Maeiou, Saeiou, 0x96);
                b1 = _mm512_permutexvar_epi64(moveThetaPrev, b0);
                b0 = _mm512_rol_epi64(_mm512_permutexvar_epi64(moveThetaNext, b0), 1);
                b2 = _mm512_permutexvar_epi64(pi1K, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Kaeiou, b0, b1, 0x96), rhoK));
                b3 = _mm512_permutexvar_epi64(pi1M, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Maeiou, b0, b1, 0x96), rhoM));
                b4 = _mm512_permutexvar_epi64(pi1S, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Saeiou, b0, b1, 0x96), rhoS));
                b5 = _mm512_permutexvar_epi64(pi1G, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Gaeiou, b0, b1, 0x96), rhoG));
                b0 = _mm512_permutexvar_epi64(pi1B, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Baeiou, b0, b1, 0x96), rhoB));
                Baeiou = _mm512_xor_si512(_mm512_ternarylogic_epi64(b0, b5, b2, 0xD2), K12RoundConst6);
                Gaeiou = _mm512_ternarylogic_epi64(b5, b2, b3, 0xD2);
                Kaeiou = _mm512_ternarylogic_epi64(b2, b3, b4, 0xD2);
                Maeiou = _mm512_ternarylogic_epi64(b3, b4, b0, 0xD2);
                Saeiou = _mm512_ternarylogic_epi64(b4, b0, b5, 0xD2);
                b0 = _mm512_permutex2var_epi64(_mm512_unpacklo_epi64(Baeiou, Gaeiou), pi2S1, Saeiou);
                b2 = _mm512_permutex2var_epi64(_mm512_unpackhi_epi64(Baeiou, Gaeiou), pi2S2, Saeiou);
                b1 = _mm512_unpacklo_epi64(Kaeiou, Maeiou);
                b3 = _mm512_unpackhi_epi64(Kaeiou, Maeiou);
                Baeiou = _mm512_permutex2var_epi64(b0, pi2BG, b1);
                Gaeiou = _mm512_permutex2var_epi64(b2, pi2BG, b3);
                Kaeiou = _mm512_permutex2var_epi64(b0, pi2KM, b1);
                Maeiou = _mm512_permutex2var_epi64(b2, pi2KM, b3);
                Saeiou = _mm512_mask_blend_epi64(0x10, _mm512_permutex2var_epi64(b0, pi2S3, b1), Saeiou);

                b0 = _mm512_ternarylogic_epi64(_mm512_ternarylogic_epi64(Baeiou, Gaeiou, Kaeiou, 0x96), Maeiou, Saeiou, 0x96);
                b1 = _mm512_permutexvar_epi64(moveThetaPrev, b0);
                b0 = _mm512_rol_epi64(_mm512_permutexvar_epi64(moveThetaNext, b0), 1);
                b2 = _mm512_permutexvar_epi64(pi1K, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Kaeiou, b0, b1, 0x96), rhoK));
                b3 = _mm512_permutexvar_epi64(pi1M, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Maeiou, b0, b1, 0x96), rhoM));
                b4 = _mm512_permutexvar_epi64(pi1S, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Saeiou, b0, b1, 0x96), rhoS));
                b5 = _mm512_permutexvar_epi64(pi1G, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Gaeiou, b0, b1, 0x96), rhoG));
                b0 = _mm512_permutexvar_epi64(pi1B, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Baeiou, b0, b1, 0x96), rhoB));
                Baeiou = _mm512_xor_si512(_mm512_ternarylogic_epi64(b0, b5, b2, 0xD2), K12RoundConst7);
                Gaeiou = _mm512_ternarylogic_epi64(b5, b2, b3, 0xD2);
                Kaeiou = _mm512_ternarylogic_epi64(b2, b3, b4, 0xD2);
                Maeiou = _mm512_ternarylogic_epi64(b3, b4, b0, 0xD2);
                Saeiou = _mm512_ternarylogic_epi64(b4, b0, b5, 0xD2);
                b0 = _mm512_permutex2var_epi64(_mm512_unpacklo_epi64(Baeiou, Gaeiou), pi2S1, Saeiou);
                b2 = _mm512_permutex2var_epi64(_mm512_unpackhi_epi64(Baeiou, Gaeiou), pi2S2, Saeiou);
                b1 = _mm512_unpacklo_epi64(Kaeiou, Maeiou);
                b3 = _mm512_unpackhi_epi64(Kaeiou, Maeiou);
                Baeiou = _mm512_permutex2var_epi64(b0, pi2BG, b1);
                Gaeiou = _mm512_permutex2var_epi64(b2, pi2BG, b3);
                Kaeiou = _mm512_permutex2var_epi64(b0, pi2KM, b1);
                Maeiou = _mm512_permutex2var_epi64(b2, pi2KM, b3);
                Saeiou = _mm512_mask_blend_epi64(0x10, _mm512_permutex2var_epi64(b0, pi2S3, b1), Saeiou);

                b0 = _mm512_ternarylogic_epi64(_mm512_ternarylogic_epi64(Baeiou, Gaeiou, Kaeiou, 0x96), Maeiou, Saeiou, 0x96);
                b1 = _mm512_permutexvar_epi64(moveThetaPrev, b0);
                b0 = _mm512_rol_epi64(_mm512_permutexvar_epi64(moveThetaNext, b0), 1);
                b2 = _mm512_permutexvar_epi64(pi1K, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Kaeiou, b0, b1, 0x96), rhoK));
                b3 = _mm512_permutexvar_epi64(pi1M, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Maeiou, b0, b1, 0x96), rhoM));
                b4 = _mm512_permutexvar_epi64(pi1S, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Saeiou, b0, b1, 0x96), rhoS));
                b5 = _mm512_permutexvar_epi64(pi1G, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Gaeiou, b0, b1, 0x96), rhoG));
                b0 = _mm512_permutexvar_epi64(pi1B, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Baeiou, b0, b1, 0x96), rhoB));
                Baeiou = _mm512_xor_si512(_mm512_ternarylogic_epi64(b0, b5, b2, 0xD2), K12RoundConst8);
                Gaeiou = _mm512_ternarylogic_epi64(b5, b2, b3, 0xD2);
                Kaeiou = _mm512_ternarylogic_epi64(b2, b3, b4, 0xD2);
                Maeiou = _mm512_ternarylogic_epi64(b3, b4, b0, 0xD2);
                Saeiou = _mm512_ternarylogic_epi64(b4, b0, b5, 0xD2);
                b0 = _mm512_permutex2var_epi64(_mm512_unpacklo_epi64(Baeiou, Gaeiou), pi2S1, Saeiou);
                b2 = _mm512_permutex2var_epi64(_mm512_unpackhi_epi64(Baeiou, Gaeiou), pi2S2, Saeiou);
                b1 = _mm512_unpacklo_epi64(Kaeiou, Maeiou);
                b3 = _mm512_unpackhi_epi64(Kaeiou, Maeiou);
                Baeiou = _mm512_permutex2var_epi64(b0, pi2BG, b1);
                Gaeiou = _mm512_permutex2var_epi64(b2, pi2BG, b3);
                Kaeiou = _mm512_permutex2var_epi64(b0, pi2KM, b1);
                Maeiou = _mm512_permutex2var_epi64(b2, pi2KM, b3);
                Saeiou = _mm512_mask_blend_epi64(0x10, _mm512_permutex2var_epi64(b0, pi2S3, b1), Saeiou);

                b0 = _mm512_ternarylogic_epi64(_mm512_ternarylogic_epi64(Baeiou, Gaeiou, Kaeiou, 0x96), Maeiou, Saeiou, 0x96);
                b1 = _mm512_permutexvar_epi64(moveThetaPrev, b0);
                b0 = _mm512_rol_epi64(_mm512_permutexvar_epi64(moveThetaNext, b0), 1);
                b2 = _mm512_permutexvar_epi64(pi1K, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Kaeiou, b0, b1, 0x96), rhoK));
                b3 = _mm512_permutexvar_epi64(pi1M, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Maeiou, b0, b1, 0x96), rhoM));
                b4 = _mm512_permutexvar_epi64(pi1S, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Saeiou, b0, b1, 0x96), rhoS));
                b5 = _mm512_permutexvar_epi64(pi1G, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Gaeiou, b0, b1, 0x96), rhoG));
                b0 = _mm512_permutexvar_epi64(pi1B, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Baeiou, b0, b1, 0x96), rhoB));
                Baeiou = _mm512_xor_si512(_mm512_ternarylogic_epi64(b0, b5, b2, 0xD2), K12RoundConst9);
                Gaeiou = _mm512_ternarylogic_epi64(b5, b2, b3, 0xD2);
                Kaeiou = _mm512_ternarylogic_epi64(b2, b3, b4, 0xD2);
                Maeiou = _mm512_ternarylogic_epi64(b3, b4, b0, 0xD2);
                Saeiou = _mm512_ternarylogic_epi64(b4, b0, b5, 0xD2);
                b0 = _mm512_permutex2var_epi64(_mm512_unpacklo_epi64(Baeiou, Gaeiou), pi2S1, Saeiou);
                b2 = _mm512_permutex2var_epi64(_mm512_unpackhi_epi64(Baeiou, Gaeiou), pi2S2, Saeiou);
                b1 = _mm512_unpacklo_epi64(Kaeiou, Maeiou);
                b3 = _mm512_unpackhi_epi64(Kaeiou, Maeiou);
                Baeiou = _mm512_permutex2var_epi64(b0, pi2BG, b1);
                Gaeiou = _mm512_permutex2var_epi64(b2, pi2BG, b3);
                Kaeiou = _mm512_permutex2var_epi64(b0, pi2KM, b1);
                Maeiou = _mm512_permutex2var_epi64(b2, pi2KM, b3);
                Saeiou = _mm512_mask_blend_epi64(0x10, _mm512_permutex2var_epi64(b0, pi2S3, b1), Saeiou);

                b0 = _mm512_ternarylogic_epi64(_mm512_ternarylogic_epi64(Baeiou, Gaeiou, Kaeiou, 0x96), Maeiou, Saeiou, 0x96);
                b1 = _mm512_permutexvar_epi64(moveThetaPrev, b0);
                b0 = _mm512_rol_epi64(_mm512_permutexvar_epi64(moveThetaNext, b0), 1);
                b2 = _mm512_permutexvar_epi64(pi1K, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Kaeiou, b0, b1, 0x96), rhoK));
                b3 = _mm512_permutexvar_epi64(pi1M, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Maeiou, b0, b1, 0x96), rhoM));
                b4 = _mm512_permutexvar_epi64(pi1S, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Saeiou, b0, b1, 0x96), rhoS));
                b5 = _mm512_permutexvar_epi64(pi1G, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Gaeiou, b0, b1, 0x96), rhoG));
                b0 = _mm512_permutexvar_epi64(pi1B, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Baeiou, b0, b1, 0x96), rhoB));
                Baeiou = _mm512_xor_si512(_mm512_ternarylogic_epi64(b0, b5, b2, 0xD2), K12RoundConst10);
                Gaeiou = _mm512_ternarylogic_epi64(b5, b2, b3, 0xD2);
                Kaeiou = _mm512_ternarylogic_epi64(b2, b3, b4, 0xD2);
                Maeiou = _mm512_ternarylogic_epi64(b3, b4, b0, 0xD2);
                Saeiou = _mm512_ternarylogic_epi64(b4, b0, b5, 0xD2);
                b0 = _mm512_permutex2var_epi64(_mm512_unpacklo_epi64(Baeiou, Gaeiou), pi2S1, Saeiou);
                b2 = _mm512_permutex2var_epi64(_mm512_unpackhi_epi64(Baeiou, Gaeiou), pi2S2, Saeiou);
                b1 = _mm512_unpacklo_epi64(Kaeiou, Maeiou);
                b3 = _mm512_unpackhi_epi64(Kaeiou, Maeiou);
                Baeiou = _mm512_permutex2var_epi64(b0, pi2BG, b1);
                Gaeiou = _mm512_permutex2var_epi64(b2, pi2BG, b3);
                Kaeiou = _mm512_permutex2var_epi64(b0, pi2KM, b1);
                Maeiou = _mm512_permutex2var_epi64(b2, pi2KM, b3);
                Saeiou = _mm512_mask_blend_epi64(0x10, _mm512_permutex2var_epi64(b0, pi2S3, b1), Saeiou);

                b0 = _mm512_ternarylogic_epi64(_mm512_ternarylogic_epi64(Baeiou, Gaeiou, Kaeiou, 0x96), Maeiou, Saeiou, 0x96);
                b1 = _mm512_permutexvar_epi64(moveThetaPrev, b0);
                b0 = _mm512_rol_epi64(_mm512_permutexvar_epi64(moveThetaNext, b0), 1);
                b2 = _mm512_permutexvar_epi64(pi1K, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Kaeiou, b0, b1, 0x96), rhoK));
                b3 = _mm512_permutexvar_epi64(pi1M, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Maeiou, b0, b1, 0x96), rhoM));
                b4 = _mm512_permutexvar_epi64(pi1S, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Saeiou, b0, b1, 0x96), rhoS));
                b5 = _mm512_permutexvar_epi64(pi1G, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Gaeiou, b0, b1, 0x96), rhoG));
                b0 = _mm512_permutexvar_epi64(pi1B, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Baeiou, b0, b1, 0x96), rhoB));
                Baeiou = _mm512_xor_si512(_mm512_ternarylogic_epi64(b0, b5, b2, 0xD2), K12RoundConst11);
                Gaeiou = _mm512_ternarylogic_epi64(b5, b2, b3, 0xD2);
                Kaeiou = _mm512_ternarylogic_epi64(b2, b3, b4, 0xD2);
                Maeiou = _mm512_ternarylogic_epi64(b3, b4, b0, 0xD2);
                Saeiou = _mm512_ternarylogic_epi64(b4, b0, b5, 0xD2);
                b0 = _mm512_permutex2var_epi64(_mm512_unpacklo_epi64(Baeiou, Gaeiou), pi2S1, Saeiou);
                b2 = _mm512_permutex2var_epi64(_mm512_unpackhi_epi64(Baeiou, Gaeiou), pi2S2, Saeiou);
                b1 = _mm512_unpacklo_epi64(Kaeiou, Maeiou);
                b3 = _mm512_unpackhi_epi64(Kaeiou, Maeiou);
                Baeiou = _mm512_permutex2var_epi64(b0, pi2BG, b1);
                Gaeiou = _mm512_permutex2var_epi64(b2, pi2BG, b3);
                Kaeiou = _mm512_permutex2var_epi64(b0, pi2KM, b1);
                Maeiou = _mm512_permutex2var_epi64(b2, pi2KM, b3);
                Saeiou = _mm512_mask_blend_epi64(0x10, _mm512_permutex2var_epi64(b0, pi2S3, b1), Saeiou);
#else
                Aba ^= ((unsigned long long*)data)[0];
                Abe ^= ((unsigned long long*)data)[1];
                Abi ^= ((unsigned long long*)data)[2];
                Abo ^= ((unsigned long long*)data)[3];
                Abu ^= ((unsigned long long*)data)[4];
                Aga ^= ((unsigned long long*)data)[5];
                Age ^= ((unsigned long long*)data)[6];
                Agi ^= ((unsigned long long*)data)[7];
                Ago ^= ((unsigned long long*)data)[8];
                Agu ^= ((unsigned long long*)data)[9];
                Aka ^= ((unsigned long long*)data)[10];
                Ake ^= ((unsigned long long*)data)[11];
                Aki ^= ((unsigned long long*)data)[12];
                Ako ^= ((unsigned long long*)data)[13];
                Aku ^= ((unsigned long long*)data)[14];
                Ama ^= ((unsigned long long*)data)[15];
                Ame ^= ((unsigned long long*)data)[16];
                Ami ^= ((unsigned long long*)data)[17];
                Amo ^= ((unsigned long long*)data)[18];
                Amu ^= ((unsigned long long*)data)[19];
                Asa ^= ((unsigned long long*)data)[20];
                rounds12
#endif
                    data += K12_rateInBytes;
                modifiedDataByteLen -= K12_rateInBytes;
            }
#if AVX512
            _mm512_mask_storeu_epi64(instance->state, 0x1F, Baeiou);
            _mm512_mask_storeu_epi64(instance->state + 40, 0x1F, Gaeiou);
            _mm512_mask_storeu_epi64(instance->state + 80, 0x1F, Kaeiou);
            _mm512_mask_storeu_epi64(instance->state + 120, 0x1F, Maeiou);
            _mm512_mask_storeu_epi64(instance->state + 160, 0x1F, Saeiou);
#else
            copyToState(stateAsLanes)
#endif
                i = dataByteLen - modifiedDataByteLen;
        }
        else
        {
            unsigned char partialBlock;
            if ((dataByteLen - i) + instance->byteIOIndex > K12_rateInBytes)
            {
                partialBlock = K12_rateInBytes - instance->byteIOIndex;
            }
            else
            {
                partialBlock = (unsigned char)(dataByteLen - i);
            }
            i += partialBlock;

            if (!instance->byteIOIndex)
            {
                unsigned int j = 0;
                for (; (j + 8) <= (unsigned int)(partialBlock >> 3); j += 8)
                {
                    ((unsigned long long*)instance->state)[j + 0] ^= ((unsigned long long*)data)[j + 0];
                    ((unsigned long long*)instance->state)[j + 1] ^= ((unsigned long long*)data)[j + 1];
                    ((unsigned long long*)instance->state)[j + 2] ^= ((unsigned long long*)data)[j + 2];
                    ((unsigned long long*)instance->state)[j + 3] ^= ((unsigned long long*)data)[j + 3];
                    ((unsigned long long*)instance->state)[j + 4] ^= ((unsigned long long*)data)[j + 4];
                    ((unsigned long long*)instance->state)[j + 5] ^= ((unsigned long long*)data)[j + 5];
                    ((unsigned long long*)instance->state)[j + 6] ^= ((unsigned long long*)data)[j + 6];
                    ((unsigned long long*)instance->state)[j + 7] ^= ((unsigned long long*)data)[j + 7];
                }
                for (; (j + 4) <= (unsigned int)(partialBlock >> 3); j += 4)
                {
                    ((unsigned long long*)instance->state)[j + 0] ^= ((unsigned long long*)data)[j + 0];
                    ((unsigned long long*)instance->state)[j + 1] ^= ((unsigned long long*)data)[j + 1];
                    ((unsigned long long*)instance->state)[j + 2] ^= ((unsigned long long*)data)[j + 2];
                    ((unsigned long long*)instance->state)[j + 3] ^= ((unsigned long long*)data)[j + 3];
                }
                for (; (j + 2) <= (unsigned int)(partialBlock >> 3); j += 2)
                {
                    ((unsigned long long*)instance->state)[j + 0] ^= ((unsigned long long*)data)[j + 0];
                    ((unsigned long long*)instance->state)[j + 1] ^= ((unsigned long long*)data)[j + 1];
                }
                if (j < (unsigned int)(partialBlock >> 3))
                {
                    ((unsigned long long*)instance->state)[j + 0] ^= ((unsigned long long*)data)[j + 0];
                }
                if (partialBlock & 7)
                {
                    unsigned long long lane = 0;
                    memcpy(&lane, data + (partialBlock & 0xFFFFFFF8), partialBlock & 7);
                    ((unsigned long long*)instance->state)[partialBlock >> 3] ^= lane;
                }
            }
            else
            {
                unsigned int _sizeLeft = partialBlock;
                unsigned int _lanePosition = instance->byteIOIndex >> 3;
                unsigned int _offsetInLane = instance->byteIOIndex & 7;
                const unsigned char* _curData = data;
                while (_sizeLeft > 0)
                {
                    unsigned int _bytesInLane = 8 - _offsetInLane;
                    if (_bytesInLane > _sizeLeft)
                    {
                        _bytesInLane = _sizeLeft;
                    }
                    if (_bytesInLane)
                    {
                        unsigned long long lane = 0;
                        memcpy(&lane, (void*)_curData, _bytesInLane);
                        ((unsigned long long*)instance->state)[_lanePosition] ^= (lane << (_offsetInLane << 3));
                    }
                    _sizeLeft -= _bytesInLane;
                    _lanePosition++;
                    _offsetInLane = 0;
                    _curData += _bytesInLane;
                }
            }

            data += partialBlock;
            instance->byteIOIndex += partialBlock;
            if (instance->byteIOIndex == K12_rateInBytes)
            {
                KeccakP1600_Permute_12rounds(instance->state);
                instance->byteIOIndex = 0;
            }
        }
    }
}

static void KangarooTwelve(unsigned char* input, unsigned int inputByteLen, unsigned char* output, unsigned long long outputByteLen)
{
    KangarooTwelve_F queueNode;
    KangarooTwelve_F finalNode;
    unsigned int blockNumber, queueAbsorbedLen;

    memset(&finalNode, 0, sizeof(KangarooTwelve_F));
    const unsigned int len = inputByteLen ^ ((K12_chunkSize ^ inputByteLen) & -(K12_chunkSize < inputByteLen));
    KangarooTwelve_F_Absorb(&finalNode, input, len);
    input += len;
    inputByteLen -= len;
    if (len == K12_chunkSize && inputByteLen)
    {
        blockNumber = 1;
        queueAbsorbedLen = 0;
        finalNode.state[finalNode.byteIOIndex] ^= 0x03;
        if (++finalNode.byteIOIndex == K12_rateInBytes)
        {
            KeccakP1600_Permute_12rounds(finalNode.state);
            finalNode.byteIOIndex = 0;
        }
        else
        {
            finalNode.byteIOIndex = (finalNode.byteIOIndex + 7) & ~7;
        }

        while (inputByteLen > 0)
        {
            const unsigned int len = K12_chunkSize ^ ((inputByteLen ^ K12_chunkSize) & -(inputByteLen < K12_chunkSize));
            memset(&queueNode, 0, sizeof(KangarooTwelve_F));
            KangarooTwelve_F_Absorb(&queueNode, input, len);
            input += len;
            inputByteLen -= len;
            if (len == K12_chunkSize)
            {
                ++blockNumber;
                queueNode.state[queueNode.byteIOIndex] ^= K12_suffixLeaf;
                queueNode.state[K12_rateInBytes - 1] ^= 0x80;
                KeccakP1600_Permute_12rounds(queueNode.state);
                queueNode.byteIOIndex = K12_capacityInBytes;
                KangarooTwelve_F_Absorb(&finalNode, queueNode.state, K12_capacityInBytes);
            }
            else
            {
                queueAbsorbedLen = len;
            }
        }

        if (queueAbsorbedLen)
        {
            if (++queueNode.byteIOIndex == K12_rateInBytes)
            {
                KeccakP1600_Permute_12rounds(queueNode.state);
                queueNode.byteIOIndex = 0;
            }
            if (++queueAbsorbedLen == K12_chunkSize)
            {
                ++blockNumber;
                queueAbsorbedLen = 0;
                queueNode.state[queueNode.byteIOIndex] ^= K12_suffixLeaf;
                queueNode.state[K12_rateInBytes - 1] ^= 0x80;
                KeccakP1600_Permute_12rounds(queueNode.state);
                queueNode.byteIOIndex = K12_capacityInBytes;
                KangarooTwelve_F_Absorb(&finalNode, queueNode.state, K12_capacityInBytes);
            }
        }
        else
        {
            memset(queueNode.state, 0, sizeof(queueNode.state));
            queueNode.byteIOIndex = 1;
            queueAbsorbedLen = 1;
        }
    }
    else
    {
        if (len == K12_chunkSize)
        {
            blockNumber = 1;
            finalNode.state[finalNode.byteIOIndex] ^= 0x03;
            if (++finalNode.byteIOIndex == K12_rateInBytes)
            {
                KeccakP1600_Permute_12rounds(finalNode.state);
                finalNode.byteIOIndex = 0;
            }
            else
            {
                finalNode.byteIOIndex = (finalNode.byteIOIndex + 7) & ~7;
            }

            memset(queueNode.state, 0, sizeof(queueNode.state));
            queueNode.byteIOIndex = 1;
            queueAbsorbedLen = 1;
        }
        else
        {
            blockNumber = 0;
            if (++finalNode.byteIOIndex == K12_rateInBytes)
            {
                KeccakP1600_Permute_12rounds(finalNode.state);
                finalNode.state[0] ^= 0x07;
            }
            else
            {
                finalNode.state[finalNode.byteIOIndex] ^= 0x07;
            }
        }
    }

    if (blockNumber)
    {
        if (queueAbsorbedLen)
        {
            blockNumber++;
            queueNode.state[queueNode.byteIOIndex] ^= K12_suffixLeaf;
            queueNode.state[K12_rateInBytes - 1] ^= 0x80;
            KeccakP1600_Permute_12rounds(queueNode.state);
            KangarooTwelve_F_Absorb(&finalNode, queueNode.state, K12_capacityInBytes);
        }
        unsigned int n = 0;
        for (unsigned long long v = --blockNumber; v && (n < sizeof(unsigned long long)); ++n, v >>= 8)
        {
        }
        unsigned char encbuf[sizeof(unsigned long long) + 1 + 2];
        for (unsigned int i = 1; i <= n; ++i)
        {
            encbuf[i - 1] = (unsigned char)(blockNumber >> (8 * (n - i)));
        }
        encbuf[n] = (unsigned char)n;
        encbuf[++n] = 0xFF;
        encbuf[++n] = 0xFF;
        KangarooTwelve_F_Absorb(&finalNode, encbuf, ++n);
        finalNode.state[finalNode.byteIOIndex] ^= 0x06;
    }
    finalNode.state[K12_rateInBytes - 1] ^= 0x80;
    KeccakP1600_Permute_12rounds(finalNode.state);
    memcpy(output, finalNode.state, outputByteLen);
}

static void KangarooTwelve64To32(unsigned char* input, unsigned char* output)
{
#if AVX512
    __m512i Baeiou = _mm512_maskz_loadu_epi64(0x1F, input);
    __m512i Gaeiou = _mm512_set_epi64(0, 0, 0, 0, 0x0700, ((unsigned long long*)input)[7], ((unsigned long long*)input)[6], ((unsigned long long*)input)[5]);

    __m512i b0 = _mm512_ternarylogic_epi64(_mm512_ternarylogic_epi64(Baeiou, Gaeiou, zero, 0x96), zero, padding, 0x96);
    __m512i b1 = _mm512_permutexvar_epi64(moveThetaPrev, b0);
    b0 = _mm512_rol_epi64(_mm512_permutexvar_epi64(moveThetaNext, b0), 1);
    __m512i b2 = _mm512_permutexvar_epi64(pi1K, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(zero, b0, b1, 0x96), rhoK));
    __m512i b3 = _mm512_permutexvar_epi64(pi1M, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(zero, b0, b1, 0x96), rhoM));
    __m512i b4 = _mm512_permutexvar_epi64(pi1S, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(padding, b0, b1, 0x96), rhoS));
    __m512i b5 = _mm512_permutexvar_epi64(pi1G, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Gaeiou, b0, b1, 0x96), rhoG));
    b0 = _mm512_permutexvar_epi64(pi1B, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Baeiou, b0, b1, 0x96), rhoB));
    Baeiou = _mm512_xor_si512(_mm512_ternarylogic_epi64(b0, b5, b2, 0xD2), K12RoundConst0);
    Gaeiou = _mm512_ternarylogic_epi64(b5, b2, b3, 0xD2);
    __m512i Kaeiou = _mm512_ternarylogic_epi64(b2, b3, b4, 0xD2);
    __m512i Maeiou = _mm512_ternarylogic_epi64(b3, b4, b0, 0xD2);
    __m512i Saeiou = _mm512_ternarylogic_epi64(b4, b0, b5, 0xD2);
    b0 = _mm512_permutex2var_epi64(_mm512_unpacklo_epi64(Baeiou, Gaeiou), pi2S1, Saeiou);
    b2 = _mm512_permutex2var_epi64(_mm512_unpackhi_epi64(Baeiou, Gaeiou), pi2S2, Saeiou);
    b1 = _mm512_unpacklo_epi64(Kaeiou, Maeiou);
    b3 = _mm512_unpackhi_epi64(Kaeiou, Maeiou);
    Baeiou = _mm512_permutex2var_epi64(b0, pi2BG, b1);
    Gaeiou = _mm512_permutex2var_epi64(b2, pi2BG, b3);
    Kaeiou = _mm512_permutex2var_epi64(b0, pi2KM, b1);
    Maeiou = _mm512_permutex2var_epi64(b2, pi2KM, b3);
    Saeiou = _mm512_mask_blend_epi64(0x10, _mm512_permutex2var_epi64(b0, pi2S3, b1), Saeiou);

    b0 = _mm512_ternarylogic_epi64(_mm512_ternarylogic_epi64(Baeiou, Gaeiou, Kaeiou, 0x96), Maeiou, Saeiou, 0x96);
    b1 = _mm512_permutexvar_epi64(moveThetaPrev, b0);
    b0 = _mm512_rol_epi64(_mm512_permutexvar_epi64(moveThetaNext, b0), 1);
    b2 = _mm512_permutexvar_epi64(pi1K, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Kaeiou, b0, b1, 0x96), rhoK));
    b3 = _mm512_permutexvar_epi64(pi1M, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Maeiou, b0, b1, 0x96), rhoM));
    b4 = _mm512_permutexvar_epi64(pi1S, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Saeiou, b0, b1, 0x96), rhoS));
    b5 = _mm512_permutexvar_epi64(pi1G, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Gaeiou, b0, b1, 0x96), rhoG));
    b0 = _mm512_permutexvar_epi64(pi1B, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Baeiou, b0, b1, 0x96), rhoB));
    Baeiou = _mm512_xor_si512(_mm512_ternarylogic_epi64(b0, b5, b2, 0xD2), K12RoundConst1);
    Gaeiou = _mm512_ternarylogic_epi64(b5, b2, b3, 0xD2);
    Kaeiou = _mm512_ternarylogic_epi64(b2, b3, b4, 0xD2);
    Maeiou = _mm512_ternarylogic_epi64(b3, b4, b0, 0xD2);
    Saeiou = _mm512_ternarylogic_epi64(b4, b0, b5, 0xD2);
    b0 = _mm512_permutex2var_epi64(_mm512_unpacklo_epi64(Baeiou, Gaeiou), pi2S1, Saeiou);
    b2 = _mm512_permutex2var_epi64(_mm512_unpackhi_epi64(Baeiou, Gaeiou), pi2S2, Saeiou);
    b1 = _mm512_unpacklo_epi64(Kaeiou, Maeiou);
    b3 = _mm512_unpackhi_epi64(Kaeiou, Maeiou);
    Baeiou = _mm512_permutex2var_epi64(b0, pi2BG, b1);
    Gaeiou = _mm512_permutex2var_epi64(b2, pi2BG, b3);
    Kaeiou = _mm512_permutex2var_epi64(b0, pi2KM, b1);
    Maeiou = _mm512_permutex2var_epi64(b2, pi2KM, b3);
    Saeiou = _mm512_mask_blend_epi64(0x10, _mm512_permutex2var_epi64(b0, pi2S3, b1), Saeiou);

    b0 = _mm512_ternarylogic_epi64(_mm512_ternarylogic_epi64(Baeiou, Gaeiou, Kaeiou, 0x96), Maeiou, Saeiou, 0x96);
    b1 = _mm512_permutexvar_epi64(moveThetaPrev, b0);
    b0 = _mm512_rol_epi64(_mm512_permutexvar_epi64(moveThetaNext, b0), 1);
    b2 = _mm512_permutexvar_epi64(pi1K, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Kaeiou, b0, b1, 0x96), rhoK));
    b3 = _mm512_permutexvar_epi64(pi1M, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Maeiou, b0, b1, 0x96), rhoM));
    b4 = _mm512_permutexvar_epi64(pi1S, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Saeiou, b0, b1, 0x96), rhoS));
    b5 = _mm512_permutexvar_epi64(pi1G, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Gaeiou, b0, b1, 0x96), rhoG));
    b0 = _mm512_permutexvar_epi64(pi1B, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Baeiou, b0, b1, 0x96), rhoB));
    Baeiou = _mm512_xor_si512(_mm512_ternarylogic_epi64(b0, b5, b2, 0xD2), K12RoundConst2);
    Gaeiou = _mm512_ternarylogic_epi64(b5, b2, b3, 0xD2);
    Kaeiou = _mm512_ternarylogic_epi64(b2, b3, b4, 0xD2);
    Maeiou = _mm512_ternarylogic_epi64(b3, b4, b0, 0xD2);
    Saeiou = _mm512_ternarylogic_epi64(b4, b0, b5, 0xD2);
    b0 = _mm512_permutex2var_epi64(_mm512_unpacklo_epi64(Baeiou, Gaeiou), pi2S1, Saeiou);
    b2 = _mm512_permutex2var_epi64(_mm512_unpackhi_epi64(Baeiou, Gaeiou), pi2S2, Saeiou);
    b1 = _mm512_unpacklo_epi64(Kaeiou, Maeiou);
    b3 = _mm512_unpackhi_epi64(Kaeiou, Maeiou);
    Baeiou = _mm512_permutex2var_epi64(b0, pi2BG, b1);
    Gaeiou = _mm512_permutex2var_epi64(b2, pi2BG, b3);
    Kaeiou = _mm512_permutex2var_epi64(b0, pi2KM, b1);
    Maeiou = _mm512_permutex2var_epi64(b2, pi2KM, b3);
    Saeiou = _mm512_mask_blend_epi64(0x10, _mm512_permutex2var_epi64(b0, pi2S3, b1), Saeiou);

    b0 = _mm512_ternarylogic_epi64(_mm512_ternarylogic_epi64(Baeiou, Gaeiou, Kaeiou, 0x96), Maeiou, Saeiou, 0x96);
    b1 = _mm512_permutexvar_epi64(moveThetaPrev, b0);
    b0 = _mm512_rol_epi64(_mm512_permutexvar_epi64(moveThetaNext, b0), 1);
    b2 = _mm512_permutexvar_epi64(pi1K, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Kaeiou, b0, b1, 0x96), rhoK));
    b3 = _mm512_permutexvar_epi64(pi1M, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Maeiou, b0, b1, 0x96), rhoM));
    b4 = _mm512_permutexvar_epi64(pi1S, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Saeiou, b0, b1, 0x96), rhoS));
    b5 = _mm512_permutexvar_epi64(pi1G, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Gaeiou, b0, b1, 0x96), rhoG));
    b0 = _mm512_permutexvar_epi64(pi1B, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Baeiou, b0, b1, 0x96), rhoB));
    Baeiou = _mm512_xor_si512(_mm512_ternarylogic_epi64(b0, b5, b2, 0xD2), K12RoundConst3);
    Gaeiou = _mm512_ternarylogic_epi64(b5, b2, b3, 0xD2);
    Kaeiou = _mm512_ternarylogic_epi64(b2, b3, b4, 0xD2);
    Maeiou = _mm512_ternarylogic_epi64(b3, b4, b0, 0xD2);
    Saeiou = _mm512_ternarylogic_epi64(b4, b0, b5, 0xD2);
    b0 = _mm512_permutex2var_epi64(_mm512_unpacklo_epi64(Baeiou, Gaeiou), pi2S1, Saeiou);
    b2 = _mm512_permutex2var_epi64(_mm512_unpackhi_epi64(Baeiou, Gaeiou), pi2S2, Saeiou);
    b1 = _mm512_unpacklo_epi64(Kaeiou, Maeiou);
    b3 = _mm512_unpackhi_epi64(Kaeiou, Maeiou);
    Baeiou = _mm512_permutex2var_epi64(b0, pi2BG, b1);
    Gaeiou = _mm512_permutex2var_epi64(b2, pi2BG, b3);
    Kaeiou = _mm512_permutex2var_epi64(b0, pi2KM, b1);
    Maeiou = _mm512_permutex2var_epi64(b2, pi2KM, b3);
    Saeiou = _mm512_mask_blend_epi64(0x10, _mm512_permutex2var_epi64(b0, pi2S3, b1), Saeiou);

    b0 = _mm512_ternarylogic_epi64(_mm512_ternarylogic_epi64(Baeiou, Gaeiou, Kaeiou, 0x96), Maeiou, Saeiou, 0x96);
    b1 = _mm512_permutexvar_epi64(moveThetaPrev, b0);
    b0 = _mm512_rol_epi64(_mm512_permutexvar_epi64(moveThetaNext, b0), 1);
    b2 = _mm512_permutexvar_epi64(pi1K, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Kaeiou, b0, b1, 0x96), rhoK));
    b3 = _mm512_permutexvar_epi64(pi1M, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Maeiou, b0, b1, 0x96), rhoM));
    b4 = _mm512_permutexvar_epi64(pi1S, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Saeiou, b0, b1, 0x96), rhoS));
    b5 = _mm512_permutexvar_epi64(pi1G, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Gaeiou, b0, b1, 0x96), rhoG));
    b0 = _mm512_permutexvar_epi64(pi1B, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Baeiou, b0, b1, 0x96), rhoB));
    Baeiou = _mm512_xor_si512(_mm512_ternarylogic_epi64(b0, b5, b2, 0xD2), K12RoundConst4);
    Gaeiou = _mm512_ternarylogic_epi64(b5, b2, b3, 0xD2);
    Kaeiou = _mm512_ternarylogic_epi64(b2, b3, b4, 0xD2);
    Maeiou = _mm512_ternarylogic_epi64(b3, b4, b0, 0xD2);
    Saeiou = _mm512_ternarylogic_epi64(b4, b0, b5, 0xD2);
    b0 = _mm512_permutex2var_epi64(_mm512_unpacklo_epi64(Baeiou, Gaeiou), pi2S1, Saeiou);
    b2 = _mm512_permutex2var_epi64(_mm512_unpackhi_epi64(Baeiou, Gaeiou), pi2S2, Saeiou);
    b1 = _mm512_unpacklo_epi64(Kaeiou, Maeiou);
    b3 = _mm512_unpackhi_epi64(Kaeiou, Maeiou);
    Baeiou = _mm512_permutex2var_epi64(b0, pi2BG, b1);
    Gaeiou = _mm512_permutex2var_epi64(b2, pi2BG, b3);
    Kaeiou = _mm512_permutex2var_epi64(b0, pi2KM, b1);
    Maeiou = _mm512_permutex2var_epi64(b2, pi2KM, b3);
    Saeiou = _mm512_mask_blend_epi64(0x10, _mm512_permutex2var_epi64(b0, pi2S3, b1), Saeiou);

    b0 = _mm512_ternarylogic_epi64(_mm512_ternarylogic_epi64(Baeiou, Gaeiou, Kaeiou, 0x96), Maeiou, Saeiou, 0x96);
    b1 = _mm512_permutexvar_epi64(moveThetaPrev, b0);
    b0 = _mm512_rol_epi64(_mm512_permutexvar_epi64(moveThetaNext, b0), 1);
    b2 = _mm512_permutexvar_epi64(pi1K, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Kaeiou, b0, b1, 0x96), rhoK));
    b3 = _mm512_permutexvar_epi64(pi1M, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Maeiou, b0, b1, 0x96), rhoM));
    b4 = _mm512_permutexvar_epi64(pi1S, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Saeiou, b0, b1, 0x96), rhoS));
    b5 = _mm512_permutexvar_epi64(pi1G, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Gaeiou, b0, b1, 0x96), rhoG));
    b0 = _mm512_permutexvar_epi64(pi1B, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Baeiou, b0, b1, 0x96), rhoB));
    Baeiou = _mm512_xor_si512(_mm512_ternarylogic_epi64(b0, b5, b2, 0xD2), K12RoundConst5);
    Gaeiou = _mm512_ternarylogic_epi64(b5, b2, b3, 0xD2);
    Kaeiou = _mm512_ternarylogic_epi64(b2, b3, b4, 0xD2);
    Maeiou = _mm512_ternarylogic_epi64(b3, b4, b0, 0xD2);
    Saeiou = _mm512_ternarylogic_epi64(b4, b0, b5, 0xD2);
    b0 = _mm512_permutex2var_epi64(_mm512_unpacklo_epi64(Baeiou, Gaeiou), pi2S1, Saeiou);
    b2 = _mm512_permutex2var_epi64(_mm512_unpackhi_epi64(Baeiou, Gaeiou), pi2S2, Saeiou);
    b1 = _mm512_unpacklo_epi64(Kaeiou, Maeiou);
    b3 = _mm512_unpackhi_epi64(Kaeiou, Maeiou);
    Baeiou = _mm512_permutex2var_epi64(b0, pi2BG, b1);
    Gaeiou = _mm512_permutex2var_epi64(b2, pi2BG, b3);
    Kaeiou = _mm512_permutex2var_epi64(b0, pi2KM, b1);
    Maeiou = _mm512_permutex2var_epi64(b2, pi2KM, b3);
    Saeiou = _mm512_mask_blend_epi64(0x10, _mm512_permutex2var_epi64(b0, pi2S3, b1), Saeiou);

    b0 = _mm512_ternarylogic_epi64(_mm512_ternarylogic_epi64(Baeiou, Gaeiou, Kaeiou, 0x96), Maeiou, Saeiou, 0x96);
    b1 = _mm512_permutexvar_epi64(moveThetaPrev, b0);
    b0 = _mm512_rol_epi64(_mm512_permutexvar_epi64(moveThetaNext, b0), 1);
    b2 = _mm512_permutexvar_epi64(pi1K, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Kaeiou, b0, b1, 0x96), rhoK));
    b3 = _mm512_permutexvar_epi64(pi1M, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Maeiou, b0, b1, 0x96), rhoM));
    b4 = _mm512_permutexvar_epi64(pi1S, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Saeiou, b0, b1, 0x96), rhoS));
    b5 = _mm512_permutexvar_epi64(pi1G, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Gaeiou, b0, b1, 0x96), rhoG));
    b0 = _mm512_permutexvar_epi64(pi1B, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Baeiou, b0, b1, 0x96), rhoB));
    Baeiou = _mm512_xor_si512(_mm512_ternarylogic_epi64(b0, b5, b2, 0xD2), K12RoundConst6);
    Gaeiou = _mm512_ternarylogic_epi64(b5, b2, b3, 0xD2);
    Kaeiou = _mm512_ternarylogic_epi64(b2, b3, b4, 0xD2);
    Maeiou = _mm512_ternarylogic_epi64(b3, b4, b0, 0xD2);
    Saeiou = _mm512_ternarylogic_epi64(b4, b0, b5, 0xD2);
    b0 = _mm512_permutex2var_epi64(_mm512_unpacklo_epi64(Baeiou, Gaeiou), pi2S1, Saeiou);
    b2 = _mm512_permutex2var_epi64(_mm512_unpackhi_epi64(Baeiou, Gaeiou), pi2S2, Saeiou);
    b1 = _mm512_unpacklo_epi64(Kaeiou, Maeiou);
    b3 = _mm512_unpackhi_epi64(Kaeiou, Maeiou);
    Baeiou = _mm512_permutex2var_epi64(b0, pi2BG, b1);
    Gaeiou = _mm512_permutex2var_epi64(b2, pi2BG, b3);
    Kaeiou = _mm512_permutex2var_epi64(b0, pi2KM, b1);
    Maeiou = _mm512_permutex2var_epi64(b2, pi2KM, b3);
    Saeiou = _mm512_mask_blend_epi64(0x10, _mm512_permutex2var_epi64(b0, pi2S3, b1), Saeiou);

    b0 = _mm512_ternarylogic_epi64(_mm512_ternarylogic_epi64(Baeiou, Gaeiou, Kaeiou, 0x96), Maeiou, Saeiou, 0x96);
    b1 = _mm512_permutexvar_epi64(moveThetaPrev, b0);
    b0 = _mm512_rol_epi64(_mm512_permutexvar_epi64(moveThetaNext, b0), 1);
    b2 = _mm512_permutexvar_epi64(pi1K, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Kaeiou, b0, b1, 0x96), rhoK));
    b3 = _mm512_permutexvar_epi64(pi1M, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Maeiou, b0, b1, 0x96), rhoM));
    b4 = _mm512_permutexvar_epi64(pi1S, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Saeiou, b0, b1, 0x96), rhoS));
    b5 = _mm512_permutexvar_epi64(pi1G, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Gaeiou, b0, b1, 0x96), rhoG));
    b0 = _mm512_permutexvar_epi64(pi1B, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Baeiou, b0, b1, 0x96), rhoB));
    Baeiou = _mm512_xor_si512(_mm512_ternarylogic_epi64(b0, b5, b2, 0xD2), K12RoundConst7);
    Gaeiou = _mm512_ternarylogic_epi64(b5, b2, b3, 0xD2);
    Kaeiou = _mm512_ternarylogic_epi64(b2, b3, b4, 0xD2);
    Maeiou = _mm512_ternarylogic_epi64(b3, b4, b0, 0xD2);
    Saeiou = _mm512_ternarylogic_epi64(b4, b0, b5, 0xD2);
    b0 = _mm512_permutex2var_epi64(_mm512_unpacklo_epi64(Baeiou, Gaeiou), pi2S1, Saeiou);
    b2 = _mm512_permutex2var_epi64(_mm512_unpackhi_epi64(Baeiou, Gaeiou), pi2S2, Saeiou);
    b1 = _mm512_unpacklo_epi64(Kaeiou, Maeiou);
    b3 = _mm512_unpackhi_epi64(Kaeiou, Maeiou);
    Baeiou = _mm512_permutex2var_epi64(b0, pi2BG, b1);
    Gaeiou = _mm512_permutex2var_epi64(b2, pi2BG, b3);
    Kaeiou = _mm512_permutex2var_epi64(b0, pi2KM, b1);
    Maeiou = _mm512_permutex2var_epi64(b2, pi2KM, b3);
    Saeiou = _mm512_mask_blend_epi64(0x10, _mm512_permutex2var_epi64(b0, pi2S3, b1), Saeiou);

    b0 = _mm512_ternarylogic_epi64(_mm512_ternarylogic_epi64(Baeiou, Gaeiou, Kaeiou, 0x96), Maeiou, Saeiou, 0x96);
    b1 = _mm512_permutexvar_epi64(moveThetaPrev, b0);
    b0 = _mm512_rol_epi64(_mm512_permutexvar_epi64(moveThetaNext, b0), 1);
    b2 = _mm512_permutexvar_epi64(pi1K, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Kaeiou, b0, b1, 0x96), rhoK));
    b3 = _mm512_permutexvar_epi64(pi1M, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Maeiou, b0, b1, 0x96), rhoM));
    b4 = _mm512_permutexvar_epi64(pi1S, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Saeiou, b0, b1, 0x96), rhoS));
    b5 = _mm512_permutexvar_epi64(pi1G, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Gaeiou, b0, b1, 0x96), rhoG));
    b0 = _mm512_permutexvar_epi64(pi1B, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Baeiou, b0, b1, 0x96), rhoB));
    Baeiou = _mm512_xor_si512(_mm512_ternarylogic_epi64(b0, b5, b2, 0xD2), K12RoundConst8);
    Gaeiou = _mm512_ternarylogic_epi64(b5, b2, b3, 0xD2);
    Kaeiou = _mm512_ternarylogic_epi64(b2, b3, b4, 0xD2);
    Maeiou = _mm512_ternarylogic_epi64(b3, b4, b0, 0xD2);
    Saeiou = _mm512_ternarylogic_epi64(b4, b0, b5, 0xD2);
    b0 = _mm512_permutex2var_epi64(_mm512_unpacklo_epi64(Baeiou, Gaeiou), pi2S1, Saeiou);
    b2 = _mm512_permutex2var_epi64(_mm512_unpackhi_epi64(Baeiou, Gaeiou), pi2S2, Saeiou);
    b1 = _mm512_unpacklo_epi64(Kaeiou, Maeiou);
    b3 = _mm512_unpackhi_epi64(Kaeiou, Maeiou);
    Baeiou = _mm512_permutex2var_epi64(b0, pi2BG, b1);
    Gaeiou = _mm512_permutex2var_epi64(b2, pi2BG, b3);
    Kaeiou = _mm512_permutex2var_epi64(b0, pi2KM, b1);
    Maeiou = _mm512_permutex2var_epi64(b2, pi2KM, b3);
    Saeiou = _mm512_mask_blend_epi64(0x10, _mm512_permutex2var_epi64(b0, pi2S3, b1), Saeiou);

    b0 = _mm512_ternarylogic_epi64(_mm512_ternarylogic_epi64(Baeiou, Gaeiou, Kaeiou, 0x96), Maeiou, Saeiou, 0x96);
    b1 = _mm512_permutexvar_epi64(moveThetaPrev, b0);
    b0 = _mm512_rol_epi64(_mm512_permutexvar_epi64(moveThetaNext, b0), 1);
    b2 = _mm512_permutexvar_epi64(pi1K, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Kaeiou, b0, b1, 0x96), rhoK));
    b3 = _mm512_permutexvar_epi64(pi1M, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Maeiou, b0, b1, 0x96), rhoM));
    b4 = _mm512_permutexvar_epi64(pi1S, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Saeiou, b0, b1, 0x96), rhoS));
    b5 = _mm512_permutexvar_epi64(pi1G, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Gaeiou, b0, b1, 0x96), rhoG));
    b0 = _mm512_permutexvar_epi64(pi1B, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Baeiou, b0, b1, 0x96), rhoB));
    Baeiou = _mm512_xor_si512(_mm512_ternarylogic_epi64(b0, b5, b2, 0xD2), K12RoundConst9);
    Gaeiou = _mm512_ternarylogic_epi64(b5, b2, b3, 0xD2);
    Kaeiou = _mm512_ternarylogic_epi64(b2, b3, b4, 0xD2);
    Maeiou = _mm512_ternarylogic_epi64(b3, b4, b0, 0xD2);
    Saeiou = _mm512_ternarylogic_epi64(b4, b0, b5, 0xD2);
    b0 = _mm512_permutex2var_epi64(_mm512_unpacklo_epi64(Baeiou, Gaeiou), pi2S1, Saeiou);
    b2 = _mm512_permutex2var_epi64(_mm512_unpackhi_epi64(Baeiou, Gaeiou), pi2S2, Saeiou);
    b1 = _mm512_unpacklo_epi64(Kaeiou, Maeiou);
    b3 = _mm512_unpackhi_epi64(Kaeiou, Maeiou);
    Baeiou = _mm512_permutex2var_epi64(b0, pi2BG, b1);
    Gaeiou = _mm512_permutex2var_epi64(b2, pi2BG, b3);
    Kaeiou = _mm512_permutex2var_epi64(b0, pi2KM, b1);
    Maeiou = _mm512_permutex2var_epi64(b2, pi2KM, b3);
    Saeiou = _mm512_mask_blend_epi64(0x10, _mm512_permutex2var_epi64(b0, pi2S3, b1), Saeiou);

    b0 = _mm512_ternarylogic_epi64(_mm512_ternarylogic_epi64(Baeiou, Gaeiou, Kaeiou, 0x96), Maeiou, Saeiou, 0x96);
    b1 = _mm512_permutexvar_epi64(moveThetaPrev, b0);
    b0 = _mm512_rol_epi64(_mm512_permutexvar_epi64(moveThetaNext, b0), 1);
    b2 = _mm512_permutexvar_epi64(pi1K, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Kaeiou, b0, b1, 0x96), rhoK));
    b3 = _mm512_permutexvar_epi64(pi1M, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Maeiou, b0, b1, 0x96), rhoM));
    b4 = _mm512_permutexvar_epi64(pi1S, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Saeiou, b0, b1, 0x96), rhoS));
    b5 = _mm512_permutexvar_epi64(pi1G, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Gaeiou, b0, b1, 0x96), rhoG));
    b0 = _mm512_permutexvar_epi64(pi1B, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Baeiou, b0, b1, 0x96), rhoB));
    Baeiou = _mm512_xor_si512(_mm512_ternarylogic_epi64(b0, b5, b2, 0xD2), K12RoundConst10);
    Gaeiou = _mm512_ternarylogic_epi64(b5, b2, b3, 0xD2);
    Kaeiou = _mm512_ternarylogic_epi64(b2, b3, b4, 0xD2);
    Maeiou = _mm512_ternarylogic_epi64(b3, b4, b0, 0xD2);
    Saeiou = _mm512_ternarylogic_epi64(b4, b0, b5, 0xD2);
    b0 = _mm512_permutex2var_epi64(_mm512_unpacklo_epi64(Baeiou, Gaeiou), pi2S1, Saeiou);
    b2 = _mm512_permutex2var_epi64(_mm512_unpackhi_epi64(Baeiou, Gaeiou), pi2S2, Saeiou);
    b1 = _mm512_unpacklo_epi64(Kaeiou, Maeiou);
    b3 = _mm512_unpackhi_epi64(Kaeiou, Maeiou);
    Baeiou = _mm512_permutex2var_epi64(b0, pi2BG, b1);
    Gaeiou = _mm512_permutex2var_epi64(b2, pi2BG, b3);
    Kaeiou = _mm512_permutex2var_epi64(b0, pi2KM, b1);
    Maeiou = _mm512_permutex2var_epi64(b2, pi2KM, b3);
    Saeiou = _mm512_mask_blend_epi64(0x10, _mm512_permutex2var_epi64(b0, pi2S3, b1), Saeiou);

    b0 = _mm512_ternarylogic_epi64(_mm512_ternarylogic_epi64(Baeiou, Gaeiou, Kaeiou, 0x96), Maeiou, Saeiou, 0x96);
    b1 = _mm512_permutexvar_epi64(moveThetaPrev, b0);
    b0 = _mm512_rol_epi64(_mm512_permutexvar_epi64(moveThetaNext, b0), 1);
    b2 = _mm512_permutexvar_epi64(pi1K, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Kaeiou, b0, b1, 0x96), rhoK));
    b3 = _mm512_permutexvar_epi64(pi1M, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Maeiou, b0, b1, 0x96), rhoM));
    b4 = _mm512_permutexvar_epi64(pi1S, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Saeiou, b0, b1, 0x96), rhoS));
    b5 = _mm512_permutexvar_epi64(pi1G, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Gaeiou, b0, b1, 0x96), rhoG));
    b0 = _mm512_permutexvar_epi64(pi1B, _mm512_rolv_epi64(_mm512_ternarylogic_epi64(Baeiou, b0, b1, 0x96), rhoB));

    _mm512_mask_storeu_epi64(output, 0xF, _mm512_permutex2var_epi64(_mm512_permutex2var_epi64(_mm512_unpacklo_epi64(_mm512_xor_si512(_mm512_ternarylogic_epi64(b0, b5, b2, 0xD2), K12RoundConst11), _mm512_ternarylogic_epi64(b5, b2, b3, 0xD2)), pi2S1, _mm512_ternarylogic_epi64(b4, b0, b5, 0xD2)), pi2BG, _mm512_unpacklo_epi64(_mm512_ternarylogic_epi64(b2, b3, b4, 0xD2), _mm512_ternarylogic_epi64(b3, b4, b0, 0xD2))));
#else
    unsigned long long Aba, Abe, Abi, Abo, Abu;
    unsigned long long Aga, Age, Agi, Ago, Agu;
    unsigned long long Aka, Ake, Aki, Ako, Aku;
    unsigned long long Ama, Ame, Ami, Amo, Amu;
    unsigned long long Asa, Ase, Asi, Aso, Asu;
    unsigned long long Bba, Bbe, Bbi, Bbo, Bbu;
    unsigned long long Bga, Bge, Bgi, Bgo, Bgu;
    unsigned long long Bka, Bke, Bki, Bko, Bku;
    unsigned long long Bma, Bme, Bmi, Bmo, Bmu;
    unsigned long long Bsa, Bse, Bsi, Bso, Bsu;
    unsigned long long Ca, Ce, Ci, Co, Cu;
    unsigned long long Da, De, Di, Do, Du;
    unsigned long long Eba, Ebe, Ebi, Ebo, Ebu;
    unsigned long long Ega, Ege, Egi, Ego, Egu;
    unsigned long long Eka, Eke, Eki, Eko, Eku;
    unsigned long long Ema, Eme, Emi, Emo, Emu;
    unsigned long long Esa, Ese, Esi, Eso, Esu;

    Ca = ((unsigned long long*)input)[0] ^ ((unsigned long long*)input)[5] ^ 0x8000000000000000;
    Ce = ((unsigned long long*)input)[1] ^ ((unsigned long long*)input)[6];
    Ci = ((unsigned long long*)input)[2] ^ ((unsigned long long*)input)[7];
    Co = ((unsigned long long*)input)[3] ^ 0x0700;

    Da = ((unsigned long long*)input)[4] ^ ROL64(Ce, 1);
    De = Ca ^ ROL64(Ci, 1);
    Di = Ce ^ ROL64(Co, 1);
    Do = Ci ^ ROL64(((unsigned long long*)input)[4], 1);
    Du = Co ^ ROL64(Ca, 1);
    Aba = ((unsigned long long*)input)[0] ^ Da;
    Bbe = ROL64(((unsigned long long*)input)[6] ^ De, 44);
    Bbi = ROL64(Di, 43);
    Bbo = ROL64(Do, 21);
    Bbu = ROL64(Du, 14);
    Eba = Aba ^ _andn_u64(Bbe, Bbi) ^ 0x000000008000808bULL;
    Ebe = Bbe ^ _andn_u64(Bbi, Bbo);
    Ebi = Bbi ^ _andn_u64(Bbo, Bbu);
    Ebo = Bbo ^ _andn_u64(Bbu, Aba);
    Ebu = Bbu ^ _andn_u64(Aba, Bbe);
    Bga = ROL64(((unsigned long long*)input)[3] ^ Do, 28);
    Bge = ROL64(Du, 20);
    Bgi = ROL64(Da, 3);
    Bgo = ROL64(De, 45);
    Bgu = ROL64(Di, 61);
    Ega = Bga ^ _andn_u64(Bge, Bgi);
    Ege = Bge ^ _andn_u64(Bgi, Bgo);
    Egi = Bgi ^ _andn_u64(Bgo, Bgu);
    Ego = Bgo ^ _andn_u64(Bgu, Bga);
    Egu = Bgu ^ _andn_u64(Bga, Bge);
    Bka = ROL64(((unsigned long long*)input)[1] ^ De, 1);
    Bke = ROL64(((unsigned long long*)input)[7] ^ Di, 6);
    Bki = ROL64(Do, 25);
    Bko = ROL64(Du, 8);
    Bku = ROL64(Da ^ 0x8000000000000000, 18);
    Eka = Bka ^ _andn_u64(Bke, Bki);
    Eke = Bke ^ _andn_u64(Bki, Bko);
    Eki = Bki ^ _andn_u64(Bko, Bku);
    Eko = Bko ^ _andn_u64(Bku, Bka);
    Eku = Bku ^ _andn_u64(Bka, Bke);
    Bma = ROL64(((unsigned long long*)input)[4] ^ Du, 27);
    Bme = ROL64(((unsigned long long*)input)[5] ^ Da, 36);
    Bmi = ROL64(De, 10);
    Bmo = ROL64(Di, 15);
    Bmu = ROL64(Do, 56);
    Ema = Bma ^ _andn_u64(Bme, Bmi);
    Eme = Bme ^ _andn_u64(Bmi, Bmo);
    Emi = Bmi ^ _andn_u64(Bmo, Bmu);
    Emo = Bmo ^ _andn_u64(Bmu, Bma);
    Emu = Bmu ^ _andn_u64(Bma, Bme);
    Bsa = ROL64(((unsigned long long*)input)[2] ^ Di, 62);
    Bse = ROL64(Do ^ 0x0700, 55);
    Bsi = ROL64(Du, 39);
    Bso = ROL64(Da, 41);
    Bsu = ROL64(De, 2);
    Esa = Bsa ^ _andn_u64(Bse, Bsi);
    Ese = Bse ^ _andn_u64(Bsi, Bso);
    Esi = Bsi ^ _andn_u64(Bso, Bsu);
    Eso = Bso ^ _andn_u64(Bsu, Bsa);
    Esu = Bsu ^ _andn_u64(Bsa, Bse);
    Ca = Eba ^ Ega ^ Eka ^ Ema ^ Esa;
    Ce = Ebe ^ Ege ^ Eke ^ Eme ^ Ese;
    Ci = Ebi ^ Egi ^ Eki ^ Emi ^ Esi;
    Co = Ebo ^ Ego ^ Eko ^ Emo ^ Eso;
    Cu = Ebu ^ Egu ^ Eku ^ Emu ^ Esu;

    Da = Cu ^ ROL64(Ce, 1);
    De = Ca ^ ROL64(Ci, 1);
    Di = Ce ^ ROL64(Co, 1);
    Do = Ci ^ ROL64(Cu, 1);
    Du = Co ^ ROL64(Ca, 1);
    Eba ^= Da;
    Bbe = ROL64(Ege ^ De, 44);
    Bbi = ROL64(Eki ^ Di, 43);
    Bbo = ROL64(Emo ^ Do, 21);
    Bbu = ROL64(Esu ^ Du, 14);
    Aba = Eba ^ _andn_u64(Bbe, Bbi) ^ 0x800000000000008bULL;
    Abe = Bbe ^ _andn_u64(Bbi, Bbo);
    Abi = Bbi ^ _andn_u64(Bbo, Bbu);
    Abo = Bbo ^ _andn_u64(Bbu, Eba);
    Abu = Bbu ^ _andn_u64(Eba, Bbe);
    Bga = ROL64(Ebo ^ Do, 28);
    Bge = ROL64(Egu ^ Du, 20);
    Bgi = ROL64(Eka ^ Da, 3);
    Bgo = ROL64(Eme ^ De, 45);
    Bgu = ROL64(Esi ^ Di, 61);
    Aga = Bga ^ _andn_u64(Bge, Bgi);
    Age = Bge ^ _andn_u64(Bgi, Bgo);
    Agi = Bgi ^ _andn_u64(Bgo, Bgu);
    Ago = Bgo ^ _andn_u64(Bgu, Bga);
    Agu = Bgu ^ _andn_u64(Bga, Bge);
    Bka = ROL64(Ebe ^ De, 1);
    Bke = ROL64(Egi ^ Di, 6);
    Bki = ROL64(Eko ^ Do, 25);
    Bko = ROL64(Emu ^ Du, 8);
    Bku = ROL64(Esa ^ Da, 18);
    Aka = Bka ^ _andn_u64(Bke, Bki);
    Ake = Bke ^ _andn_u64(Bki, Bko);
    Aki = Bki ^ _andn_u64(Bko, Bku);
    Ako = Bko ^ _andn_u64(Bku, Bka);
    Aku = Bku ^ _andn_u64(Bka, Bke);
    Bma = ROL64(Ebu ^ Du, 27);
    Bme = ROL64(Ega ^ Da, 36);
    Bmi = ROL64(Eke ^ De, 10);
    Bmo = ROL64(Emi ^ Di, 15);
    Bmu = ROL64(Eso ^ Do, 56);
    Ama = Bma ^ _andn_u64(Bme, Bmi);
    Ame = Bme ^ _andn_u64(Bmi, Bmo);
    Ami = Bmi ^ _andn_u64(Bmo, Bmu);
    Amo = Bmo ^ _andn_u64(Bmu, Bma);
    Amu = Bmu ^ _andn_u64(Bma, Bme);
    Bsa = ROL64(Ebi ^ Di, 62);
    Bse = ROL64(Ego ^ Do, 55);
    Bsi = ROL64(Eku ^ Du, 39);
    Bso = ROL64(Ema ^ Da, 41);
    Bsu = ROL64(Ese ^ De, 2);
    Asa = Bsa ^ _andn_u64(Bse, Bsi);
    Ase = Bse ^ _andn_u64(Bsi, Bso);
    Asi = Bsi ^ _andn_u64(Bso, Bsu);
    Aso = Bso ^ _andn_u64(Bsu, Bsa);
    Asu = Bsu ^ _andn_u64(Bsa, Bse);
    Ca = Aba ^ Aga ^ Aka ^ Ama ^ Asa;
    Ce = Abe ^ Age ^ Ake ^ Ame ^ Ase;
    Ci = Abi ^ Agi ^ Aki ^ Ami ^ Asi;
    Co = Abo ^ Ago ^ Ako ^ Amo ^ Aso;
    Cu = Abu ^ Agu ^ Aku ^ Amu ^ Asu;

    Da = Cu ^ ROL64(Ce, 1);
    De = Ca ^ ROL64(Ci, 1);
    Di = Ce ^ ROL64(Co, 1);
    Do = Ci ^ ROL64(Cu, 1);
    Du = Co ^ ROL64(Ca, 1);
    Aba ^= Da;
    Bbe = ROL64(Age ^ De, 44);
    Bbi = ROL64(Aki ^ Di, 43);
    Bbo = ROL64(Amo ^ Do, 21);
    Bbu = ROL64(Asu ^ Du, 14);
    Eba = Aba ^ _andn_u64(Bbe, Bbi) ^ 0x8000000000008089ULL;
    Ebe = Bbe ^ _andn_u64(Bbi, Bbo);
    Ebi = Bbi ^ _andn_u64(Bbo, Bbu);
    Ebo = Bbo ^ _andn_u64(Bbu, Aba);
    Ebu = Bbu ^ _andn_u64(Aba, Bbe);
    Bga = ROL64(Abo ^ Do, 28);
    Bge = ROL64(Agu ^ Du, 20);
    Bgi = ROL64(Aka ^ Da, 3);
    Bgo = ROL64(Ame ^ De, 45);
    Bgu = ROL64(Asi ^ Di, 61);
    Ega = Bga ^ _andn_u64(Bge, Bgi);
    Ege = Bge ^ _andn_u64(Bgi, Bgo);
    Egi = Bgi ^ _andn_u64(Bgo, Bgu);
    Ego = Bgo ^ _andn_u64(Bgu, Bga);
    Egu = Bgu ^ _andn_u64(Bga, Bge);
    Bka = ROL64(Abe ^ De, 1);
    Bke = ROL64(Agi ^ Di, 6);
    Bki = ROL64(Ako ^ Do, 25);
    Bko = ROL64(Amu ^ Du, 8);
    Bku = ROL64(Asa ^ Da, 18);
    Eka = Bka ^ _andn_u64(Bke, Bki);
    Eke = Bke ^ _andn_u64(Bki, Bko);
    Eki = Bki ^ _andn_u64(Bko, Bku);
    Eko = Bko ^ _andn_u64(Bku, Bka);
    Eku = Bku ^ _andn_u64(Bka, Bke);
    Bma = ROL64(Abu ^ Du, 27);
    Bme = ROL64(Aga ^ Da, 36);
    Bmi = ROL64(Ake ^ De, 10);
    Bmo = ROL64(Ami ^ Di, 15);
    Bmu = ROL64(Aso ^ Do, 56);
    Ema = Bma ^ _andn_u64(Bme, Bmi);
    Eme = Bme ^ _andn_u64(Bmi, Bmo);
    Emi = Bmi ^ _andn_u64(Bmo, Bmu);
    Emo = Bmo ^ _andn_u64(Bmu, Bma);
    Emu = Bmu ^ _andn_u64(Bma, Bme);
    Bsa = ROL64(Abi ^ Di, 62);
    Bse = ROL64(Ago ^ Do, 55);
    Bsi = ROL64(Aku ^ Du, 39);
    Bso = ROL64(Ama ^ Da, 41);
    Bsu = ROL64(Ase ^ De, 2);
    Esa = Bsa ^ _andn_u64(Bse, Bsi);
    Ese = Bse ^ _andn_u64(Bsi, Bso);
    Esi = Bsi ^ _andn_u64(Bso, Bsu);
    Eso = Bso ^ _andn_u64(Bsu, Bsa);
    Esu = Bsu ^ _andn_u64(Bsa, Bse);
    Ca = Eba ^ Ega ^ Eka ^ Ema ^ Esa;
    Ce = Ebe ^ Ege ^ Eke ^ Eme ^ Ese;
    Ci = Ebi ^ Egi ^ Eki ^ Emi ^ Esi;
    Co = Ebo ^ Ego ^ Eko ^ Emo ^ Eso;
    Cu = Ebu ^ Egu ^ Eku ^ Emu ^ Esu;

    Da = Cu ^ ROL64(Ce, 1);
    De = Ca ^ ROL64(Ci, 1);
    Di = Ce ^ ROL64(Co, 1);
    Do = Ci ^ ROL64(Cu, 1);
    Du = Co ^ ROL64(Ca, 1);
    Eba ^= Da;
    Bbe = ROL64(Ege ^ De, 44);
    Bbi = ROL64(Eki ^ Di, 43);
    Bbo = ROL64(Emo ^ Do, 21);
    Bbu = ROL64(Esu ^ Du, 14);
    Aba = Eba ^ _andn_u64(Bbe, Bbi) ^ 0x8000000000008003ULL;
    Abe = Bbe ^ _andn_u64(Bbi, Bbo);
    Abi = Bbi ^ _andn_u64(Bbo, Bbu);
    Abo = Bbo ^ _andn_u64(Bbu, Eba);
    Abu = Bbu ^ _andn_u64(Eba, Bbe);
    Bga = ROL64(Ebo ^ Do, 28);
    Bge = ROL64(Egu ^ Du, 20);
    Bgi = ROL64(Eka ^ Da, 3);
    Bgo = ROL64(Eme ^ De, 45);
    Bgu = ROL64(Esi ^ Di, 61);
    Aga = Bga ^ _andn_u64(Bge, Bgi);
    Age = Bge ^ _andn_u64(Bgi, Bgo);
    Agi = Bgi ^ _andn_u64(Bgo, Bgu);
    Ago = Bgo ^ _andn_u64(Bgu, Bga);
    Agu = Bgu ^ _andn_u64(Bga, Bge);
    Bka = ROL64(Ebe ^ De, 1);
    Bke = ROL64(Egi ^ Di, 6);
    Bki = ROL64(Eko ^ Do, 25);
    Bko = ROL64(Emu ^ Du, 8);
    Bku = ROL64(Esa ^ Da, 18);
    Aka = Bka ^ _andn_u64(Bke, Bki);
    Ake = Bke ^ _andn_u64(Bki, Bko);
    Aki = Bki ^ _andn_u64(Bko, Bku);
    Ako = Bko ^ _andn_u64(Bku, Bka);
    Aku = Bku ^ _andn_u64(Bka, Bke);
    Bma = ROL64(Ebu ^ Du, 27);
    Bme = ROL64(Ega ^ Da, 36);
    Bmi = ROL64(Eke ^ De, 10);
    Bmo = ROL64(Emi ^ Di, 15);
    Bmu = ROL64(Eso ^ Do, 56);
    Ama = Bma ^ _andn_u64(Bme, Bmi);
    Ame = Bme ^ _andn_u64(Bmi, Bmo);
    Ami = Bmi ^ _andn_u64(Bmo, Bmu);
    Amo = Bmo ^ _andn_u64(Bmu, Bma);
    Amu = Bmu ^ _andn_u64(Bma, Bme);
    Bsa = ROL64(Ebi ^ Di, 62);
    Bse = ROL64(Ego ^ Do, 55);
    Bsi = ROL64(Eku ^ Du, 39);
    Bso = ROL64(Ema ^ Da, 41);
    Bsu = ROL64(Ese ^ De, 2);
    Asa = Bsa ^ _andn_u64(Bse, Bsi);
    Ase = Bse ^ _andn_u64(Bsi, Bso);
    Asi = Bsi ^ _andn_u64(Bso, Bsu);
    Aso = Bso ^ _andn_u64(Bsu, Bsa);
    Asu = Bsu ^ _andn_u64(Bsa, Bse);
    Ca = Aba ^ Aga ^ Aka ^ Ama ^ Asa;
    Ce = Abe ^ Age ^ Ake ^ Ame ^ Ase;
    Ci = Abi ^ Agi ^ Aki ^ Ami ^ Asi;
    Co = Abo ^ Ago ^ Ako ^ Amo ^ Aso;
    Cu = Abu ^ Agu ^ Aku ^ Amu ^ Asu;

    Da = Cu ^ ROL64(Ce, 1);
    De = Ca ^ ROL64(Ci, 1);
    Di = Ce ^ ROL64(Co, 1);
    Do = Ci ^ ROL64(Cu, 1);
    Du = Co ^ ROL64(Ca, 1);
    Aba ^= Da;
    Bbe = ROL64(Age ^ De, 44);
    Bbi = ROL64(Aki ^ Di, 43);
    Bbo = ROL64(Amo ^ Do, 21);
    Bbu = ROL64(Asu ^ Du, 14);
    Eba = Aba ^ _andn_u64(Bbe, Bbi) ^ 0x8000000000008002ULL;
    Ebe = Bbe ^ _andn_u64(Bbi, Bbo);
    Ebi = Bbi ^ _andn_u64(Bbo, Bbu);
    Ebo = Bbo ^ _andn_u64(Bbu, Aba);
    Ebu = Bbu ^ _andn_u64(Aba, Bbe);
    Bga = ROL64(Abo ^ Do, 28);
    Bge = ROL64(Agu ^ Du, 20);
    Bgi = ROL64(Aka ^ Da, 3);
    Bgo = ROL64(Ame ^ De, 45);
    Bgu = ROL64(Asi ^ Di, 61);
    Ega = Bga ^ _andn_u64(Bge, Bgi);
    Ege = Bge ^ _andn_u64(Bgi, Bgo);
    Egi = Bgi ^ _andn_u64(Bgo, Bgu);
    Ego = Bgo ^ _andn_u64(Bgu, Bga);
    Egu = Bgu ^ _andn_u64(Bga, Bge);
    Bka = ROL64(Abe ^ De, 1);
    Bke = ROL64(Agi ^ Di, 6);
    Bki = ROL64(Ako ^ Do, 25);
    Bko = ROL64(Amu ^ Du, 8);
    Bku = ROL64(Asa ^ Da, 18);
    Eka = Bka ^ _andn_u64(Bke, Bki);
    Eke = Bke ^ _andn_u64(Bki, Bko);
    Eki = Bki ^ _andn_u64(Bko, Bku);
    Eko = Bko ^ _andn_u64(Bku, Bka);
    Eku = Bku ^ _andn_u64(Bka, Bke);
    Bma = ROL64(Abu ^ Du, 27);
    Bme = ROL64(Aga ^ Da, 36);
    Bmi = ROL64(Ake ^ De, 10);
    Bmo = ROL64(Ami ^ Di, 15);
    Bmu = ROL64(Aso ^ Do, 56);
    Ema = Bma ^ _andn_u64(Bme, Bmi);
    Eme = Bme ^ _andn_u64(Bmi, Bmo);
    Emi = Bmi ^ _andn_u64(Bmo, Bmu);
    Emo = Bmo ^ _andn_u64(Bmu, Bma);
    Emu = Bmu ^ _andn_u64(Bma, Bme);
    Bsa = ROL64(Abi ^ Di, 62);
    Bse = ROL64(Ago ^ Do, 55);
    Bsi = ROL64(Aku ^ Du, 39);
    Bso = ROL64(Ama ^ Da, 41);
    Bsu = ROL64(Ase ^ De, 2);
    Esa = Bsa ^ _andn_u64(Bse, Bsi);
    Ese = Bse ^ _andn_u64(Bsi, Bso);
    Esi = Bsi ^ _andn_u64(Bso, Bsu);
    Eso = Bso ^ _andn_u64(Bsu, Bsa);
    Esu = Bsu ^ _andn_u64(Bsa, Bse);
    Ca = Eba ^ Ega ^ Eka ^ Ema ^ Esa;
    Ce = Ebe ^ Ege ^ Eke ^ Eme ^ Ese;
    Ci = Ebi ^ Egi ^ Eki ^ Emi ^ Esi;
    Co = Ebo ^ Ego ^ Eko ^ Emo ^ Eso;
    Cu = Ebu ^ Egu ^ Eku ^ Emu ^ Esu;

    Da = Cu ^ ROL64(Ce, 1);
    De = Ca ^ ROL64(Ci, 1);
    Di = Ce ^ ROL64(Co, 1);
    Do = Ci ^ ROL64(Cu, 1);
    Du = Co ^ ROL64(Ca, 1);
    Eba ^= Da;
    Bbe = ROL64(Ege ^ De, 44);
    Bbi = ROL64(Eki ^ Di, 43);
    Bbo = ROL64(Emo ^ Do, 21);
    Bbu = ROL64(Esu ^ Du, 14);
    Aba = Eba ^ _andn_u64(Bbe, Bbi) ^ 0x8000000000000080ULL;
    Abe = Bbe ^ _andn_u64(Bbi, Bbo);
    Abi = Bbi ^ _andn_u64(Bbo, Bbu);
    Abo = Bbo ^ _andn_u64(Bbu, Eba);
    Abu = Bbu ^ _andn_u64(Eba, Bbe);
    Bga = ROL64(Ebo ^ Do, 28);
    Bge = ROL64(Egu ^ Du, 20);
    Bgi = ROL64(Eka ^ Da, 3);
    Bgo = ROL64(Eme ^ De, 45);
    Bgu = ROL64(Esi ^ Di, 61);
    Aga = Bga ^ _andn_u64(Bge, Bgi);
    Age = Bge ^ _andn_u64(Bgi, Bgo);
    Agi = Bgi ^ _andn_u64(Bgo, Bgu);
    Ago = Bgo ^ _andn_u64(Bgu, Bga);
    Agu = Bgu ^ _andn_u64(Bga, Bge);
    Bka = ROL64(Ebe ^ De, 1);
    Bke = ROL64(Egi ^ Di, 6);
    Bki = ROL64(Eko ^ Do, 25);
    Bko = ROL64(Emu ^ Du, 8);
    Bku = ROL64(Esa ^ Da, 18);
    Aka = Bka ^ _andn_u64(Bke, Bki);
    Ake = Bke ^ _andn_u64(Bki, Bko);
    Aki = Bki ^ _andn_u64(Bko, Bku);
    Ako = Bko ^ _andn_u64(Bku, Bka);
    Aku = Bku ^ _andn_u64(Bka, Bke);
    Bma = ROL64(Ebu ^ Du, 27);
    Bme = ROL64(Ega ^ Da, 36);
    Bmi = ROL64(Eke ^ De, 10);
    Bmo = ROL64(Emi ^ Di, 15);
    Bmu = ROL64(Eso ^ Do, 56);
    Ama = Bma ^ _andn_u64(Bme, Bmi);
    Ame = Bme ^ _andn_u64(Bmi, Bmo);
    Ami = Bmi ^ _andn_u64(Bmo, Bmu);
    Amo = Bmo ^ _andn_u64(Bmu, Bma);
    Amu = Bmu ^ _andn_u64(Bma, Bme);
    Bsa = ROL64(Ebi ^ Di, 62);
    Bse = ROL64(Ego ^ Do, 55);
    Bsi = ROL64(Eku ^ Du, 39);
    Bso = ROL64(Ema ^ Da, 41);
    Bsu = ROL64(Ese ^ De, 2);
    Asa = Bsa ^ _andn_u64(Bse, Bsi);
    Ase = Bse ^ _andn_u64(Bsi, Bso);
    Asi = Bsi ^ _andn_u64(Bso, Bsu);
    Aso = Bso ^ _andn_u64(Bsu, Bsa);
    Asu = Bsu ^ _andn_u64(Bsa, Bse);
    Ca = Aba ^ Aga ^ Aka ^ Ama ^ Asa;
    Ce = Abe ^ Age ^ Ake ^ Ame ^ Ase;
    Ci = Abi ^ Agi ^ Aki ^ Ami ^ Asi;
    Co = Abo ^ Ago ^ Ako ^ Amo ^ Aso;
    Cu = Abu ^ Agu ^ Aku ^ Amu ^ Asu;

    Da = Cu ^ ROL64(Ce, 1);
    De = Ca ^ ROL64(Ci, 1);
    Di = Ce ^ ROL64(Co, 1);
    Do = Ci ^ ROL64(Cu, 1);
    Du = Co ^ ROL64(Ca, 1);
    Aba ^= Da;
    Bbe = ROL64(Age ^ De, 44);
    Bbi = ROL64(Aki ^ Di, 43);
    Bbo = ROL64(Amo ^ Do, 21);
    Bbu = ROL64(Asu ^ Du, 14);
    Eba = Aba ^ _andn_u64(Bbe, Bbi) ^ 0x000000000000800aULL;
    Ebe = Bbe ^ _andn_u64(Bbi, Bbo);
    Ebi = Bbi ^ _andn_u64(Bbo, Bbu);
    Ebo = Bbo ^ _andn_u64(Bbu, Aba);
    Ebu = Bbu ^ _andn_u64(Aba, Bbe);
    Bga = ROL64(Abo ^ Do, 28);
    Bge = ROL64(Agu ^ Du, 20);
    Bgi = ROL64(Aka ^ Da, 3);
    Bgo = ROL64(Ame ^ De, 45);
    Bgu = ROL64(Asi ^ Di, 61);
    Ega = Bga ^ _andn_u64(Bge, Bgi);
    Ege = Bge ^ _andn_u64(Bgi, Bgo);
    Egi = Bgi ^ _andn_u64(Bgo, Bgu);
    Ego = Bgo ^ _andn_u64(Bgu, Bga);
    Egu = Bgu ^ _andn_u64(Bga, Bge);
    Bka = ROL64(Abe ^ De, 1);
    Bke = ROL64(Agi ^ Di, 6);
    Bki = ROL64(Ako ^ Do, 25);
    Bko = ROL64(Amu ^ Du, 8);
    Bku = ROL64(Asa ^ Da, 18);
    Eka = Bka ^ _andn_u64(Bke, Bki);
    Eke = Bke ^ _andn_u64(Bki, Bko);
    Eki = Bki ^ _andn_u64(Bko, Bku);
    Eko = Bko ^ _andn_u64(Bku, Bka);
    Eku = Bku ^ _andn_u64(Bka, Bke);
    Bma = ROL64(Abu ^ Du, 27);
    Bme = ROL64(Aga ^ Da, 36);
    Bmi = ROL64(Ake ^ De, 10);
    Bmo = ROL64(Ami ^ Di, 15);
    Bmu = ROL64(Aso ^ Do, 56);
    Ema = Bma ^ _andn_u64(Bme, Bmi);
    Eme = Bme ^ _andn_u64(Bmi, Bmo);
    Emi = Bmi ^ _andn_u64(Bmo, Bmu);
    Emo = Bmo ^ _andn_u64(Bmu, Bma);
    Emu = Bmu ^ _andn_u64(Bma, Bme);
    Bsa = ROL64(Abi ^ Di, 62);
    Bse = ROL64(Ago ^ Do, 55);
    Bsi = ROL64(Aku ^ Du, 39);
    Bso = ROL64(Ama ^ Da, 41);
    Bsu = ROL64(Ase ^ De, 2);
    Esa = Bsa ^ _andn_u64(Bse, Bsi);
    Ese = Bse ^ _andn_u64(Bsi, Bso);
    Esi = Bsi ^ _andn_u64(Bso, Bsu);
    Eso = Bso ^ _andn_u64(Bsu, Bsa);
    Esu = Bsu ^ _andn_u64(Bsa, Bse);
    Ca = Eba ^ Ega ^ Eka ^ Ema ^ Esa;
    Ce = Ebe ^ Ege ^ Eke ^ Eme ^ Ese;
    Ci = Ebi ^ Egi ^ Eki ^ Emi ^ Esi;
    Co = Ebo ^ Ego ^ Eko ^ Emo ^ Eso;
    Cu = Ebu ^ Egu ^ Eku ^ Emu ^ Esu;

    Da = Cu ^ ROL64(Ce, 1);
    De = Ca ^ ROL64(Ci, 1);
    Di = Ce ^ ROL64(Co, 1);
    Do = Ci ^ ROL64(Cu, 1);
    Du = Co ^ ROL64(Ca, 1);
    Eba ^= Da;
    Bbe = ROL64(Ege ^ De, 44);
    Bbi = ROL64(Eki ^ Di, 43);
    Bbo = ROL64(Emo ^ Do, 21);
    Bbu = ROL64(Esu ^ Du, 14);
    Aba = Eba ^ _andn_u64(Bbe, Bbi) ^ 0x800000008000000aULL;
    Abe = Bbe ^ _andn_u64(Bbi, Bbo);
    Abi = Bbi ^ _andn_u64(Bbo, Bbu);
    Abo = Bbo ^ _andn_u64(Bbu, Eba);
    Abu = Bbu ^ _andn_u64(Eba, Bbe);
    Bga = ROL64(Ebo ^ Do, 28);
    Bge = ROL64(Egu ^ Du, 20);
    Bgi = ROL64(Eka ^ Da, 3);
    Bgo = ROL64(Eme ^ De, 45);
    Bgu = ROL64(Esi ^ Di, 61);
    Aga = Bga ^ _andn_u64(Bge, Bgi);
    Age = Bge ^ _andn_u64(Bgi, Bgo);
    Agi = Bgi ^ _andn_u64(Bgo, Bgu);
    Ago = Bgo ^ _andn_u64(Bgu, Bga);
    Agu = Bgu ^ _andn_u64(Bga, Bge);
    Bka = ROL64(Ebe ^ De, 1);
    Bke = ROL64(Egi ^ Di, 6);
    Bki = ROL64(Eko ^ Do, 25);
    Bko = ROL64(Emu ^ Du, 8);
    Bku = ROL64(Esa ^ Da, 18);
    Aka = Bka ^ _andn_u64(Bke, Bki);
    Ake = Bke ^ _andn_u64(Bki, Bko);
    Aki = Bki ^ _andn_u64(Bko, Bku);
    Ako = Bko ^ _andn_u64(Bku, Bka);
    Aku = Bku ^ _andn_u64(Bka, Bke);
    Bma = ROL64(Ebu ^ Du, 27);
    Bme = ROL64(Ega ^ Da, 36);
    Bmi = ROL64(Eke ^ De, 10);
    Bmo = ROL64(Emi ^ Di, 15);
    Bmu = ROL64(Eso ^ Do, 56);
    Ama = Bma ^ _andn_u64(Bme, Bmi);
    Ame = Bme ^ _andn_u64(Bmi, Bmo);
    Ami = Bmi ^ _andn_u64(Bmo, Bmu);
    Amo = Bmo ^ _andn_u64(Bmu, Bma);
    Amu = Bmu ^ _andn_u64(Bma, Bme);
    Bsa = ROL64(Ebi ^ Di, 62);
    Bse = ROL64(Ego ^ Do, 55);
    Bsi = ROL64(Eku ^ Du, 39);
    Bso = ROL64(Ema ^ Da, 41);
    Bsu = ROL64(Ese ^ De, 2);
    Asa = Bsa ^ _andn_u64(Bse, Bsi);
    Ase = Bse ^ _andn_u64(Bsi, Bso);
    Asi = Bsi ^ _andn_u64(Bso, Bsu);
    Aso = Bso ^ _andn_u64(Bsu, Bsa);
    Asu = Bsu ^ _andn_u64(Bsa, Bse);
    Ca = Aba ^ Aga ^ Aka ^ Ama ^ Asa;
    Ce = Abe ^ Age ^ Ake ^ Ame ^ Ase;
    Ci = Abi ^ Agi ^ Aki ^ Ami ^ Asi;
    Co = Abo ^ Ago ^ Ako ^ Amo ^ Aso;
    Cu = Abu ^ Agu ^ Aku ^ Amu ^ Asu;

    Da = Cu ^ ROL64(Ce, 1);
    De = Ca ^ ROL64(Ci, 1);
    Di = Ce ^ ROL64(Co, 1);
    Do = Ci ^ ROL64(Cu, 1);
    Du = Co ^ ROL64(Ca, 1);
    Aba ^= Da;
    Bbe = ROL64(Age ^ De, 44);
    Bbi = ROL64(Aki ^ Di, 43);
    Bbo = ROL64(Amo ^ Do, 21);
    Bbu = ROL64(Asu ^ Du, 14);
    Eba = Aba ^ _andn_u64(Bbe, Bbi) ^ 0x8000000080008081ULL;
    Ebe = Bbe ^ _andn_u64(Bbi, Bbo);
    Ebi = Bbi ^ _andn_u64(Bbo, Bbu);
    Ebo = Bbo ^ _andn_u64(Bbu, Aba);
    Ebu = Bbu ^ _andn_u64(Aba, Bbe);
    Bga = ROL64(Abo ^ Do, 28);
    Bge = ROL64(Agu ^ Du, 20);
    Bgi = ROL64(Aka ^ Da, 3);
    Bgo = ROL64(Ame ^ De, 45);
    Bgu = ROL64(Asi ^ Di, 61);
    Ega = Bga ^ _andn_u64(Bge, Bgi);
    Ege = Bge ^ _andn_u64(Bgi, Bgo);
    Egi = Bgi ^ _andn_u64(Bgo, Bgu);
    Ego = Bgo ^ _andn_u64(Bgu, Bga);
    Egu = Bgu ^ _andn_u64(Bga, Bge);
    Bka = ROL64(Abe ^ De, 1);
    Bke = ROL64(Agi ^ Di, 6);
    Bki = ROL64(Ako ^ Do, 25);
    Bko = ROL64(Amu ^ Du, 8);
    Bku = ROL64(Asa ^ Da, 18);
    Eka = Bka ^ _andn_u64(Bke, Bki);
    Eke = Bke ^ _andn_u64(Bki, Bko);
    Eki = Bki ^ _andn_u64(Bko, Bku);
    Eko = Bko ^ _andn_u64(Bku, Bka);
    Eku = Bku ^ _andn_u64(Bka, Bke);
    Bma = ROL64(Abu ^ Du, 27);
    Bme = ROL64(Aga ^ Da, 36);
    Bmi = ROL64(Ake ^ De, 10);
    Bmo = ROL64(Ami ^ Di, 15);
    Bmu = ROL64(Aso ^ Do, 56);
    Ema = Bma ^ _andn_u64(Bme, Bmi);
    Eme = Bme ^ _andn_u64(Bmi, Bmo);
    Emi = Bmi ^ _andn_u64(Bmo, Bmu);
    Emo = Bmo ^ _andn_u64(Bmu, Bma);
    Emu = Bmu ^ _andn_u64(Bma, Bme);
    Bsa = ROL64(Abi ^ Di, 62);
    Bse = ROL64(Ago ^ Do, 55);
    Bsi = ROL64(Aku ^ Du, 39);
    Bso = ROL64(Ama ^ Da, 41);
    Bsu = ROL64(Ase ^ De, 2);
    Esa = Bsa ^ _andn_u64(Bse, Bsi);
    Ese = Bse ^ _andn_u64(Bsi, Bso);
    Esi = Bsi ^ _andn_u64(Bso, Bsu);
    Eso = Bso ^ _andn_u64(Bsu, Bsa);
    Esu = Bsu ^ _andn_u64(Bsa, Bse);
    Ca = Eba ^ Ega ^ Eka ^ Ema ^ Esa;
    Ce = Ebe ^ Ege ^ Eke ^ Eme ^ Ese;
    Ci = Ebi ^ Egi ^ Eki ^ Emi ^ Esi;
    Co = Ebo ^ Ego ^ Eko ^ Emo ^ Eso;
    Cu = Ebu ^ Egu ^ Eku ^ Emu ^ Esu;

    Da = Cu ^ ROL64(Ce, 1);
    De = Ca ^ ROL64(Ci, 1);
    Di = Ce ^ ROL64(Co, 1);
    Do = Ci ^ ROL64(Cu, 1);
    Du = Co ^ ROL64(Ca, 1);
    Eba ^= Da;
    Bbe = ROL64(Ege ^ De, 44);
    Bbi = ROL64(Eki ^ Di, 43);
    Bbo = ROL64(Emo ^ Do, 21);
    Bbu = ROL64(Esu ^ Du, 14);
    Aba = Eba ^ _andn_u64(Bbe, Bbi) ^ 0x8000000000008080ULL;
    Abe = Bbe ^ _andn_u64(Bbi, Bbo);
    Abi = Bbi ^ _andn_u64(Bbo, Bbu);
    Abo = Bbo ^ _andn_u64(Bbu, Eba);
    Abu = Bbu ^ _andn_u64(Eba, Bbe);
    Bga = ROL64(Ebo ^ Do, 28);
    Bge = ROL64(Egu ^ Du, 20);
    Bgi = ROL64(Eka ^ Da, 3);
    Bgo = ROL64(Eme ^ De, 45);
    Bgu = ROL64(Esi ^ Di, 61);
    Aga = Bga ^ _andn_u64(Bge, Bgi);
    Age = Bge ^ _andn_u64(Bgi, Bgo);
    Agi = Bgi ^ _andn_u64(Bgo, Bgu);
    Ago = Bgo ^ _andn_u64(Bgu, Bga);
    Agu = Bgu ^ _andn_u64(Bga, Bge);
    Bka = ROL64(Ebe ^ De, 1);
    Bke = ROL64(Egi ^ Di, 6);
    Bki = ROL64(Eko ^ Do, 25);
    Bko = ROL64(Emu ^ Du, 8);
    Bku = ROL64(Esa ^ Da, 18);
    Aka = Bka ^ _andn_u64(Bke, Bki);
    Ake = Bke ^ _andn_u64(Bki, Bko);
    Aki = Bki ^ _andn_u64(Bko, Bku);
    Ako = Bko ^ _andn_u64(Bku, Bka);
    Aku = Bku ^ _andn_u64(Bka, Bke);
    Bma = ROL64(Ebu ^ Du, 27);
    Bme = ROL64(Ega ^ Da, 36);
    Bmi = ROL64(Eke ^ De, 10);
    Bmo = ROL64(Emi ^ Di, 15);
    Bmu = ROL64(Eso ^ Do, 56);
    Ama = Bma ^ _andn_u64(Bme, Bmi);
    Ame = Bme ^ _andn_u64(Bmi, Bmo);
    Ami = Bmi ^ _andn_u64(Bmo, Bmu);
    Amo = Bmo ^ _andn_u64(Bmu, Bma);
    Amu = Bmu ^ _andn_u64(Bma, Bme);
    Bsa = ROL64(Ebi ^ Di, 62);
    Bse = ROL64(Ego ^ Do, 55);
    Bsi = ROL64(Eku ^ Du, 39);
    Bso = ROL64(Ema ^ Da, 41);
    Bsu = ROL64(Ese ^ De, 2);
    Asa = Bsa ^ _andn_u64(Bse, Bsi);
    Ase = Bse ^ _andn_u64(Bsi, Bso);
    Asi = Bsi ^ _andn_u64(Bso, Bsu);
    Aso = Bso ^ _andn_u64(Bsu, Bsa);
    Asu = Bsu ^ _andn_u64(Bsa, Bse);
    Ca = Aba ^ Aga ^ Aka ^ Ama ^ Asa;
    Ce = Abe ^ Age ^ Ake ^ Ame ^ Ase;
    Ci = Abi ^ Agi ^ Aki ^ Ami ^ Asi;
    Co = Abo ^ Ago ^ Ako ^ Amo ^ Aso;
    Cu = Abu ^ Agu ^ Aku ^ Amu ^ Asu;

    Da = Cu ^ ROL64(Ce, 1);
    De = Ca ^ ROL64(Ci, 1);
    Di = Ce ^ ROL64(Co, 1);
    Do = Ci ^ ROL64(Cu, 1);
    Du = Co ^ ROL64(Ca, 1);
    Bba = Aba ^ Da;
    Bbe = ROL64(Age ^ De, 44);
    Bbi = ROL64(Aki ^ Di, 43);
    Bbo = ROL64(Amo ^ Do, 21);
    Bbu = ROL64(Asu ^ Du, 14);
    Bga = ROL64(Abo ^ Do, 28);
    Bge = ROL64(Agu ^ Du, 20);
    Bgi = ROL64(Aka ^ Da, 3);
    Bgo = ROL64(Ame ^ De, 45);
    Bgu = ROL64(Asi ^ Di, 61);
    Bka = ROL64(Abe ^ De, 1);
    Bke = ROL64(Agi ^ Di, 6);
    Bki = ROL64(Ako ^ Do, 25);
    Bko = ROL64(Amu ^ Du, 8);
    Bku = ROL64(Asa ^ Da, 18);
    Bma = ROL64(Abu ^ Du, 27);
    Bme = ROL64(Aga ^ Da, 36);
    Bmi = ROL64(Ake ^ De, 10);
    Bmo = ROL64(Ami ^ Di, 15);
    Bmu = ROL64(Aso ^ Do, 56);
    Bsa = ROL64(Abi ^ Di, 62);
    Bse = ROL64(Ago ^ Do, 55);
    Bsi = ROL64(Aku ^ Du, 39);
    Bso = ROL64(Ama ^ Da, 41);
    Bsu = ROL64(Ase ^ De, 2);
    Eba = Bba ^ _andn_u64(Bbe, Bbi) ^ 0x0000000080000001ULL;
    Ege = Bge ^ _andn_u64(Bgi, Bgo);
    Eki = Bki ^ _andn_u64(Bko, Bku);
    Emo = Bmo ^ _andn_u64(Bmu, Bma);
    Esu = Bsu ^ _andn_u64(Bsa, Bse);
    Ca = Eba ^ Bga ^ Bka ^ Bma ^ Bsa ^ _andn_u64(Bge, Bgi) ^ _andn_u64(Bke, Bki) ^ _andn_u64(Bme, Bmi) ^ _andn_u64(Bse, Bsi);
    Ce = Bbe ^ Ege ^ Bke ^ Bme ^ Bse ^ _andn_u64(Bbi, Bbo) ^ _andn_u64(Bki, Bko) ^ _andn_u64(Bmi, Bmo) ^ _andn_u64(Bsi, Bso);
    Ci = Bbi ^ Bgi ^ Eki ^ Bmi ^ Bsi ^ _andn_u64(Bbo, Bbu) ^ _andn_u64(Bgo, Bgu) ^ _andn_u64(Bmo, Bmu) ^ _andn_u64(Bso, Bsu);
    Co = Bbo ^ Bgo ^ Bko ^ Emo ^ Bso ^ _andn_u64(Bbu, Bba) ^ _andn_u64(Bgu, Bga) ^ _andn_u64(Bku, Bka) ^ _andn_u64(Bsu, Bsa);
    Cu = Bbu ^ Bgu ^ Bku ^ Bmu ^ Esu ^ _andn_u64(Bba, Bbe) ^ _andn_u64(Bga, Bge) ^ _andn_u64(Bka, Bke) ^ _andn_u64(Bma, Bme);

    Bba = Eba ^ Cu ^ ROL64(Ce, 1);
    Bbe = ROL64(Ege ^ Ca ^ ROL64(Ci, 1), 44);
    Bbi = ROL64(Eki ^ Ce ^ ROL64(Co, 1), 43);
    Bbo = ROL64(Emo ^ Ci ^ ROL64(Cu, 1), 21);
    Bbu = ROL64(Esu ^ Co ^ ROL64(Ca, 1), 14);
    ((unsigned long long*)output)[0] = Bba ^ _andn_u64(Bbe, Bbi) ^ 0x8000000080008008ULL;
    ((unsigned long long*)output)[1] = Bbe ^ _andn_u64(Bbi, Bbo);
    ((unsigned long long*)output)[2] = Bbi ^ _andn_u64(Bbo, Bbu);
    ((unsigned long long*)output)[3] = Bbo ^ _andn_u64(Bbu, Bba);
#endif
}

void random(unsigned char* publicKey, unsigned char* nonce, unsigned char* output, unsigned int outputSize)
{
    unsigned char state[200];
    memcpy(&state[0], publicKey, 32);
    memcpy(&state[32], nonce, 32);
    memset(&state[64], 0, sizeof(state) - 64);

    for (unsigned int i = 0; i < outputSize / sizeof(state); i++)
    {
        KeccakP1600_Permute_12rounds(state);
        memcpy(output, state, sizeof(state));
        output += sizeof(state);
    }
    if (outputSize % sizeof(state))
    {
        KeccakP1600_Permute_12rounds(state);
        memcpy(output, state, outputSize % sizeof(state));
    }
}

void random2(unsigned char* publicKey, unsigned char* nonce, unsigned char* output, unsigned int outputSize) // outputSize must be a multiple of 8
{
    unsigned char state[200];
    memcpy(&state[0], publicKey, 32);
    memcpy(&state[32], nonce, 32);
    memset(&state[64], 0, sizeof(state) - 64);

    unsigned char pool[1048576 + 24]; // Need a multiple of 200

    for (unsigned int i = 0; i < sizeof(pool); i += sizeof(state))
    {
        KeccakP1600_Permute_12rounds(state);
        memcpy(&pool[i], state, sizeof(state));
    }

    unsigned int x = 0; // The same sequence is always used, exploit this for optimization
    for (unsigned long long i = 0; i < outputSize; i += 8)
    {
        *((unsigned long long*) & output[i]) = *((unsigned long long*) & pool[x & (1048576 - 1)]);
        x = x * 1664525 + 1013904223; // https://en.wikipedia.org/wiki/Linear_congruential_generator#Parameters_in_common_use
    }
}

struct RequestResponseHeader
{
private:
    unsigned char _size[3];
    unsigned char _type;
    unsigned int _dejavu;

public:
    inline unsigned int size()
    {
        return (*((unsigned int*)_size)) & 0xFFFFFF;
    }

    inline void setSize(unsigned int size)
    {
        _size[0] = (unsigned char)size;
        _size[1] = (unsigned char)(size >> 8);
        _size[2] = (unsigned char)(size >> 16);
    }

    inline bool isDejavuZero() const
    {
        return !_dejavu;
    }

    inline void zeroDejavu()
    {
        _dejavu = 0;
    }


    inline unsigned int dejavu() const
    {
        return _dejavu;
    }

    inline void setDejavu(unsigned int dejavu)
    {
        _dejavu = dejavu;
    }

    inline void randomizeDejavu()
    {
        _rdrand32_step(&_dejavu);
        if (!_dejavu)
        {
            _dejavu = 1;
        }
    }

    inline unsigned char type() const
    {
        return _type;
    }

    inline void setType(const unsigned char type)
    {
        _type = type;
    }
};

#define BROADCAST_MESSAGE 1

typedef struct
{
    unsigned char sourcePublicKey[32];
    unsigned char destinationPublicKey[32];
    unsigned char gammingNonce[32];
} Message;

char* nodeIp = NULL;
int nodePort = 0;
static constexpr unsigned long long DATA_LENGTH = 256;
static constexpr unsigned long long NUMBER_OF_HIDDEN_NEURONS = 3000;
static constexpr unsigned long long NUMBER_OF_NEIGHBOR_NEURONS = 3000;
static constexpr unsigned long long MAX_DURATION = 3000*3000;
static constexpr unsigned long long NUMBER_OF_OPTIMIZATION_STEPS = 100;
static constexpr unsigned int SOLUTION_THRESHOLD = 130;

static_assert(((DATA_LENGTH + NUMBER_OF_HIDDEN_NEURONS + DATA_LENGTH)* NUMBER_OF_NEIGHBOR_NEURONS) % 64 == 0, "Synapse size need to be a multipler of 64");
static_assert(NUMBER_OF_OPTIMIZATION_STEPS < MAX_DURATION, "Number of retries need to smaller than MAX_DURATION");

struct Miner
{
    long long data[DATA_LENGTH];
    unsigned char computorPublicKey[32];
    unsigned char currentRandomSeed[32];

    void initialize(unsigned char randomSeed[32])
    {
        random(randomSeed, randomSeed, (unsigned char*)data, sizeof(data));
        for (unsigned long long i = 0; i < DATA_LENGTH; i++)
        {
            data[i] = (data[i] >= 0 ? 1 : -1);
        }

        memcpy(currentRandomSeed, randomSeed, sizeof(currentRandomSeed));
        memset(computorPublicKey, 0, sizeof(computorPublicKey));
    }

    void getComputorPublicKey(unsigned char computorPublicKey[32])
    {
        memcpy(computorPublicKey, this->computorPublicKey, sizeof(this->computorPublicKey));
    }

    void setComputorPublicKey(unsigned char computorPublicKey[32])
    {
        memcpy(this->computorPublicKey, computorPublicKey, sizeof(this->computorPublicKey));
    }

    struct
    {
        long long input[DATA_LENGTH + NUMBER_OF_HIDDEN_NEURONS + DATA_LENGTH];
    } neurons;
    struct
    {
        unsigned long long signs[(DATA_LENGTH + NUMBER_OF_HIDDEN_NEURONS + DATA_LENGTH) * NUMBER_OF_NEIGHBOR_NEURONS / 64];
        unsigned long long sequence[MAX_DURATION];
        // Use for randomly select skipped ticks
        unsigned long long skipTicksNumber[NUMBER_OF_OPTIMIZATION_STEPS];
    } synapses;

    // Save skipped ticks
    long long skipTicks[NUMBER_OF_OPTIMIZATION_STEPS];

    // Contained all ticks possible value
    long long ticksNumbers[MAX_DURATION];


    bool findSolution(unsigned char nonce[32])
    {
        _rdrand64_step((unsigned long long*)&nonce[0]);
        _rdrand64_step((unsigned long long*)&nonce[8]);
        _rdrand64_step((unsigned long long*)&nonce[16]);
        _rdrand64_step((unsigned long long*)&nonce[24]);
        random2(computorPublicKey, nonce, (unsigned char*)&synapses, sizeof(synapses));

        unsigned int score = 0;
        long long tailTick = MAX_DURATION - 1;
        for (long long tick = 0; tick < MAX_DURATION; tick++)
        {
            ticksNumbers[tick] = tick;
        }

        for (long long l = 0; l < NUMBER_OF_OPTIMIZATION_STEPS; l++)
        {
            skipTicks[l] = -1LL;
        }

        // Calculate the score with a list of randomly skipped ticks. This list grows if an additional skipped tick
        // does not worsen the score compared to the previous one.
        // - Initialize skippedTicks = []
        // - First, use all ticks. Compute score0 and update the score with score0.
        // - In the second run, ignore ticks in skippedTicks and try skipping a random tick 'a'.
        //    + Compute score1.
        //    + If score1 is not worse than score, add tick 'a' to skippedTicks and update the score with score1.
        //    + Otherwise, ignore tick 'a'.
        // - In the third run, ignore ticks in skippedTicks and try skipping a random tick 'b'.
        //    + Compute score2.
        //    + If score2 is not worse than score, add tick 'b' to skippedTicks and update the score with score2.
        //    + Otherwise, ignore tick 'b'.
        // - Continue this process iteratively.
        unsigned long long numberOfSkippedTicks = 0;
        long long skipTick = -1;
        for (long long l = 0; l < NUMBER_OF_OPTIMIZATION_STEPS; l++)
        {
            memset(&neurons, 0, sizeof(neurons));
            memcpy(&neurons.input[0], data, sizeof(data));

            for (long long tick = 0; tick < MAX_DURATION; tick++)
            {
                // Check if current tick should be skipped
                if (tick == skipTick)
                {
                    continue;
                }

                // Skip recorded skipped ticks
                bool tickShouldBeSkipped = false;
                for (long long tickIdx = 0; tickIdx < numberOfSkippedTicks; tickIdx++)
                {
                    if (skipTicks[tickIdx] == tick)
                    {
                        tickShouldBeSkipped = true;
                        break;
                    }
                }
                if (tickShouldBeSkipped)
                {
                    continue;
                }

                // Compute neurons
                const unsigned long long neuronIndex = DATA_LENGTH + synapses.sequence[tick] % (NUMBER_OF_HIDDEN_NEURONS + DATA_LENGTH);
                const unsigned long long neighborNeuronIndex = (synapses.sequence[tick] / (NUMBER_OF_HIDDEN_NEURONS + DATA_LENGTH)) % NUMBER_OF_NEIGHBOR_NEURONS;
                unsigned long long supplierNeuronIndex;
                if (neighborNeuronIndex < NUMBER_OF_NEIGHBOR_NEURONS / 2)
                {
                    supplierNeuronIndex = (neuronIndex - (NUMBER_OF_NEIGHBOR_NEURONS / 2) + neighborNeuronIndex + (DATA_LENGTH + NUMBER_OF_HIDDEN_NEURONS + DATA_LENGTH)) % (DATA_LENGTH + NUMBER_OF_HIDDEN_NEURONS + DATA_LENGTH);
                }
                else
                {
                    supplierNeuronIndex = (neuronIndex + 1 - (NUMBER_OF_NEIGHBOR_NEURONS / 2) + neighborNeuronIndex + (DATA_LENGTH + NUMBER_OF_HIDDEN_NEURONS + DATA_LENGTH)) % (DATA_LENGTH + NUMBER_OF_HIDDEN_NEURONS + DATA_LENGTH);
                }
                const unsigned long long offset = neuronIndex * NUMBER_OF_NEIGHBOR_NEURONS + neighborNeuronIndex;

                if (!(synapses.signs[offset / 64] & (1ULL << (offset % 64))))
                {
                    neurons.input[neuronIndex] += neurons.input[supplierNeuronIndex];
                }
                else
                {
                    neurons.input[neuronIndex] -= neurons.input[supplierNeuronIndex];
                }

                if (neurons.input[neuronIndex] > 1)
                {
                    neurons.input[neuronIndex] = 1;
                }
                if (neurons.input[neuronIndex] < -1)
                {
                    neurons.input[neuronIndex] = -1;
                }
            }

            // Compute the score
            unsigned int currentScore = 0;
            for (unsigned long long i = 0; i < DATA_LENGTH; i++)
            {
                if (data[i] == neurons.input[DATA_LENGTH + NUMBER_OF_HIDDEN_NEURONS + i])
                {
                    currentScore++;
                }
            }

            // Update score if below satisfied
            // - This is the first run without skipping any ticks
            // - Current score is not worse than previous score
            if (skipTick == -1 || currentScore >= score)
            {
                score = currentScore;
                // For the first run, don't need to update the skipped ticks list
                if (skipTick != -1 )
                {
                    skipTicks[numberOfSkippedTicks] = skipTick;
                    numberOfSkippedTicks++;
                }
            }

            // Randomly choose a tick to skip for the next round and avoid duplicated pick already chosen one
            long long randomTick = synapses.skipTicksNumber[l] % (MAX_DURATION - l);
            skipTick = ticksNumbers[randomTick];
            // Replace the chosen tick position with current tail to make sure if this tick is not chosen again
            // the skipTick is still not duplicated with previous ones.
            ticksNumbers[randomTick] = ticksNumbers[tailTick];
            tailTick--;

        }

        // Check score
        if (score >= SOLUTION_THRESHOLD)
        {
            return true;
        }

        return false;
    }
};

static std::atomic<char> state(0);

static unsigned char computorPublicKey[32];
static unsigned char randomSeed[32];
static std::atomic<long long> numberOfMiningIterations(0);
static std::atomic<unsigned int> numberOfFoundSolutions(0);
static std::queue<std::array<unsigned char, 32>> foundNonce;
std::mutex foundNonceLock;


#ifdef _MSC_VER
static BOOL WINAPI ctrlCHandlerRoutine(DWORD dwCtrlType)
{
    if (!state)
    {
        state = 1;
    }
    else // User force exit quickly
    {
        std::exit(1);
    }
    return TRUE;
}
#else
void ctrlCHandlerRoutine(int signum)
{
    if (!state)
    {
        state = 1;
    }
    else // User force exit quickly
    {
        std::exit(1);
    }
}
#endif

void consoleCtrlHandler()
{
#ifdef _MSC_VER
    SetConsoleCtrlHandler(ctrlCHandlerRoutine, TRUE);
#else
    signal(SIGINT, ctrlCHandlerRoutine);
#endif
}

int getSystemProcs()
{
#ifdef _MSC_VER
#else
#endif
    return 0;
}

template<unsigned long long num>
bool isZeros(const unsigned char* value)
{
    bool allZeros = true;
    for (unsigned long long i = 0; i < num; ++i)
    {
        if (value[i] != 0)
        {
            return false;
        }
    }
    return true;
}

int miningThreadProc()
{
    std::unique_ptr<Miner> miner(new Miner());
    miner->initialize(randomSeed);
    miner->setComputorPublicKey(computorPublicKey);

    std::array<unsigned char, 32> nonce;
    while (!state)
    {
        if (miner->findSolution(nonce.data()))
        {
            {
                std::lock_guard<std::mutex> guard(foundNonceLock);
                foundNonce.push(nonce);
            }
            numberOfFoundSolutions++;
        }

        numberOfMiningIterations++;
    }
    return 0;
}

struct ServerSocket
{
#ifdef _MSC_VER
    ServerSocket()
    {
        WSADATA wsaData;
        WSAStartup(MAKEWORD(2, 2), &wsaData);
    }

    ~ServerSocket()
    {
        WSACleanup();
    }

    void closeConnection()
    {
        closesocket(serverSocket);
    }

    bool establishConnection(char* address)
    {
        serverSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (serverSocket == INVALID_SOCKET)
        {
            printf("Fail to create a socket (%d)!\n", WSAGetLastError());
            return false;
        }

        sockaddr_in addr;
        ZeroMemory(&addr, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_port = htons(nodePort);
        sscanf_s(address, "%hhu.%hhu.%hhu.%hhu", &addr.sin_addr.S_un.S_un_b.s_b1, &addr.sin_addr.S_un.S_un_b.s_b2, &addr.sin_addr.S_un.S_un_b.s_b3, &addr.sin_addr.S_un.S_un_b.s_b4);
        if (connect(serverSocket, (const sockaddr*)&addr, sizeof(addr)))
        {
            printf("Fail to connect to %d.%d.%d.%d (%d)!\n", addr.sin_addr.S_un.S_un_b.s_b1, addr.sin_addr.S_un.S_un_b.s_b2, addr.sin_addr.S_un.S_un_b.s_b3, addr.sin_addr.S_un.S_un_b.s_b4, WSAGetLastError());
            closeConnection();
            return false;
        }

        return true;
    }

    SOCKET serverSocket;
#else
    void closeConnection()
    {
        close(serverSocket);
    }
    bool establishConnection(char* address)
    {
        serverSocket = socket(AF_INET, SOCK_STREAM, 0);
        if (serverSocket == -1)
        {
            printf("Fail to create a socket (%d)!\n", errno);
            return false;
        }

        sockaddr_in addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_port = htons(nodePort);
        if (inet_pton(AF_INET, address, &addr.sin_addr) <= 0)
        {
            printf("Invalid address/ Address not supported (%s)\n", address);
            return false;
        }

        if (connect(serverSocket, (struct sockaddr*)&addr, sizeof(addr)) < 0)
        {
            printf("Fail to connect to %s (%d)\n", address, errno);
            closeConnection();
            return false;
        }

        return true;
    }

    int serverSocket;
#endif

    bool sendData(char* buffer, unsigned int size)
    {
        while (size)
        {
            int numberOfBytes;
            if ((numberOfBytes = send(serverSocket, buffer, size, 0)) <= 0)
            {
                return false;
            }
            buffer += numberOfBytes;
            size -= numberOfBytes;
        }

        return true;
    }
    bool receiveData(char* buffer, unsigned int size)
    {
        const auto beginningTime = std::chrono::steady_clock::now();
        unsigned long long deltaTime = 0;
        while (size && deltaTime <= 2000)
        {
            int numberOfBytes;
            if ((numberOfBytes = recv(serverSocket, buffer, size, 0)) <= 0)
            {
                return false;
            }
            buffer += numberOfBytes;
            size -= numberOfBytes;
            deltaTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - beginningTime).count();
        }

        return true;
    }
};

static bool getPublicKeyFromIdentity(const unsigned char* id, unsigned char* publicKey)
{
    unsigned char publicKeyBuffer[32];
    for (int i = 0; i < 4; i++)
    {
        *((unsigned long long*) & publicKeyBuffer[i << 3]) = 0;
        for (int j = 14; j-- > 0; )
        {
            if (id[i * 14 + j] < 'A' || id[i * 14 + j] > 'Z')
            {
                return false;
            }

            *((unsigned long long*) & publicKeyBuffer[i << 3]) = *((unsigned long long*) & publicKeyBuffer[i << 3]) * 26 + (id[i * 14 + j] - 'A');
        }
    }
    memcpy(publicKey, publicKeyBuffer, 32);

    return true;
}
static void hexToByte(const char* hex, uint8_t* byte, const int sizeInByte)
{
    for (int i = 0; i < sizeInByte; i++){
        sscanf(hex+i*2, "%2hhx", &byte[i]);
    }
}

int main(int argc, char* argv[])
{
    std::vector<std::thread> miningThreads;

    if (argc != 6)
    {
        printf("Usage:   Qiner [Node IP] [Node Port] [Mining ID] [Mining Seed] [Number of threads]\n");
    }
    else
    {
        nodeIp = argv[1];
        nodePort = std::atoi(argv[2]);
        char* miningID = argv[3];
        printf("Qiner is launched. Connecting to %s:%d\n", nodeIp, nodePort);

        consoleCtrlHandler();

        if (!getPublicKeyFromIdentity((const unsigned char*)miningID, computorPublicKey))
        {
            printf("The Id is invalid!\n");
        }
        else
        {
            hexToByte(argv[4], randomSeed, 32);
            unsigned int numberOfThreads = atoi(argv[5]);
            printf("%d threads are used.\n", numberOfThreads);
            miningThreads.resize(numberOfThreads);
            for (unsigned int i = numberOfThreads; i-- > 0; )
            {
                miningThreads.emplace_back(miningThreadProc);
            }
            ServerSocket serverSocket;

            auto timestamp = std::chrono::steady_clock::now();
            long long prevNumberOfMiningIterations = 0;
            while (!state)
            {
                bool haveNonceToSend = false;
                size_t itemToSend = 0;
                std::array<unsigned char, 32> sendNonce;
                {
                    std::lock_guard<std::mutex> guard(foundNonceLock);
                    haveNonceToSend = foundNonce.size() > 0;
                    if (haveNonceToSend)
                    {
                        sendNonce = foundNonce.front();
                    }
                    itemToSend = foundNonce.size();
                }
                if (haveNonceToSend)
                {
                    if (serverSocket.establishConnection(nodeIp))
                    {
                        struct
                        {
                            RequestResponseHeader header;
                            Message message;
                            unsigned char solutionMiningSeed[32];
                            unsigned char solutionNonce[32];
                            unsigned char signature[64];
                        } packet;

                        packet.header.setSize(sizeof(packet));
                        packet.header.zeroDejavu();
                        packet.header.setType(BROADCAST_MESSAGE);

                        memset(packet.message.sourcePublicKey, 0, sizeof(packet.message.sourcePublicKey));
                        memcpy(packet.message.destinationPublicKey, computorPublicKey, sizeof(packet.message.destinationPublicKey));

                        unsigned char sharedKeyAndGammingNonce[64];
                        memset(sharedKeyAndGammingNonce, 0, 32);
                        unsigned char gammingKey[32];
                        do
                        {
                            _rdrand64_step((unsigned long long*) & packet.message.gammingNonce[0]);
                            _rdrand64_step((unsigned long long*) & packet.message.gammingNonce[8]);
                            _rdrand64_step((unsigned long long*) & packet.message.gammingNonce[16]);
                            _rdrand64_step((unsigned long long*) & packet.message.gammingNonce[24]);
                            memcpy(&sharedKeyAndGammingNonce[32], packet.message.gammingNonce, 32);
                            KangarooTwelve64To32(sharedKeyAndGammingNonce, gammingKey);
                        } while (gammingKey[0]);

                        unsigned char gamma[32 + 32];
                        KangarooTwelve(gammingKey, sizeof(gammingKey), gamma, sizeof(gamma));
                        for (unsigned int i = 0; i < 32; i++)
                        {
                            packet.solutionMiningSeed[i] = randomSeed[i] ^ gamma[i];
                            packet.solutionNonce[i] = sendNonce[i] ^ gamma[i + 32];
                        }

                        _rdrand64_step((unsigned long long*) & packet.signature[0]);
                        _rdrand64_step((unsigned long long*) & packet.signature[8]);
                        _rdrand64_step((unsigned long long*) & packet.signature[16]);
                        _rdrand64_step((unsigned long long*) & packet.signature[24]);
                        _rdrand64_step((unsigned long long*) & packet.signature[32]);
                        _rdrand64_step((unsigned long long*) & packet.signature[40]);
                        _rdrand64_step((unsigned long long*) & packet.signature[48]);
                        _rdrand64_step((unsigned long long*) & packet.signature[56]);

                        if (serverSocket.sendData((char*)&packet, packet.header.size()))
                        {
                            std::lock_guard<std::mutex> guard(foundNonceLock);
                            // Send data successfully. Remove it from the queue
                            foundNonce.pop();
                            itemToSend = foundNonce.size();
                        }
                        serverSocket.closeConnection();
                    }
                }

                std::this_thread::sleep_for(std::chrono::duration < double, std::milli>(1000));

                unsigned long long delta = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - timestamp).count();
                if (delta >= 1000)
                {
                    // Get current time in UTC
                    std::time_t now_time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
                    std::tm* utc_time = std::gmtime(&now_time);
                    printf("|   %04d-%02d-%02d %02d:%02d:%02d   |   %llu it/s   |   %d solutions   |   %.10s...   |\n",
                        utc_time->tm_year + 1900, utc_time->tm_mon, utc_time->tm_mday, utc_time->tm_hour, utc_time->tm_min, utc_time->tm_sec,
                        (numberOfMiningIterations - prevNumberOfMiningIterations) * 1000 / delta, numberOfFoundSolutions.load(), miningID);
                    prevNumberOfMiningIterations = numberOfMiningIterations;
                    timestamp = std::chrono::steady_clock::now();
                }
            }
        }
        printf("Shutting down...Press Ctrl+C again to force stop.\n");

        // Wait for all threads to join
        for (auto& miningTh : miningThreads)
        {
            if (miningTh.joinable())
            {
                miningTh.join();
            }
        }
        printf("Qiner is shut down.\n");
    }

    return 0;
}