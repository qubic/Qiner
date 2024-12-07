#pragma once
bool getSubseedFromSeed(const unsigned char* seed, unsigned char* subseed);
void getPrivateKeyFromSubSeed(const unsigned char* seed, unsigned char* privateKey);
void getPublicKeyFromPrivateKey(const unsigned char* privateKey, unsigned char* publicKey);
void getIdentityFromPublicKey(const unsigned char* pubkey, char* identity, bool isLowerCase);
void getTxHashFromDigest(const unsigned char* digest, char* txHash);
void getPublicKeyFromIdentity(const char* identity, unsigned char* publicKey);
bool checkSumIdentity(char* identity);