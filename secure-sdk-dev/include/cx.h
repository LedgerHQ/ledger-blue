/*******************************************************************************
*   Ledger Blue - secure firmware
*   (c) 2016 Ledger
*
*  Licensed under the Apache License, Version 2.0 (the "License");
*  you may not use this file except in compliance with the License.
*  You may obtain a copy of the License at
*
*      http://www.apache.org/licenses/LICENSE-2.0
*
*  Unless required by applicable law or agreed to in writing, software
*  distributed under the License is distributed on an "AS IS" BASIS,
*  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*  See the License for the specific language governing permissions and
*  limitations under the License.
********************************************************************************/


#ifndef CX_H
#define CX_H

/* ####################################################################### */
/*                                    OPTIONS                              */
/* ####################################################################### */

/* ####################################################################### */
/*                                  CHIP/LIB3rd                            */
/* ####################################################################### */

/* ####################################################################### */
/*                                  COMMON                                 */
/* ####################################################################### */
/*
 * Crypto mode encoding:
 * =====================
 *
 * size:
 * -----
 *  int, a least 16 bits
 *
 * encoding:
 * ---------
 *  | bit pos   |  H constant        |   meanings
 *  ---------------------------------------------------
 *  |  0        |  CX_LAST           | last block
 *  |           |                    |
 *
 *  |  2:1      |  CX_ENCRYPT        |
 *  |           |  CX_DECRYPT        |
 *  |           |  CX_SIGN           |
 *  |           |  CX_VERIFY         |
 *
 *  |  5:3      |  CX_PAD_NONE       |
 *  |           |  CX_PAD_ISO9797M1  |
 *  |           |  CX_PAD_ISO9797M2  |
 *  |           |  CX_PAD_PKCS1_1o5    |
 *
 *  |  7:6      |  CX_CHAIN_ECB      |
 *  |           |  CX_CHAIN_CBC      |
 *
 *  |  9:8      |  CX_RND_TRNG       |
 *              |  CX_RND_RFC6979    |
 *
 *  |  11:10    |  CX_ECDH_POINT     | share full point
 *              |  CX_ECDH_X         | share only x coordinate
 *
 *  |  12        | CX_DISCARD        | do not reinitialize context on CX_LAST
 when supported

 *  |  16:13    |  RFU               |
 */

/**
 * Bit 0
 */
#define CX_LAST (1 << 0)

/**
 * Bit 1
 */
#define CX_SIG_MODE (1 << 1)

/**
 * Bit 2:1
 */
#define CX_MASK_SIGCRYPT (3 << 1)
#define CX_ENCRYPT (2 << 1)
#define CX_DECRYPT (0 << 1)
#define CX_SIGN (CX_SIG_MODE | CX_ENCRYPT)
#define CX_VERIFY (CX_SIG_MODE | CX_DECRYPT)

/**
 * Bit 5:3
 */
#define CX_MASK_PAD (7 << 3)
#define CX_PAD_NONE (0 << 3)
#define CX_PAD_ISO9797M1 (1 << 3)
#define CX_PAD_ISO9797M2 (2 << 3)
#define CX_PAD_PKCS1_1o5 (3 << 3)

/**
 * Bit 7:6
 */
#define CX_MASK_CHAIN (3 << 6)
#define CX_CHAIN_ECB (0 << 6)
#define CX_CHAIN_CBC (1 << 6)

/**
 * Bit 9:8
 */
#define CX_MASK_RND (3 << 8)
#define CX_RND_PRNG (1 << 8)
#define CX_RND_TRNG (2 << 8)
#define CX_RND_RFC6979 (3 << 8)

/**
 * Bit 11:10
 */
#define CX_MASK_ECDH (3 << 10)
#define CX_ECDH_POINT (1 << 10)
#define CX_ECDH_X (2 << 10)

/**
 * Bit 12
 */
#define CX_DISCARD (1 << 12)

/* ####################################################################### */
/*                                   RAND                                  */
/* ####################################################################### */

/**
 * generate a random char
 */
SYSCALL unsigned char cx_rng_u8(void);

/**
 * generate a random buffer
 */
SYSCALL unsigned char *cx_rng(unsigned char *buffer PLENGTH(len), int len);

/* ####################################################################### */
/*                                 HASH/HMAC                               */
/* ####################################################################### */

/* ======================================================================= */
/*                                   HASH                                 */
/* ======================================================================= */
/*
 * ripemd160 :protocole standard
 * sha256    :protocole standard
 * sha512    :bip32
 */

enum cx_md_e {
    CX_NONE,
    CX_RIPEMD160, // 20 bytes
    CX_SHA224, // 28 bytes
    CX_SHA256, // 32 bytes
    CX_SHA384, // 48 bytes
    CX_SHA512, // 64 bytes
};
typedef enum cx_md_e cx_md_t;

#define CX_RIPEMD160_SIZE 20
#define CX_SHA256_SIZE 32
#define CX_SHA512_SIZE 64

#define CX_HASH_MAX_BLOCK_COUNT 65535

struct cx_hash_header_s {
    cx_md_t algo;
    unsigned int counter;
};

struct cx_ripemd160_s {
    struct cx_hash_header_s header;
    // 64 bytes per block
    int blen;
    unsigned char block[64];
    // five 32bits words
    unsigned char acc[5 * 4];
};
typedef struct cx_ripemd160_s cx_ripemd160_t;

struct cx_sha256_s {
    struct cx_hash_header_s header;
#if defined(CX_SHA256_NES_LIB)
    tNesLibSHA256State state;
#else
    // 64 bytes per block
    int blen;
    unsigned char block[64];
    // eight 32bits words
    unsigned char acc[8 * 4];
#endif
};
typedef struct cx_sha256_s cx_sha256_t;

struct cx_sha512_s {
    struct cx_hash_header_s header;
#if defined(CX_SHA512_NES_LIB)
    tNesLibSHA512State state;
#else
    // 128 bytes per block
    int blen;
    unsigned char block[128];
    // eight 64bits words
    unsigned char acc[8 * 8];
#endif
};
typedef struct cx_sha512_s cx_sha512_t;

typedef struct cx_hash_header_s cx_hash_t;

/**
 * Init a ripmd160 context.
 *
 * @param [out] hash the context to init.
 *    The context shall be in RAM
 *
 * @return algorithm identifier
 */
SYSCALL int
cx_ripemd160_init(cx_ripemd160_t *hash PLENGTH(sizeof(cx_ripemd160_t)));

/**
 * Init a sha256 context.
 *
 * @param [out] hash the context to init.
 *    The context shall be in RAM
 *
 * @return algorithm identifier
 */
SYSCALL int cx_sha256_init(cx_sha256_t *hash PLENGTH(sizeof(cx_sha256_t)));

/**
 * Init a sha512 context.
 *
 * @param [out] hash the context to init.
 *    The context shall be in RAM
 *
 * @return algorithm identifier
 */
SYSCALL int cx_sha512_init(cx_sha512_t *hash PLENGTH(sizeof(cx_sha512_t)));

/**
 * Add more data to hash.
 *
 * @param  [in/out] hash
 *   Univers Continuation Blob.
 *   The hash context pointer shall point to  either a cx_ripemd160_t, either a
 * cx_sha256_t  or cx_sha512_t .
 *   The hash context shall be inited with 'cx_xxx_init'
 *   The hash context shall be in RAM
 *   The function should be called with a nice cast.
 *
 * @param  [in] mode
 *   16bits flags. See Above
 *   If CX_LAST is set, context is automatically re-inited.
 *   Supported flags:
 *     - CX_LAST
 *
 * @param  [in] in
 *   Input data to add to current hash
 *
 * @param  [in] len
 *   Length of input to data.
 *
 * @param [out] out
 *   Either:
 *     - NULL (ignored) if CX_LAST is NOT set
 *     - produced hash  if CX_LAST is set
 *   'out' length is implicit, no check is done
 *
 */
SYSCALL int
cx_hash(cx_hash_t *hash PLENGTH(sizeof(cx_sha512_t) /*max for sha 512*/),
        int mode, unsigned char WIDE *in PLENGTH(len), int len,
        unsigned char *out PLENGTH(64 /*max for sha 512*/));

/* ======================================================================= */
/*                                 HASH MAC                                */
/* ======================================================================= */

/*
 * hmac : bip32 seed extension
 */

/*Note: DO NOT reorder the following structures, it will break magic casts */

struct cx_hmac_ripemd160_s {
    struct cx_ripemd160_s hash;
    // 64 bytes key
    unsigned char key_len;
    unsigned char key[64];
};
typedef struct cx_hmac_ripemd160_s cx_hmac_ripemd160_t;

struct cx_hmac_sha256_s {
    struct cx_sha256_s hash;
    // 64 bytes key
    unsigned char key_len;
    unsigned char key[64];
};
typedef struct cx_hmac_sha256_s cx_hmac_sha256_t;

struct cx_hmac_sha512_s {
    struct cx_sha512_s hash;
    // 128 bytes key
    unsigned char key_len;
    unsigned char key[128];
};
typedef struct cx_hmac_sha512_s cx_hmac_sha512_t;

typedef struct cx_hash_header_s cx_hmac_t;

/**
 * Init a hmac sha512 context.
 *
 * @param  [out] hash        the context to init.
 *    The context shall be in RAM
 *
 * @param  [in] key         hmac key value
 *    Passing a NULL pointeur, will reinit the context with the previously set
 * key.
 *    If no key has already been set, passing NULL will lead into an undefined
 * behavior.
 *
 * @param  [in] key_len     hmac key length
 *    The key length shall be less than 64 bytes
 *
 * @return algorithm  identifier
 */
SYSCALL int cx_hmac_ripemd160_init(
    cx_hmac_ripemd160_t *hmac PLENGTH(sizeof(cx_hmac_ripemd160_t)),
    unsigned char WIDE *key PLENGTH(key_len), int key_len);

/**
 * Init a hmac sha256 context.
 *
 * @param [out] hash        the context to init.
 *    The context shall be in RAM
 *
 * @param [in] key         hmac key value
 *    Passing a NULL pointeur, will reinit the context with the previously set
 * key.
 *    If no key has already been set, passing NULL will lead into an undefined
 * behavior.
 *
 * @param [in] key_len     hmac key length
 *    The key length shall be less than 64 bytes
 *
 * @return algorithm  identifier
 */
SYSCALL int
cx_hmac_sha256_init(cx_hmac_sha256_t *hmac PLENGTH(sizeof(cx_hmac_sha256_t)),
                    unsigned char WIDE *key PLENGTH(key_len), int key_len);

/**
 * Init a hmac sha512 context.
 *
 * @param [out] hash        the context to init.
 *    The context shall be in RAM
 *
 * @param [in] key         hmac key value
 *    Passing a NULL pointeur, will reinit the context with the previously set
 * key.
 *    If no key has already been set, passing NULL will lead into an undefined
 * behavior.
 *
 * @param [in] key_len     hmac key length
 *    The key length shall be less than 128 bytes
 *
 * @return algorithm  identifier
 */
SYSCALL int
cx_hmac_sha512_init(cx_hmac_sha512_t *hmac PLENGTH(sizeof(cx_hmac_sha512_t)),
                    unsigned char WIDE *key PLENGTH(key_len), int key_len);

/**
 * @param [in/out] hmac
 *   Univers Continuation Blob.
 *   The hmac context pointer shall point to  either a cx_ripemd160_t, either a
 * cx_sha256_t  or cx_sha512_t .
 *   The hmac context shall be inited with 'cx_xxx_init'
 *   The hmac context shall be in RAM
 *   The function should be called with a nice cast.
 *
 * @param [in] mode
 *   16bits flags. See Above
 *   If CX_LAST is set and CX_DISCARD is not set, context is automatically
 * re-inited.
 *   Supported flags:
 *     - CX_LAST
 *     - CX_DISCARD
 *
 * @param [in] in
 *   Input data to add to current hmac
 *
 * @param [in] len
 *   Length of input to data.
 *
 * @param [out] mac
 *   Either:
 *     - NULL (ignored) if CX_LAST is NOT set
 *     - produced hmac  if CX_LAST is set
 *   'out' length is implicit, no check is done
 *
 */
SYSCALL int
cx_hmac(cx_hmac_t *hmac PLENGTH(sizeof(cx_hmac_sha512_t) /*max for sha 512*/),
        int mode, unsigned char WIDE *in PLENGTH(len), int len,
        unsigned char *mac PLENGTH(64 /*max for sha 512*/));

/* ####################################################################### */
/*                               CIPHER/SIGNATURE                          */
/* ####################################################################### */
/* - DES
 * - ECDSA
 * - ECDH
 */

/* ======================================================================= */
/*                                   DES                                   */
/* ======================================================================= */

#define CX_DES_BLOCK_SIZE 8

struct cx_des_key_s {
    unsigned char size;
    unsigned char keys[16];
};

typedef struct cx_des_key_s cx_des_key_t;

/**
 * Initialize a DES Key.
 * Once initialized, the key may be stored in non-volatile memory
 * an reused 'as-is' for any DES processing
 *
 * @param [in] rawkey
 *   raw key value
 *
 * @param [in] key_len
 *   key bytes lenght: 8,16 or 24
 *
 * @param [out] key
 *   DES key to init
 *
 * @param key
 *   ready to use key to init
 */
SYSCALL int cx_des_init_key(unsigned char WIDE *rawkey PLENGTH(key_len),
                            int key_len,
                            cx_des_key_t *key PLENGTH(sizeof(cx_des_key_t)));

/**
 * Encrypt, Decrypt, Sign or Verify data with DES algorithm.
 *
 * @param [in] key
 *   A des key fully inited with 'cx_des_init_key'
 *
 * @param [in] mode
 *   16bits crypto mode flags. See above.
 *   Supported flags:
 *     - CX_LAST
 *     - CX_ENCRYPT
 *     - CX_DECRYPT
 *     - CX_SIGN
 *     - CX_VERIFY
 *     - CX_PAD_NONE
 *     - CX_PAD_ISO9797M1
 *     - CX_PAD_ISO9797M2
 *     - CX_CHAIN_ECB
 *     - CX_CHAIN_CBC
 *
 * @param [in] in
 *   Input data to encrypt/decrypt
 *
 * @param [in] len
 *   Length of input to data.
 *   If CX_LAST is set, padding is automatically done according to  'mode'.
 *   Else  'len' shall be a multiple of DES_BLOCK_SIZE.
 *
 * @param [in] iv
 *   Initial IV for chaining mode
 *
 * @param [out] out
 *   Either:
 *     - encrypted/decrypted ouput data
 *     - produced signature
 *     - signature to check
 *   'out' buffer length is implicit, no check is done
 *
 * @return
 *   - In case of ENCRYPT, DECRYPT or SIGN mode: output lenght data
 *   - In case of VERIFY mode: 0 if signature is false, DES_BLOCK_SIZE if
 * signature is correct
 *
 * @throws INVALID_PARAMETER
 */

SYSCALL int cx_des_iv(cx_des_key_t WIDE *key PLENGTH(sizeof(cx_des_key_t)),
                      int mode, unsigned char WIDE *iv PLENGTH(8),
                      unsigned char WIDE *in PLENGTH(len), int len,
                      unsigned char *out PLENGTH(len + 8));

/**
 *  Same as cx_des_iv with initial IV assumed to be heigt zeros.
 */
SYSCALL int cx_des(cx_des_key_t WIDE *key PLENGTH(sizeof(cx_des_key_t)),
                   int mode, unsigned char WIDE *in PLENGTH(len), int len,
                   unsigned char *out PLENGTH(len + 8));

/** HW support */
void cx_des_set_hw_key(cx_des_key_t WIDE *keys, int mode);
REENTRANT(void cx_des_hw_block(unsigned char WIDE *inblock,
                               unsigned char *outblock));
void cx_des_reset_hw(void);

/* ======================================================================= */
/*                                   AES                                   */
/* ======================================================================= */

#define CX_AES_BLOCK_SIZE 16

struct cx_aes_key_s {
    int size;
    unsigned char keys[32];
};

typedef struct cx_aes_key_s cx_aes_key_t;

/**
 * Initialize a AES Key.
 * Once initialized, the key may be stored in non-volatile memory
 * an reused 'as-is' for any AES processing
 *
 * @param [in] rawkey
 *   raw key value
 *
 * @param [in] key_len
 *   key bytes lenght: 8,16 or 24
 *
 * @param [out] key
 *   AES key to init
 *
 * @param key
 *   ready to use key to init
 */
SYSCALL int cx_aes_init_key(unsigned char WIDE *rawkey PLENGTH(key_len),
                            int key_len,
                            cx_aes_key_t *key PLENGTH(sizeof(cx_aes_key_t)));

/**
 * Encrypt, Decrypt, Sign or Verify data with AES algorithm.
 *
 * @param [in] key
 *   A aes key fully inited with 'cx_aes_init_key'
 *
 * @param [in] mode
 *   16bits crypto mode flags. See above.
 *   Supported flags:
 *     - CX_LAST
 *     - CX_ENCRYPT
 *     - CX_DECRYPT
 *     - CX_SIGN
 *     - CX_VERIFY
 *     - CX_PAD_NONE
 *     - CX_PAD_ISO9797M1
 *     - CX_PAD_ISO9797M2
 *     - CX_CHAIN_ECB
 *     - CX_CHAIN_CBC
 *
 * @param [in] in
 *   Input data to encrypt/decrypt
 *
 * @param [in] len
 *   Length of input to data.
 *   If CX_LAST is set, padding is automtically done according to  'mode'.
 *   Else  'len' shall be a multiple of AES_BLOCK_SIZE.
 *
 * @param [out] out
 *   Either:
 *     - encrypted/decrypted output data
 *     - produced signature
 *     - signature to check
 *   'out' buffer length is implicit, no check is done
 *
 * @return
 *   - In case of ENCRYPT, DECRYPT or SIGN mode: output length data
 *   - In case of VERIFY mode: 0 if signature is false, AES_BLOCK_SIZE if
 * signature is correct
 *
 * @throws INVALID_PARAMETER
 */
SYSCALL int cx_aes_iv(cx_aes_key_t WIDE *key PLENGTH(sizeof(cx_aes_key_t)),
                      int mode, unsigned char WIDE *iv PLENGTH(16),
                      unsigned char WIDE *in PLENGTH(len), int len,
                      unsigned char *out PLENGTH(len + 15));

/**
 *  Same as cx_aes_iv with initial IV assumed to be sixteen zeros.
 */
SYSCALL int cx_aes(cx_aes_key_t WIDE *key PLENGTH(sizeof(cx_aes_key_t)),
                   int mode, unsigned char WIDE *in PLENGTH(len), int len,
                   unsigned char *out PLENGTH(len + 16));

/** HW support */
void cx_aes_set_hw_key(cx_aes_key_t WIDE *keys, int mode);
REENTRANT(void cx_aes_hw_block(unsigned char WIDE *inblock,
                               unsigned char *outblock));
void cx_aes_reset_hw(void);

/* ======================================================================= */
/*                                   RSASign                               */
/* ======================================================================= */
struct cx_rsa_abstract_public_key_s {
    int size;
    unsigned char e[4];
    unsigned char n[1];
};
typedef struct cx_rsa_abstract_public_key_s cx_rsa_abstract_public_key_t;

struct cx_rsa_2048_public_key_s {
    int size;
    unsigned char e[4];
    unsigned char n[256];
};
typedef struct cx_rsa_2048_public_key_s cx_rsa_2048_public_key_t;

struct cx_rsa_3072_public_key_s {
    int size;
    unsigned char e[4];
    unsigned char n[384];
};
typedef struct cx_rsa_3072_public_key_s cx_rsa_3072_public_key_t;

struct cx_rsa_4096_public_key_s {
    int size;
    unsigned char e[4];
    unsigned char n[512];
};
typedef struct cx_rsa_4096_public_key_s cx_rsa_4096_public_key_t;

/**
 * Initialize a public RSA Key.
 * Once initialized, the key may be stored in non-volatile memory
 * an reused 'as-is' for any RSA processing
 * Passing NULL as raw key initializes the key without value. The key can not be
 * used
 *
 * @param [in] rawkey
 *   Raw key value or NULL.
 *   The value shall be the public point encoded as: 04 x y, where x and y are
 *   encoded as  big endian raw value and have bits length equals to
 *   the curve size.
 *
 * @param [in] key_len
 *   Key bytes lenght
 *
 * @param [out] key
 *   Public ecfp key to init.
 *
 * @param key
 *   Ready to use key to init
 *
 * @return something
 *
 * @throws INVALID_PARAMETER
 */
SYSCALL int cx_rsa_init_public_key(
    unsigned char WIDE *exponent PLENGTH(4),
    unsigned char WIDE *modulus PLENGTH(modulus_len), int modulus_len,
    cx_rsa_abstract_public_key_t *key
        PLENGTH(sizeof(cx_rsa_abstract_public_key_t) + modulus_len));
/**
 * Verify a hash message signature according to RSA specification.
 *
 * @param [in] key
 *   A public ecfp key fully inited with 'cx_rsa_init_public_key'
 *
 * @param [in] mode
 *   16bits crypto mode flags. See above.
 *   Supported flags:
 *     - CX_LAST
 *     - CX_PAD_PKCS1_1o5
 *
 * @param [in] hashID
 *  Hash identifier used to compute the input data.
 *
 * @param [in] hash
 *   Signed input data to verify the signature.
 *   The data should be the hash of the original message.
 *   The data length must be lesser than the curve size.
 *
 * @param [in] hash_len
 *   Length of input to data.
 *
 * @param [in] sig
 *   RSA signature to verify encoded as raw bytes
 *
 * @return
 *   1 if signature is verified
 *   0 is signarure is not verified
 *
 * @throws INVALID_PARAMETER
 */
SYSCALL int cx_rsa_verify(cx_rsa_abstract_public_key_t WIDE *key PLENGTH(768),
                          int mode, cx_md_t hashID,
                          unsigned char WIDE *hash PLENGTH(hash_len),
                          int hash_len,
                          unsigned char WIDE *sig PLENGTH(sig_len),
                          int sig_len);

/** HW support */
void cx_rsa_set_hw_key(void WIDE *keys, int mode);
int cx_rsa_verify_hw(cx_rsa_abstract_public_key_t WIDE *key, int mode,
                     cx_md_t hashID, unsigned char WIDE *hash, int hash_len,
                     unsigned char WIDE *sig, int sig_len);
void cx_rsa_reset_hw(void);

/* ======================================================================= */
/*                                   ECDSA                                 */
/* ======================================================================= */
/** Only curved defined at compiled time will be supported */

enum cx_curve_e {
    CX_CURVE_NONE,
    CX_CURVE_256K1,
    CX_CURVE_256R1,
    CX_CURVE_192K1,
    CX_CURVE_192R1,
};
typedef enum cx_curve_e cx_curve_t;

typedef struct cx_curve_domain_s {
    int size;
    unsigned char la;
    unsigned char WIDE *a; // Weierstrass a coef
    unsigned char lb;
    unsigned char WIDE *b; // Weierstrass a coef
    unsigned char lp;
    unsigned char WIDE *p; // Field
    unsigned char lGx;
    unsigned char WIDE *Gx; // Point Generator x coordinate
    unsigned char lGy;
    unsigned char WIDE *Gy; // Point Generator y coordinate
    unsigned char ln;
    unsigned char WIDE *n; // Curve order
    unsigned char lh;
    unsigned char WIDE *h; // cofactor

} cx_curve_domain_t;
cx_curve_domain_t WIDE *cx_ecfp_get_domain(cx_curve_t curve);

extern cx_curve_domain_t const WIDE C_cx_secp256k1;

struct cx_ecfp_public_key_s {
    cx_curve_t curve;
    int W_len;
    unsigned char W[65];
};

struct cx_ecfp_private_key_s {
    cx_curve_t curve;
    int d_len;
    unsigned char d[32];
};

typedef struct cx_ecfp_public_key_s cx_ecfp_public_key_t;
typedef struct cx_ecfp_private_key_s cx_ecfp_private_key_t;

/**
 * Verify that a given point is really on the specified curve.
 *
 * @param [in] domain
 *   The curve domain parameters to work with.
 *
 * @param [in]  public_point
 *   The point to test  encoded as: 04 x y
 *
 * @return
 *    1 if point is on the curve
 *    0 if point is not on the curve
 *   -1 if undefined (function not impl)
 *
 * @throws INVALID_PARAMETER
 */
SYSCALL int cx_ecfp_is_valid_point(
    cx_curve_domain_t WIDE *domain PLENGTH(sizeof(cx_curve_domain_t)),
    unsigned char WIDE *point PLENGTH(1 + 32 + 32));

/**
 * Add two affine point
 *
 * @param [in] domain
 *   The curve domain parameters to work with.
 *
 * @param [out] R
 *   P+Q encoded as: 04 x y, where x and y are
 *   encoded as  big endian raw value and have bits length equals to
 *   the curve size.
 *
 * @param [in] P
 *   First point to add *
 *   The value shall be a point encoded as: 04 x y, where x and y are
 *   encoded as  big endian raw value and have bits length equals to
 *   the curve size.
 *
 * @param [in] Q
 *   Second point to add
 *
 * @param [in]  public_point
 *   The point to test  encoded as: 04 x y
 *
 * @return
 *   R encoding length, if add success
 *   -1 if failed
 *
 * @throws INVALID_PARAMETER
 */
SYSCALL int cx_ecfp_add_point(cx_curve_domain_t WIDE *domain
                                  PLENGTH(sizeof(cx_curve_domain_t)),
                              unsigned char *R PLENGTH(1 + 32 + 32),
                              unsigned char WIDE *P PLENGTH(1 + 32 + 32),
                              unsigned char WIDE *Q PLENGTH(1 + 32 + 32));

/**
 * Initialize a public ECFP Key.
 * Once initialized, the key may be stored in non-volatile memory
 * an reused 'as-is' for any ECDSA processing
 * Passing NULL as raw key initializes the key without value. The key may be
 used
 * as parameter for cx_ecfp_generate_pair.

 * @param [in] curve
 *   The curve domain parameters to work with.
 *
 * @param [in] rawkey
 *   Raw key value or NULL.
 *   The value shall be the public point encoded as: 04 x y, where x and y are
 *   encoded as  big endian raw value and have bits length equals to
 *   the curve size.
 *
 * @param [in] key_len
 *   Key bytes lenght
 *
 * @param [out] key
 *   Public ecfp key to init.
 *
 * @param key
 *   Ready to use key to init
 *
 * @return something
 *
 * @throws INVALID_PARAMETER
 */
SYSCALL int cx_ecdsa_init_public_key(
    cx_curve_t curve, unsigned char WIDE *rawkey PLENGTH(key_len), int key_len,
    cx_ecfp_public_key_t *key PLENGTH(sizeof(cx_ecfp_public_key_t)));

/**
 * Initialize a private ECFP Key.
 * Once initialized, the key may be  stored in non-volatile memory
 * and reused 'as-is' for any ECDSA processing
 * Passing NULL as raw key initializes the key without value. The key may be
 * used
 * as parameter for cx_ecfp_generate_pair.
 *
 * @param [in] curve
 *   The curve domain parameters to work with.
 *
 * @param [in] rawkey
 *   Raw key value or NULL.
 *   The value shall be the private key big endian raw value.
 *
 * @param [in] key_len
 *   Key bytes lenght
 *
 * @param [out] key
 *   Private ecfp key to init.
 *
 * @param key
 *   Ready to use key to init
 *
 * @return something
 *
 * @throws INVALID_PARAMETER
 */
SYSCALL int cx_ecdsa_init_private_key(
    cx_curve_t curve, unsigned char WIDE *rawkey PLENGTH(key_len), int key_len,
    cx_ecfp_private_key_t *key PLENGTH(sizeof(cx_ecfp_private_key_t)));
/**
 * Generate a ecfp key pair
 *
 * @param [in] curve
 *   The curve domain parameters to work with.
 *
 * @param [out] public_key
 *   A public ecfp public key to generate.
 *
 * @param [in/out] private_key
 *   A private ecfp private key to generate.
 *   Either:
 *     - if the private ecfp key is fully inited, i.e  parameter 'rawkey' of
 *       'cx_ecdsa_init_private_key' is NOT null, the private key value is kept
 *       if the 'keep_private' parameter is non zero
 *     - else a new private key is generated.
 *
 * @param [in] keep_private if set to non zero, keep the private key value if
 * set.
 *             Else generate a new random one
 *
 * @return zero
 *
 * @throws INVALID_PARAMETER
 */
SYSCALL int cx_ecfp_generate_pair(
    cx_curve_t curve,
    cx_ecfp_public_key_t *public_key PLENGTH(sizeof(cx_ecfp_public_key_t)),
    cx_ecfp_private_key_t *private_key PLENGTH(sizeof(cx_ecfp_private_key_t)),
    int keep_private);

/**
 * Sign a hash message according to ECDSA specification.
 *
 * @param [in] key
 *   A private ecfp key fully inited with 'cx_ecdsa_init_private_key'
 *
 * @param [in] mode
 *   16bits crypto mode flags. See above.
 *   Supported flags:
 *     - CX_LAST
 *     - CX_RND_TRNG
 *     - CX_RND_RFC6979
 *
 * @param [in] hashID
 *  Hash identifier used to compute the input data.
 *  This parameter is mandatory for rng of type CX_RND_RFC6979.
 *
 * @param [in] hash
 *   Input data to sign.
 *   The data should be the hash of the original message.
 *   The data length must be lesser than the curve size.
 *
 * @param [in] hash_len
 *   Length of input to data.
 *
 * @param [out] sig
 *   ECDSA signature encoded as TLV:  30 L 02 Lr r 02 Ls s
 *
 * @return
 *   Full length of signature
 *
 * @throws INVALID_PARAMETER
 */
SYSCALL int cx_ecdsa_sign(
    cx_ecfp_private_key_t WIDE *key PLENGTH(sizeof(cx_ecfp_private_key_t)),
    int mode, cx_md_t hashID, unsigned char WIDE *hash PLENGTH(hash_len),
    int hash_len, unsigned char *sig PLENGTH(1 + 1 + 2 * (1 + 1 + 33)));

/**
 * Verify a hash message signature according to ECDSA specification.
 *
 * @param [in] key
 *   A public ecfp key fully inited with 'cx_ecdsa_init_public_key'
 *
 * @param [in] mode
 *   16bits crypto mode flags. See above.
 *   Supported flags:
 *     - CX_LAST
 *
 * @param [in] hashID
 *  Hash identifier used to compute the input data.
 *
 * @param [in] hash
 *   Signed input data to verify the signature.
 *   The data should be the hash of the original message.
 *   The data length must be lesser than the curve size.
 *
 * @param [in] hash_len
 *   Length of input to data.
 *
 * @param [in] sig
 *   ECDSA signature to verify encoded as TLV:  30 L 02 Lr r 02 Ls s
 *
 * @return
 *   1 if signature is verified
 *   0 is signarure is not verified
 *
 * @throws INVALID_PARAMETER
 */
SYSCALL int cx_ecdsa_verify(
    cx_ecfp_public_key_t WIDE *key PLENGTH(sizeof(cx_ecfp_public_key_t)),
    int mode, cx_md_t hashID, unsigned char WIDE *hash PLENGTH(hash_len),
    int hash_len, unsigned char WIDE *sig PLENGTH(sig_len), int sig_len);

/** HW support */
void cx_ecfp_set_hw_key(void WIDE *keys, int mode);
int cx_ecdsa_sign_hw(cx_ecfp_private_key_t WIDE *key, unsigned char WIDE *k,
                     unsigned int k_len, unsigned char WIDE *hash,
                     unsigned int hash_len, unsigned char *sig);
int cx_ecdsa_verify_hw(cx_ecfp_public_key_t WIDE *key, unsigned char WIDE *hash,
                       int hash_len, unsigned char WIDE *sig, int sig_len);
void cx_ecfp_reset_hw(void);

/* ======================================================================= */
/*                                     ECDH                                */
/* ======================================================================= */

/**
 * Compute a shared secret according to ECDH specifiaction
 * Depending on the mode, the shared secret is either the full point or
 * only the x coordinate
 *
 * @param [in] key
 *   A private ecfp key fully inited with 'cx_ecdsa_init_private_key'
 *
 * @param [in] mode
 *   16bits crypto mode flags. See above.
 *   Supported flags:
 *     - CX_ECDH_POINT
 *     - CX_ECDH_X
 *
 * @param [in] public_point
 *   Other party public point encoded as: 04 x y, where x and y are
 *   encoded as big endian raw value and have bits length equals to
 *   the curve size.
 *
 * @param [out] secret
 *   Generated shared secret.
 *
 *
 * @return size of secret
 *
 * @throws INVALID_PARAMETER
 */
SYSCALL int
cx_ecdh(cx_ecfp_private_key_t WIDE *key PLENGTH(sizeof(cx_ecfp_private_key_t)),
        int mode, unsigned char WIDE *public_point PLENGTH(1 + 32 + 32),
        unsigned char *secret PLENGTH(1 + 32 + 32));

/** HW support */
int cx_ecdh_scalar_mult_hw(cx_ecfp_private_key_t WIDE *key,
                           unsigned char WIDE *public_point,
                           unsigned char *secret);

/* ======================================================================= */
/*                                    CRC                                */
/* ======================================================================= */

/**
 * Compute a 16 bits checksum value.
 * The 16 bits value is computed according to the CRC16 CCITT definition.
 *
 * @param [in] buffer
 *   The buffer to compute the crc over.
 *
 * @param [in]
 *   Bytes Length of the 'buffer'
 *
 * @return crc des_
 *
 */
SYSCALL unsigned short cx_crc16(void WIDE *buffer PLENGTH(len), int len);

#define CX_CRC16_INIT 0xFFFF

SYSCALL unsigned short cx_crc16_update(unsigned short crc,
                                       void WIDE *buffer PLENGTH(len), int len);

/* ======================================================================= */
/*                                    MATH                                 */
/* ======================================================================= */

/**
 * Modular addition of tow big integer of the size: r = a+b mod m
 * The maximum length supported is 64.
 *
 * @param r    where to put result
 * @param a    first operand
 * @param b    second operand
 * @param m    modulo
 * @param len  byte length of r, a, b, m
 *
 */
SYSCALL void cx_math_addm(unsigned char *r PLENGTH(len),
                          unsigned char WIDE *a PLENGTH(len),
                          unsigned char WIDE *b PLENGTH(len),
                          unsigned char WIDE *m PLENGTH(len), int len);

/**
 * Modular multiplication of tow big integer of the size: r = a*b mod m
 * The maximum length supported is 64.
 *
 * @param r    where to put result
 * @param a    first operand
 * @param b    second operand
 * @param m    modulo
 * @param len  byte length of r, a, b, m
 *
 */
SYSCALL void cx_math_multm(unsigned char *r PLENGTH(len),
                           unsigned char WIDE *a PLENGTH(len),
                           unsigned char WIDE *b PLENGTH(len),
                           unsigned char WIDE *m PLENGTH(len), int len);

/**
 * Compare to unsigned long big-endian integer
 * The maximum length supported is 64.
 *
 * @param a    first operand
 * @param b    second operand
 * @param len  byte length of a, b
 *
 * @return 0 if a==b,  negative value if a<b, positive value if a>b
 */
SYSCALL int cx_math_cmp(unsigned char WIDE *a PLENGTH(len),
                        unsigned char WIDE *b PLENGTH(len), int len);

/**
 * Compare to unsigned long big-endian integer to zero
 *
 * @param a    value to compare to zero
 * @param len  byte length of a
 *
 * @return 1 if a==0,  0 else
 */
SYSCALL int cx_math_is_zero(unsigned char WIDE *a PLENGTH(len), int len);

/**
 * Reduce in place (left zero padded) the given value.
 *    v = v mod m
 *
 * @param v        value to reduce
 * @param len_v    shall be >= len_m
 * @param m        modulus
 * @param len_m    length of modulus
 *
 */
SYSCALL void cx_math_modm(unsigned char *v PLENGTH(len_v), int len_v,
                          unsigned char WIDE *m PLENGTH(len_m), int len_m);

/* ======================================================================= */
/*                                    MATH                                 */
/* ======================================================================= */
int cx_selftest(void);

#endif // CX_H
