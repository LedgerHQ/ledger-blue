/*******************************************************************************
*   Ledger Blue - Secure firmware
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

/* MACHINE GENERATED: DO NOT MODIFY */
#include "os.h"
#include "syscalls.h"

void SVC_Call(volatile unsigned int *params) {
    // ensure params is in R0
    asm("mov r0, %0" ::"r"(params));
    // delegate svc
    asm volatile("svc #1");
}

void nvm_write(void *dst_adr, void *src_adr, unsigned int src_len) {
    unsigned int parameters[2 + 3];
    parameters[0] = (unsigned int)SYSCALL_nvm_write_ID_IN;
    parameters[1] = (unsigned int)G_try_last_open_context->jmp_buf;
    parameters[2] = (unsigned int)dst_adr;
    parameters[3] = (unsigned int)src_adr;
    parameters[4] = (unsigned int)src_len;
    SVC_Call(parameters);
    if (parameters[0] != SYSCALL_nvm_write_ID_OUT) {
        THROW(EXCEPTION_SECURITY);
    }
}

unsigned char cx_rng_u8(void) {
    unsigned int parameters[2];
    parameters[0] = (unsigned int)SYSCALL_cx_rng_u8_ID_IN;
    parameters[1] = (unsigned int)G_try_last_open_context->jmp_buf;
    SVC_Call(parameters);
    if (parameters[0] != SYSCALL_cx_rng_u8_ID_OUT) {
        THROW(EXCEPTION_SECURITY);
    }
    return (unsigned char)parameters[2];
}

unsigned char *cx_rng(unsigned char *buffer, int len) {
    unsigned int parameters[2 + 2];
    parameters[0] = (unsigned int)SYSCALL_cx_rng_ID_IN;
    parameters[1] = (unsigned int)G_try_last_open_context->jmp_buf;
    parameters[2] = (unsigned int)buffer;
    parameters[3] = (unsigned int)len;
    SVC_Call(parameters);
    if (parameters[0] != SYSCALL_cx_rng_ID_OUT) {
        THROW(EXCEPTION_SECURITY);
    }
    return (unsigned char *)parameters[2];
}

int cx_ripemd160_init(cx_ripemd160_t *hash) {
    unsigned int parameters[2 + 1];
    parameters[0] = (unsigned int)SYSCALL_cx_ripemd160_init_ID_IN;
    parameters[1] = (unsigned int)G_try_last_open_context->jmp_buf;
    parameters[2] = (unsigned int)hash;
    SVC_Call(parameters);
    if (parameters[0] != SYSCALL_cx_ripemd160_init_ID_OUT) {
        THROW(EXCEPTION_SECURITY);
    }
    return (int)parameters[2];
}

int cx_sha256_init(cx_sha256_t *hash) {
    unsigned int parameters[2 + 1];
    parameters[0] = (unsigned int)SYSCALL_cx_sha256_init_ID_IN;
    parameters[1] = (unsigned int)G_try_last_open_context->jmp_buf;
    parameters[2] = (unsigned int)hash;
    SVC_Call(parameters);
    if (parameters[0] != SYSCALL_cx_sha256_init_ID_OUT) {
        THROW(EXCEPTION_SECURITY);
    }
    return (int)parameters[2];
}

int cx_sha512_init(cx_sha512_t *hash) {
    unsigned int parameters[2 + 1];
    parameters[0] = (unsigned int)SYSCALL_cx_sha512_init_ID_IN;
    parameters[1] = (unsigned int)G_try_last_open_context->jmp_buf;
    parameters[2] = (unsigned int)hash;
    SVC_Call(parameters);
    if (parameters[0] != SYSCALL_cx_sha512_init_ID_OUT) {
        THROW(EXCEPTION_SECURITY);
    }
    return (int)parameters[2];
}

int cx_hash(cx_hash_t *hash, int mode, unsigned char *in, int len,
            unsigned char *out) {
    unsigned int parameters[2 + 5];
    parameters[0] = (unsigned int)SYSCALL_cx_hash_ID_IN;
    parameters[1] = (unsigned int)G_try_last_open_context->jmp_buf;
    parameters[2] = (unsigned int)hash;
    parameters[3] = (unsigned int)mode;
    parameters[4] = (unsigned int)in;
    parameters[5] = (unsigned int)len;
    parameters[6] = (unsigned int)out;
    SVC_Call(parameters);
    if (parameters[0] != SYSCALL_cx_hash_ID_OUT) {
        THROW(EXCEPTION_SECURITY);
    }
    return (int)parameters[2];
}

int cx_hmac_ripemd160_init(cx_hmac_ripemd160_t *hmac, unsigned char *key,
                           int key_len) {
    unsigned int parameters[2 + 3];
    parameters[0] = (unsigned int)SYSCALL_cx_hmac_ripemd160_init_ID_IN;
    parameters[1] = (unsigned int)G_try_last_open_context->jmp_buf;
    parameters[2] = (unsigned int)hmac;
    parameters[3] = (unsigned int)key;
    parameters[4] = (unsigned int)key_len;
    SVC_Call(parameters);
    if (parameters[0] != SYSCALL_cx_hmac_ripemd160_init_ID_OUT) {
        THROW(EXCEPTION_SECURITY);
    }
    return (int)parameters[2];
}

int cx_hmac_sha256_init(cx_hmac_sha256_t *hmac, unsigned char *key,
                        int key_len) {
    unsigned int parameters[2 + 3];
    parameters[0] = (unsigned int)SYSCALL_cx_hmac_sha256_init_ID_IN;
    parameters[1] = (unsigned int)G_try_last_open_context->jmp_buf;
    parameters[2] = (unsigned int)hmac;
    parameters[3] = (unsigned int)key;
    parameters[4] = (unsigned int)key_len;
    SVC_Call(parameters);
    if (parameters[0] != SYSCALL_cx_hmac_sha256_init_ID_OUT) {
        THROW(EXCEPTION_SECURITY);
    }
    return (int)parameters[2];
}

int cx_hmac_sha512_init(cx_hmac_sha512_t *hmac, unsigned char *key,
                        int key_len) {
    unsigned int parameters[2 + 3];
    parameters[0] = (unsigned int)SYSCALL_cx_hmac_sha512_init_ID_IN;
    parameters[1] = (unsigned int)G_try_last_open_context->jmp_buf;
    parameters[2] = (unsigned int)hmac;
    parameters[3] = (unsigned int)key;
    parameters[4] = (unsigned int)key_len;
    SVC_Call(parameters);
    if (parameters[0] != SYSCALL_cx_hmac_sha512_init_ID_OUT) {
        THROW(EXCEPTION_SECURITY);
    }
    return (int)parameters[2];
}

int cx_hmac(cx_hmac_t *hmac, int mode, unsigned char *in, int len,
            unsigned char *mac) {
    unsigned int parameters[2 + 5];
    parameters[0] = (unsigned int)SYSCALL_cx_hmac_ID_IN;
    parameters[1] = (unsigned int)G_try_last_open_context->jmp_buf;
    parameters[2] = (unsigned int)hmac;
    parameters[3] = (unsigned int)mode;
    parameters[4] = (unsigned int)in;
    parameters[5] = (unsigned int)len;
    parameters[6] = (unsigned int)mac;
    SVC_Call(parameters);
    if (parameters[0] != SYSCALL_cx_hmac_ID_OUT) {
        THROW(EXCEPTION_SECURITY);
    }
    return (int)parameters[2];
}

int cx_des_init_key(unsigned char *rawkey, int key_len, cx_des_key_t *key) {
    unsigned int parameters[2 + 3];
    parameters[0] = (unsigned int)SYSCALL_cx_des_init_key_ID_IN;
    parameters[1] = (unsigned int)G_try_last_open_context->jmp_buf;
    parameters[2] = (unsigned int)rawkey;
    parameters[3] = (unsigned int)key_len;
    parameters[4] = (unsigned int)key;
    SVC_Call(parameters);
    if (parameters[0] != SYSCALL_cx_des_init_key_ID_OUT) {
        THROW(EXCEPTION_SECURITY);
    }
    return (int)parameters[2];
}

int cx_des_iv(cx_des_key_t *key, int mode, unsigned char *iv, unsigned char *in,
              int len, unsigned char *out) {
    unsigned int parameters[2 + 6];
    parameters[0] = (unsigned int)SYSCALL_cx_des_iv_ID_IN;
    parameters[1] = (unsigned int)G_try_last_open_context->jmp_buf;
    parameters[2] = (unsigned int)key;
    parameters[3] = (unsigned int)mode;
    parameters[4] = (unsigned int)iv;
    parameters[5] = (unsigned int)in;
    parameters[6] = (unsigned int)len;
    parameters[7] = (unsigned int)out;
    SVC_Call(parameters);
    if (parameters[0] != SYSCALL_cx_des_iv_ID_OUT) {
        THROW(EXCEPTION_SECURITY);
    }
    return (int)parameters[2];
}

int cx_des(cx_des_key_t *key, int mode, unsigned char *in, int len,
           unsigned char *out) {
    unsigned int parameters[2 + 5];
    parameters[0] = (unsigned int)SYSCALL_cx_des_ID_IN;
    parameters[1] = (unsigned int)G_try_last_open_context->jmp_buf;
    parameters[2] = (unsigned int)key;
    parameters[3] = (unsigned int)mode;
    parameters[4] = (unsigned int)in;
    parameters[5] = (unsigned int)len;
    parameters[6] = (unsigned int)out;
    SVC_Call(parameters);
    if (parameters[0] != SYSCALL_cx_des_ID_OUT) {
        THROW(EXCEPTION_SECURITY);
    }
    return (int)parameters[2];
}

int cx_aes_init_key(unsigned char *rawkey, int key_len, cx_aes_key_t *key) {
    unsigned int parameters[2 + 3];
    parameters[0] = (unsigned int)SYSCALL_cx_aes_init_key_ID_IN;
    parameters[1] = (unsigned int)G_try_last_open_context->jmp_buf;
    parameters[2] = (unsigned int)rawkey;
    parameters[3] = (unsigned int)key_len;
    parameters[4] = (unsigned int)key;
    SVC_Call(parameters);
    if (parameters[0] != SYSCALL_cx_aes_init_key_ID_OUT) {
        THROW(EXCEPTION_SECURITY);
    }
    return (int)parameters[2];
}

int cx_aes_iv(cx_aes_key_t *key, int mode, unsigned char *iv, unsigned char *in,
              int len, unsigned char *out) {
    unsigned int parameters[2 + 6];
    parameters[0] = (unsigned int)SYSCALL_cx_aes_iv_ID_IN;
    parameters[1] = (unsigned int)G_try_last_open_context->jmp_buf;
    parameters[2] = (unsigned int)key;
    parameters[3] = (unsigned int)mode;
    parameters[4] = (unsigned int)iv;
    parameters[5] = (unsigned int)in;
    parameters[6] = (unsigned int)len;
    parameters[7] = (unsigned int)out;
    SVC_Call(parameters);
    if (parameters[0] != SYSCALL_cx_aes_iv_ID_OUT) {
        THROW(EXCEPTION_SECURITY);
    }
    return (int)parameters[2];
}

int cx_aes(cx_aes_key_t *key, int mode, unsigned char *in, int len,
           unsigned char *out) {
    unsigned int parameters[2 + 5];
    parameters[0] = (unsigned int)SYSCALL_cx_aes_ID_IN;
    parameters[1] = (unsigned int)G_try_last_open_context->jmp_buf;
    parameters[2] = (unsigned int)key;
    parameters[3] = (unsigned int)mode;
    parameters[4] = (unsigned int)in;
    parameters[5] = (unsigned int)len;
    parameters[6] = (unsigned int)out;
    SVC_Call(parameters);
    if (parameters[0] != SYSCALL_cx_aes_ID_OUT) {
        THROW(EXCEPTION_SECURITY);
    }
    return (int)parameters[2];
}

int cx_rsa_init_public_key(unsigned char *exponent, unsigned char *modulus,
                           int modulus_len, cx_rsa_abstract_public_key_t *key) {
    unsigned int parameters[2 + 4];
    parameters[0] = (unsigned int)SYSCALL_cx_rsa_init_public_key_ID_IN;
    parameters[1] = (unsigned int)G_try_last_open_context->jmp_buf;
    parameters[2] = (unsigned int)exponent;
    parameters[3] = (unsigned int)modulus;
    parameters[4] = (unsigned int)modulus_len;
    parameters[5] = (unsigned int)key;
    SVC_Call(parameters);
    if (parameters[0] != SYSCALL_cx_rsa_init_public_key_ID_OUT) {
        THROW(EXCEPTION_SECURITY);
    }
    return (int)parameters[2];
}

int cx_rsa_verify(cx_rsa_abstract_public_key_t *key, int mode, cx_md_t hashID,
                  unsigned char *hash, int hash_len, unsigned char *sig,
                  int sig_len) {
    unsigned int parameters[2 + 7];
    parameters[0] = (unsigned int)SYSCALL_cx_rsa_verify_ID_IN;
    parameters[1] = (unsigned int)G_try_last_open_context->jmp_buf;
    parameters[2] = (unsigned int)key;
    parameters[3] = (unsigned int)mode;
    parameters[4] = (unsigned int)hashID;
    parameters[5] = (unsigned int)hash;
    parameters[6] = (unsigned int)hash_len;
    parameters[7] = (unsigned int)sig;
    parameters[8] = (unsigned int)sig_len;
    SVC_Call(parameters);
    if (parameters[0] != SYSCALL_cx_rsa_verify_ID_OUT) {
        THROW(EXCEPTION_SECURITY);
    }
    return (int)parameters[2];
}

int cx_ecfp_is_valid_point(cx_curve_domain_t *domain, unsigned char *point) {
    unsigned int parameters[2 + 2];
    parameters[0] = (unsigned int)SYSCALL_cx_ecfp_is_valid_point_ID_IN;
    parameters[1] = (unsigned int)G_try_last_open_context->jmp_buf;
    parameters[2] = (unsigned int)domain;
    parameters[3] = (unsigned int)point;
    SVC_Call(parameters);
    if (parameters[0] != SYSCALL_cx_ecfp_is_valid_point_ID_OUT) {
        THROW(EXCEPTION_SECURITY);
    }
    return (int)parameters[2];
}

int cx_ecfp_add_point(cx_curve_domain_t *domain, unsigned char *R,
                      unsigned char *P, unsigned char *Q) {
    unsigned int parameters[2 + 4];
    parameters[0] = (unsigned int)SYSCALL_cx_ecfp_add_point_ID_IN;
    parameters[1] = (unsigned int)G_try_last_open_context->jmp_buf;
    parameters[2] = (unsigned int)domain;
    parameters[3] = (unsigned int)R;
    parameters[4] = (unsigned int)P;
    parameters[5] = (unsigned int)Q;
    SVC_Call(parameters);
    if (parameters[0] != SYSCALL_cx_ecfp_add_point_ID_OUT) {
        THROW(EXCEPTION_SECURITY);
    }
    return (int)parameters[2];
}

int cx_ecdsa_init_public_key(cx_curve_t curve, unsigned char *rawkey,
                             int key_len, cx_ecfp_public_key_t *key) {
    unsigned int parameters[2 + 4];
    parameters[0] = (unsigned int)SYSCALL_cx_ecdsa_init_public_key_ID_IN;
    parameters[1] = (unsigned int)G_try_last_open_context->jmp_buf;
    parameters[2] = (unsigned int)curve;
    parameters[3] = (unsigned int)rawkey;
    parameters[4] = (unsigned int)key_len;
    parameters[5] = (unsigned int)key;
    SVC_Call(parameters);
    if (parameters[0] != SYSCALL_cx_ecdsa_init_public_key_ID_OUT) {
        THROW(EXCEPTION_SECURITY);
    }
    return (int)parameters[2];
}

int cx_ecdsa_init_private_key(cx_curve_t curve, unsigned char *rawkey,
                              int key_len, cx_ecfp_private_key_t *key) {
    unsigned int parameters[2 + 4];
    parameters[0] = (unsigned int)SYSCALL_cx_ecdsa_init_private_key_ID_IN;
    parameters[1] = (unsigned int)G_try_last_open_context->jmp_buf;
    parameters[2] = (unsigned int)curve;
    parameters[3] = (unsigned int)rawkey;
    parameters[4] = (unsigned int)key_len;
    parameters[5] = (unsigned int)key;
    SVC_Call(parameters);
    if (parameters[0] != SYSCALL_cx_ecdsa_init_private_key_ID_OUT) {
        THROW(EXCEPTION_SECURITY);
    }
    return (int)parameters[2];
}

int cx_ecfp_generate_pair(cx_curve_t curve, cx_ecfp_public_key_t *public_key,
                          cx_ecfp_private_key_t *private_key,
                          int keep_private) {
    unsigned int parameters[2 + 4];
    parameters[0] = (unsigned int)SYSCALL_cx_ecfp_generate_pair_ID_IN;
    parameters[1] = (unsigned int)G_try_last_open_context->jmp_buf;
    parameters[2] = (unsigned int)curve;
    parameters[3] = (unsigned int)public_key;
    parameters[4] = (unsigned int)private_key;
    parameters[5] = (unsigned int)keep_private;
    SVC_Call(parameters);
    if (parameters[0] != SYSCALL_cx_ecfp_generate_pair_ID_OUT) {
        THROW(EXCEPTION_SECURITY);
    }
    return (int)parameters[2];
}

int cx_ecdsa_sign(cx_ecfp_private_key_t *key, int mode, cx_md_t hashID,
                  unsigned char *hash, int hash_len, unsigned char *sig) {
    unsigned int parameters[2 + 6];
    parameters[0] = (unsigned int)SYSCALL_cx_ecdsa_sign_ID_IN;
    parameters[1] = (unsigned int)G_try_last_open_context->jmp_buf;
    parameters[2] = (unsigned int)key;
    parameters[3] = (unsigned int)mode;
    parameters[4] = (unsigned int)hashID;
    parameters[5] = (unsigned int)hash;
    parameters[6] = (unsigned int)hash_len;
    parameters[7] = (unsigned int)sig;
    SVC_Call(parameters);
    if (parameters[0] != SYSCALL_cx_ecdsa_sign_ID_OUT) {
        THROW(EXCEPTION_SECURITY);
    }
    return (int)parameters[2];
}

int cx_ecdsa_verify(cx_ecfp_public_key_t *key, int mode, cx_md_t hashID,
                    unsigned char *hash, int hash_len, unsigned char *sig,
                    int sig_len) {
    unsigned int parameters[2 + 7];
    parameters[0] = (unsigned int)SYSCALL_cx_ecdsa_verify_ID_IN;
    parameters[1] = (unsigned int)G_try_last_open_context->jmp_buf;
    parameters[2] = (unsigned int)key;
    parameters[3] = (unsigned int)mode;
    parameters[4] = (unsigned int)hashID;
    parameters[5] = (unsigned int)hash;
    parameters[6] = (unsigned int)hash_len;
    parameters[7] = (unsigned int)sig;
    parameters[8] = (unsigned int)sig_len;
    SVC_Call(parameters);
    if (parameters[0] != SYSCALL_cx_ecdsa_verify_ID_OUT) {
        THROW(EXCEPTION_SECURITY);
    }
    return (int)parameters[2];
}

int cx_ecdh(cx_ecfp_private_key_t *key, int mode, unsigned char *public_point,
            unsigned char *secret) {
    unsigned int parameters[2 + 4];
    parameters[0] = (unsigned int)SYSCALL_cx_ecdh_ID_IN;
    parameters[1] = (unsigned int)G_try_last_open_context->jmp_buf;
    parameters[2] = (unsigned int)key;
    parameters[3] = (unsigned int)mode;
    parameters[4] = (unsigned int)public_point;
    parameters[5] = (unsigned int)secret;
    SVC_Call(parameters);
    if (parameters[0] != SYSCALL_cx_ecdh_ID_OUT) {
        THROW(EXCEPTION_SECURITY);
    }
    return (int)parameters[2];
}

unsigned short cx_crc16(void *buffer, int len) {
    unsigned int parameters[2 + 2];
    parameters[0] = (unsigned int)SYSCALL_cx_crc16_ID_IN;
    parameters[1] = (unsigned int)G_try_last_open_context->jmp_buf;
    parameters[2] = (unsigned int)buffer;
    parameters[3] = (unsigned int)len;
    SVC_Call(parameters);
    if (parameters[0] != SYSCALL_cx_crc16_ID_OUT) {
        THROW(EXCEPTION_SECURITY);
    }
    return (unsigned short)parameters[2];
}

unsigned short cx_crc16_update(unsigned short crc, void *buffer, int len) {
    unsigned int parameters[2 + 3];
    parameters[0] = (unsigned int)SYSCALL_cx_crc16_update_ID_IN;
    parameters[1] = (unsigned int)G_try_last_open_context->jmp_buf;
    parameters[2] = (unsigned int)crc;
    parameters[3] = (unsigned int)buffer;
    parameters[4] = (unsigned int)len;
    SVC_Call(parameters);
    if (parameters[0] != SYSCALL_cx_crc16_update_ID_OUT) {
        THROW(EXCEPTION_SECURITY);
    }
    return (unsigned short)parameters[2];
}

void cx_math_addm(unsigned char *r, unsigned char *a, unsigned char *b,
                  unsigned char *m, int len) {
    unsigned int parameters[2 + 5];
    parameters[0] = (unsigned int)SYSCALL_cx_math_addm_ID_IN;
    parameters[1] = (unsigned int)G_try_last_open_context->jmp_buf;
    parameters[2] = (unsigned int)r;
    parameters[3] = (unsigned int)a;
    parameters[4] = (unsigned int)b;
    parameters[5] = (unsigned int)m;
    parameters[6] = (unsigned int)len;
    SVC_Call(parameters);
    if (parameters[0] != SYSCALL_cx_math_addm_ID_OUT) {
        THROW(EXCEPTION_SECURITY);
    }
}

void cx_math_multm(unsigned char *r, unsigned char *a, unsigned char *b,
                   unsigned char *m, int len) {
    unsigned int parameters[2 + 5];
    parameters[0] = (unsigned int)SYSCALL_cx_math_multm_ID_IN;
    parameters[1] = (unsigned int)G_try_last_open_context->jmp_buf;
    parameters[2] = (unsigned int)r;
    parameters[3] = (unsigned int)a;
    parameters[4] = (unsigned int)b;
    parameters[5] = (unsigned int)m;
    parameters[6] = (unsigned int)len;
    SVC_Call(parameters);
    if (parameters[0] != SYSCALL_cx_math_multm_ID_OUT) {
        THROW(EXCEPTION_SECURITY);
    }
}

int cx_math_cmp(unsigned char *a, unsigned char *b, int len) {
    unsigned int parameters[2 + 3];
    parameters[0] = (unsigned int)SYSCALL_cx_math_cmp_ID_IN;
    parameters[1] = (unsigned int)G_try_last_open_context->jmp_buf;
    parameters[2] = (unsigned int)a;
    parameters[3] = (unsigned int)b;
    parameters[4] = (unsigned int)len;
    SVC_Call(parameters);
    if (parameters[0] != SYSCALL_cx_math_cmp_ID_OUT) {
        THROW(EXCEPTION_SECURITY);
    }
    return (int)parameters[2];
}

int cx_math_is_zero(unsigned char *a, int len) {
    unsigned int parameters[2 + 2];
    parameters[0] = (unsigned int)SYSCALL_cx_math_is_zero_ID_IN;
    parameters[1] = (unsigned int)G_try_last_open_context->jmp_buf;
    parameters[2] = (unsigned int)a;
    parameters[3] = (unsigned int)len;
    SVC_Call(parameters);
    if (parameters[0] != SYSCALL_cx_math_is_zero_ID_OUT) {
        THROW(EXCEPTION_SECURITY);
    }
    return (int)parameters[2];
}

void cx_math_modm(unsigned char *v, int len_v, unsigned char *m, int len_m) {
    unsigned int parameters[2 + 4];
    parameters[0] = (unsigned int)SYSCALL_cx_math_modm_ID_IN;
    parameters[1] = (unsigned int)G_try_last_open_context->jmp_buf;
    parameters[2] = (unsigned int)v;
    parameters[3] = (unsigned int)len_v;
    parameters[4] = (unsigned int)m;
    parameters[5] = (unsigned int)len_m;
    SVC_Call(parameters);
    if (parameters[0] != SYSCALL_cx_math_modm_ID_OUT) {
        THROW(EXCEPTION_SECURITY);
    }
}

void os_perso_wipe(void) {
    unsigned int parameters[2];
    parameters[0] = (unsigned int)SYSCALL_os_perso_wipe_ID_IN;
    parameters[1] = (unsigned int)G_try_last_open_context->jmp_buf;
    SVC_Call(parameters);
    if (parameters[0] != SYSCALL_os_perso_wipe_ID_OUT) {
        THROW(EXCEPTION_SECURITY);
    }
}

void os_perso_set_pin(unsigned char *pin, unsigned int length) {
    unsigned int parameters[2 + 2];
    parameters[0] = (unsigned int)SYSCALL_os_perso_set_pin_ID_IN;
    parameters[1] = (unsigned int)G_try_last_open_context->jmp_buf;
    parameters[2] = (unsigned int)pin;
    parameters[3] = (unsigned int)length;
    SVC_Call(parameters);
    if (parameters[0] != SYSCALL_os_perso_set_pin_ID_OUT) {
        THROW(EXCEPTION_SECURITY);
    }
}

void os_perso_set_seed(unsigned char *seed, unsigned int length) {
    unsigned int parameters[2 + 2];
    parameters[0] = (unsigned int)SYSCALL_os_perso_set_seed_ID_IN;
    parameters[1] = (unsigned int)G_try_last_open_context->jmp_buf;
    parameters[2] = (unsigned int)seed;
    parameters[3] = (unsigned int)length;
    SVC_Call(parameters);
    if (parameters[0] != SYSCALL_os_perso_set_seed_ID_OUT) {
        THROW(EXCEPTION_SECURITY);
    }
}

void os_perso_finalize(void) {
    unsigned int parameters[2];
    parameters[0] = (unsigned int)SYSCALL_os_perso_finalize_ID_IN;
    parameters[1] = (unsigned int)G_try_last_open_context->jmp_buf;
    SVC_Call(parameters);
    if (parameters[0] != SYSCALL_os_perso_finalize_ID_OUT) {
        THROW(EXCEPTION_SECURITY);
    }
}

unsigned int os_perso_isonboarded(void) {
    unsigned int parameters[2];
    parameters[0] = (unsigned int)SYSCALL_os_perso_isonboarded_ID_IN;
    parameters[1] = (unsigned int)G_try_last_open_context->jmp_buf;
    SVC_Call(parameters);
    if (parameters[0] != SYSCALL_os_perso_isonboarded_ID_OUT) {
        THROW(EXCEPTION_SECURITY);
    }
    return (unsigned int)parameters[2];
}

unsigned int os_global_pin_is_validated(void) {
    unsigned int parameters[2];
    parameters[0] = (unsigned int)SYSCALL_os_global_pin_is_validated_ID_IN;
    parameters[1] = (unsigned int)G_try_last_open_context->jmp_buf;
    SVC_Call(parameters);
    if (parameters[0] != SYSCALL_os_global_pin_is_validated_ID_OUT) {
        THROW(EXCEPTION_SECURITY);
    }
    return (unsigned int)parameters[2];
}

unsigned int os_global_pin_check(unsigned char *pin_buffer,
                                 unsigned char pin_length) {
    unsigned int parameters[2 + 2];
    parameters[0] = (unsigned int)SYSCALL_os_global_pin_check_ID_IN;
    parameters[1] = (unsigned int)G_try_last_open_context->jmp_buf;
    parameters[2] = (unsigned int)pin_buffer;
    parameters[3] = (unsigned int)pin_length;
    SVC_Call(parameters);
    if (parameters[0] != SYSCALL_os_global_pin_check_ID_OUT) {
        THROW(EXCEPTION_SECURITY);
    }
    return (unsigned int)parameters[2];
}

void os_global_pin_invalidate(void) {
    unsigned int parameters[2];
    parameters[0] = (unsigned int)SYSCALL_os_global_pin_invalidate_ID_IN;
    parameters[1] = (unsigned int)G_try_last_open_context->jmp_buf;
    SVC_Call(parameters);
    if (parameters[0] != SYSCALL_os_global_pin_invalidate_ID_OUT) {
        THROW(EXCEPTION_SECURITY);
    }
}

unsigned int os_global_pin_retries(void) {
    unsigned int parameters[2];
    parameters[0] = (unsigned int)SYSCALL_os_global_pin_retries_ID_IN;
    parameters[1] = (unsigned int)G_try_last_open_context->jmp_buf;
    SVC_Call(parameters);
    if (parameters[0] != SYSCALL_os_global_pin_retries_ID_OUT) {
        THROW(EXCEPTION_SECURITY);
    }
    return (unsigned int)parameters[2];
}

unsigned int os_registry_count(void) {
    unsigned int parameters[2];
    parameters[0] = (unsigned int)SYSCALL_os_registry_count_ID_IN;
    parameters[1] = (unsigned int)G_try_last_open_context->jmp_buf;
    SVC_Call(parameters);
    if (parameters[0] != SYSCALL_os_registry_count_ID_OUT) {
        THROW(EXCEPTION_SECURITY);
    }
    return (unsigned int)parameters[2];
}

void os_registry_get(unsigned int index, application_t *out_application_entry) {
    unsigned int parameters[2 + 2];
    parameters[0] = (unsigned int)SYSCALL_os_registry_get_ID_IN;
    parameters[1] = (unsigned int)G_try_last_open_context->jmp_buf;
    parameters[2] = (unsigned int)index;
    parameters[3] = (unsigned int)out_application_entry;
    SVC_Call(parameters);
    if (parameters[0] != SYSCALL_os_registry_get_ID_OUT) {
        THROW(EXCEPTION_SECURITY);
    }
}

void os_sched_exec(unsigned int application_index) {
    unsigned int parameters[2 + 1];
    parameters[0] = (unsigned int)SYSCALL_os_sched_exec_ID_IN;
    parameters[1] = (unsigned int)G_try_last_open_context->jmp_buf;
    parameters[2] = (unsigned int)application_index;
    SVC_Call(parameters);
    if (parameters[0] != SYSCALL_os_sched_exec_ID_OUT) {
        THROW(EXCEPTION_SECURITY);
    }
}

void os_sched_exit(unsigned int exit_code) {
    unsigned int parameters[2 + 1];
    parameters[0] = (unsigned int)SYSCALL_os_sched_exit_ID_IN;
    parameters[1] = (unsigned int)G_try_last_open_context->jmp_buf;
    parameters[2] = (unsigned int)exit_code;
    SVC_Call(parameters);
    if (parameters[0] != SYSCALL_os_sched_exit_ID_OUT) {
        THROW(EXCEPTION_SECURITY);
    }
}

void os_ux_register(bolos_ux_params_t *parameter_ram_pointer) {
    unsigned int parameters[2 + 1];
    parameters[0] = (unsigned int)SYSCALL_os_ux_register_ID_IN;
    parameters[1] = (unsigned int)G_try_last_open_context->jmp_buf;
    parameters[2] = (unsigned int)parameter_ram_pointer;
    SVC_Call(parameters);
    if (parameters[0] != SYSCALL_os_ux_register_ID_OUT) {
        THROW(EXCEPTION_SECURITY);
    }
}

unsigned int os_ux(bolos_ux_params_t *params) {
    unsigned int parameters[2 + 1];
    parameters[0] = (unsigned int)SYSCALL_os_ux_ID_IN;
    parameters[1] = (unsigned int)G_try_last_open_context->jmp_buf;
    parameters[2] = (unsigned int)params;
    SVC_Call(parameters);
    if (parameters[0] != SYSCALL_os_ux_ID_OUT) {
        THROW(EXCEPTION_SECURITY);
    }
    return (unsigned int)parameters[2];
}

void io_seproxyhal_spi_send(unsigned char *buffer, unsigned short length) {
    unsigned int parameters[2 + 2];
    parameters[0] = (unsigned int)SYSCALL_io_seproxyhal_spi_send_ID_IN;
    parameters[1] = (unsigned int)G_try_last_open_context->jmp_buf;
    parameters[2] = (unsigned int)buffer;
    parameters[3] = (unsigned int)length;
    SVC_Call(parameters);
    if (parameters[0] != SYSCALL_io_seproxyhal_spi_send_ID_OUT) {
        THROW(EXCEPTION_SECURITY);
    }
}

unsigned short io_seproxyhal_spi_recv(unsigned char *buffer,
                                      unsigned short maxlength,
                                      unsigned int flags) {
    unsigned int parameters[2 + 3];
    parameters[0] = (unsigned int)SYSCALL_io_seproxyhal_spi_recv_ID_IN;
    parameters[1] = (unsigned int)G_try_last_open_context->jmp_buf;
    parameters[2] = (unsigned int)buffer;
    parameters[3] = (unsigned int)maxlength;
    parameters[4] = (unsigned int)flags;
    SVC_Call(parameters);
    if (parameters[0] != SYSCALL_io_seproxyhal_spi_recv_ID_OUT) {
        THROW(EXCEPTION_SECURITY);
    }
    return (unsigned short)parameters[2];
}

void io_seproxyhal_spi_init(void) {
    unsigned int parameters[2];
    parameters[0] = (unsigned int)SYSCALL_io_seproxyhal_spi_init_ID_IN;
    parameters[1] = (unsigned int)G_try_last_open_context->jmp_buf;
    SVC_Call(parameters);
    if (parameters[0] != SYSCALL_io_seproxyhal_spi_init_ID_OUT) {
        THROW(EXCEPTION_SECURITY);
    }
}
