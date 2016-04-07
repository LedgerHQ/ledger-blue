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
#ifndef SYSCALL_DEFS_H
#define SYSCALL_DEFS_H

#define EXC_RETURN_THREAD_PSP 0xFFFFFFFD
#define EXC_RETURN_THREAD_MSP 0xFFFFFFF9
#define EXC_RETURN_HANDLER_MSP 0xFFFFFFF1

// called when entering the SVC Handler C
void syscall_enter(unsigned int entry_lr);

unsigned int syscall_check_app_address(unsigned int *parameters,
                                       unsigned int checked_idx,
                                       unsigned int checked_length);
unsigned int syscall_check_app_flags(unsigned int flags);

void os_sched_hold_r4_r11(void);
void os_sched_task_save_current(void);

#define SYSCALL_nvm_write_ID_IN 0x6000017fUL
#define SYSCALL_nvm_write_ID_OUT 0x900001bcUL
void nvm_write(void *dst_adr, void *src_adr, unsigned int src_len);

#define SYSCALL_cx_rng_u8_ID_IN 0x600002c0UL
#define SYSCALL_cx_rng_u8_ID_OUT 0x90000225UL
unsigned char cx_rng_u8(void);

#define SYSCALL_cx_rng_ID_IN 0x600003a3UL
#define SYSCALL_cx_rng_ID_OUT 0x900003bcUL
unsigned char *cx_rng(unsigned char *buffer, int len);

#define SYSCALL_cx_ripemd160_init_ID_IN 0x6000047fUL
#define SYSCALL_cx_ripemd160_init_ID_OUT 0x900004f8UL
int cx_ripemd160_init(cx_ripemd160_t *hash);

#define SYSCALL_cx_sha256_init_ID_IN 0x600005dbUL
#define SYSCALL_cx_sha256_init_ID_OUT 0x90000564UL
int cx_sha256_init(cx_sha256_t *hash);

#define SYSCALL_cx_sha512_init_ID_IN 0x600006c1UL
#define SYSCALL_cx_sha512_init_ID_OUT 0x900006eeUL
int cx_sha512_init(cx_sha512_t *hash);

#define SYSCALL_cx_hash_ID_IN 0x60000735UL
#define SYSCALL_cx_hash_ID_OUT 0x9000075eUL
int cx_hash(cx_hash_t *hash, int mode, unsigned char *in, int len,
            unsigned char *out);

#define SYSCALL_cx_hmac_ripemd160_init_ID_IN 0x6000089eUL
#define SYSCALL_cx_hmac_ripemd160_init_ID_OUT 0x9000080bUL
int cx_hmac_ripemd160_init(cx_hmac_ripemd160_t *hmac, unsigned char *key,
                           int key_len);

#define SYSCALL_cx_hmac_sha256_init_ID_IN 0x600009a6UL
#define SYSCALL_cx_hmac_sha256_init_ID_OUT 0x90000997UL
int cx_hmac_sha256_init(cx_hmac_sha256_t *hmac, unsigned char *key,
                        int key_len);

#define SYSCALL_cx_hmac_sha512_init_ID_IN 0x60000a79UL
#define SYSCALL_cx_hmac_sha512_init_ID_OUT 0x90000ae9UL
int cx_hmac_sha512_init(cx_hmac_sha512_t *hmac, unsigned char *key,
                        int key_len);

#define SYSCALL_cx_hmac_ID_IN 0x60000b81UL
#define SYSCALL_cx_hmac_ID_OUT 0x90000bc4UL
int cx_hmac(cx_hmac_t *hmac, int mode, unsigned char *in, int len,
            unsigned char *mac);

#define SYSCALL_cx_des_init_key_ID_IN 0x60000c4cUL
#define SYSCALL_cx_des_init_key_ID_OUT 0x90000c37UL
int cx_des_init_key(unsigned char *rawkey, int key_len, cx_des_key_t *key);

#define SYSCALL_cx_des_iv_ID_IN 0x60000dc3UL
#define SYSCALL_cx_des_iv_ID_OUT 0x90000d1aUL
int cx_des_iv(cx_des_key_t *key, int mode, unsigned char *iv, unsigned char *in,
              int len, unsigned char *out);

#define SYSCALL_cx_des_ID_IN 0x60000e56UL
#define SYSCALL_cx_des_ID_OUT 0x90000e4eUL
int cx_des(cx_des_key_t *key, int mode, unsigned char *in, int len,
           unsigned char *out);

#define SYSCALL_cx_aes_init_key_ID_IN 0x60000fb5UL
#define SYSCALL_cx_aes_init_key_ID_OUT 0x90000fe0UL
int cx_aes_init_key(unsigned char *rawkey, int key_len, cx_aes_key_t *key);

#define SYSCALL_cx_aes_iv_ID_IN 0x600010bcUL
#define SYSCALL_cx_aes_iv_ID_OUT 0x9000103aUL
int cx_aes_iv(cx_aes_key_t *key, int mode, unsigned char *iv, unsigned char *in,
              int len, unsigned char *out);

#define SYSCALL_cx_aes_ID_IN 0x600011c3UL
#define SYSCALL_cx_aes_ID_OUT 0x900011eaUL
int cx_aes(cx_aes_key_t *key, int mode, unsigned char *in, int len,
           unsigned char *out);

#define SYSCALL_cx_rsa_init_public_key_ID_IN 0x60001287UL
#define SYSCALL_cx_rsa_init_public_key_ID_OUT 0x9000122eUL
int cx_rsa_init_public_key(unsigned char *exponent, unsigned char *modulus,
                           int modulus_len, cx_rsa_abstract_public_key_t *key);

#define SYSCALL_cx_rsa_verify_ID_IN 0x600013f4UL
#define SYSCALL_cx_rsa_verify_ID_OUT 0x900013f4UL
int cx_rsa_verify(cx_rsa_abstract_public_key_t *key, int mode, cx_md_t hashID,
                  unsigned char *hash, int hash_len, unsigned char *sig,
                  int sig_len);

#define SYSCALL_cx_ecfp_is_valid_point_ID_IN 0x600014a2UL
#define SYSCALL_cx_ecfp_is_valid_point_ID_OUT 0x900014d8UL
int cx_ecfp_is_valid_point(cx_curve_domain_t *domain, unsigned char *point);

#define SYSCALL_cx_ecfp_add_point_ID_IN 0x60001599UL
#define SYSCALL_cx_ecfp_add_point_ID_OUT 0x900015edUL
int cx_ecfp_add_point(cx_curve_domain_t *domain, unsigned char *R,
                      unsigned char *P, unsigned char *Q);

#define SYSCALL_cx_ecdsa_init_public_key_ID_IN 0x600016ecUL
#define SYSCALL_cx_ecdsa_init_public_key_ID_OUT 0x900016b4UL
int cx_ecdsa_init_public_key(cx_curve_t curve, unsigned char *rawkey,
                             int key_len, cx_ecfp_public_key_t *key);

#define SYSCALL_cx_ecdsa_init_private_key_ID_IN 0x60001738UL
#define SYSCALL_cx_ecdsa_init_private_key_ID_OUT 0x9000175aUL
int cx_ecdsa_init_private_key(cx_curve_t curve, unsigned char *rawkey,
                              int key_len, cx_ecfp_private_key_t *key);

#define SYSCALL_cx_ecfp_generate_pair_ID_IN 0x600018b0UL
#define SYSCALL_cx_ecfp_generate_pair_ID_OUT 0x90001891UL
int cx_ecfp_generate_pair(cx_curve_t curve, cx_ecfp_public_key_t *public_key,
                          cx_ecfp_private_key_t *private_key, int keep_private);

#define SYSCALL_cx_ecdsa_sign_ID_IN 0x6000192eUL
#define SYSCALL_cx_ecdsa_sign_ID_OUT 0x90001942UL
int cx_ecdsa_sign(cx_ecfp_private_key_t *key, int mode, cx_md_t hashID,
                  unsigned char *hash, int hash_len, unsigned char *sig);

#define SYSCALL_cx_ecdsa_verify_ID_IN 0x60001a9eUL
#define SYSCALL_cx_ecdsa_verify_ID_OUT 0x90001a6bUL
int cx_ecdsa_verify(cx_ecfp_public_key_t *key, int mode, cx_md_t hashID,
                    unsigned char *hash, int hash_len, unsigned char *sig,
                    int sig_len);

#define SYSCALL_cx_ecdh_ID_IN 0x60001b16UL
#define SYSCALL_cx_ecdh_ID_OUT 0x90001b61UL
int cx_ecdh(cx_ecfp_private_key_t *key, int mode, unsigned char *public_point,
            unsigned char *secret);

#define SYSCALL_cx_crc16_ID_IN 0x60001c47UL
#define SYSCALL_cx_crc16_ID_OUT 0x90001cdcUL
unsigned short cx_crc16(void *buffer, int len);

#define SYSCALL_cx_crc16_update_ID_IN 0x60001d47UL
#define SYSCALL_cx_crc16_update_ID_OUT 0x90001dbdUL
unsigned short cx_crc16_update(unsigned short crc, void *buffer, int len);

#define SYSCALL_cx_math_addm_ID_IN 0x60001e2cUL
#define SYSCALL_cx_math_addm_ID_OUT 0x90001eb3UL
void cx_math_addm(unsigned char *r, unsigned char *a, unsigned char *b,
                  unsigned char *m, int len);

#define SYSCALL_cx_math_multm_ID_IN 0x60001f96UL
#define SYSCALL_cx_math_multm_ID_OUT 0x90001facUL
void cx_math_multm(unsigned char *r, unsigned char *a, unsigned char *b,
                   unsigned char *m, int len);

#define SYSCALL_cx_math_cmp_ID_IN 0x600020a6UL
#define SYSCALL_cx_math_cmp_ID_OUT 0x90002070UL
int cx_math_cmp(unsigned char *a, unsigned char *b, int len);

#define SYSCALL_cx_math_is_zero_ID_IN 0x6000217eUL
#define SYSCALL_cx_math_is_zero_ID_OUT 0x900021c0UL
int cx_math_is_zero(unsigned char *a, int len);

#define SYSCALL_cx_math_modm_ID_IN 0x6000229aUL
#define SYSCALL_cx_math_modm_ID_OUT 0x90002270UL
void cx_math_modm(unsigned char *v, int len_v, unsigned char *m, int len_m);

#define SYSCALL_os_perso_wipe_ID_IN 0x6000232aUL
#define SYSCALL_os_perso_wipe_ID_OUT 0x90002345UL
void os_perso_wipe(void);

#define SYSCALL_os_perso_set_pin_ID_IN 0x600024ccUL
#define SYSCALL_os_perso_set_pin_ID_OUT 0x900024dfUL
void os_perso_set_pin(unsigned char *pin, unsigned int length);

#define SYSCALL_os_perso_set_seed_ID_IN 0x60002565UL
#define SYSCALL_os_perso_set_seed_ID_OUT 0x900025b1UL
void os_perso_set_seed(unsigned char *seed, unsigned int length);

#define SYSCALL_os_perso_finalize_ID_IN 0x60002680UL
#define SYSCALL_os_perso_finalize_ID_OUT 0x90002654UL
void os_perso_finalize(void);

#define SYSCALL_os_perso_isonboarded_ID_IN 0x6000279aUL
#define SYSCALL_os_perso_isonboarded_ID_OUT 0x900027d5UL
unsigned int os_perso_isonboarded(void);

#define SYSCALL_os_global_pin_is_validated_ID_IN 0x60002889UL
#define SYSCALL_os_global_pin_is_validated_ID_OUT 0x90002845UL
unsigned int os_global_pin_is_validated(void);

#define SYSCALL_os_global_pin_check_ID_IN 0x6000296fUL
#define SYSCALL_os_global_pin_check_ID_OUT 0x9000291eUL
unsigned int os_global_pin_check(unsigned char *pin_buffer,
                                 unsigned char pin_length);

#define SYSCALL_os_global_pin_invalidate_ID_IN 0x60002ad0UL
#define SYSCALL_os_global_pin_invalidate_ID_OUT 0x90002afbUL
void os_global_pin_invalidate(void);

#define SYSCALL_os_global_pin_retries_ID_IN 0x60002b59UL
#define SYSCALL_os_global_pin_retries_ID_OUT 0x90002b18UL
unsigned int os_global_pin_retries(void);

#define SYSCALL_os_registry_count_ID_IN 0x60002c40UL
#define SYSCALL_os_registry_count_ID_OUT 0x90002c06UL
unsigned int os_registry_count(void);

#define SYSCALL_os_registry_get_ID_IN 0x60002d65UL
#define SYSCALL_os_registry_get_ID_OUT 0x90002db2UL
void os_registry_get(unsigned int index, application_t *out_application_entry);

#define SYSCALL_os_sched_exec_ID_IN 0x60002e79UL
#define SYSCALL_os_sched_exec_ID_OUT 0x90002e48UL
void os_sched_exec(unsigned int application_index);

#define SYSCALL_os_sched_exit_ID_IN 0x60002fe1UL
#define SYSCALL_os_sched_exit_ID_OUT 0x90002f6fUL
void os_sched_exit(unsigned int exit_code);

#define SYSCALL_os_ux_register_ID_IN 0x60003015UL
#define SYSCALL_os_ux_register_ID_OUT 0x900030b9UL
void os_ux_register(bolos_ux_params_t *parameter_ram_pointer);

#define SYSCALL_os_ux_ID_IN 0x60003158UL
#define SYSCALL_os_ux_ID_OUT 0x9000311fUL
unsigned int os_ux(bolos_ux_params_t *params);

#define SYSCALL_io_seproxyhal_spi_send_ID_IN 0x6000320fUL
#define SYSCALL_io_seproxyhal_spi_send_ID_OUT 0x90003215UL
void io_seproxyhal_spi_send(unsigned char *buffer, unsigned short length);

#define SYSCALL_io_seproxyhal_spi_recv_ID_IN 0x600033d1UL
#define SYSCALL_io_seproxyhal_spi_recv_ID_OUT 0x9000332bUL
unsigned short io_seproxyhal_spi_recv(unsigned char *buffer,
                                      unsigned short maxlength,
                                      unsigned int flags);

#define SYSCALL_io_seproxyhal_spi_init_ID_IN 0x600034aaUL
#define SYSCALL_io_seproxyhal_spi_init_ID_OUT 0x900034cbUL
void io_seproxyhal_spi_init(void);

#endif // SYSCALL_DEFS_H
