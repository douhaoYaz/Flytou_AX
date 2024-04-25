/*
    embedtls.h - MbedTLS customization Header

    Override mbedtls-config.h settings
 */

#ifndef _h_EMBEDTLS
#define _h_EMBEDTLS 1

#include "osdep.h"

#if ME_UNIX_LIKE
    #define MBEDTLS_DEPRECATED_WARNING
#endif
#define MBEDTLS_DEPRECATED_REMOVED
#undef  MBEDTLS_SELF_TEST

#if ME_COM_MPR || ME_MPR_PRODUCT || ME_MULTITHREAD
    #define MBEDTLS_THREADING_C
    #define MBEDTLS_THREADING_ALT
    typedef struct MprMutex* mbedtls_threading_mutex_t;
#endif

#if ME_DEBUG
    #define MBEDTLS_SSL_DEBUG_ALL
    #define MBEDTLS_DEBUG_C
#endif
#if ME_CPU_ARCH == ME_CPU_X86 || ME_CPU_ARCH == ME_CPU_X64
    #define MBEDTLS_HAVE_SSE2
#endif

/*
    Map MakeMe configuration into MbedTLS defines.
    If mbedtls.NAME is defined, then override the MbedTLS definition from config.h
    mbedtls.compact defines an optimized general compact/embedded configuration.
 */
#if ME_MBEDTLS_COMPACT
    #undef MBEDTLS_ARC4_C
    #undef MBEDTLS_AES_ROM_TABLES
    #undef MBEDTLS_BLOWFISH_C
    #undef MBEDTLS_CAMELLIA_C
    #undef MBEDTLS_DES_C
    #undef MBEDTLS_KEY_EXCHANGE_PSK_ENABLED
    #undef MBEDTLS_KEY_EXCHANGE_RSA_PSK_ENABLED
    #undef MBEDTLS_KEY_EXCHANGE_DHE_PSK_ENABLED
    #undef MBEDTLS_KEY_EXCHANGE_ECDHE_PSK_ENABLED
    #undef MBEDTLS_PADLOCK_C
    #undef MBEDTLS_PEM_WRITE_C
    #undef MBEDTLS_RIPEMD160_C
    #undef MBEDTLS_SSL3
    #undef MBEDTLS_SSL_PROTO_DTLS
    #undef MBEDTLS_SSL_DTLS_ANTI_REPLAY
    #undef MBEDTLS_SSL_DTLS_HELLO_VERIFY
    #undef MBEDTLS_SSL_DTLS_BADMAC_LIMIT
    #undef MBEDTLS_SSL_DTLS_CLIENT_PORT_REUSE
    #undef MBEDTLS_TIMING_C
    #undef MBEDTLS_VERSION_C
    #undef MBEDTLS_VERSION_FEATURES
    #undef MBEDTLS_XTEA_C
    #define MBEDTLS_SSL_SRV_SUPPORT_SSLV2_CLIENT_HELLO
    #define MBEDTLS_REMOVE_ARC4_CIPHERSUITES
#endif

/*
    Feature selection based on main.me settings.mbedtls configuration.
 */
#if ME_MBEDTLS_AES_ROM_TABLES
    #define MBEDTLS_AES_ROM_TABLES
#elif defined(ME_MBEDTLS_AES_ROM_TABLES) && ME_MBEDTLS_AES_ROM_TABLES == 0
    #undef MBEDTLS_AES_ROM_TABLES
#endif

#if ME_MBEDTLS_ARC4
    #define MBEDTLS_ARC4_C
#elif defined(ME_MBEDTLS_ARC4) && ME_MBEDTLS_ARC4 == 0
    #undef MBEDTLS_ARC4_C
    #define MBEDTLS_REMOVE_ARC4_CIPHERSUITES
#endif

#if ME_MBEDTLS_CAMELLIA
    #define MBEDTLS_CAMELLIA_C
#elif defined(ME_MBEDTLS_CAMELLIA) && ME_MBEDTLS_CAMELLIA == 0
    #undef MBEDTLS_CAMELLIA_C
#endif

#if ME_MBEDTLS_CBC
    #define MBEDTLS_CIPHER_MODE_CBC
#elif defined(ME_MBEDTLS_CBC) && ME_MBEDTLS_CBC == 0
    #undef MBEDTLS_CIPHER_MODE_CBC
#endif

#if ME_MBEDTLS_CCM
    #define MBEDTLS_CCM_C
#elif defined(ME_MBEDTLS_CCM) && ME_MBEDTLS_CCM == 0
    #undef MBEDTLS_CCM_C
#endif

#if ME_MBEDTLS_DES
    #define MBEDTLS_DES_C
#elif defined(ME_MBEDTLS_DES) && ME_MBEDTLS_DES == 0
    #undef MBEDTLS_DES_C
#endif

#if ME_MBEDTLS_PADLOCK
    #define MBEDTLS_PADLOCK_C
#elif defined(ME_MBEDTLS_PADLOCK) && ME_MBEDTLS_PADLOCK == 0
    #undef MBEDTLS_PADLOCK_C
#endif

#if ME_MBEDTLS_PSK
    #define MBEDTLS_KEY_EXCHANGE_PSK_ENABLED
    #define MBEDTLS_KEY_EXCHANGE_RSA_PSK_ENABLED
    #define MBEDTLS_KEY_EXCHANGE_DHE_PSK_ENABLED
    #define MBEDTLS_KEY_EXCHANGE_ECDHE_PSK_ENABLED
#elif defined(ME_MBEDTLS_PSK) && ME_MBEDTLS_PSK == 0
    #undef MBEDTLS_KEY_EXCHANGE_PSK_ENABLED
    #undef MBEDTLS_KEY_EXCHANGE_RSA_PSK_ENABLED
    #undef MBEDTLS_KEY_EXCHANGE_DHE_PSK_ENABLED
    #undef MBEDTLS_KEY_EXCHANGE_ECDHE_PSK_ENABLED
#endif

#if ME_MBEDTLS_XTEA
    #define MBEDTLS_XTEA_C
#elif defined(ME_MBEDTLS_XTEA) && ME_MBEDTLS_XTEA == 0
    #undef MBEDTLS_XTEA_C
#endif

/*
    This is needed for some old clients (baiduspider)
    Default to enabled.
 */
#if ME_MBEDTLS_SSLV2_HELLO
    #define MBEDTLS_SSL_SRV_SUPPORT_SSLV2_CLIENT_HELLO
#elif defined(ME_MBEDTLS_SSLV2_HELLO) && ME_MBEDTLS_SSLV2_HELLO == 0
    #undef MBEDTLS_SSL_SRV_SUPPORT_SSLV2_CLIENT_HELLO
#endif

#ifndef MBEDTLS_SSL_CIPHERSUITES
    /*
        Modified to push down to remove obsolete SHA-1 ciphers
     */
    #define MBEDTLS_SSL_CIPHERSUITES \
    \
    /* All AES-256 ephemeral suites */ \
    MBEDTLS_TLS_ECDHE_ECDSA_WITH_AES_256_GCM_SHA384, \
    MBEDTLS_TLS_ECDHE_RSA_WITH_AES_256_GCM_SHA384, \
    MBEDTLS_TLS_DHE_RSA_WITH_AES_256_GCM_SHA384, \
    MBEDTLS_TLS_ECDHE_ECDSA_WITH_AES_256_CCM, \
    MBEDTLS_TLS_DHE_RSA_WITH_AES_256_CCM, \
    MBEDTLS_TLS_ECDHE_ECDSA_WITH_AES_256_CBC_SHA384, \
    MBEDTLS_TLS_ECDHE_RSA_WITH_AES_256_CBC_SHA384, \
    MBEDTLS_TLS_DHE_RSA_WITH_AES_256_CBC_SHA256, \
    MBEDTLS_TLS_ECDHE_ECDSA_WITH_AES_256_CCM_8, \
    MBEDTLS_TLS_DHE_RSA_WITH_AES_256_CCM_8, \
    \
    /* All AES-128 ephemeral suites */ \
    MBEDTLS_TLS_ECDHE_ECDSA_WITH_AES_128_GCM_SHA256, \
    MBEDTLS_TLS_ECDHE_RSA_WITH_AES_128_GCM_SHA256, \
    MBEDTLS_TLS_DHE_RSA_WITH_AES_128_GCM_SHA256, \
    MBEDTLS_TLS_ECDHE_ECDSA_WITH_AES_128_CCM, \
    MBEDTLS_TLS_DHE_RSA_WITH_AES_128_CCM, \
    MBEDTLS_TLS_ECDHE_ECDSA_WITH_AES_128_CBC_SHA256, \
    MBEDTLS_TLS_ECDHE_RSA_WITH_AES_128_CBC_SHA256, \
    MBEDTLS_TLS_DHE_RSA_WITH_AES_128_CBC_SHA256, \
    MBEDTLS_TLS_ECDHE_ECDSA_WITH_AES_128_CCM_8, \
    MBEDTLS_TLS_DHE_RSA_WITH_AES_128_CCM_8, \
    \
    /* All AES-256 suites */ \
    MBEDTLS_TLS_RSA_WITH_AES_256_GCM_SHA384, \
    MBEDTLS_TLS_RSA_WITH_AES_256_CCM, \
    MBEDTLS_TLS_RSA_WITH_AES_256_CBC_SHA256, \
    MBEDTLS_TLS_ECDH_RSA_WITH_AES_256_GCM_SHA384, \
    MBEDTLS_TLS_ECDH_RSA_WITH_AES_256_CBC_SHA384, \
    MBEDTLS_TLS_ECDH_ECDSA_WITH_AES_256_GCM_SHA384, \
    MBEDTLS_TLS_ECDH_ECDSA_WITH_AES_256_CBC_SHA384, \
    MBEDTLS_TLS_RSA_WITH_AES_256_CCM_8, \
    \
    /* All AES-128 suites */ \
    MBEDTLS_TLS_RSA_WITH_AES_128_GCM_SHA256, \
    MBEDTLS_TLS_RSA_WITH_AES_128_CCM, \
    MBEDTLS_TLS_RSA_WITH_AES_128_CBC_SHA256, \
    MBEDTLS_TLS_ECDH_RSA_WITH_AES_128_GCM_SHA256, \
    MBEDTLS_TLS_ECDH_RSA_WITH_AES_128_CBC_SHA256, \
    MBEDTLS_TLS_ECDH_ECDSA_WITH_AES_128_GCM_SHA256, \
    MBEDTLS_TLS_ECDH_ECDSA_WITH_AES_128_CBC_SHA256, \
    MBEDTLS_TLS_RSA_WITH_AES_128_CCM_8, \
    \
    /* Obsolete Compatibility suites  */ \
    MBEDTLS_TLS_ECDHE_ECDSA_WITH_AES_256_CBC_SHA, \
    MBEDTLS_TLS_ECDHE_RSA_WITH_AES_256_CBC_SHA, \
    MBEDTLS_TLS_DHE_RSA_WITH_AES_256_CBC_SHA, \
    MBEDTLS_TLS_ECDHE_ECDSA_WITH_AES_128_CBC_SHA, \
    MBEDTLS_TLS_ECDHE_RSA_WITH_AES_128_CBC_SHA, \
    MBEDTLS_TLS_DHE_RSA_WITH_AES_128_CBC_SHA, \
    MBEDTLS_TLS_RSA_WITH_AES_256_CBC_SHA, \
    MBEDTLS_TLS_ECDH_RSA_WITH_AES_256_CBC_SHA, \
    MBEDTLS_TLS_ECDH_ECDSA_WITH_AES_256_CBC_SHA, \
    MBEDTLS_TLS_RSA_WITH_AES_128_CBC_SHA, \
    MBEDTLS_TLS_ECDH_RSA_WITH_AES_128_CBC_SHA, \
    MBEDTLS_TLS_ECDH_ECDSA_WITH_AES_128_CBC_SHA
#endif

#endif /* _h_EMBEDTLS */
