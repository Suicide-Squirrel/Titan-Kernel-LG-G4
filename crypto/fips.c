/*
 * FIPS 200 support.
 *
 * Copyright (c) 2008 Neil Horman <nhorman@tuxdriver.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 */

#include "internal.h"
#include <crypto/sha.h>

// in FIPS kernel build mode, we have sensible default settings
#ifdef CONFIG_CRYPTO_FIPS
int cc_mode = 0;
int cc_mode_flag = 0;
int fips_enabled = 0;
int fips_panic = 0;
int fips_allow_others = 1;
static u8 fips_ic[SHA1_DIGEST_SIZE];
#else
int cc_mode;
int cc_mode_flag;
int fips_enabled;
int fips_panic;
int fips_allow_others;
static u8 *fips_ic = NULL;
#endif

EXPORT_SYMBOL_GPL(cc_mode);
EXPORT_SYMBOL_GPL(cc_mode_flag);
EXPORT_SYMBOL_GPL(fips_enabled);
EXPORT_SYMBOL_GPL(fips_panic);
EXPORT_SYMBOL_GPL(fips_allow_others);

// note: CC mode and FIPS mode are the same thing here; this exists
// solely for legacy reasons, as does the handling of the cc_mode
// parameter, which also turns FIPS mode on
int get_cc_mode_state(void)
{
	return cc_mode_flag;
}
EXPORT_SYMBOL_GPL(get_cc_mode_state);

/* conversion routine to handle command line HMAC argument */

void hextobin(const char *str, u8 *bytes, size_t blen)
{
   u8 pos;
   u8 idx0;
   u8 idx1;

   // mapping of ASCII characters to hex values
   const uint8_t hashmap[] =
   {
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // ........
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // ........
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // ........
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // ........
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //  !"#$%&'
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // ()*+,-./
     0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, // 01234567
     0x08, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 89:;<=>?
     0x00, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x00, // @ABCDEFG
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // HIJKLMNO
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // PQRSTUVW
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // XYZ[\]^_
     0x00, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x00, // `abcdefg
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // hijklmno
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // pqrstuvw
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // xyz{|}~.
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // ........
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // ........
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // ........
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // ........
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // ........
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // ........
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // ........
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // ........
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // ........
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // ........
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // ........
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // ........
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // ........
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // ........
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // ........
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00  // ........
   };

   memset(bytes, 0, blen);
   for (pos = 0; ((pos < (blen * 2)) && (pos < strlen(str))); pos += 2)
   {
      idx0 = (u8) str[pos + 0];
      idx1 = (u8) str[pos + 1];
      bytes[pos/2] = (u8) (hashmap[idx0] << 4) | hashmap[idx1];
   };
}

/* Process kernel command-line parameter at boot time. fips=0 or fips=1 */
static int fips_enable(char *str)
{
	fips_enabled = !!simple_strtol(str, NULL, 0);
	printk(KERN_INFO "FIPS: FIPS capability %s\n",
		fips_enabled ? "enabled" : "disabled");
	return 1;
}

static int cc_mode_enable(char *str)
{
	cc_mode_flag = simple_strtol(str, NULL, 10);
	cc_mode = cc_mode_flag & 0x01;
	printk(KERN_INFO "CCMODE: CC mode %s\n",
		cc_mode ? "enabled" : "disabled");
	return 1;
}

static int fips_panic_enable(char *str)
{
    fips_panic = !!simple_strtol(str, NULL, 0);
    printk(KERN_INFO "FIPS: panic on test fail %s\n",
        fips_panic ? "enabled" : "disabled");
    return 1;
}

static int fips_allow_others_enable(char *str)
{
    fips_allow_others = !!simple_strtol(str, NULL, 0);
    printk(KERN_INFO "FIPS: non-FIPS algorithms in FIPS mode %s\n",
        fips_allow_others ? "allowed" : "prohibited");
    return 1;
}

#ifdef CONFIG_CRYPTO_FIPS
static int fips_ic_set(char *str)
{
	int i;

	if (strlen(str) != 2 * SHA1_DIGEST_SIZE) {
		printk(KERN_ERR "FIPS: invalid integrity check HMAC parameter %s"
			" (must be %d characters long)\n", str, 2 * SHA1_DIGEST_SIZE);
		memset(fips_ic, 0, SHA1_DIGEST_SIZE);
	} else {
		hextobin(str, fips_ic, SHA1_DIGEST_SIZE);
		printk(KERN_INFO "FIPS: FIPS expected integrity check HMAC = ");
		for (i = 0; i < SHA1_DIGEST_SIZE; i++) {
			printk(KERN_CONT "%02x", fips_ic[i]);
		}
	}

	return 1;
}
#else
static int fips_ic_set(char *str)
{
	printk(KERN_INFO "FIPS: integrity check argument ignored in non-FIPS kernel\n");
	return 1;
}
#endif

u8 *fips_ic_expected(void)
{
	return (u8 *) fips_ic;
}
EXPORT_SYMBOL_GPL(fips_ic_expected);

__setup("fips=", fips_enable);
__setup("cc_mode=", cc_mode_enable);
__setup("fips_panic=", fips_panic_enable);
__setup("fips_allow_others=", fips_allow_others_enable);
__setup("fips_ic=", fips_ic_set);
