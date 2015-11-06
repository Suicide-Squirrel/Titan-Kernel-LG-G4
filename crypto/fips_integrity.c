/*
 * Integrity check code for crypto module.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 */
#include <crypto/hash.h>
#include <crypto/sha.h>
#include <linux/err.h>
#include <linux/scatterlist.h>
#include <asm-generic/sections.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <linux/memblock.h>

#include "internal.h"

// the key for the HMAC-SHA-1 integrity check
#define KEY "LG Electronics Kernel FIPS Module"

static bool need_integrity_check = true;

void fips_integrity_check(void)
{
	u8 *rbuf = 0;
	u32 len;
	u8 hmac[SHA1_DIGEST_SIZE];
	struct hash_desc desc;
	struct scatterlist sg;
	u8 *key = KEY;
    int i, err, step_len = PAGE_SIZE;
    u8 *pAllocBuf = 0;
	u8 *expected = fips_ic_expected();

    printk(KERN_INFO "FIPS: kernel integrity check start\n");

	if (unlikely(fips_error())) {
		printk(KERN_INFO "FIPS: skipping integrity check because of "
			"algorithm self-test errors\n");
		return;
	}

	if (unlikely(!need_integrity_check)) {
        printk(KERN_INFO "FIPS: skipping integrity check because it already passed\n");
		return;
	}

	rbuf = (u8*) phys_to_virt((unsigned long) CONFIG_CRYPTO_FIPS_INTEG_COPY_ADDRESS);

	if (*((u32 *) &rbuf[56]) != 0x644d5241) {
		printk(KERN_ERR "FIPS: invalid image magic number 0x%08x\n", *((u32 *) &rbuf[56]));
		set_fips_error();
		goto err1;
	}

	len = (u32) (__bss_start - _text);

    printk(KERN_INFO "FIPS: kernel integrity Image length = %d\n", len);
    printk(KERN_INFO "FIPS: kernel integrity check address = 0x%08lx \n", (unsigned long) rbuf);

	desc.tfm = crypto_alloc_hash("hmac(sha1)", 0, 0);

	if (IS_ERR(desc.tfm)) {
		printk(KERN_ERR "FIPS: failed to allocate tfm %ld\n",
		       PTR_ERR(desc.tfm));
		set_fips_error();
		goto err1;
	}
#if FIPS_FUNC_TEST == 3
    rbuf[1024] = rbuf[1024] + 1;
#endif
	crypto_hash_setkey(desc.tfm, key, strlen(key));

	pAllocBuf = kmalloc(step_len,GFP_KERNEL);
	if (!pAllocBuf) {
		printk(KERN_INFO "FIPS: failed to allocate memory, length %d\n", step_len);
		set_fips_error();
		goto err1;
	}

	err = crypto_hash_init(&desc);
	if (err) {
		printk(KERN_INFO "FIPS: failed at crypto_hash_init\n");
        set_fips_error();
		kfree(pAllocBuf);
		goto err1;
	}

	for (i = 0; i < len; i += step_len) {
		if (i + step_len >= len - 1)  {
			memcpy(pAllocBuf, &rbuf[i], len - i);
			sg_init_one(&sg, pAllocBuf, len - i);
			err = crypto_hash_update(&desc, &sg, len - i);
			if (err) {
				printk(KERN_INFO "FIPS: failed at crypto_hash_update (1)\n");
                set_fips_error();
				goto err;
			}
			err = crypto_hash_final(&desc, hmac);
			if (err) {
				printk(KERN_INFO "FIPS: failed at crypto_hash_final\n");
                set_fips_error();
				goto err;
			}
		} else {
		    memcpy(pAllocBuf, &rbuf[i], step_len);
		    sg_init_one(&sg, pAllocBuf, step_len);
		    err = crypto_hash_update(&desc, &sg, step_len);

		    if (err) {
			    printk(KERN_INFO "FIPS: failed at crypto_hash_update (2)\n");
                set_fips_error();
			    goto err;
		    }
        }
	}
#if FIPS_FUNC_TEST == 3
    rbuf[1024] = rbuf[1024] - 1;
#endif

	printk(KERN_INFO "FIPS: result of hmac_sha1 is ");
	for (i = 0; i < SHA1_DIGEST_SIZE; i++) {
		printk(KERN_CONT "%02x", hmac[i]);
	}
	printk(KERN_CONT "\n");

	if (!strncmp(hmac, expected, SHA1_DIGEST_SIZE)) {
		printk(KERN_INFO "FIPS: kernel integrity check passed\n");
	} else {
		printk(KERN_ERR "FIPS: kernel integrity check failed, expected value was ");
		for (i = 0; i < SHA1_DIGEST_SIZE; i++) {
			printk(KERN_CONT "%02x", expected[i]);
		}
		set_fips_error();
	}

 err:
	kfree(pAllocBuf);
	crypto_free_hash(desc.tfm);
 err1:
	need_integrity_check = false;
	// free the memory block reserved for the kernel image
	memblock_free((unsigned long) CONFIG_CRYPTO_FIPS_INTEG_COPY_ADDRESS,
			(unsigned long) (__bss_start - _text));
}

EXPORT_SYMBOL_GPL(fips_integrity_check);
