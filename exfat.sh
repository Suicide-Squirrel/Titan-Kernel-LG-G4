#!/bin/sh

export PATH=/home/pavlaras/android/toolchains/aarch64-linux-android-4.9/bin:$PATH
export CROSS_COMPILE=aarch64-linux-android-

sh ./tuxera_update.sh --target target/lg.d/mobile-msm8992-3.10.84 --use-cache --latest --max-cache-entries 2 --source-dir . --output-dir . -a --user lg-mobile --pass AumlTsj0ou
tar -xzf tuxera-exfat*.tgz
perl ./scripts/sign-file sha1 signing_key.priv signing_key.x509 tuxera-exfat*/exfat/kernel-module/texfat.ko
