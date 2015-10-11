#ifndef _FIPS_H
#define _FIPS_H

#ifdef CONFIG_CRYPTO_FIPS
extern int fips_enabled;
extern int fips_panic;
extern int fips_allow_others;
extern int get_fips_error_state(void);
extern int get_cc_mode_state(void);
#else
#define fips_enabled 0
#define fips_panic 0
#define fips_allow_others 0

static inline int get_fips_error_state(void)
{
	return 0;
}
static inline int get_cc_mode_state(void)
{
	return 0;
}
#endif

#endif
