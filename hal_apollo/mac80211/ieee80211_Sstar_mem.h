#ifndef _IEEE80211_SSTAR_MEM_H_
#define _IEEE80211_SSTAR_MEM_H_
#if defined (SSTAR_ALLOC_MEM_DEBUG)
extern void *ieee80211_Sstar_kzalloc(size_t s, gfp_t gfp,const char *func);
extern void *ieee80211_Sstar_kmalloc(size_t s, gfp_t gfp,const char *func);
extern void *ieee80211_Sstar_kcalloc(size_t n, size_t size, gfp_t gfp,const char *func);
extern void *ieee80211_Sstar_krealloc(void *p, size_t new_size, gfp_t gfp,const char *func);
extern void ieee80211_Sstar_kfree(void *p);
extern void ieee80211_Sstar_mem_exit(void);
extern void ieee80211_Sstar_mem_int(void);


#define Sstar_kzalloc(_s,_gfp)  					ieee80211_Sstar_kzalloc(_s,_gfp,__func__)
#define Sstar_kmalloc(_s,_gfp)  					ieee80211_Sstar_kmalloc(_s,_gfp,__func__)
#define Sstar_kcalloc(_n,_size,_gfp)  			ieee80211_Sstar_kcalloc(_n,_size,_gfp,__func__)
#define Sstar_krealloc(_p,_new_size,_gfp)		ieee80211_Sstar_krealloc(_p,_new_size,_gfp,__func__)
#define Sstar_kfree(_p)							ieee80211_Sstar_kfree(_p)
#else
#define Sstar_kzalloc(_s,_gfp)  					kzalloc(_s,_gfp)
#define Sstar_kmalloc(_s,_gfp)  					kmalloc(_s,_gfp)
#define Sstar_kcalloc(_n,_size,_gfp)  			kcalloc(_n,_size,_gfp)
#define Sstar_krealloc(_p,_new_size,_gfp)		krealloc(_p,_new_size,_gfp)
#define Sstar_kfree(_p)							kfree(_p)
#define ieee80211_Sstar_mem_exit()
#define ieee80211_Sstar_mem_int()
#endif

#endif  //_SSTAR_P2P_H_

