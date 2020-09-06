/*
 *  HT-related code for sigmastar APOLLO driver
 * *
 * Copyright (c) 2016, sigmastar
 * Author:
 *
 * Based on apollo code
 * Copyright (c) 2010, ST-Ericsson
 * Author: Dmitry Tarnyagin <dmitry.tarnyagin@stericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef SSTAR_APOLLO_HT_H_INCLUDED
#define SSTAR_APOLLO_HT_H_INCLUDED

#include <net/Sstar_mac80211.h>

struct Sstar_ht_info {
	struct ieee80211_sta_ht_cap	ht_cap;
	enum nl80211_channel_type	channel_type;
	u16				operation_mode;
};

static inline int Sstar_is_ht(const struct Sstar_ht_info *ht_info)
{
	return ht_info->channel_type != NL80211_CHAN_NO_HT;
}

static inline int Sstar_ht_greenfield(const struct Sstar_ht_info *ht_info)
{
	return Sstar_is_ht(ht_info) &&
		(ht_info->ht_cap.cap & IEEE80211_HT_CAP_GRN_FLD) &&
		!(ht_info->operation_mode &
			IEEE80211_HT_OP_MODE_NON_GF_STA_PRSNT);
}

static inline int Sstar_ht_ampdu_density(const struct Sstar_ht_info *ht_info)
{
	if (!Sstar_is_ht(ht_info))
		return 0;
	return ht_info->ht_cap.ampdu_density;
}

#endif /* SSTAR_APOLLO_HT_H_INCLUDED */
