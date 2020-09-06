#include "apollo.h"
#include "hwio.h"
#include "fwio.h"
#include "bh.h"
#include "apollo_plat.h"
int Sstar_config_dpll(struct Sstar_common *hw_priv,char* value,int prjType,int dpllClock);
int Sstar_config_dcxo(struct Sstar_common *hw_priv,char *value,int prjType,int dcxoType,int dpllClock);
int Sstar_wait_wlan_rdy(struct Sstar_common *hw_priv);
int Sstar_system_done(struct Sstar_common *hw_priv);
int Sstar_config_jtag_mode(struct Sstar_common *hw_priv);
void Sstar_set_config_to_smu_apolloC(struct Sstar_common *hw_priv,int dpllClock);
void Sstar_set_config_to_smu_apolloB(struct Sstar_common *hw_priv,int dpllClock);
