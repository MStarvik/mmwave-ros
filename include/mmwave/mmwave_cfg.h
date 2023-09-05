#pragma once

#ifdef __cplusplus
extern "C" {
#endif

int mmwave_cfg_start(const char *dev);
int mmwave_cfg_stop(const char *dev);
int mmwave_cfg_write_file(const char *dev, const char *cfg);

#ifdef __cplusplus
}
#endif
