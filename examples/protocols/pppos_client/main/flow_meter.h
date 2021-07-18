#ifndef __FLOW_METER_H__
#define __FLOW_METER_H__

void flow_meter_task(void *pvParameter);
void flow_meter_data_get(float *flow_rate, double *flow_total);
#endif