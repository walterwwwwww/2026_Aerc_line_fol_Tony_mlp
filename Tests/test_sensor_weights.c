#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "app_sensor_weights.h"

int main(void)
{
    float weights[14] = SENSOR_WEIGHT_DEFAULTS;
    float old[7] = {.1f,1.f,2.f,3.f,4.f,5.f,6.5f};
    unsigned seed=1, checked=0;
    for (int i=0;i<7;i++)
    {
        int j=13-i;
        assert(weights[i]==old[6-i] && weights[j]==old[6-i]);
        for (int n=0;n<500;n++)
        {
            seed=seed*1664525u+1013904223u;
            float left=(seed%10001)/1000.f;
            seed=seed*1664525u+1013904223u;
            float right=(seed%10001)/1000.f;
            for (int mode=1;mode<4;mode++)
            {
                int lb=mode&1,rb=mode&2;
                float expected=lb&&rb?(right-left)*old[6-i]:lb?-10.f*old[6-i]:10.f*old[6-i];
                float actual=sensor_pair_error(left,right,weights[i],weights[j],lb,rb);
                assert(memcmp(&expected,&actual,sizeof(float))==0);
                checked++;
            }
        }
        float lw=weights[i]+1.f,rw=weights[j];
        assert(sensor_pair_error(2.f,3.f,lw,rw,1,1)==3.f*rw-2.f*lw);
        assert(sensor_pair_error(2.f,3.f,lw,rw,0,1)==10.f*rw);
        assert(sensor_pair_error(2.f,3.f,lw,rw,1,0)==-10.f*lw);
    }
    printf("PASS: %u legacy cases remain bit-identical; asymmetric sides are independent.\n",checked);
}
