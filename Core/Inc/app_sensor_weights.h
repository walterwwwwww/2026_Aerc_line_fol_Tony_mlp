#ifndef APP_SENSOR_WEIGHTS_H
#define APP_SENSOR_WEIGHTS_H

#define SENSOR_WEIGHT_DEFAULTS {6.5f, 5.0f, 4.0f, 3.0f, 2.0f, 1.0f, 0.1f, \
                                0.1f, 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.5f}
#define SENSOR_WEIGHT_MAX 100.0f

/* Preserve the exact legacy expression for equal mirror weights. Independent
 * weights only change the contribution of the corresponding sensor. */
static inline float sensor_pair_error(float left, float right,
                                      float left_weight, float right_weight,
                                      int left_black, int right_black)
{
    if (left_black && right_black)
    {
        if (left_weight == right_weight)
            return (right - left) * left_weight;
        return right * right_weight - left * left_weight;
    }
    if (left_black) return -10.0f * left_weight;
    if (right_black) return 10.0f * right_weight;
    return 0.0f;
}
#endif
