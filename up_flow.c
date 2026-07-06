// #include "up_flow.h"

// #define LIMIT(x, min, max) (((x) < (min)) ? (min) : (((x) > (max)) ? (max) : (x)))
// #define LPF_1_(hz, t, in, out) ((out) += (1 / (1 + 1 / ((hz) * 3.14f * (t)))) * ((in) - (out)))
// #define safe_div(numerator, denominator, safe_value) ((denominator == 0) ? (safe_value) : ((numerator) / (denominator)))

// void filter_1(float base_hz, float gain_hz, float dT, float in, _filter_1_st *f1) {
//     // 一阶低通滤波
//     LPF_1_(gain_hz, dT, (in - f1->out), f1->a);
//     f1->b = my_pow(in - f1->out);
//     f1->e_nr = LIMIT(safe_div(my_pow(f1->a), ((f1->b) + my_pow(f1->a)), 0), 0, 1);
// }

// _xyz_f_st heading_coordinate_acc;
// _xyz_f_st heading_coordinate_speed_fus;
// _xyz_f_st heading_coordinate_speed_err_i;

// float fx_gyro_fix, fy_gyro_fix;
// float fx_o, fy_o;
// float f_out_x, f_out_y;
// float gyro_lpf_x, gyro_lpf_y;

// _filter_1_st f1_fx;
// _filter_1_st f1_fy;

// float f1_b, f1_g;

// // flow_dat.qual: x,y 方向光流都有效:255, x 有效:2, y 有效:1, 都无效:0;

// float gyro_x, gyro_y;

// flow_fusion(float dT, float fx, float fy, s32 flow_height, u8 pos_hold) {
//     float flow_t1 = 1.0, flow_t2 = 1.0;
//     fx_o = fx * 10 * flow_height; // fx, fy (rad/s) --> flow speed (m/s)
//     fy_o = fy * 10 * flow_height;
//     u32 use_height = 100;

//     if (flow_height < 200) { // input height (cm)
//         use_height = 100;
//     } else if (flow_height < 300) {
//         use_height = 150;
//     } else if (flow_height < 400) {
//         use_height = 200;
//     } else if (flow_height < 500) {
//         use_height = 250;
//     } else if (flow_height < 600) {
//         use_height = 300;
//     } else if (flow_height < 1000) {
//         use_height = 350;
//     } else {
//         use_height = 400;
//     }

//     if (pos_hold == 1) { // in pose hold mode
//         gyro_y = (((s16)sensor.Gyro_deg.y / 2) * 2); // degree per second
//         gyro_x = (((s16)sensor.Gyro_deg.x / 2) * 2);

//         LPF_1_(3.0f, dT, gyro_y, gyro_lpf_y); // gyro low pass filter (delay) for fitting flow data
//         LPF_1_(3.0f, dT, gyro_x, gyro_lpf_x);

//         fx_gyro_fix = ((fx + LIMIT(((gyro_lpf_y) / 57.3f), -flow_t1, flow_t1)) * 10 * use_height); // rotation compensation
//         fy_gyro_fix = ((fy - LIMIT(((gyro_lpf_x) / 57.3f), -flow_t2, flow_t2)) * 10 * use_height);

//         // 保持加速度与光流没有偏航方向的影响
//         vec_3d_transition(&imu_data.z_vec, &imu_data.a_acc, &heading_coordinate_acc);
//         f1_fx.out += ((s32)heading_coordinate_acc.x / 10 * 10) * dT; // integrated acceleration for speed

//         if (flow_dat.qual < 3) {
//             // 光流效果不好时
//         } else if (f1_b < 1e-5f) {
//             f1_b += 0.02f;
//         }

//         filter_1(f1_b, f1_g, dT, fx_gyro_fix, &f1_fx); // flow_data with acc integrated data complementary filtering
//         filter_1(f1_b, f1_g, dT, fy_gyro_fix, &f1_fy);

//         heading_coordinate_speed_fus.x = f1_fx.out; // 融合速度
//         heading_coordinate_speed_fus.y = f1_fy.out;

//         f_out_x = heading_coordinate_speed_fus.x + heading_coordinate_speed_err_i.x * 0.02f; // 最终输出结果
//         f_out_y = heading_coordinate_speed_fus.y + heading_coordinate_speed_err_i.y * 0.02f;

//         heading_coordinate_speed_err_i.x += (fx_gyro_fix - f_out_x) * dT;
//         heading_coordinate_speed_err_i.y += (fy_gyro_fix - f_out_y) * dT;
//     }
// }